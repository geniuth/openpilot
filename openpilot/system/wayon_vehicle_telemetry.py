#!/usr/bin/env python3
"""ID.4(MEB) 차량 상태를 Wayon Cloud로 올린다.

배터리 잔량·오도미터·외기온·12V 전압과 GPS를 주기적으로 POST해서 휴대폰 앱이
차 상태를 볼 수 있게 한다.

배터리는 CAN(Motor_16)에서 직접 디코딩한다. carState는 card 프로세스가 온로드에서만
돌아 주차 중엔 비어 있어, 충전 중 감시가 되려면 CAN 직접 샘플링이어야 한다.
충전 전력은 배터리량 증가 기울기로 추정한다(전용 신호 MEB_HVEM_01 수신 여부 미검증).

- 주행 중: TELEMETRY_ONROAD_S 간격
- 주차 중: TELEMETRY_OFFROAD_S 간격 + 배터리 변화 감지 시 즉시
- /data/wayon_cloud/config.json 의 endpoint/token 이 있어야 동작 (없으면 조용히 대기)
"""
import json
import os
import time
from pathlib import Path

import requests

from openpilot.cereal import messaging
from openpilot.common.params import Params

CONFIG_PATH = Path(os.getenv("WAYON_CLOUD_CONFIG", "/data/wayon_cloud/config.json"))
STATE_PATH = Path(os.getenv("WAYON_VEHICLE_STATE", "/data/wayon_cloud/vehicle_state.json"))

TELEMETRY_ONROAD_S = 30.0
TELEMETRY_OFFROAD_S = 600.0
# 주차 중 배터리량이 이만큼(Wh) 변하면 즉시 업로드 -> 충전 시작/종료를 빨리 반영
BATTERY_DELTA_TRIGGER_WH = 300.0
# ID.4 Pro 사용가능 용량[Wh] (carstate의 MEB_USABLE_BATTERY_WH와 같은 값)
USABLE_BATTERY_WH = 77000.0
# ID.4 Pro 총 용량[Wh] (SOH = 현재 만충용량 / 이 값)
NOMINAL_BATTERY_WH = 82000.0
# 충전으로 판정할 최소 증가율[W]
CHARGING_MIN_W = 300.0


def read_config() -> dict:
  try:
    with CONFIG_PATH.open("r", encoding="utf-8") as handle:
      config = json.load(handle)
      return config if isinstance(config, dict) else {}
  except (OSError, ValueError):
    return {}


def load_last_state() -> dict:
  try:
    return json.loads(STATE_PATH.read_text(encoding="utf-8"))
  except (OSError, ValueError):
    return {}


def save_last_state(payload: dict) -> None:
  STATE_PATH.parent.mkdir(parents=True, exist_ok=True)
  tmp = STATE_PATH.with_suffix(".tmp")
  tmp.write_text(json.dumps(payload, separators=(",", ":")), encoding="utf-8")
  os.replace(tmp, STATE_PATH)


def utc_now() -> str:
  from datetime import datetime, timezone
  return datetime.now(timezone.utc).isoformat(timespec="seconds").replace("+00:00", "Z")


def build_vehicle_block(battery_wh: float | None, charge_power_w: float | None,
                        capacity_wh: float | None = None) -> dict:
  vehicle: dict = {}
  usable = capacity_wh if (capacity_wh and capacity_wh > 0) else USABLE_BATTERY_WH
  if battery_wh is not None and battery_wh > 0:
    vehicle["battery_wh"] = round(battery_wh, 1)
    vehicle["capacity_wh"] = round(usable, 0)
    vehicle["soc_percent"] = round(min(100.0, battery_wh / usable * 100.0), 1)
  if charge_power_w is not None and charge_power_w >= CHARGING_MIN_W:
    vehicle["charging"] = True
    vehicle["charge_power_w"] = round(charge_power_w, 0)
  else:
    vehicle["charging"] = False
  return vehicle


def merge_last_known(vehicle: dict, last_known: dict) -> dict:
  """차가 꺼지면 CAN이 끊겨 값이 사라진다. 마지막 측정값을 채워
  주차 후에도 앱이 '주차 시점의 잔량/주행거리'를 보여주게 한다."""
  merged = dict(vehicle)
  fresh = False
  for key in ("battery_wh", "capacity_wh", "soc_percent", "odometer_km", "outside_temp_c",
              "soh_percent", "hv_voltage"):
    if merged.get(key) is None and last_known.get(key) is not None:
      merged[key] = last_known[key]
    elif merged.get(key) is not None and key in ("battery_wh", "odometer_km"):
      fresh = True
  if not fresh and last_known.get("measured_at"):
    # 값이 실시간이 아니라 '마지막 시동 시점' 측정치임을 앱에 알린다
    merged["measured_at"] = last_known["measured_at"]
    merged["stale"] = True
  return merged


def sample_vehicle_can(timeout_s: float = 4.0) -> dict:
  """차량 값들을 CAN에서 짧게 샘플링한다.

  carState는 card 프로세스가 온로드에서만 돌아 주차 중엔 비어 있다.
  그래서 배터리까지 여기서 직접 디코딩해야 충전 중 모니터링이 된다 (주행 코드 무수정).

  bus0
  - Motor_16.MO_Energieinhalt_BMS      : HV 배터리 에너지량[Wh]  <- 잔량/충전 판정의 핵심
  - Diagnose_01.KBI_Kilometerstand     : 계기판 총 주행거리[km]
  bus1
  - Klima_Sensor_02.BCM1_Aussen_Temp_ungef : 외기온도[degC]
  - BMS_04.BMS_Kapazitaet_02           : 배터리 용량[Ah] -> 실제 용량/SOH (수신 여부 미검증)
  - MEB_HVEM_01.Battery_Voltage        : HV 전압[V]      (수신 여부 미검증)

  BMS_04/MEB_HVEM_01은 이 하네스에서 수신되는지 아직 확인 전이라, 값이 오면 쓰고
  없으면 조용히 건너뛴다(공칭 용량으로 폴백).
  """
  result: dict = {}
  try:
    from opendbc.can import CANParser
    cp0 = CANParser("vw_meb", [("Motor_16", 10), ("Diagnose_01", 1)], 0)
    cp1 = CANParser("vw_meb", [("Klima_Sensor_02", 1), ("BMS_04", 1), ("MEB_HVEM_01", 1)], 1)
  except Exception as exc:
    print(f"Wayon telemetry: CAN parser unavailable: {exc}", flush=True)
    return result

  sock = messaging.sub_sock("can", timeout=200)
  deadline = time.monotonic() + max(0.5, timeout_s)
  hv_voltage = None
  capacity_ah = None
  while time.monotonic() < deadline:
    msgs = messaging.drain_sock(sock)
    if not msgs:
      continue
    frames = [(m.logMonoTime, [(f.address, f.dat, f.src) for f in m.can]) for m in msgs]
    try:
      cp0.update(frames)
      cp1.update(frames)
    except Exception:
      break

    # vl은 미수신이어도 기본값 0을 주므로, vl_all(이번 update에서 실제 수신된 값들)로 판정한다
    if cp0.vl_all["Motor_16"].get("MO_Energieinhalt_BMS"):
      wh = cp0.vl["Motor_16"]["MO_Energieinhalt_BMS"]
      if wh > 0:
        result["battery_wh"] = float(wh)
    if cp0.vl_all["Diagnose_01"].get("KBI_Kilometerstand"):
      odo = cp0.vl["Diagnose_01"]["KBI_Kilometerstand"]
      if odo > 0:
        result["odometer_km"] = float(odo)
    if cp1.vl_all["Klima_Sensor_02"].get("BCM1_Aussen_Temp_ungef"):
      result["outside_temp_c"] = float(cp1.vl["Klima_Sensor_02"]["BCM1_Aussen_Temp_ungef"])
    if cp1.vl_all["MEB_HVEM_01"].get("Battery_Voltage"):
      v = cp1.vl["MEB_HVEM_01"]["Battery_Voltage"]
      if v > 0:
        hv_voltage = float(v)
    if cp1.vl_all["BMS_04"].get("BMS_Kapazitaet_02"):
      ah = cp1.vl["BMS_04"]["BMS_Kapazitaet_02"]
      if ah > 0:
        capacity_ah = float(ah)

    if "battery_wh" in result and "odometer_km" in result and "outside_temp_c" in result:
      break

  # 실제 용량/SOH: BMS 용량[Ah] x HV 전압[V] = 현재 만충 용량[Wh]
  if capacity_ah and hv_voltage:
    capacity_wh = capacity_ah * hv_voltage
    if 20000 < capacity_wh < 150000:  # 상식 범위 밖이면 스케일 해석이 틀린 것 -> 버림
      result["capacity_wh"] = round(capacity_wh, 0)
      result["soh_percent"] = round(capacity_wh / NOMINAL_BATTERY_WH * 100.0, 1)
  if hv_voltage:
    result["hv_voltage"] = round(hv_voltage, 1)
  return result


def post_state(config: dict, payload: dict) -> bool:
  endpoint = str(config.get("endpoint") or "").rstrip("/")
  token = str(config.get("token") or "")
  if not endpoint or not token:
    return False
  try:
    response = requests.post(
      f"{endpoint}/api/telemetry",
      json=payload,
      headers={"Authorization": f"Bearer {token}", "Content-Type": "application/json"},
      timeout=(10, 30),
    )
    return 200 <= response.status_code < 300
  except requests.RequestException as exc:
    print(f"Wayon telemetry: upload failed: {exc}", flush=True)
    return False


def main() -> None:
  params = Params()
  # carrot에는 liveLocationKalman이 없다 (gpsLocation / gpsLocationExternal 사용)
  sm = messaging.SubMaster(["carState", "gpsLocation", "gpsLocationExternal", "peripheralState"])

  last = load_last_state()
  last_battery_wh = last.get("battery_wh")
  last_battery_at = None  # monotonic, 이번 실행에서만 사용
  last_upload_at = 0.0
  charge_power_w = None

  print("Wayon telemetry: started", flush=True)

  while True:
    time.sleep(1.0)
    sm.update(0)

    config = read_config()
    if not config.get("endpoint") or not config.get("token"):
      continue

    onroad = params.get_bool("IsOnroad")
    now = time.monotonic()

    # 배터리 잔량: 주행 중엔 carState(가벼움), 주차 중엔 CAN 직접 샘플링으로 얻는다.
    # (card 프로세스가 온로드 전용이라 주차 중엔 carState가 비어 충전 감시가 안 됐다)
    battery_wh = None
    if sm.updated["carState"] and sm.valid["carState"]:
      gauge = sm["carState"].fuelGauge
      if gauge > 0:
        battery_wh = gauge * USABLE_BATTERY_WH

    # 주차 중 배터리 증가 기울기로 충전 전력 추정
    if battery_wh is not None:
      if last_battery_wh is not None and last_battery_at is not None:
        dt_h = (now - last_battery_at) / 3600.0
        if dt_h > 0.02:  # 최소 ~1분 간격에서만 산출 (노이즈 방지)
          delta_wh = battery_wh - last_battery_wh
          charge_power_w = delta_wh / dt_h if delta_wh > 0 else None
      if last_battery_wh is None or last_battery_at is None or \
         abs(battery_wh - (last_battery_wh or 0)) >= BATTERY_DELTA_TRIGGER_WH:
        last_battery_wh = battery_wh
        last_battery_at = now

    interval = TELEMETRY_ONROAD_S if onroad else TELEMETRY_OFFROAD_S
    battery_moved = (
      battery_wh is not None and last.get("battery_wh") is not None
      and abs(battery_wh - float(last["battery_wh"])) >= BATTERY_DELTA_TRIGGER_WH
    )
    if now - last_upload_at < interval and not battery_moved:
      continue

    # 배터리/오도미터/외기온·용량은 CAN에서 직접 샘플링 (업로드 직전에만).
    # 차가 완전히 잠들면 CAN이 없어 빈 dict가 온다.
    sampled = sample_vehicle_can()

    # 주차 중엔 carState가 없으므로 CAN 샘플값이 배터리의 유일한 소스가 된다.
    if sampled.get("battery_wh"):
      battery_wh = sampled["battery_wh"]
      # 충전 판정도 이 값으로 다시 계산 (증가 기울기)
      if last_battery_at is not None and last_battery_wh is not None:
        dt_h = (now - last_battery_at) / 3600.0
        if dt_h > 0.02:
          delta_wh = battery_wh - last_battery_wh
          charge_power_w = delta_wh / dt_h if delta_wh > 0 else None
      if last_battery_at is None or abs(battery_wh - (last_battery_wh or 0)) >= BATTERY_DELTA_TRIGGER_WH:
        last_battery_wh, last_battery_at = battery_wh, now

    vehicle = build_vehicle_block(battery_wh, charge_power_w, sampled.get("capacity_wh"))
    for key in ("odometer_km", "outside_temp_c", "soh_percent", "hv_voltage"):
      if sampled.get(key) is not None:
        vehicle[key] = sampled[key]

    # 12V 보조 배터리: 판다가 차량 전원선에서 측정한 입력 전압
    if sm.valid.get("peripheralState") and sm.seen.get("peripheralState"):
      mv = sm["peripheralState"].voltage
      if mv and mv > 0:
        vehicle["aux_voltage"] = round(mv / 1000.0, 2)

    # 이번에 새로 측정된 값이 있으면 '마지막 알려진 값'으로 저장
    for key in ("battery_wh", "capacity_wh", "soc_percent", "odometer_km", "outside_temp_c",
                "soh_percent", "hv_voltage", "aux_voltage"):
      if vehicle.get(key) is not None:
        last[key] = vehicle[key]
        last["measured_at"] = utc_now()
    vehicle = merge_last_known(vehicle, last)

    # Worker(/api/telemetry)는 camelCase + 중첩 gps 를 기대하며, 페이로드 전체를 raw_json 에 저장한다.
    # 앱은 raw_json.vehicle 을 읽어 배터리/충전 정보를 표시한다.
    dongle = params.get("DongleId")
    if isinstance(dongle, bytes):
      dongle = dongle.decode("utf-8", "replace")
    payload: dict = {
      "deviceId": str(config.get("device_id") or dongle or "unknown"),
      "updatedAt": utc_now(),
      "onroad": 1 if onroad else 0,
      "ignition": 1 if onroad else 0,
      "enabled": 0,
      "vehicle": vehicle,
      "gps": {},
    }

    for gps_service in ("gpsLocationExternal", "gpsLocation"):
      if sm.valid.get(gps_service) and sm.seen.get(gps_service):
        gps = sm[gps_service]
        if gps.hasFix:
          payload["gps"]["latitude"] = float(gps.latitude)
          payload["gps"]["longitude"] = float(gps.longitude)
          payload["gps"]["speedMps"] = float(gps.speed)
          payload["gps"]["bearingDeg"] = float(gps.bearingDeg)
          payload["gps"]["accuracyM"] = float(gps.horizontalAccuracy)
          break

    if post_state(config, payload):
      last_upload_at = now
      last["updated_at"] = payload["updatedAt"]
      save_last_state(last)   # 마지막 측정값 유지 (주차 후에도 표시하기 위함)
      print(f"Wayon telemetry: uploaded soc={vehicle.get('soc_percent')}% "
            f"odo={vehicle.get('odometer_km')} charging={vehicle.get('charging')} "
            f"stale={vehicle.get('stale', False)}", flush=True)


if __name__ == "__main__":
  main()
