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
# 잔량% 계산 분모. 차량 계기판 표시와 맞추기 위해 64kWh로 고정한다.
# (BMS가 주는 실측 용량은 값이 계속 변해 계기판과 어긋난다 -> SOH 계산에만 쓴다)
USABLE_BATTERY_WH = 64000.0
# ID.4 Pro 총 용량[Wh] (SOH = 현재 만충용량 / 이 값)
NOMINAL_BATTERY_WH = 82000.0
# 충전으로 판정할 최소 증가율[W]
CHARGING_MIN_W = 300.0

# can 구독 소켓 (재사용)
_CAN_SOCK = None


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
  # 분모는 항상 고정값(계기판 일치). capacity_wh(BMS 실측)는 SOH 전용.
  usable = USABLE_BATTERY_WH
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
              "soh_percent", "hv_voltage", "measured_capacity_wh",
              "ac_on", "blower_volt", "blower_level", "seat_heat_left", "seat_heat_right", "recirc"):
    if merged.get(key) is None and last_known.get(key) is not None:
      merged[key] = last_known[key]
    elif merged.get(key) is not None and key in ("battery_wh", "odometer_km"):
      fresh = True
  if not fresh and last_known.get("measured_at"):
    # 값이 실시간이 아니라 '마지막 시동 시점' 측정치임을 앱에 알린다
    merged["measured_at"] = last_known["measured_at"]
    merged["stale"] = True
  return merged


def sample_vehicle_can(timeout_s: float = 6.0) -> dict:
  """차량 값들을 CAN에서 짧게 샘플링한다.

  carState는 card 프로세스가 온로드에서만 돌아 주차 중엔 비어 있다.
  그래서 배터리까지 여기서 직접 디코딩해야 충전 중 모니터링이 된다 (주행 코드 무수정).

  주차/충전 중 실측(2026-08): 아래가 모두 bus1로 수신된다. 주행 중엔 bus0로 오므로
  양쪽 버스를 다 파싱한다.
  - Motor_16.MO_Energieinhalt_BMS      : HV 배터리 에너지량[Wh]  <- 잔량/충전 판정의 핵심
  - HVEM_02.HVEM_Nutzbare_Energie      : 가용 에너지[Wh] (Motor_16과 동일값으로 교차검증됨)
  - MEB_HVEM_01.Battery_Voltage        : HV 전압[V]  (실측 369.2V)
  - BMS_04.BMS_Kapazitaet_02           : 배터리 용량[Ah] -> 실제 용량/SOH
  - Diagnose_01.KBI_Kilometerstand     : 계기판 총 주행거리[km]
  - Klima_Sensor_02.BCM1_Aussen_Temp_ungef : 외기온도[degC] (주행 중에만 수신)
  """
  result: dict = {}
  # 같은 메시지가 주행 중엔 bus0, 주차/충전 중엔 bus1로 온다(실측). 양쪽 다 파싱한다.
  MSGS = [("Motor_16", 2), ("HVEM_02", 10), ("MEB_HVEM_01", 100),
          ("BMS_04", 2), ("Diagnose_01", 1), ("Klima_Sensor_02", 1),
          ("Klima_11", 5), ("Klima_12", 5)]
  try:
    from opendbc.can import CANParser
    parsers = [CANParser("vw_meb", MSGS, bus) for bus in (0, 1)]
  except Exception as exc:
    print(f"Wayon telemetry: CAN parser unavailable: {exc}", flush=True)
    return result

  # 소켓을 매번 새로 열면 첫 호출(프로세스 기동 직후)에 연결 워밍업 때문에 아무것도 못 받는다.
  # 한 번 만들어 재사용한다.
  global _CAN_SOCK
  if _CAN_SOCK is None:
    _CAN_SOCK = messaging.sub_sock("can", timeout=200)
    time.sleep(0.5)          # 구독 성립 대기
    messaging.drain_sock(_CAN_SOCK)
  sock = _CAN_SOCK
  deadline = time.monotonic() + max(0.5, timeout_s)
  hv_voltage = None
  capacity_ah = None
  while time.monotonic() < deadline:
    msgs = messaging.drain_sock(sock)
    if not msgs:
      continue
    frames = [(m.logMonoTime, [(f.address, f.dat, f.src) for f in m.can]) for m in msgs]
    for cp in parsers:
      try:
        cp.update(frames)
      except Exception:
        continue

      # vl은 미수신이어도 기본값 0을 주므로, vl_all(이번 update에서 실제 수신된 값)로 판정한다
      wh = cp.vl_all["Motor_16"].get("MO_Energieinhalt_BMS") or \
           cp.vl_all["HVEM_02"].get("HVEM_Nutzbare_Energie")
      if wh:
        v_wh = wh[-1] if isinstance(wh, (list, tuple)) else wh
        if v_wh > 0:
          result["battery_wh"] = float(v_wh)
      if cp.vl_all["Diagnose_01"].get("KBI_Kilometerstand"):
        odo = cp.vl["Diagnose_01"]["KBI_Kilometerstand"]
        if odo > 0:
          result["odometer_km"] = float(odo)
      if cp.vl_all["Klima_Sensor_02"].get("BCM1_Aussen_Temp_ungef"):
        result["outside_temp_c"] = float(cp.vl["Klima_Sensor_02"]["BCM1_Aussen_Temp_ungef"])
      if cp.vl_all["MEB_HVEM_01"].get("Battery_Voltage"):
        v = cp.vl["MEB_HVEM_01"]["Battery_Voltage"]
        if v > 0:
          hv_voltage = float(v)
      if cp.vl_all["BMS_04"].get("BMS_Kapazitaet_02"):
        ah = cp.vl["BMS_04"]["BMS_Kapazitaet_02"]
        if ah > 0:
          capacity_ah = float(ah)

      # --- 공조 상태 (2026-08 실차 확인: Klima_11=bus1, Klima_12=bus0) ---
      if cp.vl_all["Klima_11"].get("KL_AC_Schalter") is not None:
        result["ac_on"] = bool(cp.vl["Klima_11"]["KL_AC_Schalter"])
      if cp.vl_all["Klima_12"].get("KL_Geblspng_Soll"):
        volt = float(cp.vl["Klima_12"]["KL_Geblspng_Soll"])
        if volt > 0:
          result["blower_volt"] = round(volt, 2)
          # 1.45V=꺼짐, 14V=최대. 0~10단으로 환산해 표시한다.
          result["blower_level"] = max(0, min(10, round((volt - 1.45) / (14.0 - 1.45) * 10)))
      for sig, key in (("KL_SIH_Soll_li", "seat_heat_left"), ("KL_SIH_Soll_re", "seat_heat_right")):
        if cp.vl_all["Klima_12"].get(sig) is not None:
          result[key] = int(cp.vl["Klima_12"][sig])
      if cp.vl_all["Klima_12"].get("KL_Umluftklappe_Status") is not None:
        result["recirc"] = int(cp.vl["Klima_12"]["KL_Umluftklappe_Status"])

    if "battery_wh" in result and "odometer_km" in result:
      break

  # 실제 용량/SOH: BMS 용량[Ah] x HV 전압[V] = 현재 만충 용량[Wh]
  # 주의: 이 값은 온도/충전상태에 따라 세션마다 72~90kWh로 요동친다(실측). 즉 엄밀한 SOH가
  # 아니라 '현재 가용 용량' 추정치다. 그래서 100%를 넘지 않게 자르고, 분모(잔량%)에는 쓰지 않는다.
  if capacity_ah and hv_voltage:
    capacity_wh = capacity_ah * hv_voltage
    if 20000 < capacity_wh < 150000:  # 상식 범위 밖이면 스케일 해석이 틀린 것 -> 버림
      result["measured_capacity_wh"] = round(capacity_wh, 0)
      result["soh_percent"] = round(min(100.0, capacity_wh / NOMINAL_BATTERY_WH * 100.0), 1)
  if hv_voltage:
    result["hv_voltage"] = round(hv_voltage, 1)
  return result


def haversine_m(a: tuple, b: tuple) -> float:
  """두 위경도 사이 거리[m]."""
  import math
  lat1, lon1 = a
  lat2, lon2 = b
  r = 6371000.0
  p1, p2 = math.radians(lat1), math.radians(lat2)
  dp = math.radians(lat2 - lat1)
  dl = math.radians(lon2 - lon1)
  h = math.sin(dp / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2
  return 2 * r * math.asin(min(1.0, math.sqrt(h)))


class TripRecorder:
  """주행 시작~종료를 기록해 Wayon Cloud에 올린다.

  onroad 전환으로 시작/종료를 잡고, 주행 중 GPS를 샘플링해 경로와 거리를 만든다.
  이게 없으면 앱의 '주행 이력'이 계속 비어 있다 (기존에 업로더가 아예 없었음).
  """

  MIN_POINT_GAP_S = 5.0      # 경로 점 최소 간격
  MIN_TRIP_M = 300.0         # 이보다 짧으면 버림(주차장 이동 등)
  MAX_POINTS = 720           # 한 트립 최대 점 수 (1시간 @5초)

  def __init__(self):
    self.active = False
    self.started_at = None
    self.route: list = []
    self.distance_m = 0.0
    self.last_point = None
    self.last_sample_at = 0.0
    self.trip_id = None

  def start(self):
    import uuid
    self.active = True
    self.started_at = utc_now()
    self.route = []
    self.distance_m = 0.0
    self.last_point = None
    self.last_sample_at = 0.0
    self.trip_id = str(uuid.uuid4())
    print("Wayon telemetry: trip started", flush=True)

  def add_point(self, lat: float, lon: float, speed_mps: float | None, now: float):
    if not self.active or now - self.last_sample_at < self.MIN_POINT_GAP_S:
      return
    self.last_sample_at = now
    if self.last_point is not None:
      d = haversine_m(self.last_point, (lat, lon))
      if d < 2000:      # GPS 튐 방지
        self.distance_m += d
    self.last_point = (lat, lon)
    if len(self.route) < self.MAX_POINTS:
      point = {"latitude": round(lat, 6), "longitude": round(lon, 6), "t": utc_now()}
      if speed_mps is not None:
        point["speedMps"] = round(speed_mps, 2)
      self.route.append(point)

  def finish(self, config: dict, device_id: str) -> bool:
    """주행 종료 -> 업로드. 성공/실패와 무관하게 상태는 초기화한다."""
    if not self.active:
      return False
    self.active = False
    if self.distance_m < self.MIN_TRIP_M or len(self.route) < 2:
      print(f"Wayon telemetry: trip discarded ({self.distance_m:.0f}m)", flush=True)
      return False

    ended_at = utc_now()
    from datetime import datetime
    try:
      t0 = datetime.strptime(self.started_at, "%Y-%m-%dT%H:%M:%SZ")
      t1 = datetime.strptime(ended_at, "%Y-%m-%dT%H:%M:%SZ")
      duration_s = max(0, int((t1 - t0).total_seconds()))
    except Exception:
      duration_s = 0

    payload = {
      "id": self.trip_id,
      "deviceId": device_id,
      "startedAt": self.started_at,
      "endedAt": ended_at,
      "durationS": duration_s,
      "distanceM": round(self.distance_m, 1),
      "route": self.route,
    }
    ok = post_json(config, "/api/trips", payload)
    print(f"Wayon telemetry: trip upload {'ok' if ok else 'FAILED'} "
          f"({self.distance_m / 1000:.2f}km, {len(self.route)}pts)", flush=True)
    return ok


def post_json(config: dict, path: str, payload: dict) -> bool:
  endpoint = str(config.get("endpoint") or "").rstrip("/")
  token = str(config.get("token") or "")
  if not endpoint or not token:
    return False
  try:
    response = requests.post(
      f"{endpoint}{path}",
      json=payload,
      headers={"Authorization": f"Bearer {token}", "Content-Type": "application/json"},
      timeout=(10, 30),
    )
    return 200 <= response.status_code < 300
  except requests.RequestException as exc:
    print(f"Wayon telemetry: POST {path} failed: {exc}", flush=True)
    return False


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
  last_upload_at = None   # None = 아직 한 번도 안 올림 -> 즉시 업로드
  charge_power_w = None
  trip = TripRecorder()
  onroad_prev = None

  print("Wayon telemetry: started", flush=True)

  while True:
    time.sleep(1.0)
    sm.update(0)

    config = read_config()
    if not config.get("endpoint") or not config.get("token"):
      continue

    onroad = params.get_bool("IsOnroad")
    now = time.monotonic()

    dongle_id = params.get("DongleId")
    if isinstance(dongle_id, bytes):
      dongle_id = dongle_id.decode("utf-8", "replace")
    device_id = str(config.get("device_id") or dongle_id or "unknown")

    # --- 주행 기록: onroad 전환으로 시작/종료 ---
    if onroad_prev is not None and onroad != onroad_prev:
      if onroad:
        trip.start()
      else:
        trip.finish(config, device_id)
    onroad_prev = onroad

    if onroad and trip.active:
      for svc in ("gpsLocationExternal", "gpsLocation"):
        if sm.valid.get(svc) and sm.seen.get(svc):
          g = sm[svc]
          if g.hasFix:
            trip.add_point(float(g.latitude), float(g.longitude), float(g.speed), now)
            break

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
    # 부팅 직후엔 monotonic이 작아 'now - 0 < interval'이 참이 되어 첫 업로드가
    # 최대 10분 지연되던 문제가 있었다. 첫 회는 무조건 올린다.
    if last_upload_at is not None and now - last_upload_at < interval and not battery_moved:
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
    for key in ("odometer_km", "outside_temp_c", "soh_percent", "hv_voltage", "measured_capacity_wh",
                "ac_on", "blower_volt", "blower_level", "seat_heat_left", "seat_heat_right", "recirc"):
      if sampled.get(key) is not None:
        vehicle[key] = sampled[key]

    # 12V 보조 배터리: 판다가 차량 전원선에서 측정한 입력 전압
    if sm.valid.get("peripheralState") and sm.seen.get("peripheralState"):
      mv = sm["peripheralState"].voltage
      # 차가 잠들면 판다가 자체 대기전원(~5V)을 보고한다. 12V계 실측으로 볼 수 있는
      # 범위(9V 이상)일 때만 채택한다.
      if mv and mv >= 9000:
        vehicle["aux_voltage"] = round(mv / 1000.0, 2)

    # 이번에 새로 측정된 값이 있으면 '마지막 알려진 값'으로 저장
    for key in ("battery_wh", "capacity_wh", "soc_percent", "odometer_km", "outside_temp_c",
                "soh_percent", "hv_voltage", "aux_voltage", "measured_capacity_wh",
                "ac_on", "blower_volt", "blower_level", "seat_heat_left", "seat_heat_right", "recirc"):
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
