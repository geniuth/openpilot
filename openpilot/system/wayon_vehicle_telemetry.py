#!/usr/bin/env python3
"""ID.4(MEB) 차량 상태를 Wayon Cloud로 올린다.

carState.fuelGauge(= Motor_16의 HV 배터리 에너지량 정규화)와 GPS를 주기적으로 POST해서
휴대폰 앱이 배터리 잔량·주차위치를 볼 수 있게 한다. 충전 전력은 주차 중 배터리량 증가
기울기로 추정한다(MEB_HVEM_01은 이 하네스에 없음).

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


def build_vehicle_block(battery_wh: float | None, charge_power_w: float | None) -> dict:
  vehicle: dict = {}
  if battery_wh is not None and battery_wh > 0:
    vehicle["battery_wh"] = round(battery_wh, 1)
    vehicle["capacity_wh"] = USABLE_BATTERY_WH
    vehicle["soc_percent"] = round(battery_wh / USABLE_BATTERY_WH * 100.0, 1)
  if charge_power_w is not None and charge_power_w >= CHARGING_MIN_W:
    vehicle["charging"] = True
    vehicle["charge_power_w"] = round(charge_power_w, 0)
  else:
    vehicle["charging"] = False
  return vehicle


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
  sm = messaging.SubMaster(["carState", "gpsLocation", "gpsLocationExternal"])

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

    vehicle = build_vehicle_block(battery_wh, charge_power_w)

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
      if battery_wh is not None:
        last = {"battery_wh": battery_wh, "updated_at": payload["updated_at"]}
        save_last_state(last)
      print(f"Wayon telemetry: uploaded soc={vehicle.get('soc_percent')}% "
            f"charging={vehicle.get('charging')}", flush=True)


if __name__ == "__main__":
  main()
