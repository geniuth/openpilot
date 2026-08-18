#!/usr/bin/env python3
"""ID.4 원격 예열/예냉 — 기기측 명령 수신 + 언락 재생.

클라우드(Worker) 명령 큐를 짧은 주기로 폴링해서, preheat 명령이 오면
언락 신호(Blinkmodi_02.BM_ZV_auf)를 재현한다. 차량에 '잠금 해제 시 독립
공조 작동' 옵션이 켜져 있으면 이걸로 예열/예냉이 시작된다.

=== CAN 송신 경로 ===
정식 경로는 messaging.pub_sock("sendcan") 이다. 여기로 프레임을 publish 하면
pandad 가 받아 판다 safety hook 을 통과시킨 뒤 실제 버스로 내보낸다.
(재생검증 도구의 allOutput 은 일회성 검증용, 이건 상시 서비스 경로다.)

*** 전제: Blinkmodi_02 가 판다 safety TX 화이트리스트에 있어야 한다. ***
없으면 pandad 가 이 프레임을 '차단'한다. 그 경우 언락은 안 되고, 우리는
Klima_11(공조)에서 변화를 못 봐서 결과를 "blocked" 로 보고한다.

=== 안전장치 ===
- 주행 중(onroad)에는 절대 송신하지 않는다. 폴링도 오프로드에서만.
- config.json 에 preheat_enabled: true 가 있어야 실제 송신한다.
  없으면 명령을 받아도 dry-run(로그만) — safety 패치 검증 전 오작동 방지.
- 재생은 짧게(기본 3프레임 × 0.1s). 무한 반복 금지.
"""
import os
import sys
import time
import json
from pathlib import Path

sys.path.insert(0, "/data/openpilot")

import requests

from openpilot.cereal import messaging
from openpilot.common.params import Params
from openpilot.selfdrive.pandad import can_list_to_can_capnp

CONFIG_PATH = Path(os.getenv("WAYON_CLOUD_CONFIG", "/data/wayon_cloud/config.json"))

# Blinkmodi_02 = 0x366, 언락 비트 BM_ZV_auf. bus1 에서 관측됨(정찰 확인).
UNLOCK_MSG = "Blinkmodi_02"
UNLOCK_BUS = 1
# Klima_11 = 0x3B5, KL_AC_Schalter = byte0 bit2 (공조 반영 확인용)
KLIMA_11 = 0x3B5
AC_BYTE, AC_BIT = 0, 2

POLL_OFFROAD_S = 15.0     # 오프로드일 때 명령 폴링 주기
POLL_ONROAD_S = 120.0     # 주행 중엔 느리게(어차피 송신 안 함)
REPLAY_FRAMES = 3         # 언락 프레임 재생 횟수
REPLAY_INTERVAL_S = 0.1
AC_POLL_S = 8.0           # 재생 후 공조 반영 확인 시간


def read_config() -> dict:
  try:
    with CONFIG_PATH.open("r", encoding="utf-8") as handle:
      config = json.load(handle)
      return config if isinstance(config, dict) else {}
  except (OSError, ValueError):
    return {}


def poll_command(config: dict, device_id: str) -> dict | None:
  """대기 중인 명령을 가져온다(있으면 서버가 taken 으로 바꾼다)."""
  endpoint = str(config.get("endpoint") or "").rstrip("/")
  token = str(config.get("token") or "")
  if not endpoint or not token:
    return None
  try:
    r = requests.get(
      f"{endpoint}/api/command",
      params={"deviceId": device_id},
      headers={"Authorization": f"Bearer {token}"},
      timeout=(10, 20),
    )
    if r.status_code != 200:
      return None
    return r.json().get("command")
  except requests.RequestException:
    return None


def ack_command(config: dict, device_id: str, cmd_id: str, state: str, result: str) -> None:
  endpoint = str(config.get("endpoint") or "").rstrip("/")
  token = str(config.get("token") or "")
  if not endpoint or not token:
    return
  try:
    requests.post(
      f"{endpoint}/api/command/ack",
      json={"deviceId": device_id, "id": cmd_id, "state": state, "result": result},
      headers={"Authorization": f"Bearer {token}", "Content-Type": "application/json"},
      timeout=(10, 20),
    )
  except requests.RequestException as exc:
    print(f"Wayon preheat: ack 실패: {exc}", flush=True)


def replay_unlock(sendcan, sm) -> str:
  """언락 프레임을 sendcan 으로 재생하고, 공조 반영을 확인한다.
  반환: 'ac_on' / 'no_ac' / 'blocked' (blocked 는 Klima 자체를 못 받은 경우)."""
  try:
    from opendbc.can import CANPacker
    packer = CANPacker("vw_meb")
  except Exception as exc:
    print(f"Wayon preheat: CANPacker 사용 불가: {exc}", flush=True)
    return "no_ac"

  # Blinkmodi_02 는 카운터/체크섬이 없어 BM_ZV_auf=1 프레임을 그대로 반복 송신하면 된다.
  addr, dat, _ = packer.make_can_msg(UNLOCK_MSG, UNLOCK_BUS, {"BM_ZV_auf": 1})
  if not dat:
    print("Wayon preheat: 언락 프레임 생성 실패", flush=True)
    return "no_ac"

  print(f"Wayon preheat: 언락 재생 — {UNLOCK_MSG} bus{UNLOCK_BUS} x{REPLAY_FRAMES} "
        f"[{bytes(dat).hex(' ').upper()}]", flush=True)
  frame = [(addr, bytes(dat), UNLOCK_BUS)]   # [(address, data, src)]
  for _ in range(REPLAY_FRAMES):
    sendcan.send(can_list_to_can_capnp(frame, msgtype='sendcan'))
    time.sleep(REPLAY_INTERVAL_S)

  # 공조(Klima_11) 반영 확인
  print(f"Wayon preheat: 공조 {AC_POLL_S:.0f}초 확인 (Klima_11)", flush=True)
  deadline = time.monotonic() + AC_POLL_S
  seen_klima = False
  ac_on = False
  while time.monotonic() < deadline:
    for m in messaging.drain_sock(sm):
      for f in m.can:
        if int(f.address) != KLIMA_11:
          continue
        seen_klima = True
        data = bytes(f.dat)
        if len(data) > AC_BYTE and (data[AC_BYTE] >> AC_BIT) & 1:
          ac_on = True
    if ac_on:
      break
    time.sleep(0.05)

  if ac_on:
    print("Wayon preheat: 공조 ON 확인 — 예열 성공", flush=True)
    return "ac_on"
  if not seen_klima:
    # Klima 프레임 자체를 못 받았다 = 차가 잠들었거나 판다가 프레임을 차단한 정황
    print("Wayon preheat: Klima_11 미수신 (차단/슬립 정황)", flush=True)
    return "blocked"
  print("Wayon preheat: 공조 ON 신호 못 봄 (인증거부/상태신호/충돌 중 하나)", flush=True)
  return "no_ac"


def main() -> None:
  params = Params()
  print("Wayon preheat: 시작", flush=True)

  sm = messaging.sub_sock("can", timeout=200)
  sendcan = messaging.pub_sock("sendcan")
  time.sleep(0.5)
  messaging.drain_sock(sm)

  while True:
    config = read_config()
    if not config.get("endpoint") or not config.get("token"):
      time.sleep(POLL_OFFROAD_S)
      continue

    dongle = params.get("DongleId")
    if isinstance(dongle, bytes):
      dongle = dongle.decode("utf-8", "replace")
    device_id = str(config.get("device_id") or dongle or "unknown")
    onroad = params.get_bool("IsOnroad")

    # 주행 중엔 명령을 실행하지 않는다(폴링만 느리게 유지).
    if onroad:
      time.sleep(POLL_ONROAD_S)
      continue

    cmd = poll_command(config, device_id)
    if not cmd:
      time.sleep(POLL_OFFROAD_S)
      continue

    cmd_id = str(cmd.get("id") or "")
    cmd_type = str(cmd.get("type") or "")
    print(f"Wayon preheat: 명령 수신 type={cmd_type} id={cmd_id}", flush=True)

    if cmd_type != "preheat":
      ack_command(config, device_id, cmd_id, "done", "ignored")
      continue

    # safety 패치 검증 전에는 실제 송신을 막는 안전 플래그.
    if not config.get("preheat_enabled"):
      print("Wayon preheat: preheat_enabled 아님 → dry-run(송신 안 함)", flush=True)
      ack_command(config, device_id, cmd_id, "done", "dry_run")
      time.sleep(POLL_OFFROAD_S)
      continue

    # 재생 직전 다시 오프로드 확인(주행 시작과의 경합 방지)
    if params.get_bool("IsOnroad"):
      ack_command(config, device_id, cmd_id, "done", "onroad_abort")
      continue

    result = replay_unlock(sendcan, sm)
    state = "blocked" if result == "blocked" else "done"
    ack_command(config, device_id, cmd_id, state, result)
    time.sleep(POLL_OFFROAD_S)


if __name__ == "__main__":
  main()
