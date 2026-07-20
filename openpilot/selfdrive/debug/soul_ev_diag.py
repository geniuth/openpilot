#!/usr/bin/env python3
"""
Kia Soul EV (KIA_EV_SK3) 조향/크루즈 풀림 진단 — 무개입(읽기 전용) 라이브 모니터.

목적: 코드/거동을 전혀 바꾸지 않고, 풀림이 발생하는 순간의 신호를 시간축으로 박제해
      (A) 토크 과다로 MDPS가 토크요청을 거부(ToiFlt) 하는지, (B) 토크 부족으로 차선을
      못 잡는지, 그리고 크루즈가 ACCEnable 1~2프레임 블립으로 풀리는지를 "추론 없이" 확정.

증상이 "인게이지 유지(시스템 정상)인데 조향 액추에이션만 안 됨"인 경우를 위해,
ToiFlt 플래그가 안 서도 'latActive+토크명령 중인데 MDPS가 실제로 안 도는(ToiActive=0)'
상태가 지속되면 [STEER-NOACT] 로 잡아서 저장한다.

SSH 실행:
    cd /data/openpilot
    python selfdrive/debug/soul_ev_diag.py
    # 풀림이 발생하면 콘솔에 한 줄 요약이 뜨고, 전후 윈도우가 파일로 저장됩니다.
    # 평소처럼 운전하다가 좌/우회전 후 조향이 풀리거나 크루즈가 풀릴 때까지 켜두세요.

저장 위치: /data/media/0/soul_ev_diag/diag_*.txt  (없으면 ./soul_ev_diag/ 로 폴백)

읽는 신호 (모두 원시 또는 발행된 메시지 그대로, 가공/추정 없음):
  carControl : latActive, actuators.torque(정규화), actuators.torqueOutputCan(실제 송신값)
  carState   : steeringAngleDeg, steeringTorque(운전자), steeringTorqueEps,
               steeringPressed, steerFaultTemporary, accFaulted, cruiseState.{enabled,available}, vEgo
  원시 CAN(bus0, hyundai_kia_generic):
    MDPS12 : CF_Mdps_ToiActive / CF_Mdps_ToiFlt / CF_Mdps_ToiUnavail / CR_Mdps_StrColTq / CR_Mdps_OutTq
    TCS13  : ACCEnable(0=SCC ready,1=temp,2/3=perm), ACC_REQ
"""
import os
import time
from collections import deque

import cereal.messaging as messaging
from opendbc.can import CANParser

DBC = "hyundai_kia_generic"
HZ = 100
WINDOW_SEC = 3.0                       # 이벤트 전후로 보관/덤프할 시간
BUF = int(WINDOW_SEC * HZ)
POST_FRAMES = int(1.0 * HZ)            # 이벤트 후 추가로 더 모을 프레임
NOACT_TQ_MIN = 30                      # 이 이상 토크를 보내고 있으면 '실제 조향 시도 중'으로 간주
NOACT_THRESH_FRAMES = int(0.5 * HZ)    # latActive+토크명령인데 ToiActive=0가 이만큼 지속되면 '미작동'
HB_SEC = 1.0                           # 하트비트(살아있는지+실시간 값) 출력 주기

ACCENABLE_VAL = {0: "SCC ready", 1: "SCC temp fault", 2: "SCC perm fault", 3: "SCC perm fault(comm)"}


def out_dir():
  for d in ("/data/media/0/soul_ev_diag", os.path.join(os.getcwd(), "soul_ev_diag")):
    try:
      os.makedirs(d, exist_ok=True)
      return d
    except OSError:
      continue
  return "."


def fmt(rec):
  return (f"t={rec['t']:7.2f} v={rec['vEgo']:5.1f} lat={int(rec['latActive'])} "
          f"ang={rec['ang']:7.1f} | tq_cmd={rec['tq']:+.2f} outTq={rec['outTq']:+6.0f} "
          f"drvTq={rec['drvTq']:+6.0f} epsTq={rec['epsTq']:+6.1f} prs={int(rec['prs'])} "
          f"| Toi[act={rec['toiAct']} flt={rec['toiFlt']} unav={rec['toiUnav']}] "
          f"stFltTmp={int(rec['stFlt'])} | ACCEn={rec['accEn']}({ACCENABLE_VAL.get(rec['accEn'],'?')}) "
          f"ACC_REQ={rec['accReq']} accFaulted={int(rec['accFaulted'])} "
          f"cruise[en={int(rec['cEn'])} av={int(rec['cAv'])}]")


def dump(odir, tag, ring, note):
  ts = time.strftime("%Y%m%d_%H%M%S")
  path = os.path.join(odir, f"diag_{tag}_{ts}.txt")
  with open(path, "w") as f:
    f.write(f"# Soul EV 진단 이벤트: {tag}\n# {note}\n")
    f.write(f"# window={len(ring)} frames (~{len(ring)/HZ:.1f}s)\n\n")
    for rec in ring:
      f.write(fmt(rec) + "\n")
  print(f"  -> saved: {path}")
  return path


def main():
  odir = out_dir()
  print(f"[soul_ev_diag] 읽기 전용 모니터 시작. 저장 위치: {odir}")
  print("[soul_ev_diag] 풀림이 발생할 때까지 평소처럼 주행하세요. 종료: Ctrl-C\n")

  cp = CANParser(DBC, [("MDPS12", 0), ("TCS13", 0), ("SAS11", 0)], 0)
  logcan = messaging.sub_sock("can", timeout=100)
  sm = messaging.SubMaster(["carState", "carControl", "carOutput"])

  ring = deque(maxlen=BUF)
  t0 = time.monotonic()

  prev_toi_fault = False     # ToiFlt or ToiUnavail (원시)
  prev_acc_nonzero = False   # TCS13.ACCEnable != 0 (원시)
  acc_blip_frames = 0        # ACCEnable이 연속 비0으로 유지된 프레임 수
  noact_frames = 0           # latActive+토크명령인데 ToiActive=0가 연속 유지된 프레임 수
  prev_lat = False           # 직전 latActive (True->False 엣지 감지)
  prev_cEn = False           # 직전 cruiseState.enabled (디스인게이지 엣지)
  prev_pressed = False       # 직전 steeringPressed (핸들 흔듦 상승엣지)
  prev_toi_act = 1           # 직전 ToiActive (0->1 복구 엣지 감지)
  last_hb = 0.0              # 마지막 하트비트 시각
  frame = 0
  pending = None             # (tag, note, frames_left) — 이벤트 후 추가 수집

  while True:
    frame += 1
    cans = messaging.drain_sock(logcan)
    can_packets = [(m.logMonoTime, [(c.address, c.dat, c.src) for c in m.can]) for m in cans]
    if can_packets:
      cp.update(can_packets)
    sm.update(0)

    cs, cc, co = sm["carState"], sm["carControl"], sm["carOutput"]

    # 원시 CAN: 이번 drain 안의 '모든' 샘플을 봐서 1프레임 블립도 놓치지 않음
    acc_samples = [int(round(x)) for x in cp.vl_all["TCS13"]["ACCEnable"]] or [int(round(cp.vl["TCS13"]["ACCEnable"]))]
    acc_now = int(round(cp.vl["TCS13"]["ACCEnable"]))
    toi_flt = int(round(cp.vl["MDPS12"]["CF_Mdps_ToiFlt"]))
    toi_unav = int(round(cp.vl["MDPS12"]["CF_Mdps_ToiUnavail"]))
    toi_act = int(round(cp.vl["MDPS12"]["CF_Mdps_ToiActive"]))
    # openpilot이 실제 적용한 토크 (carOutput.actuatorsOutput = card.py가 발행하는 진짜 출력)
    out_tq = co.actuatorsOutput.torqueOutputCan

    rec = {
      "t": time.monotonic() - t0,
      "vEgo": cs.vEgo,
      "latActive": cc.latActive,
      "ang": cs.steeringAngleDeg,
      "tq": cc.actuators.torque,
      "outTq": out_tq,
      "drvTq": cp.vl["MDPS12"]["CR_Mdps_StrColTq"],
      "epsTq": cp.vl["MDPS12"]["CR_Mdps_OutTq"],
      "prs": cs.steeringPressed,
      "toiAct": toi_act, "toiFlt": toi_flt, "toiUnav": toi_unav,
      "stFlt": cs.steerFaultTemporary,
      "accEn": acc_now, "accReq": int(round(cp.vl["TCS13"]["ACC_REQ"])),
      "accFaulted": cs.accFaulted,
      "cEn": cs.cruiseState.enabled, "cAv": cs.cruiseState.available,
    }
    ring.append(rec)

    # ---- 이벤트 후 추가 수집 마무리 ----
    if pending is not None:
      tag, note, left = pending
      left -= 1
      if left <= 0:
        dump(odir, tag, list(ring), note)
        pending = None
      else:
        pending = (tag, note, left)

    # ---- 조향 풀림 후보: ToiFlt/ToiUnavail 상승엣지가 '인게이지(latActive) 중'에 발생 ----
    toi_fault = bool(toi_flt or toi_unav)
    if toi_fault and not prev_toi_fault and cc.latActive:
      sat = abs(rec["outTq"]) >= 380   # STEER_MAX=409 기준 포화 근접
      hint = "A 의심(토크과다→ToiFlt): outTq 포화" if sat else "토크 비포화 상태에서 ToiFlt"
      note = (f"조향: MDPS Toi 폴트(flt={toi_flt},unav={toi_unav}) @latActive. {hint}. "
              f"ang={rec['ang']:.0f} outTq={rec['outTq']:.0f} drvTq={rec['drvTq']:.0f}")
      print(f"[STEER] {note}")
      print("        " + fmt(rec))
      if pending is None:
        pending = ("STEER", note, POST_FRAMES)
    prev_toi_fault = toi_fault

    # ---- 조향 미작동: 인게이지+토크명령 중인데 MDPS가 실제로 안 도는(ToiActive=0) 지속 ----
    # ToiFlt 플래그가 안 서도 '조용한 거부'를 잡는다 (인게이지 유지·시스템 정상으로 보이는 증상).
    commanding = cc.latActive and abs(rec["tq"]) >= 0.05
    if commanding and toi_act == 0:
      noact_frames += 1
    else:
      noact_frames = 0
    if noact_frames == NOACT_THRESH_FRAMES:   # 임계 처음 도달한 순간 1회만 덤프
      # 실제 출력토크(carOutput)로 판별: OP가 토크를 내는데 MDPS가 무시 vs OP 출력이 0
      if abs(out_tq) >= 30:
        side = f"MDPS측: OP 출력 torqueOutputCan={out_tq:.0f}인데 ToiActive=0 → MDPS가 무시(하드웨어)"
      else:
        side = f"OP측: OP 출력 torqueOutputCan={out_tq:.0f}(거의0) → tq_cmd는 큰데 OP가 토크를 안 냄"
      note = (f"조향 미작동: latActive+명령(tq_cmd={rec['tq']:+.2f}) 중 MDPS ToiActive=0가 "
              f"~{NOACT_THRESH_FRAMES/HZ:.1f}s 지속. engaged(cEn)={int(rec['cEn'])} toiFlt={toi_flt} "
              f"stFltTmp={int(rec['stFlt'])} ang={rec['ang']:.0f} drvTq={rec['drvTq']:.0f}. {side}")
      print(f"[STEER-NOACT] {note}")
      print("        " + fmt(rec))
      if pending is None:
        pending = ("STEER_NOACT", note, POST_FRAMES)

    # ---- 크루즈 디스인게이지 순간: cruiseState.enabled True->False (조향까지 끊기는지 같이 본다) ----
    if prev_cEn and not cs.cruiseState.enabled:
      note = (f"크루즈 en True->False (디스인게이지 순간). latActive={int(cc.latActive)} outTq={out_tq:.0f} "
              f"ToiAct={toi_act} ang={rec['ang']:.0f} v={rec['vEgo']:.1f} ACC_REQ={rec['accReq']} accFaulted={int(rec['accFaulted'])}")
      print(f"[CRUISE-OFF] {note}")
      if pending is None:
        pending = ("CRUISE_OFF", note, POST_FRAMES)
    prev_cEn = cs.cruiseState.enabled

    # ---- 횡제어 꺼짐: latActive True->False (인게이지(크루즈)는 유지인데 조향만 빠지는 경우) ----
    if prev_lat and not cc.latActive:
      note = (f"횡제어 OFF: latActive True->False. cruise.enabled={int(rec['cEn'])} "
              f"(종방향 인게이지 유지 여부). ToiAct={toi_act} toiFlt={toi_flt} stFltTmp={int(rec['stFlt'])} "
              f"ang={rec['ang']:.0f} outTq_직전={rec['outTq']:.0f} v={rec['vEgo']:.1f}")
      print(f"[LAT-OFF] {note}")
      print("        " + fmt(rec))
      if pending is None:
        pending = ("LAT_OFF", note, POST_FRAMES)
    prev_lat = cc.latActive

    # ---- 복구 동작: 운전자가 핸들 흔든 순간(steeringPressed 상승엣지). 직전 3초가 곧 증상 구간 ----
    if cs.steeringPressed and not prev_pressed and cs.cruiseState.enabled:
      note = (f"핸들 흔듦(복구동작) 감지 @cruise.enabled. 직전 윈도우가 증상 구간. "
              f"latActive={int(cc.latActive)} ToiAct={toi_act} toiFlt={toi_flt} stFltTmp={int(rec['stFlt'])} "
              f"ang={rec['ang']:.0f} drvTq={rec['drvTq']:.0f}")
      print(f"[NUDGE] {note}")
      if pending is None:
        pending = ("NUDGE", note, POST_FRAMES)
    prev_pressed = cs.steeringPressed

    # ---- engage 감지: ToiActive 0->1 (MDPS가 LKAS를 무는 순간). 모든 케이스 + 정확한 순간값 기록 ----
    if toi_act == 1 and prev_toi_act == 0 and cc.latActive:
      hands_off = (not cs.steeringPressed) and abs(rec["drvTq"]) < 60
      mode = "손뗌(hands-off)" if hands_off else "운전자개입(driver-assist)"
      print(f"[ENGAGE 0->1] @t={rec['t']:.1f} v={rec['vEgo']:.1f} outTq={out_tq:.0f} "
            f"drvTq={rec['drvTq']:.0f} prs={int(rec['prs'])} ang={rec['ang']:.0f}  <{mode}>")
    prev_toi_act = toi_act

    # ---- 하트비트: 살아있는지 + 데이터 흐르는지 + 실시간 값 (1초마다) ----
    now = rec["t"]
    if now - last_hb >= HB_SEC:
      last_hb = now
      alive = "OK" if (sm.alive["carState"] and sm.alive["carControl"] and cp.can_valid) else \
              f"DATA?(cs={int(sm.alive['carState'])} cc={int(sm.alive['carControl'])} can={int(cp.can_valid)})"
      print(f"[hb {alive}] t={now:6.1f} v={rec['vEgo']:4.1f} en={int(rec['cEn'])} lat={int(rec['latActive'])} "
            f"ToiAct={toi_act} toiFlt={toi_flt} tq_cmd={rec['tq']:+.2f} outTq={out_tq:+5.0f} "
            f"ang={rec['ang']:+5.0f} drvTq={rec['drvTq']:+5.0f} accEn={acc_now}")

    # ---- 크루즈 풀림 후보: ACCEnable 0 -> 비0 (이번 drain의 모든 샘플 기준) ----
    nonzero_in_drain = any(s != 0 for s in acc_samples)
    if nonzero_in_drain:
      acc_blip_frames += 1
    else:
      if prev_acc_nonzero and acc_blip_frames > 0:
        # 비0이 끝남 -> 블립 길이 확정
        print(f"[CRUISE] ACCEnable 비0 구간 종료: 지속 {acc_blip_frames} drain, "
              f"마지막값들={acc_samples}, accFaulted(디바운스후)={int(rec['accFaulted'])}")
      acc_blip_frames = 0

    if nonzero_in_drain and not prev_acc_nonzero:
      vals = sorted(set(s for s in acc_samples if s != 0))
      note = (f"크루즈: TCS13.ACCEnable 0->비0 (값={vals} "
              f"= {[ACCENABLE_VAL.get(v,'?') for v in vals]}). "
              f"cruise.enabled={int(rec['cEn'])} ACC_REQ={rec['accReq']} vEgo={rec['vEgo']:.1f}")
      print(f"[CRUISE] {note}")
      if pending is None:
        pending = ("CRUISE", note, POST_FRAMES)
    prev_acc_nonzero = nonzero_in_drain

    time.sleep(1.0 / HZ)


if __name__ == "__main__":
  try:
    main()
  except KeyboardInterrupt:
    print("\n[soul_ev_diag] 종료.")
