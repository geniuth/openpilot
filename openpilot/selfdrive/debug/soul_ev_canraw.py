#!/usr/bin/env python3
"""
Kia Soul EV (KIA_EV_SK3) — 조향 주체/거부 판별용 원본 CAN·송신 덤프.

확정된 사실(이전 로그):
  - openpilot은 LKAS11(0x340)을 bus0으로 100Hz '정상 송신' 중 (sendcan bus=0). (수신 스트림엔
    자기 송신이 안 비쳐서 b0=0으로 보였을 뿐.)
  - 순정 카메라는 bus2에서 LKAS11을 쏘지만 ActToi=0/토크=0 → 조향지시 없음(충돌 아님).
  - 그런데도 MDPS ToiActive가 안 붙는 주행이 있었음.
  → 남은 질문: openpilot이 '쏘는 그 LKAS11 안에' 실제로 ActToi=1과 토크가 실려 있는가?
    실려 있는데도 MDPS가 거부하면 = MDPS 수용 문제. 안 실려 있으면 = openpilot측 요청 안 함.

이 스크립트: sendcan의 LKAS11을 '디코드'해서 openpilot이 실제로 요청하는 ActToi/토크/폴트를
  MDPS 응답(ToiActive)과 나란히 본다. 읽기 전용(무개입).

SSH:
    cd /data/openpilot
    python selfdrive/debug/soul_ev_canraw.py
    # 조향 인게이지하고 정차→출발/저속주행. 구간 자동 저장. 종료: Ctrl-C

저장: /data/media/0/soul_ev_diag/canraw_*.txt
"""
import os
import time
from collections import deque, Counter

import cereal.messaging as messaging
from opendbc.can import CANParser

DBC = "hyundai_kia_generic"
HZ = 100
LKAS11 = 832    # 0x340
MDPS12 = 593    # 0x251

PRE_SEC = 2.0
POST_SEC = 5.0
STOP_V = 0.3
GO_V = 1.5


def out_dir():
  for d in ("/data/media/0/soul_ev_diag", os.path.join(os.getcwd(), "soul_ev_diag")):
    try:
      os.makedirs(d, exist_ok=True)
      return d
    except OSError:
      continue
  return "."


def line(r):
  return (f"t={r['t']:8.2f} v={r['v']:5.1f} lat={r['lat']} prs={r['prs']} ang={r['ang']:+6.1f} || "
          f"opTX-LKAS11[ActToi={r['op_act']} Toq={r['op_toq']:+5.0f} Flt={r['op_flt']} Sys={r['op_sys']} Msg={r['op_msg']:2d}] || "
          f"cam-b2[ActToi={r['b2_act']} Toq={r['b2_toq']:+5.0f}] || "
          f"MDPS[ToiAct={r['m_act']} Flt={r['m_flt']} Unav={r['m_unav']} StrColTq={r['m_col']:+5.0f}] || "
          f"0x340 rx_b0={r['n0']} rx_b2={r['n2']} opTX={r['ntx']}")


def main():
  cp0 = CANParser(DBC, [("MDPS12", 0)], 0)     # bus0 수신: MDPS12
  cp2 = CANParser(DBC, [("LKAS11", 0)], 2)     # bus2 수신: 순정 카메라 LKAS11
  cptx = CANParser(DBC, [("LKAS11", 0)], 0)    # openpilot 송신 LKAS11 (sendcan을 src0으로 주입)

  logcan = messaging.sub_sock("can", timeout=100)
  sendcan = messaging.sub_sock("sendcan", timeout=100)
  sm = messaging.SubMaster(["carState", "carControl"])

  outd = out_dir()
  print(f"[canraw] 시작. 저장: {outd}")
  print("[canraw] 조향 인게이지 후 정차→출발/저속 주행. 구간 자동 저장. 종료: Ctrl-C")
  print(f"DBC: {DBC}")

  buf = deque(maxlen=int(6.0 * HZ))
  t0 = time.monotonic()
  was_stopped = False
  capturing = False
  cap_lines = []
  cap_end = 0.0
  last_hb = 0.0
  hb = Counter()
  prev_m_act = None   # None = 아직 첫 샘플 전 (시작 시점 착시 WAKE 방지)
  wake_tag = False

  while True:
    cans = messaging.drain_sock(logcan)
    rx_packets = [(m.logMonoTime, [(c.address, c.dat, c.src) for c in m.can]) for m in cans]
    n0 = n2 = 0
    for m in cans:
      for c in m.can:
        if c.address == LKAS11:
          if c.src == 0:
            n0 += 1; hb["b0"] += 1
          elif c.src == 2:
            n2 += 1; hb["b2"] += 1
          elif c.src == 1:
            hb["b1"] += 1

    # openpilot 실제 송신 LKAS11 → cptx에 src0으로 주입해서 '내용' 디코드
    ntx = 0
    tx_packets = []
    for m in messaging.drain_sock(sendcan):
      frames = [(c.address, c.dat, 0) for c in m.sendcan if c.address == LKAS11]
      if frames:
        tx_packets.append((m.logMonoTime, frames))
        ntx += len(frames); hb["tx"] += len(frames)

    if rx_packets:
      cp0.update(rx_packets)
      cp2.update(rx_packets)
    if tx_packets:
      cptx.update(tx_packets)
    sm.update(0)

    t = time.monotonic() - t0
    v = float(sm["carState"].vEgo)
    lat = int(sm["carControl"].latActive)
    prs = int(sm["carState"].steeringPressed)
    ang = float(sm["carState"].steeringAngleDeg)

    r = {
      "t": t, "v": v, "lat": lat, "n0": n0, "n2": n2, "ntx": ntx, "prs": prs, "ang": ang,
      "op_act": int(round(cptx.vl["LKAS11"]["CF_Lkas_ActToi"])),
      "op_toq": cptx.vl["LKAS11"]["CR_Lkas_StrToqReq"],
      "op_flt": int(round(cptx.vl["LKAS11"]["CF_Lkas_ToiFlt"])),
      "op_sys": int(round(cptx.vl["LKAS11"]["CF_Lkas_LdwsSysState"])),
      "op_msg": int(round(cptx.vl["LKAS11"]["CF_Lkas_MsgCount"])),
      # 모드 필드(자동 engage 허가와 관련 의심): openpilot이 쏘울EV엔 순정값 그대로 통과시킴
      "op_actm": int(round(cptx.vl["LKAS11"]["CF_Lkas_LdwsActivemode"])),
      "op_ldwo": int(round(cptx.vl["LKAS11"]["CF_Lkas_LdwsOpt_USM"])),
      "op_fcwo": int(round(cptx.vl["LKAS11"]["CF_Lkas_FcwOpt_USM"])),
      "b2_actm": int(round(cp2.vl["LKAS11"]["CF_Lkas_LdwsActivemode"])),
      "b2_ldwo": int(round(cp2.vl["LKAS11"]["CF_Lkas_LdwsOpt_USM"])),
      "b2_act": int(round(cp2.vl["LKAS11"]["CF_Lkas_ActToi"])),
      "b2_toq": cp2.vl["LKAS11"]["CR_Lkas_StrToqReq"],
      "m_act": int(round(cp0.vl["MDPS12"]["CF_Mdps_ToiActive"])),
      "m_flt": int(round(cp0.vl["MDPS12"]["CF_Mdps_ToiFlt"])),
      "m_unav": int(round(cp0.vl["MDPS12"]["CF_Mdps_ToiUnavail"])),
      "m_col": cp0.vl["MDPS12"]["CR_Mdps_StrColTq"],
    }
    buf.append(r)

    # === 핵심: MDPS가 '물리는' 바로 그 순간(ToiActive 0->1)을 잡아 직전 조건을 분석 ===
    if r["m_act"] == 1 and prev_m_act == 0:
      pre = list(buf)[-int(5.0 * HZ):]          # 직전 ~5초 (운전자토크 arm 윈도우 가설 검증용)
      if pre:
        vmax = max(x["v"] for x in pre); vmin = min(x["v"] for x in pre)
        col_absmax = max(abs(x["m_col"]) for x in pre)
        prs_any = any(x["prs"] for x in pre)
        ang_absmax = max(abs(x["ang"]) for x in pre)
        # openpilot ActToi 요청비트의 상승엣지가 직전 윈도우에 있었나 (H-edge 가설 검증)
        acts = [x["op_act"] for x in pre]
        act_edge = any(a == 1 and b == 0 for a, b in zip(acts[1:], acts[:-1]))
        secs = pre[-1]["t"] - pre[0]["t"]
      else:
        vmax = vmin = col_absmax = ang_absmax = secs = 0; prs_any = False; act_edge = False
      trig = []
      if prs_any or col_absmax > 40: trig.append(f"운전자토크(colmax={col_absmax:.0f},prs={int(prs_any)})")
      if act_edge: trig.append("ActToi상승엣지")
      if v > 2.0: trig.append(f"속도({v:.1f})")
      cause = " + ".join(trig) if trig else "??(손뗌·저토크·엣지없음인데 물림)"
      print(f"[WAKE 0->1] @t={t:.1f} v={v:.1f} ang={ang:+.0f} prs={prs} col={r['m_col']:+.0f} "
            f"opTQ={r['op_toq']:+.0f} | 직전{secs:.1f}s: v {vmin:.1f}~{vmax:.1f}, |col|max={col_absmax:.0f}, "
            f"손댐={int(prs_any)}, 엣지={int(act_edge)}, |ang|max={ang_absmax:.0f} | 추정트리거: {cause}")
      # 이 순간 전후를 파일로도 박제
      capturing = True
      cap_lines = [line(x) for x in buf]
      cap_end = t + 2.0
      wake_tag = True
    prev_m_act = r["m_act"]

    if v < STOP_V:
      was_stopped = True
    if was_stopped and (not capturing) and v > GO_V:
      capturing = True
      was_stopped = False
      cap_lines = [line(x) for x in buf]
      cap_end = t + POST_SEC
      wake_tag = False
      print(f"[TRIGGER] 정차→출발 감지 @t={t:.1f} v={v:.1f} — {PRE_SEC:.0f}s 전부터 {POST_SEC:.0f}s 녹화")

    if capturing:
      cap_lines.append(line(r))
      if t >= cap_end:
        capturing = False
        tag = "WAKE" if wake_tag else "startstop"
        fn = os.path.join(outd, f"canraw_{tag}_{time.strftime('%Y%m%d_%H%M%S')}.txt")
        with open(fn, "w") as fp:
          fp.write("\n".join(cap_lines) + "\n")
        print(f"  -> 저장: {fn}  ({len(cap_lines)} lines)")
        wake_tag = False

    if t - last_hb >= 1.0:
      last_hb = t
      diag = ""
      if lat and hb["tx"] > 0 and r["op_act"] == 0:
        diag = "  <<< lat인데 opTX ActToi=0 (openpilot이 조향요청 안 함!)"
      elif lat and r["op_act"] == 1 and r["m_act"] == 0:
        diag = "  <<< opTX ActToi=1인데 MDPS 거부 (MDPS 수용문제)"
      print(f"[hb] t={t:6.1f} v={v:5.1f} lat={lat} | "
            f"opTX[Act={r['op_act']} Toq={r['op_toq']:+5.0f} Sys={r['op_sys']} 모드:Actm={r['op_actm']} LdwO={r['op_ldwo']} FcwO={r['op_fcwo']}] "
            f"cam[Act={r['b2_act']} Actm={r['b2_actm']} LdwO={r['b2_ldwo']}] MDPS[Act={r['m_act']} col={r['m_col']:+5.0f}] "
            f"| opTX/s={hb['tx']}{diag}")
      hb.clear()

    time.sleep(0.003)


if __name__ == "__main__":
  try:
    main()
  except KeyboardInterrupt:
    print("\n[canraw] 종료.")
