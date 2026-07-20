#!/usr/bin/env python3
"""MEB 순정 카메라 CAN 출력(MEB_Camera_01/02/03/10) 리버싱 하네스.

rlog 안의 openpilot 자체 비전(modelV2 차선/엣지)과 radarState 리드를 정답지로,
카메라 메시지의 모든 8/16비트 필드 후보와 상관을 전수 계산한다.
1차 결과(2026-07): 0x183 byte41/57 등에서 차선 확률 ~0.81, byte27/61 도로엣지 ~0.83,
0x234 byte36 리드거리 ~0.83. 슬롯->차선 고정 배정은 아님(동적 추적 객체로 추정).
진행 기록: docs/id4_camera_rev.md

사용법: python3 openpilot/selfdrive/debug/vw_meb_camera_rev.py <rlog.zst> [...]
"""
import sys, bisect
from collections import defaultdict
from pathlib import Path
import capnp, zstandard

REPO = Path("/home/user/openpilot_carrot")
capnp.remove_import_hook()
log_capnp = capnp.load(str(REPO/"openpilot/cereal/log.capnp"), imports=[str(REPO/"opendbc_repo/opendbc/car")])

def read_events(path):
    with open(path,"rb") as f:
        raw = zstandard.ZstdDecompressor().stream_reader(f).read()
    try:
        yield from log_capnp.Event.read_multiple_bytes(raw)
    except Exception:
        return

TARGETS = {0x183: "Camera_01", 0x234: "Camera_02", 0x318: "Camera_03", 0x17335B11: "Camera_10"}
msgs = {a: [] for a in TARGETS}
gt = defaultdict(lambda: ([], []))   # name -> (ts, vals)

def add(name, t, v):
    gt[name][0].append(t); gt[name][1].append(float(v))

for path in sys.argv[1:]:
    for ev in read_events(path):
        w = ev.which()
        t = ev.logMonoTime*1e-9
        if w == "can":
            for m in ev.can:
                if m.src in (0,2) and m.address in TARGETS:
                    msgs[m.address].append((t, bytes(m.dat)))
        elif w == "carState":
            add("vEgo", t, ev.carState.vEgo)
            add("steer", t, ev.carState.steeringAngleDeg)
        elif w == "modelV2":
            md = ev.modelV2
            if len(md.laneLines) >= 3 and len(md.laneLines[1].y) > 0:
                add("laneL_y", t, md.laneLines[1].y[0])
                add("laneR_y", t, md.laneLines[2].y[0])
                add("laneL_p", t, md.laneLineProbs[1])
                add("laneR_p", t, md.laneLineProbs[2])
            if len(md.roadEdges) >= 2 and len(md.roadEdges[0].y) > 0:
                add("edgeL_y", t, md.roadEdges[0].y[0])
                add("edgeR_y", t, md.roadEdges[1].y[0])
        elif w == "radarState":
            l = ev.radarState.leadOne
            if l.status:
                add("lead_d", t, l.dRel)
                add("lead_y", t, l.yRel)
                add("lead_v", t, l.vRel)

def gt_at(name, t):
    ts, vs = gt[name]
    if not ts: return None
    i = min(max(bisect.bisect_left(ts, t), 0), len(vs)-1)
    if abs(ts[i]-t) > 0.3: return None
    return vs[i]

def corr(a, b):
    n = len(a)
    if n < 30: return 0.0
    ma, mb = sum(a)/n, sum(b)/n
    ca=[x-ma for x in a]; cb=[x-mb for x in b]
    num=sum(x*y for x,y in zip(ca,cb)); da=sum(x*x for x in ca)**0.5; db=sum(y*y for y in cb)**0.5
    return num/(da*db) if da>0 and db>0 else 0.0

GT_NAMES = ["vEgo","steer","laneL_y","laneR_y","laneL_p","laneR_p","edgeL_y","edgeR_y","lead_d","lead_y","lead_v"]

for addr, mname in TARGETS.items():
    rows = msgs[addr]
    if len(rows) < 100: continue
    n = min(len(p) for _, p in rows)
    print(f"\n===== {mname} (0x{addr:X}) {len(rows)}프레임 {n}B")
    # GT 리샘플
    gts = {g: [gt_at(g, t) for t, _ in rows] for g in GT_NAMES}
    hits = []
    for lo in range(0, n-1):
        for w, off in ((8,0),(16,0),(16,0x8000)):
            nb = w//8
            if lo+nb > n: continue
            f = [int.from_bytes(p[lo:lo+nb],"little")-off for _, p in rows]
            if len(set(f)) < 8: continue
            for g in GT_NAMES:
                pair = [(x,y) for x,y in zip(f, gts[g]) if y is not None]
                if len(pair) < 100: continue
                c = corr([x for x,_ in pair], [y for _,y in pair])
                if abs(c) > 0.55:
                    hits.append((abs(c), c, lo, w, off, g))
    hits.sort(reverse=True)
    seen = set()
    for _, c, lo, w, off, g in hits:
        key = (lo//2*2, g)   # 비슷한 오프셋 중복 제거
        if key in seen: continue
        seen.add(key)
        print(f"  byte{lo:2d} u{w}{'-8000h' if off else '':7s} ~ {g:8s}: corr={c:+.3f}")
        if len(seen) > 24: break
