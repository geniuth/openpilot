#!/usr/bin/env python3
"""MEB_Side_Assist_02 (0x24D) 후측방 코너레이더 리버싱 스캐너.

rlog에서 0x24D 프레임을 뽑아
  1) 메시지 생존 여부/바이트별 변동성 (구조 개관)
  2) 8바이트 x 6슬롯(바이트 16~) 구조로 잘라 슬롯 점유 타임라인
  3) 현재 가설 필드 디코드 (b4=슬롯ID, [b5,b6]=절대속도 km/h x0.01,
     b7=후방거리 후보, [b0,b1]=상대속도 후보 0x8000 오프셋)
  4) 슬롯별 연속 점유 구간(run)에서 "거리 미분 == 속도" 상관 후보 필드 탐색
를 출력한다. 진행 상황/확정 필드: docs/id4_side_assist_rev.md

사용법:
  python3 openpilot/selfdrive/debug/vw_meb_side_assist_scan.py <rlog.zst> [rlog2.zst ...]
"""
import itertools
import statistics
import sys
from collections import Counter
from pathlib import Path

import capnp
import zstandard

REPO_ROOT = Path(__file__).resolve().parents[3]
ADDR = 0x24D
SLOT_BASE = 16   # 슬롯 영역 시작 오프셋
SLOT_SIZE = 8
N_SLOTS = 6

capnp.remove_import_hook()
log_capnp = capnp.load(str(REPO_ROOT / "openpilot/cereal/log.capnp"),
                       imports=[str(REPO_ROOT / "opendbc_repo/opendbc/car")])


def read_events(path):
  with open(path, "rb") as f:
    raw = zstandard.ZstdDecompressor().stream_reader(f).read()
  try:
    yield from log_capnp.Event.read_multiple_bytes(raw)
  except Exception:
    return


def u(b, lo, nbits):
  return (int.from_bytes(b, "little") >> lo) & ((1 << nbits) - 1)


def corr(a, b):
  n = len(a)
  ma, mb = sum(a) / n, sum(b) / n
  ca = [x - ma for x in a]
  cb = [x - mb for x in b]
  num = sum(x * y for x, y in zip(ca, cb))
  da = sum(x * x for x in ca) ** 0.5
  db = sum(y * y for y in cb) ** 0.5
  return num / (da * db) if da > 0 and db > 0 else 0.0


def smooth_fields(run, topn=16):
  """run(시계열 슬롯 페이로드)에서 연속적으로 변하는 필드 후보를 찾는다."""
  N = len(run)
  out = []
  for lo in range(0, SLOT_SIZE * 8):
    for w in (8, 10, 12, 16):
      if lo + w > SLOT_SIZE * 8:
        continue
      series = [u(s, lo, w) for _, s in run]
      distinct = len(set(series))
      rng = max(series) - min(series)
      if rng == 0 or distinct < max(5, N // 25):
        continue
      diffs = [abs(series[j + 1] - series[j]) for j in range(N - 1)]
      big = sum(1 for x in diffs if x > rng * 0.5)
      med = statistics.median(diffs)
      out.append(((med / rng) + big / N, lo, w))
  out.sort()
  return [(lo, w) for _, lo, w in out[:topn]]


def find_dist_vel_pairs(run, name):
  """거리 필드의 시간 미분이 속도 필드와 비례하는 쌍을 찾는다 (물리 정합 검증)."""
  N = len(run)
  if N < 40:
    return []
  dt = (run[-1][0] - run[0][0]) / max(1, N - 1)
  if dt <= 0:
    return []
  fields = smooth_fields(run)
  win = max(5, N // 12)
  found = []
  for (dlo, dw), (vlo, vw) in itertools.permutations(fields, 2):
    if not (dlo + dw <= vlo or vlo + vw <= dlo):
      continue
    D = [u(s, dlo, dw) for _, s in run]
    V = [u(s, vlo, vw) for _, s in run]
    slopes, vavg = [], []
    for j in range(0, N - win, max(1, win // 2)):
      slopes.append((D[j + win] - D[j]) / (win * dt))
      vavg.append(sum(V[j:j + win]) / win)
    if len(slopes) < 6:
      continue
    c = corr(slopes, vavg)
    if abs(c) > 0.7:
      vv = statistics.pvariance(vavg)
      k = (statistics.covariance(slopes, vavg) / vv) if len(vavg) > 1 and vv > 0 else 0
      found.append((abs(c), c, (dlo, dw), (vlo, vw), k))
  found.sort(reverse=True)
  if found:
    print(f"\n== {name}: N={N} dt={dt * 1000:.0f}ms")
    for f in found[:6]:
      print(f"  corr={f[1]:+.3f} dist@{f[2]} vel@{f[3]} scale_ratio={f[4]:.4f}")
  return found


def main():
  paths = sys.argv[1:]
  if not paths:
    print(__doc__)
    sys.exit(1)

  frames = []  # (t_sec, payload)
  addr_hits = Counter()
  for path in paths:
    for ev in read_events(path):
      if ev.which() != "can":
        continue
      for m in ev.can:
        if m.address == ADDR:
          addr_hits[m.src] += 1
          frames.append((ev.logMonoTime * 1e-9, bytes(m.dat)))

  print(f"0x{ADDR:03X} frames: {len(frames)} (src별: {dict(addr_hits)})")
  if not frames:
    return
  print(f"길이: {set(len(p) for _, p in frames)} | 유니크 페이로드: {len(set(p for _, p in frames))}")

  n = len(frames[0][1])
  var_bytes = [len({p[i] for _, p in frames}) for i in range(n)]
  print("바이트별 유니크 값 수:")
  for i in range(0, n, 16):
    print(f"  [{i:2d}]", var_bytes[i:i + 16])

  # 슬롯 분해: 연속 점유 구간(run) 수집
  runs = {i: [] for i in range(N_SLOTS)}
  cur = {i: None for i in range(N_SLOTS)}
  occupancy = Counter()
  for t, p in frames:
    for i in range(N_SLOTS):
      s = p[SLOT_BASE + i * SLOT_SIZE: SLOT_BASE + (i + 1) * SLOT_SIZE]
      if len(s) < SLOT_SIZE:
        continue
      if s == b"\xff" * SLOT_SIZE:
        if cur[i]:
          runs[i].append(cur[i])
          cur[i] = None
      else:
        occupancy[i] += 1
        if cur[i] is None:
          cur[i] = []
        cur[i].append((t, s))
  for i in range(N_SLOTS):
    if cur[i]:
      runs[i].append(cur[i])

  print(f"\n슬롯 점유 프레임 수: {dict(occupancy)}")
  print(f"슬롯별 run 수: {{i: len(r) for...}} = { {i: len(runs[i]) for i in range(N_SLOTS)} }")

  # 가설 디코드 샘플 (각 슬롯 최장 run의 중간 프레임)
  print("\n가설 디코드 (b4=ID, [b5,b6]=절대속도 km/h, b7=거리후보, [b0,b1]-0x8000=상대속도후보):")
  for i in range(N_SLOTS):
    if not runs[i]:
      continue
    r = max(runs[i], key=len)
    t, s = r[len(r) // 2]
    spd = int.from_bytes(s[5:7], "little") * 0.01
    rel = int.from_bytes(s[0:2], "little") - 0x8000
    print(f"  slot{i} (run {len(r)}프레임): raw={s.hex()} id=0x{s[4]:02X} v={spd:.1f}km/h b7={s[7]} rel_raw={rel}")

  # 물리 정합 탐색: 거리 미분 vs 속도
  allfound = {}
  for i in range(N_SLOTS):
    for r in sorted(runs[i], key=len, reverse=True)[:2]:
      if len(r) >= 100:
        for f in find_dist_vel_pairs(r, f"slot{i}/len{len(r)}")[:3]:
          allfound.setdefault((f[2], f[3]), []).append((i, round(f[1], 3), round(f[4], 4)))

  print("\n== 여러 run에서 일관된 dist/vel 필드 쌍:")
  for key, hits in sorted(allfound.items(), key=lambda kv: -len(kv[1])):
    if len(hits) >= 2:
      print(f"  dist@{key[0]} vel@{key[1]}: {hits}")


if __name__ == "__main__":
  main()
