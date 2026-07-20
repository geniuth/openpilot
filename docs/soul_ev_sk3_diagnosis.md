# Kia Soul EV (KIA_EV_SK3) 최적화 — 사실 기반 진단 + 1차 수정

> 대상: **Kia Soul EV 2019 = 코드상 `KIA_EV_SK3`** (별도 "soulbooster" 포트는 없음)
> 베이스: **`carrot-wip`** (id4/VW-MEB 작업이 전혀 섞이지 않은 깨끗한 계보. tip `9ba24bf`)
> 원칙: **할루시네이션 없이 코드로 검증된 사실만.** 모든 라인 인용은 carrot-wip 기준이며 재검증 가능.

플랫폼 정의 (`opendbc_repo/opendbc/car/hyundai/values.py:787-791`):

```python
KIA_EV_SK3 = HyundaiPlatformConfig(
  [HyundaiCarDocs("Kia Soul EV 2019", car_parts=CarParts.common([CarHarness.hyundai_c]))],
  CarSpecs(mass=1695, wheelbase=2.6, steerRatio=13.75),
  flags=HyundaiFlags.CHECKSUM_CRC8 | HyundaiFlags.EV,
)
```

→ **non-CAN-FD, 토크식 LKAS11/MDPS, `ANGLE_CONTROL` 없음, `CAMERA_SCC` 없음.** DBC = `hyundai_kia_generic` (bus 0).

---

## 문제 2 — 간헐적 크루즈(롱컨) 풀림 → **이번에 수정함**

### 코드로 확정된 디스인게이지 체인 (디바운스 전혀 없음)

1. `opendbc/car/hyundai/carstate.py:340` (non-CAN-FD 경로):
   ```python
   ret.accFaulted = cp.vl["TCS13"]["ACCEnable"] != 0  # 0 ACC ENABLED, 1-3 DISABLED
   ```
   **유예/디바운스/카운터 없는 raw 1프레임 신호.**
2. `selfdrive/car/car_specific.py:219-220` → `if CS.accFaulted: events.add(EventName.accFaulted)` (무조건)
3. `selfdrive/selfdrived/events.py:941` → `EventName.accFaulted: { ET.IMMEDIATE_DISABLE: ... }`

→ **stock `TCS13.ACCEnable`이 단 1프레임이라도 0이 아니면 즉시 크루즈 해제.**
일시적 `1`("SCC temp fault") 한 프레임 블립으로도 즉시 풀린다. 롱컨개조차는 stock ACC/SCC가
버스에 살아 있어 휠슬립·일시 상태변화로 `ACCEnable`이 순간 튀면 즉시 해제될 수 있음 =
"간헐적 자발 풀림"의 코드로 설명 가능한 유력 원인.

### 적용한 수정 (최소·안전)

`carstate.py`에 **N프레임 디바운스** 추가 (Soul EV가 타는 non-CAN-FD 경로만):

```python
ACC_FAULT_DEBOUNCE_FRAMES = 7   # carState 100Hz -> ~70ms

if cp.vl["TCS13"]["ACCEnable"] != 0:
    self.acc_fault_frames = min(self.acc_fault_frames + 1, ACC_FAULT_DEBOUNCE_FRAMES)
else:
    self.acc_fault_frames = 0
ret.accFaulted = self.acc_fault_frames >= ACC_FAULT_DEBOUNCE_FRAMES
```

- 1~6프레임짜리 순간 블립은 무시 → 자발 풀림 억제.
- **진짜 지속 폴트는 여전히 ~70ms 내 해제** (사람 반응시간 대비 무시 가능, 안전 트레이드오프 미미).
- 값 `7`은 보수적 기본값. 진단스크립트가 실제 블립 길이를 찍어주므로 그걸로 조정 가능.
- CAN-FD 경로(canfd 경로의 동일 라인)는 Soul EV 비해당이라 **의도적으로 미변경** (blast radius 최소화).

---

## 문제 1 — 조향 풀림: **실차 로그로 원인 확정 → 자동 재latch 수정 적용**

### 로그로 확정된 사실 (carOutput.outTq + MDPS12.ToiActive)
- 증상은 상시조향(always-on lateral) 사용 중 발생. `latActive=1` 유지, `cruiseState`와 무관.
- 풀리는 순간: **`carControl.actuatorsOutput.torqueOutputCan`(OP 실제 출력)은 크고(±100~300), 운전자 손 뗌(`steeringPressed=0`), 그런데 `CF_Mdps_ToiActive=0`(MDPS 미작동), `ToiFlt/Unavail=0`(폴트 없음).**
- 트리거: 직전에 **유턴/큰 회전에서 운전자가 핸들을 강하게 잡아(`CR_Mdps_StrColTq` −200~−300) MDPS가 양보(ToiActive=0)** → 회전 후 손 떼도 MDPS Toi가 **재engage 안 된 채 latch**. OP는 토크를 계속 쏘지만 MDPS가 무시 → 차선이탈, **핸들 흔들면(강제 1→0→1 엣지) 복귀**.
- → **openpilot 출력은 정상. MDPS 하드웨어가 override 후 재engage를 안 하는 것이 원인**(폴트가 아니라 단순 미재engage라 `steerFaultTemporary`도 안 뜸).

### 시도한 수정 — MDPS 자동 재latch → **역효과로 되돌림(revert)**
`apply_steer_req`를 3프레임 끊어 1→0→1 엣지로 MDPS를 강제 재engage("핸들 흔들기" 자동화)하려 했으나, **실차 로그에서 역효과 확인**:
- 펄스 시 `torque_fault=1`(=`CF_Lkas_ToiFlt=1`)이 나가는데, 이 Soul EV MDPS는 그걸 받고 **`CF_Mdps_ToiFlt=1`로 폴트** → `steerFaultTemporary=1` → openpilot이 **횡제어를 통째로 끔(`latActive→0`)**.
- 즉 작은 각도·손뗌에서도 `[STEER] ToiFlt=1` → `[LAT-OFF]`가 빈발. `[RECOVER]` 0회. **조향요청 토글이 이 MDPS에선 폴트를 유발해 더 나빠짐.**
- → carcontroller 변경 **전량 revert**(원본 동일). 같은 계열(요청비트/ToiFlt 토글) 접근은 이 차에선 부적합.

### 다음 단계 (재진단 필요)
- 테스트 로그가 전부 **저속(2~6 m/s) 좁은 구간 반복기동**이라, 운전자 강개입 시 MDPS가 양보(ToiActive=0)하는 정상거동과 실제 버그가 섞여 있을 수 있음.
- **정상 주행속도에서, 손 안 대고 직진/완만한 코너 중 조향이 빠지는 순간**(운전자 개입 없이 ToiActive=0)을 잡은 로그가 필요. 그래야 "MDPS 양보(정상)" vs "진짜 미재engage(버그)"를 분리 가능.
- 진단 스크립트(`soul_ev_diag.py`)는 그대로 유효(`[STEER-NOACT]`/`outTq`/`ToiAct`/`[RECOVER]`).

---

## (참고) 진단 스크립트 — `selfdrive/debug/soul_ev_diag.py`

### 코드로 확정된 사실

1. **carrot의 부드러운 토크복구(`recover_level`/`lkas_max_torque`)는 Soul EV 미적용.**
   `carcontroller.py`에서 계산은 되지만(라인 ~256-291) `lkas_max_torque`는 **CAN-FD 경로**
   (`create_steering_messages*`, 라인 ~363/365)에만 전달됨. Soul EV는 non-CAN-FD 분기
   `hyundaican.create_lkas11(..., apply_torque, apply_steer_req, torque_fault, ...)` (라인 ~414)
   → "회전 후 복구가 느려 풀린다"는 가설은 이 차종엔 **코드적으로 성립 안 함(배제).**
2. Soul EV 실제 조향 경로:
   - `new_torque = round(actuators.torque * STEER_MAX)`, `apply_torque = apply_driver_steer_torque_limits(...)` (라인 ~218)
   - 한계값: `STEER_MAX=409, STEER_DELTA_UP=3, STEER_DELTA_DOWN=7, STEER_DRIVER_ALLOWANCE=50` (특수 목록 미포함 = 기본 최대값).
   - `apply_steer_req`는 `|steeringAngleDeg| ≥ 85°` 지속 시 EPS 폴트 방지로 잠깐 끊김(`common_fault_avoidance`).
   - `torque_fault = CC.latActive and not apply_steer_req` (라인 ~302)
3. **MDPS 폴트 파싱(`carstate.py:314`):** `steerFaultTemporary = (CF_Mdps_ToiUnavail≠0) or (CF_Mdps_ToiFlt≠0)`.
   - **중요: `steerFaultTemporary`는 디스인게이지를 유발하지 않고 경고만 함** → "OP는 인게이지(조향중)로
     보이는데 실제 차는 MDPS가 토크를 거부해 안 도는" 증상과 정확히 일치. 핸들을 살짝 흔들면 MDPS Toi
     래치가 풀려 복귀. 좌/우회전 후 多 = 회전 중 고토크로 MDPS 폴트 누적.

### 코드만으로는 단정 불가 → 로그로 확정 (A/B 방향이 정반대)

- **(A) 토크 과다**: 곡선 지속 고토크(STEER_MAX 409 / rate 한계)가 구형 Soul EV MDPS 허용범위를 넘겨 `ToiFlt`. → 대응: `CustomSteerMax`를 409→384 등으로 ↓.
- **(B) 토크 부족**: rate(UP=3)가 느려 곡선을 못 잡아 차선 이탈, 운전자 개입으로 재동기. → 대응: `CustomSteerDeltaUp`를 3→4~5로 ↑.

두 방향이 상반되므로 **로그로 가설을 먼저 확정한 뒤 한 번에 한 값씩만** 변경. 그래서 이번 브랜치는 조향 거동을 바꾸지 않고 **진단 스크립트만 추가**한다.

### 진단 스크립트 (읽기 전용, SSH 실행) — `selfdrive/debug/soul_ev_diag.py`

코드/거동을 전혀 바꾸지 않고 풀림 순간의 신호를 박제한다.

```bash
# 디바이스에 SSH 접속 후
cd /data/openpilot
python selfdrive/debug/soul_ev_diag.py
# 평소처럼 주행. 좌/우회전 후 조향이 풀리거나 크루즈가 풀릴 때까지 켜둔다.
# 이벤트 발생 시 콘솔 한 줄 요약 + 전후 윈도우가 파일로 저장됨:
#   /data/media/0/soul_ev_diag/diag_*.txt   (없으면 ./soul_ev_diag/)
```

스크립트가 기록하는 것 (모두 발행 메시지/원시 CAN 그대로, 추정 없음):
- **조향 이벤트**: `MDPS12.CF_Mdps_ToiFlt/ToiUnavail` 상승엣지가 `carControl.latActive` 중 발생한 순간. 동시에 `actuators.torqueOutputCan`(실제 송신 토크), `steeringAngleDeg`, 운전자토크(`CR_Mdps_StrColTq`)/EPS토크(`CR_Mdps_OutTq`)를 같이 찍어 **A(토크포화→ToiFlt) vs B(비포화)** 를 즉시 판별.
- **크루즈 이벤트**: `TCS13.ACCEnable` 0→비0 전이와 **연속 비0 프레임 수(블립 길이)**, 그 순간 디바운스 후 `accFaulted` 값. → 디바운스 임계값 `7`이 적절한지 실측으로 검증.

### 로그 회수
`/data/media/0/soul_ev_diag/diag_*.txt`를 SSH `scp`로 받거나 파일 내용을 붙여주시면, A/B를 사실로 확정한 뒤 **한 방향만** 최소 튜닝(런타임 파라미터 `CustomSteerMax` 또는 `CustomSteerDeltaUp`, 코드변경 없이)으로 진행한다.

---

## 이번 브랜치에서 한 것 / 안 한 것

| 구분 | 내용 |
|---|---|
| 베이스 | **`carrot-wip`** (id4/VW-MEB 무관, 깨끗한 계보) |
| 수정(코드) | 크루즈: `carstate.py` accFaulted **N프레임 디바운스** (non-CAN-FD 경로) |
| 추가(무개입) | 조향/크루즈 진단 스크립트 `selfdrive/debug/soul_ev_diag.py` (읽기 전용) |
| **안 한 것** | 조향 토크/레이트/STEER_MAX 등 **거동 변경 없음** — 실차 로그로 A/B 확정 후 별도 진행 |

다음 단계: 진단 스크립트 로그 → A/B 확정 → 조향 1개 파라미터만 조정.
