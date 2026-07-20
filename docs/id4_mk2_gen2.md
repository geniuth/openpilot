# ID.4 MK2 (2024-25, E8 섀시) — MEB GEN2 규명 기록

MK1 포팅을 MK2(2024년식 이후, 페이스리프트) 차량에 설치하자 CAN 오류가 났다.
테스터 차량의 rlog를 리플레이해서 원인을 규명하고 수정했다. 결과는
`VolkswagenFlags.MEB_GEN2` 플래그와 `vw_meb_2024.dbc`로 분리되어 있다.
검증 도구: `openpilot/selfdrive/debug/vw_meb_mk2_replay.py` (rlog 입력).

## GEN2에서 달라진 것 (rlog 실측으로 확정)

| 항목 | MK1 (GEN1) | MK2 (GEN2) |
|---|---|---|
| CRC | 기존 VW MQB/MEB 방식 | **새 매직 상수 + 절단된 계산 길이** |
| ESC_51 | 48B | **64B** |
| Motor_51 | 32B | **48B** |
| 기어 | `Getriebe_11` @50Hz | **`Gateway_73` @10Hz** (`GE_Fahrstufe`) — Getriebe_11은 PT 버스에 없음 |
| TA_01 | 50Hz | 10Hz |
| EA_01/02 | 10Hz | 2Hz |
| KLR_01 | 50Hz | 10Hz |
| MEB_ACC_01 (ACC_19) | 16Hz | 10Hz |
| BSM | 확장 CAN | **PT 버스**, Driver/Passenger 신호명 |

- CRC 자체는 처음부터 100% 맞았다 (리플레이에서 미스매치 0건). 초기 "CAN 오류"의
  실제 원인은 **기어 소스(Getriebe_11 부재)와 메시지 주기 불일치**로 파서 타임아웃이
  난 것. 주기/소스 수정 후 리플레이 canValid 100% 확인.
- GEN2 CRC 구현은 `mqbcan.py`의 `volkswagen_mqb_meb_gen2_checksum()` — d[0] 자가검증
  후 불일치 시 구식 체크섬으로 폴백. 메시지별 상수는
  `VOLKSWAGEN_MQB_MEB_GEN2_CONSTANTS` (0x0DB/0xFC/0x102/0x10B/0x13D/0x139).
- 판다 안전 쪽은 `ALT_CRC_VARIANT_1` safety flag(=2)로 분기, GEN2 rx 체크는
  ESC_51 64B / Motor_51 48B 기준 (`safety_volkswagen_meb.h`).

## 핑거프린트/차종

- `VOLKSWAGEN_ID4_MK2` (mass 2224, wheelbase 2.77, 섀시코드 E8), 자동 감지.
- 토크 파라미터는 MK1과 동일값으로 시작 (`override.toml`: 2.0/2.0/0.1).

## 함께 잡은 버그: TA 해제 후 SET 재인게이지 시 조향 미복구

증상: 주행 중 트래블어시스트 버튼으로 인게이지 해제 → SET 버튼으로 재인게이지하면
속도제어만 붙고 조향이 안 붙음. 속도 +/− 스크롤을 건드리면 그제야 조향이 붙음.

원인: `openpilot/selfdrive/car/cruise.py`의 상시조향(`_lat_enabled`) 복구가
가감속 버튼 핸들러에만 있고 SET/RESUME 핸들러에 없었다.

수정: MEB에서 `setCruise`/`resumeCruise` 버튼 눌림 시에도 `_lat_enabled = True` 복구.
실차에서 SET 재인게이지·속도 상향 재인게이지 양쪽 확인 완료.

## 리플레이 검증 절차 (다음에 또 CAN 오류가 나면)

1. 테스터에게 rlog 요청 (오류가 난 주행 구간 포함)
2. `vw_meb_mk2_replay.py <rlog...>` — carstate 파서에 실제 CAN을 먹여
   canValid 분포/기어값/CRC 미스매치를 확인
3. 특정 메시지 타임아웃이면: 해당 메시지의 실제 수신 주기를 세서 파서 주기와 대조
