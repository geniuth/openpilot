# ID.4 전방 레이더 raw 트랙 조사

목표: 현대기아(Mando SCC)의 `EnableRadarTracks`처럼, 순정 전방 레이더가 가공 전 오브젝트
트랙을 그대로 뿌리도록 켤 수 있는지 확인한다.

## 1. 현재 우리가 받는 데이터 (실측)

`Strukturen_01` (0x24F, 64B, 25Hz, bus 2)에 오브젝트가 6슬롯으로 들어온다.
같은차선 2 + 좌측차선 2 + 우측차선 2. 슬롯마다 ObjectID / 종거리 / 횡거리 / 상대속도.
`opendbc/car/volkswagen/radar_interface.py`가 이미 파싱해서 radard로 넘긴다.

주행 로그 약 120초 실측 결과:

| 항목 | 값 |
|---|---|
| 프레임당 유효 객체 수 | 0개 17%, 1개 12%, 2개 20%, 3개 27%, 4개 21%, 5개 2% |
| 6슬롯 포화 | 0회 (최대 5개) |
| 최대 종거리 | 127.3 m |
| 횡거리 분포 | 대부분 ±8 m |
| 정지 물체 | 잡힘 (절대속도 1.5 m/s 미만이 로그에 따라 2~12%) |

두 가지 결론.

- 슬롯 부족으로 객체를 놓치는 상황은 실제로 발생하지 않고 있다.
- **정지 물체도 올라온다.** "순정은 ACC 관련만 필터링해서 정지차를 안 준다"는 통설은
  적어도 우리 차에는 그대로 적용되지 않는다. 정차 차량 늦은 감속 문제는 레이더 데이터
  부재보다 리드 선택/추종 튜닝 쪽을 먼저 봐야 한다.

Mando 트랙모드 대비 실제로 부족한 건 FOV다. Mando는 ±45도 이상이라 바로 옆 차선
근접 차량까지 보이는데, 우리는 횡 ±8 m 밴드에 갇혀 있다.

## 2. 숨은 raw 리스트는 없다 (확정)

bus 0/1/2 전체 주소를 전수조사했다. 32객체급 대역폭을 쓰는 미해독 메시지가 없다.
미해독으로 남아 있던 0x268 / 0x20B / 0x555는 전부 값이 변하지 않는 상수다.
0x14D는 32B 50Hz라 후보로 보였으나 DBC에 `ACC_18`로 이미 전부 디코딩돼 있다
(TSK가 레이더로 보내는 가감속 요청이지 객체 데이터가 아니다).

즉 언락 없이는 raw 트랙을 얻을 수 없다.

## 3. 레이더 정체

로그의 carParams에서:

```
fwdRadar  addr=0x757  fw=b'\xf1\x871EA907572H \xf1\x890234'
```

`1EA907572H`, SW `0234`. **Continental 계열이고 Mando가 아니다.**
따라서 `selfdrive/debug/car/hyundai_enable_radar_points.py`는 그대로 적용 불가.
그 스크립트는 Mando SCC 펌웨어 화이트리스트 기반이다.

참고로 위 FW 문자열은 `F187`/`F189` DID를 UDS로 **실제로 읽어온** 값이다.
즉 판다에서 0x757로 `0x22 ReadDataByIdentifier`가 이미 성공하고 있다.

## 4. 현대 방식의 메커니즘

1. 진단세션 `0x07` 진입 (표준 아닌 공급사 커스텀 세션)
2. DID `0x0142`를 `0x22`로 읽음 → 6바이트 config
3. 마지막 바이트를 `0x00` → `0x01`로 `0x2E` 쓰기
4. 재시동 후 트랙 출력 시작 (0x500~0x51F, 32객체)

스크립트 주석의 경고를 그대로 옮긴다. "새 레이더에 시도하려면 기본 config 값을 반드시
기록해둘 것. 다른 레이더와 다를 수 있고 되돌려야 할 수도 있다."

## 5. SFD 제약

ID.4는 2021년형부터 SFD(Schutz Fahrzeug Diagnose)가 걸려 있다. 두 군데서 막힌다.

**게이트웨이 진단 필터.** 19번 모듈에 진단 필터가 있어 OBD 포트로 13번 모듈
(Auto Dist. Reg)에 접근이 안 되는 사례가 보고돼 있다(ID.Buzz "Cannot be reached 0100").
해제하려면 19번의 `IDE16611-Diagnosis filter`를 temporary deactivation으로 바꿔야 하고,
그 자체가 19번 SFD 언락을 요구한다. 게다가 해제 효과가 20 km 주행 후 다시 잠긴다.

**13번 모듈 자체의 SFD.** VCID 끝에 "SFD"가 붙으면 VW 백엔드에서 시간제한 토큰을
받아야 쓰기가 허용된다.

우리에게 유리한 점: 판다는 OBD 포트 뒤가 아니라 버스에 직결이므로 게이트웨이 진단
필터의 영향을 받지 않는다(위 FW 읽기 성공이 증거). 반면 SFD는 타겟 ECU가 직접
강제하므로 우회 불가. 쓰기 시도에서 NRC `0x33 securityAccessDenied`가 오면 그게 SFD다.

## 6. 진행 계획

- **0단계 (읽기 전용, 위험 없음).** `selfdrive/debug/car/vw_meb_radar_did_scan.py`.
  확장세션(0x03) 열고 `0x22`로 DID 대역을 훑어 존재하는 DID를 전부 덤프한다.
  부정응답 코드로 존재 여부가 갈리는 게 핵심이다.
  `0x31` = 없음, `0x33` = 있는데 잠김(SFD 후보, 가장 중요), `0x22`/`0x7E`/`0x7F` = 있는데 조건 불일치.
- **1단계 (대조).** OBDeleven/VCDS로 13번 모듈 어댑션 채널 목록을 뽑아 이름과 대조.
- **2단계 (쓰기).** 후보가 특정된 경우에만. 원본 값 기록 → 1바이트 변경 → 재시동 →
  DTC 확인 → 버스 전수조사 재실행으로 새 주소 출현 확인.
- **3단계 (파싱).** 새 메시지가 뜨면 DBC 작성 + `radar_interface.py` 파서 추가.

## 7. 리스크

- 이 레이더가 AEB/FCW/Front Assist를 담당한다. 오코딩 시 안전기능 상실 또는
  ODIS 없이는 못 지우는 DTC 발생 가능.
- 쓰기는 재시동해도 유지된다. 원본 값을 먼저 읽어두지 않으면 되돌릴 방법이 없다.
- 게이트웨이 하네스에서는 순정 ACC가 이 레이더를 계속 소비한다. 출력 포맷 변경 시
  순정 측 반응은 미지수.
- 시동이 걸린 상태에서는 세션 요청이 거부된다(우리 DISABLE_RADAR 코드도 동일 전제).
  반드시 ACC ON / 시동 OFF / 정차 / openpilot 정지 상태에서만.

## 8. 어댑션 뽑을 때 캡처 목록

1. 13번 모듈 접근 여부 (안 되면 19번 진단필터부터)
2. 컨트롤러 정보 화면 전체. 부품번호, SW/HW 버전, **VCID에 SFD 표시 유무**
3. 어댑션 채널 전체 목록: 채널명 + `IDExxxxx` 번호 + 현재값 (번호까지 있어야 DID 역추적 가능)
4. 롱 코딩 문자열 원본
5. 측정값 블록(Messwerte) 목록
6. 현재 DTC (나중에 비교할 기준선)

주목할 독일어 키워드:
`Objektliste` `Objekt` `Rohdaten` `Messwertausgabe` `Debug` `Entwickler` `Applikation`
`Traceausgabe` `Sensordaten` `CAN-Botschaft` `Diagnoseausgabe`

## 9. 기대치

낮다. 현대 건은 Mando 레이더가 널리 쓰여 공급사 자료 접근이 있었기에 발견된 것이고,
Continental VW 레이더의 raw 출력은 보통 설정 비트 하나가 아니라 별도 계측 프로토콜로
열게 되어 있다. 다만 0단계는 위험이 없으므로 "없다"를 확정하는 것만으로도 가치가 있다.

## 참고

- https://wiki.ross-tech.com/wiki/index.php/SFD
- https://www.vag-coding.net/tutorials-information/vcds-what-is-sfd-protection/
- https://www.t6forum.com/threads/vcds-help-id-buzz-module-13-auto-dist-reg-status-cannot-be-reached-0100.65482/
- https://www.vwidtalk.com/threads/im-back-with-a-2024-id-4-pro-s-awd.16143/page-3
- https://www.vag-coding.net/vw/id-4-e2/
