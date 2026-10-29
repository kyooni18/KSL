# KSP 비행 데이터 수집 가이드 (D1 후속)

`NEXT.md`의 D1("실제 KSP STS-N 아음속 aero를 비행 로그로 피팅")은 **대부분 이미 끝났다.**
main의 `b377e95`가 138개 실측 KSP 로그로 `stsn_aero_identified.csv`를 만들었고
(34개 비행을 통째로 hold-out, 착륙 영역 CL rms 0.063 / CD rms 0.012), 이것이 기본 프로파일
`identified`다. 그래서 새 비행은 **그 피팅이 아직 못 본 영역만** 채우면 된다.

## 1. 남은 공백 (근거: `ShuttleSim/reference-model/*.report.json`)

| # | 공백 | 근거 | 막고 있는 것 |
|---|---|---|---|
| G1 | **M 0.2–0.5에서 α 13° 이상 데이터가 없다.** Mach 매듭별 α 5–95 % 범위가 M0.2: 5.4–13.4°, M0.25: 2.8–12.2°, M0.3–0.5: ≈1–12.7°. 13–22°는 외삽이고, 실속각(`--alpha-stall 20`)도 가정이다. | aero report `knots[].alpha_p05/p95` | D2(15° 한계가 실제 한계인가), Final 플레어 튜닝(16) |
| G2 | **기어와 에어브레이크의 항력 증분이 측정 불가(≈0).** 피팅값 dCD_gear −0.003, dCD_airbrake −0.0002. | aero report `increments` | D4(스피드브레이크 허용 여부), 18 |
| G3 | **M 0.6–0.9 저α(2–10°) 데이터가 없다.** 이 구간은 샘플 550–840개, α 12–18°만 있다. | aero report `knots` | MM305 진입 에너지(우선순위 낮음) |
| G4 | **롤/요 직접제어 플랜트가 식별되지 않았다** (hold-out R² 0.24/0.37). 피치 감쇠도 10 Hz에서 식별 불가. | `stsn_attitude_identified.report.json` | NEXT 6, FCS(20) |

G1과 G2가 D1–D4에 직접 걸리므로 먼저 한다. G3, G4는 같은 세션에서 여유가 있으면 한다.

## 2. 새 파일 형식은 필요 없다

앱이 KSP에 **연결되는 순간** 기록이 시작된다. 자동조종을 engage하지 않고 손으로 날아도 기록된다.

- 위치: `<repo>/FlightLogs/<UTC시각>-STS-N-vehicle.jsonl` (`KSP_LANDER_ROOT`가 있으면 그 아래)
- 한 번 연결 = 한 파일 = 피팅에서 **한 비행**. hold-out은 파일 이름의 crc32로 비행 단위로 나뉜다.
  → 카드마다 **연결을 끊고 다시 연결**해서 파일을 분리한다 (퀵로드만 하면 한 파일에 섞인다).
- 기록 주기: 동압 > 25 Pa면 0.10 s, 각속도 > 4°/s 또는 스틱 > 60 %면 0.05 s, 5 s마다 keyframe.

피팅 도구가 읽는 필드 (전부 이미 기록됨):

| 용도 | 필드 |
|---|---|
| aero | `aero.mach`, `aero.dynamicPressure`, `aero.liftForce`, `aero.dragForce`, `attitude.angleOfAttack`, `attitude.sideslip`, `vehicle.currentThrust`, `state.gear`, `state.airbrakes`, `state.vesselSituation`, `position.radarAltitude` |
| 자세 | 위 항목 + `appliedControl.reportedPitch/Roll/Yaw`, `attitude.bodyPitchRate/bodyRollRate/bodyYawRate` |

## 3. 샘플이 버려지지 않게 하는 조건

도구가 아래를 벗어나는 샘플은 버린다.

- **엔진 추력 0** (`currentThrust` ≤ 1 N): 모든 엔진 끄기
- **옆미끄럼 |β| < 4°** (aero는 5°, 자세는 4°): 날개 수평, 러더 중립
- **동압 ≥ 500 Pa** (aero는 300 Pa; 해면 기준 약 30 m/s 이상)
- **전파고도 ≥ 15 m**, 상황 `flying`
- **physics warp와 time warp 끄기**: 자세 피팅은 연속 샘플의 각속도 차분을 쓴다
- **SAS 끄기**: 카드 F에서 기록되는 스틱이 실제 입력이어야 한다

## 4. 비행 카드

공통 시작점: KSC 근처 고도 8–10 km, 속도 200–250 m/s, 엔진 끔, 날개 수평에서 퀵세이브.
각 카드는 **서로 다른 날(또는 다른 속도)로 최소 2회**. 비행이 많을수록 hold-out 점수가 믿을 만해진다.

### A. 착륙 영역 고α 스윕 (G1, 최우선) — 3–4회

1. 강하하며 속도를 M 0.2–0.45(약 70–150 m/s)로 만든다.
2. α를 **2 → 6 → 10 → 13 → 16 → 18 → 20 → 22 → 25°** 로 올린다. 각 단계에서 **5 s 유지**, 단계 사이 피치 변화는 2°/s 이하로 천천히.
3. 20° 이후 양력이 줄거나 기수가 떨어지면 그 α를 메모한다 (실속각, D2).
4. 속도가 M 0.2 아래로 떨어지면 기수를 내려 속도를 회복한 뒤 남은 단계를 이어간다.
5. 회마다 다른 속도대(예: 80, 110, 140 m/s)에서 한다.

### B. 기어 증분 (G2) — 2회

M 0.25–0.4, α 8–12°를 유지하면서 **기어를 10 s 내림 / 10 s 올림, 4사이클.** α와 속도는 최대한 일정하게 유지한다.

### C. 에어브레이크 증분 (G2, D4) — 2회

B와 같은 조건에서 **에어브레이크(Brakes 액션 그룹)를 10 s 켬 / 10 s 끔, 4사이클.**
먼저 로그의 `state.airbrakesAvailable`이 `true`인지 확인한다. `false`면 기체에 에어브레이크 부품이 없거나 그룹에 묶여 있지 않다는 뜻이고, 그러면 G2 증분이 0으로 나온 것 자체가 답이다.

### D. 고아음속 저α (G3) — 1–2회

고도 10–15 km에서 M 0.6–0.9 동안 α를 **2 → 5 → 8 → 10°**, 단계마다 5 s 유지.

### E. 자세 더블릿 (G4) — 동압대별 1회씩

SAS 끔. 동압 **2–4 kPa, 4–7 kPa, 7–12 kPa** 각각에서:

- 피치 스틱 +50 % 0.5 s → −50 % 0.5 s → 중립 5 s, 3회
- 롤 스틱 같은 방식, 3회
- 요(러더) 같은 방식, 3회 (β가 4°를 넘지 않게 짧게)

스틱이 60 %를 넘으면 기록이 20 Hz로 올라가므로 피치 감쇠 식별에도 도움이 된다.

## 5. 수집 후 다시 피팅하기

로그를 `FlightLogs/`에 두고 (기존 설정 그대로):

```sh
cd ShuttleSim
python3 tools/identify_stsn_aero.py --constrained \
  --normalized data/fitted/ksp-databook-training.csv data/fitted/ksp-heldout-full.csv data/fitted/ksp-calibration.csv \
  --live-logs ../FlightLogs --heldout-log 2026-09-15T14-17-20Z-STS-N-vehicle.jsonl \
  --supersonic-table data/fitted/stsn_aero_ksp_robust.csv \
  --table reference-model/stsn_aero_identified.csv \
  --report reference-model/stsn_aero_identified.report.json \
  --prior-book reference-model/stsn_certified_prior_identified.csv
```

카드 A로 α 22–25° 데이터가 생기면 `--alpha-max`를 25로 올리고, 실측 실속각에 맞춰
`--alpha-stall`도 바꾼다. 자세는 `tools/identify_stsn_attitude.py`로 같은 로그 디렉터리를 다시 돌린다.

**완료 기준** (NEXT 1-1과 같음): hold-out 비행에서 L과 D 오차 ≤ 10 %. 추가로 report의
M 0.2–0.45 매듭 `alpha_p95`가 20° 이상이고, `increments.gear/airbrake`가 0이 아니거나
0이라는 것이 카드 B/C로 확인되어야 한다.
