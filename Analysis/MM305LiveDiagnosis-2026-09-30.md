# MM305 / HAC live KSP 로그 분석

분석일: 2026-09-30. 비행 코드는 변경하지 않았고 신규 live 비행도 수행하지 않았다.

## 결론

최근 실행의 직접적인 장애는 **HAC에서 활주로 정렬까지는 도달하지만 Final 인계의 착륙 에너지 조건을 통과하지 못하는 것**이다. 최신 실행에서는 정렬점의 위치·고도·속도·하강각·bank 조건이 충족되는 순간에도 예상 착륙속도가 목표보다 낮아 인계가 차단된다. 계속 TAEM 상태로 진행하다 정렬점 뒤 500 m의 miss 경계에서 Abort한다.

그러나 그 예상 착륙속도 자체에 이전 live 실행에서 심한 인계 불연속이 관측됐다. 따라서 에너지 게이트가 거부한다는 사실과 실제 기체가 착륙 에너지 부족이라는 주장은 구분해야 한다. 반경이나 인계 허용폭을 더 조정하기 전에 예측과 실제 Final 실행의 일관성을 검증해야 한다.

## 조사 범위와 실행 신원

- `FlightLogs/2026-09-28T03-50-35Z`, `08-30-23Z`, `08-44-32Z`, `08-49-40Z`, `09-24-05Z`, `09-44-46Z`의 vehicle/planner 쌍. vehicle delta는 `Tools/postflight_acceptance.py::load_vehicle`로 복원했다. motion 그룹은 변경 생략 때문에 틱 사이의 값이 유지될 수 있으므로 정확한 인계 시점 수치는 native stderr 진단으로 교차 확인했다.
- 최신 native 로그: `Runtime/Headless/mm305-alignment-capture-live.log`. 비교 로그: `mm305-native-acquisition-convergence-live.log`, `mm305-controlled-convergence-live.log`, `mm305-touchdown-predictor-diagnostic-live.log`.
- 위 최근 6개 비행은 같은 저장 상태 부근(UT 21899, runway along 약 -48.5 km, cross 약 +10.3 km, 고도 약 21.5 km ASL, 속도 약 763 m/s)의 HAC 단독 시험이다. 일반 MM304→MM305 전 구간 검증으로 간주할 수 없다. `09-23-24Z`는 시뮬레이션 시간이 흐르지 않은 idle 연결 로그라 비행 비교에서 제외했다.
- 최신 campaign identity: `089b1c2eb4080dc3-cacabe6f3a9c7d6f-23edce430a42f1af`; native build 문자열 `Sep 28 2026 13:55:56`.
- 해당 acceptance artifact manifest와 현재 파일 SHA256 비교에서 `taem_alignment.h`, `taem_candidate_search.c`, `guidance/final/planning.inc`, `controller/prediction_workers.inc`는 일치한다. `guidance_taem.c`는 다르므로 그 파일의 현재 구현 설명은 실행 당시와의 차이를 고려해야 한다. Git HEAD만으로 실행 코드를 특정할 수 없다.

## 반복되는 종료 패턴

아래는 **마지막 TAEM vehicle sample**이며 충돌이나 착륙 상태가 아니다. 시각은 파일명의 UTC, 고도는 활주로 기준이다.

| 실행 | 경과 UT (s) | along (m) | cross (m) | 고도 (m) | TAS (m/s) | FPA (°) |
|---|---:|---:|---:|---:|---:|---:|
| 03:50:35 | 116.1 | -8500.0 | +3.2 | 3788.2 | 157.5 | -21.70 |
| 08:30:23 | 121.2 | -8507.6 | +2.7 | 3792.8 | 157.1 | -21.81 |
| 08:44:32 | 122.2 | -8507.1 | -7.1 | 3791.5 | 156.2 | -21.79 |
| 08:49:40 | 121.4 | -8514.0 | -11.0 | 3796.0 | 157.3 | -21.87 |
| 09:24:05 | 122.0 | -8513.2 | +1.1 | 3793.3 | 156.2 | -21.82 |
| 09:44:46 | 118.6 | -8510.3 | +10.4 | 3793.9 | 156.4 | -21.79 |

모두 TAEM→Abort로 종료했다. native 로그의 종료 이유는 `MM305 missed the Final alignment point; late runway-line capture is not permitted.`이다. 목표 station은 -9000m, miss 경계는 -8500m다. 마지막 TAEM sample 다음 틱에서 이 경계를 넘는다. 최근 실행의 직접적인 종료 원인은 횡방향 오차 증가가 아니다.

## 1. 최신 실행은 기하 조건을 만족하지만 Final 에너지 게이트가 거부한다

`mm305-alignment-capture-live.log:19720` 및 직전의 `FINAL_DELIVERY_ENERGY`、UT 22014.31:

| 상태 | 실측 / 진단 | frozen target / ready 조건 |
|---|---:|---:|
| along | -9006 m (tracker frame) | -9000 ±250 m |
| cross | +6 m | 절댓값 <35 m |
| runway course | 약 90.8° | heading error <2° |
| bank | -2.7° | 절댓값 <5° |
| bank rate | 인접 vehicle sample에서 약 -0.09°/s | 절댓값 <3°/s |
| runway height | 4008 m | 4000 ±150 m |
| TAS | 161.50 m/s | 165 ±10 m/s |
| FPA | -23.94° | -23.962 ±3° |

같은 틱의 Final 진단은 `predictedTouchdownV=69.88`, `touchdownMargin=-370.9`, `lowMargin=0.0`, `highMargin=10000.0`이다. 설정의 `touchdownSpeed=75`에 대해 `0.5*(Vtd²-75²)`가 음수가 된다.

`guidance/final/planning.inc:532`는 이 margin을 `specific_energy_margin`의 최솟값에 포함하고, `guidance/taem_exec.c:103`이 음수를 거부한다. 따라서 다른 계약 조건과 무관하게 이 틱의 Final delivery는 허용되지 않는다. 이는 게이트의 동작을 확인한 것이며, 예측의 물리적 정확성을 확인한 것은 아니다.

이후에도 인계하지 못하고, station 통과 뒤 TAEM reference가 얕아지면서 속도와 고도를 잃고 miss 경계에 도달한다. 마지막 상태만 보면 최초 거부 시점에 이미 기하 조건이 충족됐다는 사실을 놓친다.

## 2. 착륙 예측이 MM305→Final에서 불연속이다

이전 `mm305-touchdown-predictor-diagnostic-live.log:5413` 이후:

- UT 22009.65, 인계 직전: V=168.86m/s, h=4118.6m, FPA=-24.24°, 예상 착륙속도 72.83 / 72.41m/s.
- UT 22009.75, 첫 Final 틱: V=168.75m/s, h=4111.7m, FPA=-24.22°, 예상 착륙속도 135.02m/s, `FINAL_TRACE vtd=134.7`.
- 다음 틱에서 `vtd=119.0`, 이어서 113.9, 109.1m/s.

0.1초의 작은 실측 상태 변화에 비해 착륙 예측이 약 62m/s 뛴다. 인계 전 예측과 Final 실행의 상태·모델·학습값 불일치를 의심할 강한 근거다. 구체적으로 어떤 변수가 원인인지는 이번 조사에서 확정하지 않았다. 이전 비행에서는 이번의 음수 착륙 에너지 게이트와 동일하게 처리되지 않았고, 인계가 실제로 허용됐다.

이 불연속이 남아 있는 동안 최신의 69.88m/s만으로 실제 착륙 에너지가 부족하다고 단정할 수 없다. 게이트를 완화하는 것만으로 안전한 착륙이 증명되지도 않는다.

## 3. planner 성공 판정이 live의 최종 요구까지 검증하지 않는다

`taem_candidate_search.c:121`은 `TERMINAL_SOLVER_UNQUALIFIED && path_constraints_ok`를 aligned exit로 취급한다. `:220`의 채택 후보도 `TAEM_PLAN_UNQUALIFIED`이고, 이유는 `Final tail qualification is pending`이다.

native replay는 HAC와 runway alignment까지 평가한다. live가 추가로 요구하는 Final delivery와 touchdown / rollout 완료를 증명하지 않는다. 따라서 `found=1; adopted=1`이나 후보 exitV≈165는 착륙 가능성의 증명이 아니다. 후보 선택·채택과 live 인계가 평가하는 범위에 차이가 있다.

## 4. 비동기 replan은 크게 달라진 상태에서 채택된다

최신 run의 request→adoption UT 차이는 11.12, 11.64, 11.18, 10.84초다. 그동안 along 이동은 약 5.81, 4.86, 3.13, 2.34km이고, ASL 고도 하강은 약 2.84, 2.56, 1.65, 1.25km다.

`controller/prediction_workers.inc:132`의 current 판정은 generation / phase / request token / snapshot identity를 확인한다. 실측 상태 이동량이나 채택 시점부터의 replay / splice qualification은 확인하지 않는다. `guidance_mm305_accept_plan`은 route cursor를 0으로 되돌린다.

계획의 출발 상태와 실제 채택 상태 사이의 차이는 구체적인 문제다. 다만 최종 정렬에는 도달하므로 이번 음수 에너지 게이트 거부의 유일한 원인으로 증명되지는 않았다. 채택 전 현재 상태에서 재검증하거나 상태 차이에 기반해 결과의 유효기간을 판정해야 한다.

## 5. JSONL의 block reason이 실제 거부 이유를 보여주지 않는다

최신 planner JSONL은 alignment 중에도 `taemTerminalEvaluationValid=false`, `taemTerminalBlockReason="Terminal contract invalid"`, margin=0을 기록한다.

MM305 alignment에서는 `final_contract`와 `final_eval`을 로컬로 계산하지만, planner logger는 `guidance_taem_exec.terminal_*`을 읽는다. 현재 해당 branch에는 평가 결과를 저장하는 코드가 없다. JSONL의 invalid / 0은 실제 Final 거부 이유를 나타내는 값으로 사용할 수 없다. 이번 진단은 native의 `FINAL_DELIVERY_ENERGY`를 근거로 한다.

또한 최신 planner JSON의 `candidateFinalDistance`는 약 11036.9m지만, native `MM305_ROUTE finalDistance`와 게이트의 station은 9000m다. 전자는 HAC exit 좌표와 일치한다. 정확한 Final alignment station과 혼동하면 안 된다.

## 수정·검증 순서

1. **같은 실측 state와 완전한 Final 내부 state / aero / response 조건에서 인계 전후 예측을 비교하는 replay**를 만든다. 약72→135m/s의 변화를 재현하고 initialization / parameter 중 무엇이 변하는지 확인한다.
2. **planner의 종단 판정을 live Final delivery와 맞추고 필요한 Final tail까지 평가한다.** 정렬까지만 검증한 후보와 전체 착륙이 검증된 후보를 구분해 기록한다.
3. **비동기 결과를 현재 state에서 재검증한 뒤 채택한다.** 시간뿐 아니라 위치·에너지·자세 변화를 포함한다.
4. **실제로 평가한 gate의 ready 여부, block reason, 모든 margin을 JSONL에 저장한다.** 기하 조건 미달과 Final 계약 거부를 구분할 수 있어야 한다.

테스트나 빌드는 추가 실행하지 않았다. JSONL 복원, native 로그 교차 확인, 실행 artifact와 source hash 대조, 관련 코드 확인을 수행했고 주요 원인 구분은 Jev로도 검토했다. 서로 다른 초기 상태에 대한 견고성과 전체 착륙 성공은 이번 조사에서 검증하지 않았다.
