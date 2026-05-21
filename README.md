# AGV Traffic Control Visualizer

AGV(Automated Guided Vehicle) 교통 제어 알고리즘을 시각적으로 테스트하는 Python Qt GUI 애플리케이션입니다. 캔버스에서 노드와 엣지로 작업장 맵을 만들고, 여러 AGV의 출발지/목적지/작업 시간/속도/왕복 횟수를 설정한 뒤 RHCR 기반의 rolling-horizon 시뮬레이션을 실행할 수 있습니다.

## 주요 기능

- 그래프 기반 작업장 맵 편집
  - 캔버스 더블 클릭으로 노드 추가
  - 노드에서 다른 노드로 드래그해 엣지 추가
  - 우측 패널에서 선택 노드 추가/삭제
- AGV 시뮬레이션
  - 여러 AGV의 시작 노드, 작업 목표 노드, 작업 후 목표 노드 설정
  - AGV별 이동 속도, 작업 대기 시간, 왕복 횟수 설정
  - Play, Pause, Step, Reset 제어
- 교통 제어 시각화
  - RHCR(Rolling Horizon Collision Resolution) 방식의 주기적 재계획
  - PBS(Priority-Based Search) window solver 기반 경로 계획
  - 노드/엣지 충돌, swap conflict, 동일 엣지 점유 충돌 감지
  - 예약 엣지, 계획 경로, conflict zone, deadlock 상태 표시
- 샘플 맵 로드
  - `Load Sample Map` 버튼으로 기본 맵과 AGV 3대를 바로 구성

## 요구 사항

- Python 3.10 이상 권장
- Qt 바인딩 중 하나
  - 권장: `PySide6`
  - 대안: `PyQt6`

## 설치

```bash
python -m pip install PySide6
```

`PySide6` 대신 `PyQt6`를 사용할 수도 있습니다.

```bash
python -m pip install PyQt6
```

## 실행

```bash
python main.py
```

애플리케이션은 먼저 `PySide6`를 사용하고, 설치되어 있지 않으면 `PyQt6`로 fallback합니다.

## 사용 방법

1. `Load Sample Map`을 눌러 기본 맵을 불러오거나, 캔버스에서 직접 맵을 만듭니다.
2. 직접 맵을 만들 때는 캔버스 빈 공간을 더블 클릭해 노드를 추가합니다.
3. 노드를 클릭한 상태로 다른 노드까지 드래그해 엣지를 추가합니다.
4. `Add AGV` 영역에서 시작 노드, 작업 목표 노드, 작업 후 목표 노드, 작업 시간, 속도, 왕복 횟수를 설정합니다.
5. `Add AGV` 버튼으로 AGV를 추가합니다.
6. `Play`로 자동 실행하거나 `Step`으로 한 tick씩 진행합니다.
7. `Pause`로 일시정지하고, `Reset`으로 AGV를 시작 상태로 되돌립니다.

캔버스 위에서 마우스 휠을 사용하면 확대/축소할 수 있습니다.

## UI 구성

- 왼쪽 캔버스
  - 노드, 엣지, AGV 위치, 계획 경로, 예약 상태, conflict zone을 표시합니다.
- 오른쪽 패널
  - `Clock`: 현재 시뮬레이션 tick
  - `Status`: 실행, 일시정지, 완료, deadlock 상태
  - `Node Tools`: 노드 선택, 자동 추가, 삭제
  - `Add AGV`: 새 AGV 추가
  - `Edit AGV`: 기존 AGV 수정/삭제
  - `AGV State`: AGV별 현재 상태, 경로, 대기 횟수, 재계획 횟수
  - `Simulation Log`: 계획/충돌/이동 로그

## 샘플 맵

`Load Sample Map`은 다음 구성을 생성합니다.

- 노드: `1`부터 `16` 일부 번호를 사용하는 작업장 형태의 그래프
- 버퍼 노드: `7`, `10`
- AGV:
  - `A`: `16 -> 1`
  - `B`: `13 -> 2`
  - `C`: `15 -> 2`

## 알고리즘 개요

교통 제어는 `TrafficController`와 `PBSWindowSolver`가 담당합니다.

- RHCR 시뮬레이션 window: `5` tick
- RHCR planning window: `25` tick
- 매 planning 주기마다 현재 AGV 상태를 기준으로 windowed MAPF 문제를 풉니다.
- PBS solver는 AGV 간 우선순위 제약을 탐색하면서 각 AGV의 state-time A* 경로를 생성합니다.
- 실행 단계에서는 다음 이동이 예약, 노드 점유, 엣지 충돌, swap conflict를 일으키는지 검사합니다.
- 일정 시간 이상 전체 AGV가 움직이지 못하면 deadlock 상태로 판단합니다.

## 파일 구조

```text
.
├── main.py                         # 애플리케이션 진입점
├── qt_compat.py                    # PySide6/PyQt6 호환 레이어
├── main_window.py                  # 메인 윈도우와 UI 이벤트 처리
├── graph_canvas.py                 # 그래프/AGV 시각화 캔버스
├── graph_model.py                  # 노드/엣지 그래프 모델
├── models.py                       # AGV, 충돌, 예약 관련 dataclass
├── traffic_controller.py           # RHCR 기반 교통 제어 로직
├── pbs_window_solver.py            # PBS window solver
├── constants.py                    # UI/시뮬레이션 상수
└── legacy_traffic_algorithm_commented.py
```

## 개발 메모

- 노드 ID `7`, `10`은 기본 버퍼 노드로 취급됩니다.
- 차수가 3 이상인 노드는 UI 표시용 conflict zone 중심으로 계산됩니다.
- 그래프를 수정하면 기존 계획, 예약, lock 상태가 무효화되고 다음 실행에서 다시 계획합니다.
- `legacy_traffic_algorithm_commented.py`는 이전 알고리즘을 주석과 함께 보존한 참고 파일입니다.
