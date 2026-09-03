# LeKiwi YOLO + GRIP — 빨간 큐브 자율 집기

LeKiwi(모바일 베이스 + SO-101 팔)가 **학습 정책 없이** 규칙 기반으로 빨간 큐브를 집는 파이프라인입니다.
YOLO(`roboseasylabs/red_cube_yolo`)로 큐브를 찾고, front 카메라로 다가가고, 손목 카메라를 보며 팔을
내려 집고, 제대로 집었는지 색으로 판별한 뒤, 큐브를 문 채 시작 자세로 돌아옵니다.
모든 설정은 CLI 인자(`--approach.*`, `--pick.*`, `--grasp.*`, `--check.*`)이고, 로봇에서 측정한 자세·목표 값은
`poses/*.json` 파일로 관리합니다. 2026-09-04 실물에서 전체 플로우 성공을 확인했습니다.

### 과제 정의

| 항목 | 내용 |
| --- | --- |
| 목표 | 테이블 위 빨간 큐브 1개를 찾아 집어 들고, 시작 자세로 돌아온다 |
| 입력 | front 카메라(640×480), wrist 카메라(640×480), 팔 관절 위치 |
| 출력 | 베이스 속도(`x.vel`, `y.vel`, `theta.vel`) + 팔 6관절 목표 위치 |
| 성공 판정 | front·wrist 두 뷰 모두에서 큐브 박스 좌/우에 보라색 그리퍼가 보임 |
| 실패 처리 | 집기 재시도 5회 → 시작 자세 복귀 후 3초 뒤 재접근, pick 시도 5회까지 |

---

## 목차

1. [파일 구성](#1-파일-구성)
2. [설치 (한 번만)](#2-설치-한-번만)
3. [실행 순서](#3-실행-순서) ← **처음이면 여기부터 순서대로**
4. [전체 플로우와 상태](#4-전체-플로우와-상태)
5. [화면과 조작키](#5-화면과-조작키)
6. [주요 옵션](#6-주요-옵션)
7. [저장 파일과 현재 값](#7-저장-파일과-현재-값)
8. [튜닝 가이드](#8-튜닝-가이드)
9. [문제 해결](#9-문제-해결)
10. [설계 메모](#10-설계-메모)

---

## 1. 파일 구성

| 파일 | 역할 |
| --- | --- |
| [download_hf_model.py](download_hf_model.py) | Hugging Face 에서 YOLO 가중치(`best.pt`) 내려받기 → `weights/` |
| [lekiwi_yolo_view.py](lekiwi_yolo_view.py) | front/wrist 뷰 실시간 YOLO 추론 표시 (로봇은 안 움직임). 박스 크기 재기·모델 확인용 |
| [lekiwi_save_pose.py](lekiwi_save_pose.py) | 지금 팔 관절 위치를 `poses/<이름>.json` 으로 저장 |
| [lekiwi_yolo_pick.py](lekiwi_yolo_pick.py) | **메인.** 접근 → 팔 자세 → 손목 서보 → 집기 → 판별 → 재시도/복귀 전체 흐름 |
| [wrist_servo.py](wrist_servo.py) | 손목 뷰 비주얼 서보 + 집기 관련 설정(`GraspArgs`) |
| [grasp_check.py](grasp_check.py) | 보라색 그리퍼 색 판별로 집기 성공/실패 결정(`GraspCheckArgs`) |
| `poses/` | 로봇에서 측정한 자세·참조 파일 (7장) |
| `weights/` | YOLO 가중치 (git 제외, 스크립트로 다시 받음) |

---

## 2. 설치 (한 번만)

상위 [README](../README.md) 1장의 `lerobot` conda 환경이 이미 있다는 전제입니다. 그 위에 두 패키지만 더 필요합니다.

```bash
conda activate lerobot
pip install ultralytics huggingface_hub
cd ~/workspace/lekiwi/yolo_and_pick
python download_hf_model.py            # → weights/best.pt (5MB)
```

- `download_hf_model.py --list` 로 레포 파일 확인, `--filename last.pt` 로 다른 가중치, `--force` 로 덮어쓰기.
- GPU 가 있으면 실행 시 `--yolo.device=0`, 없으면 생략(CPU). 이 스크립트들은 두 뷰를 한 배치로 추론합니다.

---

## 3. 실행 순서

### 0) 라즈베리파이에서 호스트 켜기 (세션마다)

```bash
python -m lerobot.robots.lekiwi.lekiwi_host \
    --robot.id=lekiwi01 \
    --robot.cameras='{
      front: {type: opencv, index_or_path: /dev/video0, width: 640, height: 480, fps: 30, rotation: 0, fourcc: MJPG},
      wrist: {type: opencv, index_or_path: /dev/video2, width: 640, height: 480, fps: 30, rotation: 0, fourcc: MJPG}
    }' \
    --host.connection_time_s=14400
```

호스트에는 **클라이언트가 하나만** 붙습니다. `lekiwi-teleoperate.py` / `lekiwi-record.py` 가 붙어 있으면 먼저 끄세요.
호스트가 살아 있는지 PC 에서 확인: `ping 192.168.0.201` 이 되고 5555/5556 포트가 열려 있어야 합니다.

### 1) 모델과 카메라 확인

```bash
python lekiwi_yolo_view.py
```

front(왼쪽)·wrist(오른쪽) 뷰에 박스가 그려지고 초록 십자선이 뜨면 정상입니다. 터미널에 뷰별 검출 수 / 최고 conf /
중심 좌표 / Hz 가 나옵니다. 이 화면으로 **박스 폭(px)** 을 읽어 목표 크기를 정합니다.

### 2) 자세 3개 저장 (로봇/환경이 바뀌었을 때만)

리더암(`lekiwi-teleoperate.py`)으로 팔을 원하는 자세로 만들고 → teleoperate 를 **끄고** → 저장합니다.
(호스트는 마지막 목표 위치를 유지하므로 teleoperate 를 꺼도 팔이 떨어지지 않습니다.)

| 이름 | 어떤 자세 | 명령 |
| --- | --- | --- |
| `pre_pick` | 큐브 앞에 도착했을 때 팔을 보낼 **집기 직전 자세** (그리퍼는 아무래도 됨, 실행 중 열림) | `python lekiwi_save_pose.py --name pre_pick` |
| `grasp` | 큐브를 **집는 순간의 자세** — 손목 서보가 뻗어 나가는 방향의 끝점 | `python lekiwi_save_pose.py --name grasp` |
| `grasp_closed` | 큐브를 **물고 있는 상태**. 그리퍼 값만 쓴다 (닫기 목표) | `python lekiwi_save_pose.py --name grasp_closed` |

`--samples 10` 으로 여러 번 읽어 평균낼 수 있고, `--list=true` 로 저장된 자세를 봅니다.

### 3) 손목 뷰 참조 구성 저장 (한 번)

집기 직전에 손목 카메라에 큐브가 **어떻게 보여야 하는지**(박스 왼쳝 변 위치, 위 변 위치, 폭)를 `poses/grasp_ref.json` 에 둡니다.
가장 쉬운 방법은 실행 중 그 화면이 나왔을 때 **`S` 키**를 누르는 것입니다 (다음 실행부터 적용). 현재 파일은 이미 검증된 값이라
큐브·카메라 위치가 바뀌지 않았다면 건너뛰어도 됩니다.

### 4) 먼저 dry run 으로 판단만 보기

```bash
python lekiwi_yolo_pick.py --dry_run=true
```

바퀴·팔 명령은 보내지 않고(정지만) 화면과 터미널에 상태·오차·명령 값을 보여 줍니다. 아래를 확인하세요.

- 큐브가 화면 오른쪽이면 `th=-…`(우회전), 왼쪽이면 `th=+…` 가 나오는지
- 큐브가 멀면 `x=+…`, 아래에 잘려 납작하면 `TOO_CLOSE` 와 `x=-…` 가 나오는지
- 손목 뷰에 박스가 **하나만** 그려지는지 (왼쪽 그리퍼 오검출이 걸러지는지)

### 5) 실제 실행

```bash
python lekiwi_yolo_pick.py
```

처음엔 `--start_paused=true` 로 시작해 화면을 보고 SPACE 로 출발시키거나, `--approach.max_speed=0.05` 로 느리게 시작하는 것을 권합니다.
끝나면 `=== TASK 완료 ===` 가 찍히고 팔은 큐브를 문 채 시작 자세로 유지됩니다. 종료는 Q/ESC 또는 Ctrl+C (바퀴 정지 후 연결 해제).

### 6) 자주 쓰는 변형

```bash
python lekiwi_yolo_pick.py --grasp.exit_when_done=true          # 집고 돌아오면 스크립트 종료
python lekiwi_yolo_pick.py --grasp.enabled=false                # 접근 + pick 자세까지만
python lekiwi_yolo_pick.py --pick.enabled=false                 # 접근·정렬만 (팔 고정)
python lekiwi_yolo_pick.py --approach.target_size_px=130        # front 뷰에서 더 가까이 멈춤
python lekiwi_yolo_pick.py --grasp.gripper_close_extra_pct=3    # 더 세게 물기
python lekiwi_yolo_pick.py --display=none                       # SSH/헤드리스
```

---

## 4. 전체 플로우와 상태

```
 [베이스]  SEARCHING ─ 큐브 없음/가로선 위 → 정지
           ROTATING  ─ 박스 중심 x 가 세로선 ±tol 밖 → 제자리 회전 (전진 안 함)
           ALIGNING  ─ 폭 < 목표 → 전진 / 폭 > 목표+tol → 후진
           TOO_CLOSE ─ 박스가 바닥에 붙고 납작(h/w<0.8) 또는 작음 → 후진 (잘린 것)
           ALIGNED   ─ 폭·중심 10프레임 연속 OK
 [팔]      ARM_TO_PICK → PICK_READY        pre_pick 자세로 2s 보간 (이후 바퀴 잠금)
           GRIPPER_OPEN                    그리퍼 100% (1s)
           WRIST_CENTERING                 손목 뷰: 박스 왼쪽 변→세로선, 화면 중앙 높이→박스 안 (pan/tilt 보정)
           WRIST_APPROACH                  맞으면 grasp 자세 방향으로 뻗기 (박스가 커짐)
           WRIST_REFINE                    크기는 됐는데 가로/세로가 안 맞음 → 2s 보정 후 강제 진행
           GRASP_READY                     폭 ≥ 목표-20 & 가로/세로 OK,  또는 front 정중앙에 보라색(높이 힌트)
           GRIPPER_CLOSE → GRASPED         15.1% 로 닫기 (1s)
 [판별]    GRASP_CHECK                     두 뷰 박스 좌/우 띠의 보라색 비율 ≥ 0.15, 5프레임 연속 → OK / 3s 초과 → FAIL
           GRASP_FAIL → (재시도 ≤5)        벌리고 목표 폭 +30px, 경로 +15% 더 → 다시 READY→닫기→판별
                      → (pick 재시도 ≤5)   벌리고 시작 자세 복귀 → HOME_WAIT 3s → 접근부터 다시
                      → GIVE_UP            모두 실패 → 그 자리에서 정지
           GRASP_OK → CARRY_HOME → DONE    큐브 문 채 3s 동안 시작 자세로  ← task 끝
```

팔이 `HOME` 이 아닌 동안(자세 이동·서보·집기·판별·운반)에는 바퀴 명령을 절대 보내지 않습니다.
검출이 끊기면 그 자리에서 멈추고(`WRIST_LOST`), 호스트 워치독도 명령이 끊기면 스스로 바퀴를 세웁니다.

---

## 5. 화면과 조작키

창 하나에 **front(왼쪽) | wrist(오른쪽)** 가 붙어 나옵니다.

| 표시 | 뜻 |
| --- | --- |
| 초록 십자선 | 화면 정중앙 (두 뷰 모두) |
| front: 초록 세로 띠 / `target zone` 가로선 | 좌우 허용 범위 / 이 선 **아래**의 검출만 목표 (위는 회색 X 로 무시 표시) |
| front: 자홍 박스 + 노란 참조 사각형 | 목표 큐브 / 도착해야 할 폭(117px) |
| front: 중앙 작은 사각형 `hint 0.xx` | 서보 중 정중앙 보라색 비율 (높이 힌트, 초록이면 문턱 이상) |
| wrist: 두꺼운 왼쪽 변·위 변 / 노란 십자 | 세로선·참조 위치에 맞출 기준 (맞으면 초록) / 기준이 와야 할 목표점 |
| 양 뷰: 박스 좌/우 반투명 띠 `0.xx` | 집기 판별용 보라색 비율 (자주=있음, 회색=없음), `GRIP?`/`GRIP OK`/`GRIP FAIL` |
| 하단 상태바 | 현재 상태와 바퀴 명령 `x= y= m/s th= deg/s`, 팔 단계 진행률 |

| 키 | 동작 |
| --- | --- |
| `SPACE` | 일시정지/재개 (정지 명령만 보냄, 팔 단계도 멈춤) |
| `S` | 지금 손목 뷰의 박스-화면중심 관계를 `poses/grasp_ref.json` 으로 저장 |
| `Q` / `ESC` | 종료 (바퀴 정지 → 연결 해제). 창을 닫아도 같음 |

터미널 한 줄 상태: `[WRIST_APPROACH] … | wrist APPROACHING width=210/288 left-dx=+3✓ dy=-20✓ reach 60% pan+0.8 tilt-1.2 | 29.7 Hz`

---

## 6. 주요 옵션

전체 목록은 `python lekiwi_yolo_pick.py --help`. 자주 만지는 것만:

**접근 `--approach.*`** (front 뷰, 베이스)

| 옵션 | 기본 | 뜻 |
| --- | --- | --- |
| `target_size_px` / `size_tolerance_px` | 117 / 10 | 멈출 박스 폭. `lekiwi_yolo_view.py` 로 재서 넣기 |
| `center_tolerance_px` | 100 | 세로선 좌우 허용. 작을수록 회전 정렬을 더 정확히 |
| `lateral_mode` | `rotate` | `rotate`(제자리 회전 우선) / `strafe`(옆 이동, 전진과 동시) |
| `max_speed` / `min_speed` | 0.1 / 0.03 m/s | 전후 속도 상/하한 |
| `max_theta_speed` / `min_theta_speed` | 30 / 8 deg/s | 회전 속도 상/하한 |
| `line_ratio` / `only_below_line` | 0.5 / true | 가로선 위치 / 위쪽 검출 무시 |
| `clip_aspect_min` / `bottom_margin_px` | 0.8 / 5 | 잘림(너무 가까움) 판정 |

**pick 자세 `--pick.*`**: `pose_file`(poses/pre_pick.json), `move_time_s`(2), `drift_frames`(30, pick 자세에서 큐브가 벗어나면 복귀)

**집기 `--grasp.*`** (wrist 뷰, 팔)

| 옵션 | 기본 | 뜻 |
| --- | --- | --- |
| `ref_file` | poses/grasp_ref.json | 손목 뷰 목표 구성 (있으면 아래 x/y/size 목표를 덮어씀) |
| `x_anchor` / `x_tolerance_px` | left / 25 | 세로선에 맞출 박스 변 (그리퍼 왼쪽에 카메라 → 왼쪽 변) |
| `y_anchor` / `y_tolerance_px` | top / 40 | 위 변 기준 (아래 변은 잘리므로) |
| `target_size_px` / `size_tolerance_px` | 288 / 20 | 도착 폭 |
| `refine_timeout_s` | 2 | 크기 도달 후 가로/세로가 안 맞아도 이 시간 뒤 진행 |
| `pan_gain` `pan_sign` / `tilt_gain` `tilt_sign` | 0.15 ±1 | 보정 게인·부호 (shoulder_pan / wrist_flex) |
| `max_correction_deg` / `max_joint_speed_dps` | 20 / 20 | 보정 누적 한계 / 관절 속도 한계 (안전장치) |
| `approach_mode` | pose | `pose`(grasp 자세 방향 보간) / `joints`(`reach_joints` 관절별 deg/s) |
| `gripper_open_pct` / `close_pose_file` / `gripper_close_extra_pct` | 100 / grasp_closed.json / 0 | 열기·닫기 목표 |
| `front_hint` / `front_hint_min_ratio` | true / 0.5 | front 정중앙 보라색 = 알맞은 높이 (첫 시도만) |
| `single_box` / `left_region_ratio` / `left_min_conf` | true / 0.5 / 0.85 | 손목 뷰 박스 하나만, 왼쪽(그리퍼) 쪽은 conf 빡빡하게 |
| `max_retries` / `retry_size_step_px` / `retry_overreach` | 5 / 30 / 0.15 | 집기 재시도 |
| `max_pick_attempts` / `pick_retry_wait_s` | 5 / 3 | pick 재시도 (시작 자세 복귀 후 대기) |
| `carry_time_s` / `exit_when_done` | 3 / false | 성공 후 시작 자세 복귀 시간 / 끝나면 종료 |

**판별 `--check.*`**: `hue_min/hue_max/sat_min/val_min`(115/165/70/40, 보라색 HSV), `band_px`(40), `min_purple_ratio`(0.15),
`confirm_frames`(5), `timeout_s`(3), `require_both_sides`(true), `exit_on_result`(false)

---

## 7. 저장 파일과 현재 값

모두 `poses/` 아래. 팔 관절은 lerobot DEGREES 모드(각 관절 기계적 가동범위 중심이 0°), 그리퍼는 0~100%.

| 파일 | 용도 | 현재 값 (2026-09-04 실물 측정, 성공 확인) |
| --- | --- | --- |
| `pre_pick.json` | 집기 직전 팔 자세 | pan -1.9, lift 21.6, elbow 5.2, wrist_flex 50.3, roll -6.6, gripper 3.5 |
| `grasp.json` | 집는 순간 자세 (뻗는 방향 끝점) | pan -3.8, lift 70.5, elbow -23.8, wrist_flex 51.7, roll -9.5, gripper 27.8 |
| `grasp_closed.json` | 물고 있는 상태 (그리퍼만 사용) | **gripper 15.1** |
| `grasp_ref.json` | 손목 뷰 목표 구성 | 왼쪽 변 dx **-23**, 위 변 dy **-139**, 폭 **288** (화면 중심 320,240 기준) |
| `*_prev.json` | 직전 값 백업 | — |

다른 로봇에 옮길 때: 캘리브레이션에서 각 관절을 양쪽 하드스톱까지 훑었다면 `pan/lift/elbow/wrist_flex` 는 거의 그대로 재현되지만,
**`wrist_roll` 은 homing 때 잡은 각도에 의존**하므로 재현되지 않습니다. 가장 확실한 방법은 로봇마다 2)·3) 을 다시 하는 것(1~2분).

---

## 8. 튜닝 가이드

| 증상 | 조치 |
| --- | --- |
| front 에서 회전이 반대 방향 | `--dry_run=true` 로 `th` 부호 확인. LeKiwi 는 `theta +` 가 좌회전. 코드 부호가 맞지 않으면 알려줄 것 |
| 큐브 앞에 너무 멀리/가까이 멈춤 | `lekiwi_yolo_view.py` 로 원하는 거리에서 폭을 읽어 `--approach.target_size_px` |
| 손목 서보가 오차를 키우는 쪽으로 돎 | `--grasp.max_correction_deg=5` 로 낮춰 방향을 보고 `--grasp.pan_sign=-1` / `--grasp.tilt_sign=-1` |
| 계속 `WRIST_REFINE` 만 하다 진행 | `x_tolerance_px`/`y_tolerance_px` 를 넉넉히, 또는 `refine_timeout_s` 로 강제 진행 (기본 2s) |
| 너무 얕게/깊게 집음 | `S` 로 알맞은 순간의 참조를 다시 저장. 또는 `--grasp.target_size_px` |
| 물었는데 미끄러짐 | `--grasp.gripper_close_extra_pct=2~5` (STS3215 과부하 주의, 서서히) |
| 판별이 항상 FAIL (실제론 집음) | 상태줄의 `front L/R`, `wrist L/R` 비율을 보고 `--check.min_purple_ratio` 를 낮추거나 `--check.band_px` 를 넓힘. 조명 바뀌면 `hue/sat/val` |
| 재시도하다 테이블에 닿음 | `--grasp.retry_overreach=0.08`, `--grasp.max_retries=3` |
| 왼쪽 그리퍼가 큐브로 잡힘 | `--grasp.left_min_conf=0.9`, `--grasp.left_region_ratio=0.55` |
| 추론이 느림 (Hz 낮음) | `--yolo.device=0`(GPU), `--yolo.imgsz=480`, `--views='[front]'`(집기 단계는 wrist 필요) |

---

## 9. 문제 해결

**`LeKiwi 호스트(192.168.0.201)에 연결하지 못했습니다`**
라즈베리파이에서 `lekiwi_host` 가 꺼져 있거나 다른 클라이언트가 붙어 있습니다. `ping` 이 되는데 5555/5556 이 닫혀 있으면 호스트 프로세스가 없는 것.
이 PC 에서 `pgrep -af lekiwi` 로 다른 스크립트가 붙어 있는지 확인.

**`ultralytics 가 설치돼 있지 않습니다`** → `pip install ultralytics` (conda env `lerobot` 에서).

**`모델 파일이 없습니다`** → `python download_hf_model.py`.

**`pick/grasp/닫힘 자세 파일이 없습니다`** → 3장 2) 대로 저장. 팔을 안 움직이려면 `--pick.enabled=false`.

**큐브가 파랗게 보임 / 검출이 안 됨** → LeKiwi 프레임은 RGB 라 BGR 변환이 필요한데 이 스크립트들은 이미 처리합니다. 다른 코드에 붙일 때 주의.

**화면이 안 뜸 (SSH)** → `lekiwi_yolo_view.py --display=rerun`, `lekiwi_yolo_pick.py --display=none`.

**`--help` 가 에러** → 필드 주석에 `%` 가 들어가면 argparse 가 깨집니다. 주석에는 "퍼센트" 로 쓰기.

**연결/파싱만 확인하고 싶을 때** → 반드시 도달 불가 IP 로: `--robot.remote_ip=10.255.255.1 --robot.connect_timeout_s=1`.
`--dry_run=true` 도 실제 호스트에 접속하고 제어 루프를 돌립니다(명령은 정지만 보냄).

---

## 10. 설계 메모

- **왜 회전 우선인가**: 옆 이동(strafe)은 카메라 시야에서 큐브가 빠르게 미끄러지지만, 제자리 회전은 큐브를 화면 중앙으로 부드럽게 가져오고 그 뒤 직진만 하면 됩니다. 히스테리시스(`center_hysteresis_px`)로 경계에서의 떨림을 막습니다.
- **왜 폭(width)인가**: 큐브가 가까워지면 아래 변이 화면 밖으로 잘려 높이·중심 y 를 믿을 수 없습니다. 같은 이유로 손목 뷰 참조도 **위 변**을 씁니다.
- **왜 왼쪽 변인가**: 손목 카메라가 그리퍼 **왼쪽**에 달려 있어 큐브가 화면 오른쪽 절반에 있어야 두 손가락 사이에 들어옵니다.
- **왜 grasp 자세 보간인가**: 관절 부호를 추측해 뻗는 것보다, 사람이 만든 집기 자세 방향으로 보간하면서 pan/tilt 만 보정하는 게 안전하고 재현성이 높습니다. 보정 누적과 관절 속도에 한계를 두어 부호가 틀려도 크게 다치지 않게 했습니다.
- **왜 색으로 판별하나**: 집기 성공은 YOLO 가 알려주지 않습니다. 그리퍼(보라)와 큐브(빨강), 테이블(흰색)이 HSV 에서 잘 갈라져 박스 좌/우 띠의 보라색 비율만으로 충분했습니다.
- **정보 흐름**: `LeKiwiClient.get_observation()` → RGB→BGR → YOLO 배치 추론(두 뷰) → `Approacher`(베이스) / `ArmSequencer`(팔 상태기) → `WristServo`(팔 보정) → `GraspChecker`(판별) → `send_action({팔 6관절, x/y/theta.vel})`.
