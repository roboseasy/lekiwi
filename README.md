# LeKiwi 조작 · 녹화 · 재생 · 정책 실행 스크립트

[LeRobot](https://github.com/huggingface/lerobot) 의 LeKiwi(모바일 베이스 + SO-101 팔) 를
**PC 에서 원격으로** 조작하고, 데이터셋을 녹화하고, 학습한 정책을 실물에서 돌려 보기 위한
스크립트 모음입니다. 모든 설정은 CLI 인자로 넘기며, 코드를 고칠 일이 없습니다.

lerobot 이 기본 제공하는 `lerobot-teleoperate` / `lerobot-record` / `lerobot-replay` /
`lerobot-rollout` / `lerobot-eval` 은 LeKiwi 에 그대로 쓸 수 없습니다. 이 저장소의
스크립트는 각각 그 빈틈을 메웁니다.

| 스크립트 | 하는 일 | lerobot 기본 CLI 를 못 쓰는 이유 |
| --- | --- | --- |
| [lekiwi-teleoperate.py](lekiwi-teleoperate.py) | 리더암 + 키보드로 실시간 조작 (녹화 없음) | `lerobot-teleoperate` 는 teleop 을 **하나만** 받는다. LeKiwi 는 팔(리더암) + 베이스(키보드) 두 개가 동시에 필요 |
| [lekiwi-record.py](lekiwi-record.py) | 조작하면서 LeRobotDataset 녹화 | `lerobot-record` 도 같은 이유로 teleop 하나만 받음 |
| [lekiwi-replay.py](lekiwi-replay.py) | 녹화된 에피소드의 action 을 그대로 재생 | `lerobot-replay` 는 `lekiwi_client` 를 import 하지 않아 `--robot.type=lekiwi_client` 를 모르고, 중간에 멈출 방법도 없다 |
| [lekiwi-rollout.py](lekiwi-rollout.py) | 학습한 정책으로 자율 실행 | `lerobot-rollout` 은 위와 같고, 더해서 `rollout/context.py` 가 `.pos` 만 남기고 **`.vel`(베이스 3채널)을 버린다** → 베이스가 아예 안 움직이고 채널 수도 어긋남 |
| [lekiwi-evaluate.py](lekiwi-evaluate.py) | N 에피소드 돌리고 성공률 · 지연 리포트 | `lerobot-eval` 은 gym 시뮬레이션 환경 전용. 실물에는 환경도 리워드도 없다 |


### 목차

1. [설치](#1-설치) — conda 환경부터 lerobot 설치·캘리브레이션까지
2. [실행 전 준비](#2-실행-전-준비) — 라즈베리파이 호스트, 카메라 규칙
3. [공통 인자](#3-모든-스크립트가-공유하는-인자)
4. [스크립트별 설명](#4-스크립트별-설명)
5. [전체 워크플로](#5-전체-워크플로)
6. [공통 설계](#6-공통-설계)
7. [문제 해결](#7-문제-해결)

### 한눈에 보는 빠른 시작

```bash
# PC (한 번만)
conda create -n lerobot python=3.12 -y && conda activate lerobot
conda install ffmpeg -c conda-forge
git clone https://github.com/huggingface/lerobot.git && cd lerobot
pip install -e ".[core_scripts,lekiwi]"

# Lekiwi[라즈베리파이] (세션마다)
python -m lerobot.robots.lekiwi.lekiwi_host --robot.id=lekiwi01 --robot.cameras='{...}'

# PC (세션마다)
conda activate lerobot && cd ~/workspace/lekiwi
python lekiwi-teleoperate.py
```

---

## 1. 설치

### 1.1 conda 환경 만들기 (PC)

lerobot 은 **Python 3.12 이상**이 필요합니다.

```bash
conda create -n lerobot python=3.12 -y
conda activate lerobot
```

셸을 새로 열 때마다 `conda activate lerobot` 를 해 줘야 합니다.

> conda 가 없다면 [miniforge](https://conda-forge.org/download/) 를 먼저 설치하세요.
> ```bash
> wget "https://github.com/conda-forge/miniforge/releases/latest/download/Miniforge3-$(uname)-$(uname -m).sh"
> bash Miniforge3-$(uname)-$(uname -m).sh
> ```

### 1.2 ffmpeg 설치 (영상 인코딩/디코딩)

데이터셋 영상은 TorchCodec 으로 다루며 `ffmpeg` 가 필요합니다. conda 환경 안에 넣는 방식이
PyTorch 버전을 안 타서 가장 안전합니다.

```bash
conda install ffmpeg -c conda-forge
```

`libsvtav1` 인코더가 없다는 오류(`ffmpeg -encoders` 로 확인)나 torchcodec 버전 충돌이 나면
7.1.1 로 고정합니다.

```bash
conda install ffmpeg=7.1.1 -c conda-forge
```

### 1.3 lerobot 설치 (PC)

**소스에서 (권장 — LeKiwi 는 코드를 들여다볼 일이 많습니다)**

```bash
git clone https://github.com/huggingface/lerobot.git
cd lerobot
pip install -e ".[core_scripts,lekiwi]"
```

**PyPI 에서**

```bash
pip install 'lerobot[core_scripts,lekiwi]'
```

extra 가 무엇을 넣는지:

| extra | 내용 | 왜 필요한가 |
| --- | --- | --- |
| `core_scripts` | `dataset` + `hardware` + `viz` | 녹화(datasets/torchcodec), 키보드·시리얼(pynput/pyserial), rerun 시각화 |
| `lekiwi` | `feetech` + `pyzmq` | LeKiwi 모터 SDK 와 PC↔라즈베리파이 ZMQ 통신 |
| `training` | `dataset` + `accelerate` + `wandb` | 정책 학습(`lerobot-train`) 을 이 PC 에서 할 때 추가 |

정책을 이 PC 에서 학습까지 한다면:

```bash
pip install -e ".[core_scripts,lekiwi,training]"
```

`smolvla`, `pi` 같은 언어 조건부 정책을 쓸 거라면 해당 extra 도 함께 넣습니다
(예: `pip install -e ".[core_scripts,lekiwi,training,smolvla]"`).

> **자주 겪는 함정:** `pip install -e .` 만 한 환경에는 `datasets` 가 없어서
> [lekiwi-record.py](lekiwi-record.py) 임포트부터 실패합니다. 반드시 `dataset` 이 포함된
> extra(`core_scripts` 또는 `dataset`)로 설치하세요.
> ```bash
> python -c "import datasets, lerobot; print(lerobot.__version__)"
> ```

`torch` 는 lerobot 의 기본 의존성이라 따로 설치할 필요가 없지만, GPU 로 추론/학습하려면
CUDA 빌드가 맞는지 확인하세요.

```bash
python -c "import torch; print(torch.__version__, torch.cuda.is_available())"
```

### 1.4 라즈베리파이(LeKiwi 본체) 설치

라즈베리파이에도 같은 순서로 설치합니다. 여기서는 카메라를 열고 모터를 돌리기만 하므로
학습 관련 extra 는 필요 없습니다.

```bash
# 라즈베리파이에서 (ssh 접속 후)
conda create -n lerobot python=3.12 -y
conda activate lerobot

git clone https://github.com/huggingface/lerobot.git
cd lerobot
pip install -e ".[lekiwi,viz]"
```

> 라즈베리파이(aarch64)에는 torchcodec 휠이 없을 수 있고, 그럴 땐 lerobot 이 자동으로
> pyav 로 넘어갑니다. 호스트는 영상을 인코딩하지 않으므로 문제되지 않습니다.

### 1.5 USB 포트 찾기와 캘리브레이션

포트 찾기 (케이블을 뺐다 꽂으며 확인):

```bash
lerobot-find-port
```

리눅스에서 권한이 없다면:

```bash
sudo chmod 666 /dev/ttyACM0
```

**팔로워 팔 캘리브레이션 — 라즈베리파이에서**

```bash
lerobot-calibrate \
    --robot.type=lekiwi \
    --robot.id=lekiwi01 \
    --robot.port=/dev/ttyACM0 \
    --robot.cameras='{}'
```

**리더암 캘리브레이션 — PC 에서**

```bash
lerobot-calibrate \
    --teleop.type=so101_leader \
    --teleop.port=/dev/ttyACM0 \
    --teleop.id=leader
```

각 관절을 가운데로 두고 Enter, 그다음 모든 관절을 가동 범위 끝까지 움직입니다.
결과는 `~/.cache/huggingface/lerobot/calibration/` 아래에 `<id>.json` 으로 저장되고,
스크립트의 `--robot.id` / `--teleop.id` 가 그 파일을 가리킵니다. 바퀴 모터는 캘리브레이션이
필요 없습니다.

**(선택) 리더암 포트 이름 고정** — `/dev/ttyACM0` 번호는 꽂는 순서에 따라 바뀝니다.
udev 규칙으로 고정해 두면 `--teleop.port=/dev/so101-leader` 기본값을 그대로 쓸 수 있습니다.

```bash
udevadm info -a -n /dev/ttyACM0 | grep -m1 -E 'idVendor|idProduct|\{serial\}'
sudo tee /etc/udev/rules.d/99-so101-leader.rules <<'RULE'
SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="55d3", SYMLINK+="so101-leader"
RULE
sudo udevadm control --reload-rules && sudo udevadm trigger
```

`idVendor`/`idProduct` 는 위 `udevadm` 출력에서 나온 값으로 바꾸고, 같은 칩의 보드가 여러 개면
`ATTRS{serial}=="..."` 조건을 추가하세요.

### 1.6 이 저장소 받기

```bash
git clone <this-repo> ~/workspace/lekiwi
cd ~/workspace/lekiwi
```

스크립트는 설치 없이 그대로 실행합니다 (`python lekiwi-record.py ...`).
`conda activate lerobot` 된 셸에서 실행하면 됩니다.

---

## 2. 실행 전 준비

### 라즈베리파이(LeKiwi) 쪽 호스트 실행

모든 스크립트는 라즈베리파이에서 `lekiwi_host` 가 이미 돌고 있다고 가정합니다.

```bash
python -m lerobot.robots.lekiwi.lekiwi_host \
    --robot.id=lekiwi01 \
    --robot.cameras='{
      front: {type: opencv, index_or_path: /dev/video0, width: 640, height: 480, fps: 30, rotation: 0, fourcc: MJPG},
      wrist: {type: opencv, index_or_path: /dev/video2, width: 640, height: 480, fps: 30, rotation: 0, fourcc: MJPG}
    }' \
    --host.connection_time_s=14400
```

`--host.connection_time_s` 이 지나면 호스트가 스스로 종료하므로, 긴 세션에서는 넉넉히 줍니다.

### 카메라 해상도 규칙 (자주 걸리는 함정)

- 호스트 설정의 `width`/`height` 는 **rotation 을 적용한 뒤의 출력 크기**입니다.
  캡처 해상도는 rotation 이 90/270 일 때만 뒤집혀 설정됩니다
  (`cameras/opencv/camera_opencv.py`).
  센서가 640x480 인 웹캠이면 → rotation 0/180 은 `width 640, height 480`,
  rotation 90/-90 은 `width 480, height 640`. 다른 조합은 캡처 설정에 실패합니다.
- **클라이언트(PC)는 카메라를 열지 않습니다.** 스크립트의 `--robot.cameras` 는
  "호스트가 이런 이름/크기로 프레임을 보내온다"는 *선언*이고, `index_or_path` 는 무시됩니다.
  **이름**은 호스트와 반드시 같아야 프레임을 받습니다.
- 녹화/정책 스크립트는 연결 직후 실제 프레임 shape 을 확인해서, 설정과 다르면 자동 보정하고
  경고를 남깁니다. 엄격하게 검사하려면 `--auto_camera_shape=false`, 검사 자체를 끄려면
  `--check_cameras=false`.

---

## 3. 모든 스크립트가 공유하는 인자

인자 파싱은 lerobot 과 동일한 draccus 기반이라 `--그룹.필드=값` 형태입니다.
전체 목록은 언제든 `python <스크립트> --help`.

### `--robot.*` — LeKiwi 호스트 접속

| 인자 | 기본값 | 설명 |
| --- | --- | --- |
| `--robot.remote_ip` | `192.168.0.201` | 라즈베리파이 IP |
| `--robot.id` | `lekiwi01` | 로봇 id (캘리브레이션 파일 이름) |
| `--robot.port_zmq_cmd` | `5555` | 명령 채널 포트 |
| `--robot.port_zmq_observations` | `5556` | 관측 채널 포트 |
| `--robot.connect_timeout_s` | `5` | 접속 타임아웃 |
| `--robot.cameras` | `front`(/dev/video0), `wrist`(/dev/video2), 640x480@30 | 호스트가 보내오는 카메라 선언 |

### `--teleop.*` — PC 에 USB 로 물린 리더암 (teleoperate / record 전용)

| 인자 | 기본값 | 설명 |
| --- | --- | --- |
| `--teleop.port` | `/dev/so101-leader` | 리더암 시리얼 포트 |
| `--teleop.id` | `leader` | 캘리브레이션 id |
| `--teleop.use_degrees` | `true` | 관절 단위를 degree 로 |

### 시각화 · 기타 공통

| 인자 | 기본값 | 설명 |
| --- | --- | --- |
| `--display_data` | `true` | rerun 으로 카메라/액션 스트리밍 |
| `--display_mode` | `rerun` | 시각화 백엔드 |
| `--display_ip` / `--display_port` | `None` | 원격 rerun 서버로 보낼 때 |
| `--display_compressed_images` | `false` | 전송량 줄이기 (원격 rerun 이면 자동 활성) |
| `--play_sounds` | `true` | 음성 안내 (record / replay / rollout / evaluate) |
| `--print_status` | `true` | 한 줄짜리 상태 표시 (베이스 속도, 루프 Hz 등) |
| `--fps` | `30` | 제어 루프 주기 (record 는 `--dataset.fps`) |

### 키보드 백엔드 `--base_control` (teleoperate / record)

베이스 주행에는 연속 키 입력이 필요해서 백엔드를 고를 수 있습니다.

| 값 | 동작 |
| --- | --- |
| `auto` (기본) | X11 이면 `pynput`, Wayland/헤드리스면 터미널 입력 |
| `pynput` | 전역 키 후킹. X11 전용. 키를 **누르고 있는 동안** 이동 |
| `terminal` | 이 터미널의 stdin 을 읽음. Wayland 에서도 동작하지만 **터미널 창에 포커스** 필요 |
| `none` | 베이스 주행 없음 (팔만 조작, 베이스 속도는 0 으로 기록) |

터미널 백엔드에는 key-release 이벤트가 없어서, `--base_hold_s`(기본 0.35초) 동안 키가
다시 안 들어오면 "뗐다"고 봅니다. 키 자동반복 간격보다 커야 합니다.

`replay` / `rollout` / `evaluate` 는 연속 주행키가 필요 없으므로 항상 터미널 리스너만 씁니다.

---

## 4. 스크립트별 설명

### 4.1 [lekiwi-teleoperate.py](lekiwi-teleoperate.py) — 조작만

녹화 없이 로봇을 움직여 보는 용도. 리더암 캘리브레이션 확인, 카메라 시야 잡기,
작업 공간 배치 확인에 씁니다.

```bash
python lekiwi-teleoperate.py --display_data=true
```

전부 명시하려면:

```bash
python lekiwi-teleoperate.py \
    --robot.remote_ip=192.168.0.201 \
    --robot.id=lekiwi01 \
    --teleop.port=/dev/so101-leader \
    --teleop.id=leader \
    --fps=30 \
    --teleop_time_s=60 \
    --display_data=true
```

**전용 인자**

| 인자 | 기본값 | 설명 |
| --- | --- | --- |
| `--teleop_time_s` | `None` | 이 시간(초) 뒤 자동 종료. `None` 이면 ESC/Ctrl+C 까지 |
| `--check_cameras` | `true` | 시작 시 실제로 오는 카메라 이름/shape 을 로그로 남김 (실패해도 계속 진행) |

**조작키**

```
리더암      : 그대로 따라 움직인다
베이스 주행 : W/S 전후, A/D 좌우, Z/X 회전, R/F 속도 단계, SPACE 정지
종료        : ESC (Ctrl+C 도 됨)
```

종료 시 베이스를 정지시키고, 팔은 **현재 자세를 그대로 목표로 줘서 튀지 않게** 한 뒤
연결을 끊습니다. R/F(속도 단계) 키는 매 프레임 반복 적용되면 순식간에 최대/최소로 튀기 때문에
0.4초 간격으로 throttle 합니다.

---

### 4.2 [lekiwi-record.py](lekiwi-record.py) — 데이터셋 녹화

조작하면서 LeRobotDataset 을 만듭니다. 녹화 루프 자체는 lerobot 의 `record_loop` 를
그대로 쓰므로 데이터 포맷은 표준과 동일합니다.

```bash
HF_USER=your_hf_id
TASK_NAME=lekiwi_pick_cube

python lekiwi-record.py \
    --dataset.repo_id=${HF_USER}/${TASK_NAME} \
    --dataset.single_task="Pick up the cube and place it in the box" \
    --dataset.num_episodes=10 \
    --dataset.episode_time_s=15 \
    --dataset.reset_time_s=3 \
    --display_data=true \
    --dataset.push_to_hub=false
```

`--dataset.repo_id` 에 `/` 가 없으면 `$HF_USER`(없으면 로그인 계정명)를 앞에 붙여 줍니다.

**`--dataset.*` — lerobot `DatasetRecordConfig` 와 동일, 기본값만 LeKiwi 에 맞춤**

| 인자 | 기본값 | 설명 |
| --- | --- | --- |
| `--dataset.repo_id` | (필수) | `계정/데이터셋이름` |
| `--dataset.single_task` | (필수) | 태스크 설명 문장 |
| `--dataset.root` | `None` | 로컬 저장 경로. 기본은 `$HF_LEROBOT_HOME/repo_id` |
| `--dataset.fps` | `30` | 녹화 주기 |
| `--dataset.num_episodes` | `10` | 녹화할 에피소드 수 |
| `--dataset.episode_time_s` | `30` | 에피소드 한 번의 길이(초) |
| `--dataset.reset_time_s` | `10` | 에피소드 사이 환경 리셋 시간(초, 녹화 안 됨) |
| `--dataset.push_to_hub` | `false` | 끝나고 허브 업로드 (lerobot 기본은 true 이지만 여기선 로컬이 기본) |
| `--dataset.streaming_encoding` | `true` | 실시간 인코딩 → `save_episode()` 가 거의 즉시 끝남. 문제 생기면 `false` |
| `--dataset.encoder_threads` | `2` | 인코더 스레드 수 |
| `--dataset.video` | `true` | 프레임을 mp4 로 인코딩 |
| `--dataset.num_image_writer_threads_per_camera` | `4` | 카메라당 이미지 기록 스레드 |

**녹화 전용 인자**

| 인자 | 기본값 | 설명 |
| --- | --- | --- |
| `--resume` | `false` | 기존 데이터셋에 이어서 녹화 (로봇 호환성 검사 포함) |
| `--stamp_repo_id` | `false` | repo_id 뒤에 날짜/시간을 붙여 매 세션 새 이름 |
| `--check_cameras` | `true` | 첫 관측으로 카메라 이름/해상도 검사 |
| `--auto_camera_shape` | `true` | 실제 프레임에 맞춰 설정 자동 보정. `false` 면 불일치 시 에러 |

**조작키**

```
리더암      : 그대로 따라 움직인다
베이스 주행 : W/S 전후, A/D 좌우, Z/X 회전, R/F 속도 단계, SPACE 정지
녹화 제어   : →(또는 N) 현재 에피소드 종료, ← 재녹화, ESC(또는 Q) 녹화 중단
```

> 베이스 주행의 R/F 가 lerobot 기본 단축키(r=재녹화, q=중단)와 겹치기 때문에,
> **재녹화는 ← 키에만** 할당했습니다.

**동작 순서**

1. 데이터셋 폴더를 만들기 **전에** 먼저 로봇에 연결합니다 — 연결 실패 시 빈 데이터셋이
   남아 다음 실행에서 "이미 존재하는 repo_id" 로 막히는 걸 막기 위함입니다.
2. 카메라 shape 을 확정한 뒤에 데이터셋 feature 를 만듭니다.
3. 에피소드 녹화 → (마지막이 아니면) 리셋 구간 → `save_episode()` 반복.
4. 끝나면 `finalize()`, 필요 시 허브 업로드, 데이터셋 위치를 로그로 남깁니다.

---

### 4.3 [lekiwi-replay.py](lekiwi-replay.py) — 에피소드 재생

녹화된 action 을 그대로 다시 실행합니다. **리더암이 필요 없습니다.**
녹화가 제대로 됐는지, 로봇이 그 궤적을 실제로 따라갈 수 있는지 확인하는 용도입니다.

```bash
python lekiwi-replay.py \
    --dataset.repo_id=${HF_USER}/${TASK_NAME} \
    --dataset.episode=0
```

**전용 인자**

| 인자 | 기본값 | 설명 |
| --- | --- | --- |
| `--dataset.repo_id` | (필수) | 재생할 데이터셋 |
| `--dataset.episode` | `0` | 재생할 에피소드 번호 |
| `--dataset.root` | `None` | 로컬 데이터셋 경로 |
| `--dataset.fps` | `None` | 재생 주기. `None` 이면 데이터셋에 기록된 fps 사용 (권장) |
| `--warmup_s` | `2.0` | 현재 자세 → 첫 프레임 자세로 선형 보간 이동하는 시간 |
| `--countdown_s` | `2.0` | 워밍업 후 재생 시작까지 대기 (로봇에서 손 뗄 여유) |

**조작키**: `ESC` 즉시 중단 (Ctrl+C 도 됨)

**안전**

- 리플레이는 녹화 당시의 **절대 관절 각도와 베이스 속도**를 그대로 재생합니다.
  주변 환경이 녹화 때와 다르면 그대로 충돌합니다. 반드시 주변을 비우고 실행하세요.
- 시작 시 팔이 첫 프레임 자세로 튀지 않도록 `--warmup_s` 동안 선형 보간으로 이동합니다
  (이 구간에서 베이스는 정지).
- 데이터셋에 팔 관절(`.pos`)이 하나라도 빠져 있으면 **에러로 멈춥니다** (0도로 보내는 건 위험).
  베이스 속도(`.vel`)가 빠져 있으면 0(정지)으로 채우고 경고만 남깁니다.

---

### 4.4 [lekiwi-rollout.py](lekiwi-rollout.py) — 정책 롤아웃

학습한 정책으로 로봇을 자율 실행합니다. 팔 6채널(`.pos`) + 베이스 3채널(`.vel`) =
**9차원 action 을 그대로 유지**하는 추론 루프를 직접 돕니다.

```bash
python lekiwi-rollout.py \
    --policy.path=outputs/train/${TASK_NAME}/checkpoints/last/pretrained_model \
    --task="Pick up the cube and place it in the box" \
    --duration_s=60 \
    --display_data=true
```

허브에 올린 정책도 같은 방식:

```bash
python lekiwi-rollout.py \
    --policy.path=${HF_USER}/${TASK_NAME}_act \
    --task="Pick up the cube and place it in the box"
```

처음 돌려보는 정책이라면 베이스를 천천히:

```bash
python lekiwi-rollout.py --policy.path=... --task="..." \
    --base_speed_scale=0.3 --duration_s=30
```

**전용 인자**

| 인자 | 기본값 | 설명 |
| --- | --- | --- |
| `--policy.path` | (필수) | 로컬 체크포인트 경로 또는 허브 repo_id |
| `--policy.<필드>` | — | 체크포인트 설정 덮어쓰기 (예: `--policy.n_action_steps=50`) |
| `--task` | `""` | 정책에 넘길 태스크 문자열. 언어 조건부 정책(SmolVLA, pi0 등)에 필수, ACT/Diffusion 은 무시 |
| `--duration_s` | `None` | 이 시간(초) 뒤 자동 종료 |
| `--device` | `None` | 추론 장치. `None` 이면 체크포인트 설정 → 자동 선택 |
| `--warmup_s` | `2.0` | 첫 목표 자세까지 선형 보간 이동 시간 |
| `--base_speed_scale` | `1.0` | 베이스 속도에 곱할 계수. 처음엔 `0.3` 권장 |
| `--rename_map` | `{}` | 관측 키 이름 변환 |

카메라 이름이 학습 데이터셋과 다르면:

```bash
--rename_map='{"observation.images.front": "observation.images.cam_high"}'
```

**조작키**

```
SPACE : 일시정지 / 재개 (정지 중에는 팔을 현재 자세로 고정하고 베이스를 멈춘다)
ESC   : 종료 (Ctrl+C 도 됨)
```

재개할 때는 정지 전에 만들어 둔 액션 청크를 버리고(`policy.reset()`) 지금 관측부터 다시 시작합니다.

**사전 검사** — 로봇을 움직이기 전에 다음을 확인하고, 안 맞으면 멈춥니다.

- 정책의 action 차원 vs LeKiwi 의 9채널 (6차원이면 팔만 학습한 정책 → 다시 학습 필요)
- 정책의 `observation.state` 차원
- 정책이 기대하는 카메라 키 vs 로봇이 주는 카메라 키 (`--rename_map` 이 있으면 건너뜀)

---

### 4.5 [lekiwi-evaluate.py](lekiwi-evaluate.py) — 정책 평가

에피소드/리셋 구간을 나눠 N 번 돌리고, 사람이 매번 성공/실패를 눌러 주면
**성공률과 추론 지연 통계를 JSON 리포트**로 남깁니다.

| | rollout | evaluate |
| --- | --- | --- |
| 실행 | 계속 돌린다 | 에피소드 N 번 |
| 판정 | 사람이 눈으로 | S/F 키로 기록 |
| 결과물 | 없음 | 표 + JSON 리포트 |

```bash
python lekiwi-evaluate.py \
    --policy.path=outputs/train/${TASK_NAME}/checkpoints/last/pretrained_model \
    --task="Pick up the cube and place it in the box" \
    --eval.n_episodes=10 \
    --eval.episode_time_s=30 \
    --eval.reset_time_s=10
```

두 체크포인트를 비교하려면 같은 조건으로 두 번 돌리고 리포트를 비교합니다:

```bash
python lekiwi-evaluate.py --policy.path=.../040000/pretrained_model --eval.tag=step040k ...
python lekiwi-evaluate.py --policy.path=.../080000/pretrained_model --eval.tag=step080k ...
```

**`--eval.*` 인자**

| 인자 | 기본값 | 설명 |
| --- | --- | --- |
| `--eval.n_episodes` | `10` | 평가할 에피소드 수 |
| `--eval.episode_time_s` | `30` | 에피소드 최대 길이(초). 지나면 판정을 물어봄 |
| `--eval.reset_time_s` | `10` | 에피소드 사이 환경 리셋 시간(초) |
| `--eval.output_dir` | `None` | 리포트 저장 폴더. 기본 `outputs/eval` |
| `--eval.tag` | `""` | 리포트 파일 이름 꼬리표 (체크포인트 비교용) |
| `--eval.drop_unjudged` | `false` | 판정 없이 끝난 에피소드를 제외할지. `false` 면 실패로 셈 |

`--policy.path` / `--task` / `--fps` / `--device` / `--warmup_s` / `--base_speed_scale` /
`--rename_map` 은 rollout 과 동일합니다.

**조작키**

```
에피소드 진행 중 : S 성공으로 종료, F 실패로 종료, →(N) 판정 보류하고 종료,
                   SPACE 일시정지/재개, ESC 평가 전체 중단
판정 대기 중     : S 성공, F 실패, ESC 중단
리셋 구간        : →(N) 리셋 끝내고 다음 에피소드로, ESC 중단
```

리셋 구간에는 로봇이 현재 자세를 유지한 채 멈춰 있으므로, 그동안 물체를 제자리에
돌려놓으면 됩니다. 판정을 보류한 채 끝난 에피소드는 바로 뒤에서 S/F 를 물어봅니다.
일시정지 중에는 에피소드 시간이 흐르지 않으므로, 손댄 시간이 에피소드 길이를 깎지 않습니다.

**리포트**

`outputs/eval/lekiwi_eval_YYYYmmdd_HHMMSS[_tag].json` 에 저장되며, 터미널에도 표로 출력됩니다.

```
  ep      판정   길이(s)     스텝   추론(ms)   루프(Hz)
   1      성공      12.4      372       18.2       29.8
   2      실패      30.0      900       17.9       29.9
...
성공률: 7/10 = 70.0%
평균 추론 18.1 ms | 평균 루프 29.8 Hz
```

JSON 에는 `policy_path`, `policy_type`, `task`, `tag`, `robot_id`, `fps`, `device`,
`base_speed_scale`, `eval` 설정 전체, 시작/종료 시각, `summary`(성공률·평균 지연),
`episodes`(에피소드별 판정·길이·스텝·추론 mean/p95·루프 Hz)가 들어갑니다.

---

## 5. 전체 워크플로

```
0) 설치/캘리브레이션  (1장)  conda create -n lerobot python=3.12 -y / lerobot-calibrate ...
0') 호스트 실행   (라즈베리파이) python -m lerobot.robots.lekiwi.lekiwi_host --robot.id=lekiwi01 --robot.cameras='{...}'
1) 조작 확인      python lekiwi-teleoperate.py
2) 데이터 녹화    python lekiwi-record.py --dataset.repo_id=... --dataset.single_task="..."
3) 녹화 검증      python lekiwi-replay.py --dataset.repo_id=... --dataset.episode=0
4) 학습           lerobot-train --dataset.repo_id=... --policy.type=act \
                      --output_dir=outputs/train/<task> --job_name=<task> --policy.device=cuda
5) 정책 확인      python lekiwi-rollout.py  --policy.path=outputs/train/<task>/checkpoints/last/pretrained_model --task="..."
6) 정책 평가      python lekiwi-evaluate.py --policy.path=... --task="..." --eval.n_episodes=10
```

4번 학습은 lerobot 이 제공하는 CLI 를 그대로 씁니다 (이 저장소에 스크립트 없음).

---

## 6. 공통 설계

모든 스크립트가 같은 방식으로 동작합니다.

- **설정은 전부 CLI 인자.** `lerobot.configs.parser.wrap()` + dataclass 조합이라
  코드에 하드코딩된 상수가 없습니다. `--help` 로 전체 목록을 볼 수 있습니다.
- **연결 실패는 친절한 에러로.** 호스트에 못 붙으면 `ping` / 포트 / 방화벽 체크리스트를 출력합니다.
- **카메라 동기화.** 클라이언트가 선언한 shape 과 호스트가 실제로 보내는 프레임을 맞춥니다
  (`sync_cameras`). 보정하면 `observation_features` 의 cached_property 캐시를 비웁니다.
- **부드러운 시작.** replay/rollout/evaluate 는 첫 목표 자세까지 `--warmup_s` 동안 선형 보간으로
  이동합니다. 없으면 첫 `send_action` 에서 팔이 최대 속도로 튑니다.
- **안전한 종료.** 어떤 경로로 끝나든(`finally`) 베이스 속도를 0 으로 보내고, 팔은 현재 자세를
  그대로 목표로 줘서 튀지 않게 한 뒤 연결을 끊습니다.
- **베이스 채널 보강.** 호스트의 `send_action` 은 `x.vel`/`y.vel`/`theta.vel` 이 반드시 있다고
  가정하므로, 데이터셋이나 정책에 베이스 채널이 없어도 0 을 깔아 KeyError 를 막습니다.

---

## 7. 문제 해결

| 증상 | 확인할 것 |
| --- | --- |
| 호스트 접속 실패 | 라즈베리파이에서 `lekiwi_host` 실행 중인지, `ping <ip>`, 포트 5555/5556, 방화벽, `--host.connection_time_s` 만료 여부 |
| 카메라 프레임이 안 옴 | 호스트와 클라이언트의 `--robot.cameras` **이름**이 같은지 (`front`/`wrist`) |
| 카메라 해상도 불일치 경고 | 호스트의 rotation 설정 때문에 가로/세로가 뒤집힌 경우가 대부분. 학습/녹화 때와 같은 해상도인지 확인 |
| 호스트 카메라 설정 실패 | rotation 90/270 이면 `width`/`height` 를 뒤집어 적어야 함 |
| 키가 안 먹음 (Wayland) | `--base_control=terminal` (기본 `auto` 가 이미 선택), **터미널 창에 포커스** 필요 |
| 베이스 속도가 순식간에 최대로 | R/F throttle 이 0.4초 간격. 그래도 빠르면 키 자동반복 속도 확인 |
| 재녹화가 안 됨 | `r` 이 아니라 **←** 키 (r/f 는 베이스 속도 단계) |
| 임포트 에러 (`datasets`) | `dataset` extra 없이 설치된 환경. `pip install -e ".[core_scripts,lekiwi]"` |
| 임포트 에러 (`zmq`, `scservo_sdk`) | `lekiwi` extra 누락. `pip install -e ".[lekiwi]"` |
| 영상 인코딩/디코딩 실패 | `conda install ffmpeg -c conda-forge` (안 되면 `ffmpeg=7.1.1`) |
| 리더암 포트가 매번 바뀜 | udev 규칙으로 `/dev/so101-leader` 고정 (1.5절) |
| 캘리브레이션 파일 못 찾음 | `--robot.id` / `--teleop.id` 가 `~/.cache/huggingface/lerobot/calibration/` 의 파일 이름과 같은지 |
| 정책 action 차원 6 vs 9 | 팔만 학습한 정책. LeKiwi 데이터셋(팔 6 + 베이스 3)으로 다시 학습 |
| 정책 카메라 키 불일치 | `--rename_map='{"observation.images.front": "observation.images.cam_high"}'` |
| `이미 존재하는 repo_id` | 다른 이름을 쓰거나 `--resume=true`, 또는 `--stamp_repo_id=true` |
