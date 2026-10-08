# 교육용 모델 가져오기 (2026-09-18)

- 출처: `lekiwi_classroom_members/isaac_sim/assets/lekiwi_soarm`
- 출처 프로젝트 커밋: `b4688d238448d86f4ac5a202e0ee8bfa6a3df006`
- 복사 범위: STL 15개, ROS용 Xacro 6개, 원본 `SOURCE.md`.
- 원본 프로젝트는 수정하지 않았다. 파일별 SHA-256과 경로는 `import_manifest.json`에 기록했다.
- 형상, 축 방향, 팔 장착 위치, 재질은 교육용 ROS Xacro와 동일하다.
  기본 팔 자세도 교육용 `teleop_bridge.py`와 같이 wrist_roll=-90도, 나머지=0도다.

현재 패키지의 진입점은 `../urdf/lekiwi.urdf.xacro`다. 이 파일에서 mesh 경로를
`package://lekiwi_description/meshes/...`으로 지정하고 아래의 카메라를 추가한다.
복사한 Xacro와 STL은 원본 바이트를 유지한다. 원본 프로젝트 설치나 경로에 의존하지 않는다.

Isaac 전용 생성 URDF/USD는 가져오지 않았다. 해당 파일에 추가된 질량·관성·36개
수동 롤러 충돌 링크는 물리 시뮬레이션용이며, 몸체·바퀴·팔의 시각 mesh는 같다.
ROS 원본의 `base_footprint` (base_link 아래 0.075 m)는 유지한다.

`display.launch.py`는 이 교육용 모델을 기본으로 표시하며 가상 joint state만 발행한다.
팔 제어, ros2_control, MoveIt, 모터 드라이버, 센서 드라이버를 시작하지 않는다.
RViz를 닫으면 모델 표시용 publisher도 종료한다.

실물 bringup도 같은 최신 모델을 사용한다. `use_hardware_sensors:=true`로 기존 lidar/imu
장착 좌표를 새 기준에 맞춰 추가한다. 주행·센서 좌표 변환은 [FRAME_MIGRATION.md](FRAME_MIGRATION.md)에 기록했다.
실물에서 wheel ID·주행·엔코더·스캔 방향은 별도 재확인이 필요하다.

## 카메라 좌표계 추가 (2026-09-18)

같은 교육용 프로젝트의 `isaac_sim/assets/cameras/mounts.json`을 이 디렉터리의
`../config/camera_mounts.json`으로 그대로 복사했다. SHA-256은 다음과 같다.

```text
dcb8efa8e325abb8ce1f4efb38ca839205a1e3831cd05061ee997a57c26a99fb  camera_mounts.json
```

현재 프로젝트에서 작성한 `../urdf/cameras.xacro`가 이 설정을 읽어 optical frame과
고정 관절을 만든다. 위치는 렌즈 중심이며 단위는 m다.

- `base_link → front_camera_optical_frame`
- `wrist_link → wrist_camera_optical_frame`

방향 quaternion(x,y,z,w)은 같은 회전을 나타내는 URDF RPY로 변환한다.
광학 좌표축은 +X 오른쪽, +Y 아래, +Z 촬영 정면이다. Isaac의 USD Camera 전용
추가 X축 180도 회전은 ROS optical frame에 적용하지 않는다.

카메라의 상자·렌즈 외형은 교육용 `isaac_sim/robot_cameras.py`의 크기·offset·색상을
현재 Xacro에 옮겼다. 참조 코드 SHA-256:
`15cd9d7f1f69cf0704de05f545b0276e642b46ce14211dcd337e03ad2e9e8013`.
물리 충돌, 센서 드라이버, 영상 발행은 추가하지 않았다.

원본의 `calibrated: false`를 그대로 유지한다. 도면 재현 자세와 손목 카메라의
wrist_roll 이전 부모(`wrist_link`) 역시 원본 설정이며 실물 보정 완료를 뜻하지 않는다.
`base_footprint` 높이와 몸체·바퀴·팔의 기존 연결은 변경하지 않는다.

추가 후 검증:

- description build 및 기존 등록 CTest 2개 통과.
- 두 카메라의 원본 quaternion과 URDF RPY 회전 행렬이 일치한다
  (최대 성분 오차 약 `6.93e-12`).
- 원본 `measurement.joint_positions_deg` 자세에서 SOARM base 기준 렌즈 위치가
  front `(0.08691, 0, -0.05928)` m, wrist `(0.20540, 0, 0.20858)` m와 일치한다.
  촬영 정면도 각각 +X, 전방 아래 45도로 일치한다.
- 오프라인 관절 변환 검사에서 어깨 각도를 바꾸면 손목 카메라는 팔을 따라 움직이고
  전방 카메라의 base 기준 위치는 유지된다. 실물 관절은 움직이지 않았다.
- 실행 중인 미리보기에서 18개 link, 17개 TF 연결 및 두 카메라의 부모·위치·방향을 확인했다.
  기존 RViz 창을 유지한 채 robot description을 갱신했고 `Global Status: Ok`를 확인했다.
- 카메라를 제외한 URDF 내용과 기존 22개 복사 파일은 변경되지 않았다.
  원본 프로젝트 Git 상태와 카메라 설정·참조 코드 해시도 작업 전후 동일하다.
- 실물 카메라 위치 보정과 영상 수신 검증은 수행하지 않았다.

## 최초 모델 가져오기 검증 (카메라 추가 전)

- `colcon build --symlink-install --packages-select lekiwi_description`: 성공.
- `colcon test --packages-select lekiwi_description`: 기존 등록 CTest 2개 통과.
  README에 기록된 기존 pytest 함수 수집 누락은 이 결과와 별개다.
- `lekiwi_bringup/test/test_launch_contracts.py`: 20개 통과.
- 복사한 22개 파일의 해시와 원본 프로젝트 Git 상태가 작업 전후 동일하다.
- 생성 URDF: 16개 link, 15개 joint, 38개 mesh 참조 모두 현재 패키지에서 해석된다.
  패키지 경로와 최상위 robot 이름을 맞추면 교육용 원본 Xacro 출력과 동일하다.
- 로컬 ROS domain 83에서 robot description, 9개 가상 관절 값, 전체 TF 연결을 확인했다.
  `/cmd_vel` publisher는 0개이며 하드웨어 노드는 실행하지 않았다.
- 실제 RViz 창에서 몸체·바퀴 3개·팔 표시 및 `Global Status: Ok`를 확인했다.
- 실제 장비 주행 및 센서 TF 검증은 수행하지 않았다.
