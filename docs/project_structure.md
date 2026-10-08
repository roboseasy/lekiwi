# 현재 폴더 구조

ROS 실행 코드는 8개 패키지에 두고, 개발 진단·통합 검사·모델 내보내기를 분리한다.
모델의 기준 소스는 `lekiwi_description`이다. ZIP은 여기서 생성하는 배포 산출물이다.

```text
lekiwi/
├── README.md, LICENSE, THIRD_PARTY_NOTICES.md  고객 안내·배포 권리
├── .github/workflows/ros2-jazzy.yml  새 amd64·ARM64 환경 설치 및 통합 검사
├── lekiwi.repos               고정 버전 YDLIDAR SDK·드라이버
├── lekiwi/                    ROS 메타 패키지
├── lekiwi_description/
│   ├── urdf/lekiwi.urdf.xacro 기본 모델 진입점
│   ├── urdf/base_and_arm.urdf.xacro  교육용 몸체·팔 복사본
│   ├── urdf/so101/            교육용 팔 매크로 복사본
│   ├── urdf/cameras.xacro     전방·손목 카메라 TF와 외형
│   ├── urdf/hardware_sensors.xacro  실물 센서 좌표 변환
│   ├── config/               카메라 장착값, 주행 좌표계 변환
│   ├── meshes/base/          현재 몸체와 바퀴 STL 2개
│   ├── meshes/soarm/         SO101 STL 13개
│   ├── launch/display.launch.py
│   ├── rviz/                 모델 미리보기·주행 표시 설정
│   ├── docs/                 출처·해시·좌표계 변경 설명
│   └── test/                 모델 및 좌표 변환 검사
├── lekiwi_bringup/
│   ├── launch/, param/        base·센서 실행, 장비별 설정 예시
│   ├── lekiwi_bringup/base_arguments.py  공통 실행 인자·기본값·전달
│   ├── tools/diagnostics/     선택 설치하는 개발용 토픽·서비스 진단
│   └── test/                  실행·설치 계약 검사, diagnostics/ 회귀 테스트
├── lekiwi_node/               모터 명령, 엔코더 odom, 안전 처리
├── lekiwi_sensors/            라이다·USB 카메라·BMI160 드라이버와 필터
├── lekiwi_cartographer/       Cartographer 2D 지도 작성
├── lekiwi_navigation2/        Omni AMCL·MPPI·충돌 감시
├── lekiwi_teleop/             실물 키보드 조작·Nav2 모터 감독·거리 비교 시험
│   └── lekiwi_teleop/motor_session.py  공통 모터 서비스·준비 상태·세션 정리
├── tests/
│   ├── integration/          설치된 패키지의 모의 pub/sub·SLAM·Nav2 검사
│   │   └── process_cleanup.py  검사에서 만든 자식 프로세스만 종료
│   └── unit/                 통합 검사 종료 처리 회귀 검사
├── tools/export_model_bundle.py  모델 ZIP 생성 및 검증
├── exports/                  최신 모델 ZIP
├── docs/                     하드웨어 절차·검증 기록
└── build/, install/, log/    로컬 빌드 산출물
```

`lekiwi_description/meshes/`의 STL 15개는 모두 현재 Xacro에서 참조된다.
PNG·JPEG 등 별도 이미지 파일은 없으며, `exports/`의 ZIP은 모델 배포본이다.
과거 SLAM·Nav2 구현과 개발 계획 초안은 Git 이력에 보존하고 현재 소스 트리에서는 제외한다.
별도 worktree는 각각 독립된 Git 작업 공간이다.

## 기본 설치와 개발 도구

기본 설치는 주행·센서·모델·SLAM·Nav2와 모터 안전 처리를 포함한다.
패키지의 `test/`와 루트 `tests/`, `tools/`는 고객 실행 경로에 설치하지 않는다.
회귀 테스트는 유지하며, CMake 패키지는 `BUILD_TESTING=OFF`로 검사 대상 생성을 생략할 수 있다.

`lekiwi_bringup/tools/diagnostics/`는 기본값 `LEKIWI_INSTALL_DIAGNOSTICS=OFF`로 제외한다.
개발자가 ON으로 빌드하면 검사 서버·worker·일괄 실행기와 진단 launch만 추가 설치한다.
설치와 실행은 [개발용 진단 안내](hardware/topic_tests.md)를 따른다.
진단 코드를 bringup 패키지 안에 두어 설치 시 저장소 밖의 파일을 참조하지 않도록 했다.

기존 Conda 개발 설정은 제거했다. 지원하는 설치 경로는 README의
Ubuntu 24.04 / ROS 2 Jazzy와 시스템 Python이다.

## 모델 실행과 내보내기

빌드한 workspace를 source한 PC에서:

```bash
ROS_DOMAIN_ID=83 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
  ros2 launch lekiwi_description display.launch.py
```

저장소 루트에서 최신 ZIP 생성:

```bash
python3 tools/export_model_bundle.py
```

출력은 `exports/lekiwi_urdf_bundle.zip`이다. 압축 검증 후 같은 파일을 교체한다.
실물 센서 프레임도 포함한 URDF를 내보내려면 `--hardware-sensors`를 명시한다.
ZIP은 ROS/RViz 모델의 portable URDF이며 Isaac 물리 주행 검증을 뜻하지 않는다.

실물 주행 설정의 좌표계 변경 근거는
[FRAME_MIGRATION.md](../lekiwi_description/docs/FRAME_MIGRATION.md)를 참고한다.

## 2026-10-01 배포 전 구조 점검

- 활성 ROS 패키지는 8개다. 루트 `scripts/`의 개발 도구를 용도별로 옮겼으며,
  통합 검사와 패키지 회귀 테스트의 CI 경로도 함께 변경했다.
- bringup 설치 검사는 개발 도구 없는 패키지의 기본 설치와 진단 선택 설치를
  각각 임시 폴더에서 확인한다. 설치 대상의 Python cache 제외도 검사한다.
- 공통 launch 인자와 전달 코드는 bringup의 Python 모듈로 기본 설치한다.
  launch 검사는 pytest로 자동 발견하며, 실행 인자와 장비별 보정값 우선순위를 확인한다.
- teleop·Nav2 감독·거리 측정은 공통 모터 연결 모듈을 사용한다.
  각 도구의 준비 상태 확인과 정지 순서는 유지한다.
- USB 카메라 프로파일은 `lekiwi_sensors/launch/cameras.launch.py` 한 곳에 있다.
  일괄 검사 launch는 이를 include한다.
- STL 15개는 모두 모델에서 사용되고 동일 바이트 중복이 없다. 배포용 ZIP은
  재생성 가능한 산출물이며 현재 Xacro·설정·출처 문서를 포함한다.
  좌표계 비교 fixture와 회귀 테스트는 활성 기능 검증에 사용한다.
- `build/`, `install/`, `log/`, 로컬 오류 로그와 Python cache는 Git 추적·배포 소스에서 제외한다.
  ROS 패키지의 디렉터리 설치 규칙도 bytecode를 제외한다.
- 현재 사용하지 않는 과거 패키지 사본과 설계 초안은 Git 이력에서 확인한다.
  별도 worktree와 현장 검사 기록은 현재 소스 정리 대상에 포함하지 않는다.

검사 결과는 서버의 `~/.ros/lekiwi-tests/`에 작업별로 저장한다. 프로젝트 소스 폴더에
결과 JSON·카메라 프레임·현장 로그를 누적하지 않는다.

루트의 SLAM·Nav2 패키지 출처·변경값·실행 순서는 [이식 기록](hardware/slam_navigation.md)을 따른다.
