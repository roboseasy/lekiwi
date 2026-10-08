# 하부 역방향 라이다 장착과 SLAM 후속 개발 메모

기록일: 2026-09-12

2026-09-12 상태: **후속 개발 대기**. 사용자는 당시 검토 내용을 추후 개발 항목으로 보관하도록 요청했다. 이 메모는 실물 동작 승인을 의미하지 않는다.

## 2026-09-22 정지 스캔 방향 확인

- 로봇을 바닥에 놓고 주변 시야를 확보한 상태에서 `/scan_raw`와 `/scan`의 약 10Hz 수신을 확인했다.
- 사용자 제공 SOARM base 기준 위치 `(-20.67, 0, -102.50)` mm와 뒤집힌 장착 방향에 따라
  라이다 TF를 `soarm_base_link` 아래 XYZ `(-0.02067, 0, -0.10250)` m, RPY `(π, 0, 0)`으로 수정했다.
  라이다에 적용돼 있던 약 120° yaw는 제거했으며 모터 계산의 좌표 변환은 유지했다.
- 설치된 SDK 코드를 대조한 뒤 `reversion=false`, 기존 `inverted=true`로 실행했다.
  두 옵션은 각각 추가 180° 회전과 스캔 각도 부호 반전이다. TF와 함께 적용한 최종 화면에서
  사용자가 실제 물체의 앞뒤·좌우와 RViz 점 위치가 일치함을 확인했고 이 조합을 기본 설정에 저장했다.
- 비교 도중 로봇 또는 물체 위치가 바뀌었으므로 전후 스캔의 수치 차이를 정밀 보정 근거로 쓰지 않는다.
  이번 확인은 정지 상태의 방향과 수신 확인이다. 거리 정확도, 시야 가림, 이동 중 시간 대응과 SLAM 품질은 별도 검증 대상이다.
- 최신 TF 상세는 [좌표계 변경 기록](../../lekiwi_description/docs/FRAME_MIGRATION.md)을 참고한다.

## 2026-09-29 저장된 ROS bag의 시간 정보 확인

2026-09-23에 PC에서 저장한 `~/.ros/lekiwi-tests/map-acquisition-20260923-01/rosbag`을
읽기 전용으로 분석했다. `/scan_raw` 2,886개와 `/scan` 2,885개는 모두 430개 거리값을
포함했다. `/scan_raw`의 `time_increment`는 0보다 컸지만 정규화된 `/scan`은 모두 0이었다.
따라서 현재 SLAM 입력에는 측정점별 시간 간격이 전달되지 않는다.

bag 기록 시각에서 메시지 header 시각을 뺀 값은 `/scan_raw`에서 중앙값 0.110초,
95백분위 0.231초, 최대 0.981초였고, `/odom`에서는 중앙값 0.007초,
95백분위 0.104초, 최대 0.866초였다. 가장 큰 지연은 두 토픽 모두 14:11:47~48경에
나타났다. 이 차이에는 Pi·PC 시계 오차, 전송, 수신 및 기록 지연이 함께 들어갈 수 있어
Wi-Fi나 SDK를 단독 원인으로 지목할 수 없다. 반전 후 측정점 순서와 실제 취득 시각,
이동 중 지도 왜곡의 인과 관계는 아직 실측하지 않았다.

아래는 2026-09-12 당시의 검토 기록이며 현재 기본 설정과 구분한다.

## 사용자에게 확인한 현상

- YDLIDAR Tmini Plus를 기존 하부 위치에 거꾸로 장착했다.
- 세 바퀴 부근에서 라이다 시야가 많이 가려진다.
- 이동·회전하면 벽이 겹치거나 지도가 틀어진다.
- 사용자는 뒤집힌 장착에 따른 좌우 반전을 의심한다. 최종 ROS 좌표에서의 반전 여부는 아직 실측하지 않았다.

## 검토 기준과 확인한 사실

- 현재 프로젝트: `feature/lekiwi-ros2`, commit `57c6716`.
- 참조 저장소: `adityakamath/lekiwi_ros2`, commit `301f98b6e121477c5b7bdb5000d45e8f878beadf`.
- 검토 중 코드·설정·하드웨어 상태는 변경하지 않았다. Pi의 실행 설정과 설치된 SDK 버전은 미확인이다.
- [당시 URDF 스냅샷](../../lekiwi_description/test/fixtures/legacy_base.urdf)은 `lidar_joint`에 `roll=π`를 적용한다. 센서 좌표의 `(x, y, z)`는 회전에 의해 `(x, -y, -z)`가 된다. 2026-09-18 이후 모델과의 변환은 [좌표계 변경 기록](../../lekiwi_description/docs/FRAME_MIGRATION.md)을 참고한다.
- 당시 YDLIDAR 설정은 `inverted: true`, `reversion: true`, `ignore_array: ""`였다. 공개 SDK에서 두 옵션은 각각 각도 부호 반전과 180도 회전에 해당한다. 센서 원래 각도 규약까지 확인하기 전에는 이 조합을 이중 반전 오류로 단정하지 않는다. [현재 설정](../../lekiwi_sensors/param/ydlidar.yaml)은 위 2026-09-22 확인 결과를 반영한다.
- URDF의 `base_footprint → base_link` 높이 `0.055m`와 `base_link → lidar_link` 높이 `-0.053m`를 합치면 스캔 좌표는 바닥 기준 `0.002m`다. 실제 레이저 출사 높이와 일치하는지 미확인이다. 시각용 원통의 offset은 스캔 좌표를 바꾸지 않는다.
- [스캔 시간 처리](../../lekiwi_sensors/scripts/scan_timing_normalizer)는 기본적으로 `time_increment=0`을 적용한다. 측정점별 시간 차이를 제거하므로 이동 중 왜곡의 점검 대상이다. 이것이 현장 실패 원인이라는 실측 증거는 아직 없다.
- 당시 Cartographer 기본·IMU 설정은 모두 `min_range=0.30m`였다. 모든 방향의 가까운 점을 버리는 처리이며 바퀴만 식별하는 필터는 아니다.
- 당시 Nav2 설정은 같은 `/scan`을 사용하지만 장애물 최소 거리는 `0.12m`였다. 두 처리 경로에서 자체 반사가 다르게 취급될 수 있다. 과거 설정 원문은 Git 커밋 `da1910e`의 `archive/ros_packages/`에서 확인한다.
- 당시 Cartographer의 기본 launch는 IMU 미사용 설정을 선택했다. 현재 표준 launch는 IMU 설정을 사용하며, 실제 지도 취득에 IMU가 사용됐는지는 실행 로그로 확인한다.

## 레퍼런스에서 참고할 부분

1. 작성자는 LD19를 35mm 스페이서 네 개로 Raspberry Pi 위에 장착했다. 하부 바퀴 사이에 장착한 현재 구조와 시야 조건이 다르다.
2. 이후 팬틸트 카메라가 가리는 약 70도 구간을 필터링하고 `/scan_raw → laser_filters → /scan` 구조로 SLAM/Nav2에 전달했다.
3. 검토한 commit은 `LaserScanAngularBoundsFilter`로 `0.56~5.72rad` 구간을 유지한다. 드라이버는 `0~2π` 배열을 사용한다. 현재 YDLIDAR의 `-π~π` 범위에 숫자를 그대로 복사하지 않는다. 블로그의 InPlace 예제와 GitHub의 현재 필터 종류도 다르다.
4. 바퀴의 전후·좌우 속도와 BNO055의 자세·각속도를 `robot_localization` EKF로 결합한다. 현재 commit은 50Hz, `two_d_mode: true`이며 EKF가 odometry TF를 발행한다.
5. 지도 작성은 비동기 `slam_toolbox`, 최대 거리 8m, 지도 해상도 5cm, 처리 빈도 제한을 사용한다. 알고리즘 교체만으로 가림·좌표·시간 문제가 해결된다는 근거는 없다.
6. Gibbard 문서는 LD06 통신 파싱과 실시간 점 표시 예제다. LeKiwi SLAM 구현 사례가 아니며 Tmini Plus에 통신 파서를 그대로 사용할 수 없다.

## 후속 작업 우선순위

- [x] **정지 상태의 앞뒤·좌우 확인:** 2026-09-22 최종 TF·드라이버 조합을 적용한 RViz에서 사용자가 실제 물체와 방향 일치를 확인했다. 정밀 각도 오차 측정은 포함하지 않는다.
- [ ] **장착 위치 실측:** 광학 중심의 x/y/z, 수평도, 전방 기준과 뒤집힌 축을 측정한다. 높이 오차 자체를 평면 지도 겹침의 직접 원인으로 단정하지 않는다.
- [ ] **스캔 시간 검증:** header 시각, 실제 측정 순서, 각도별 배열 재배치, 반전 후 시간 대응, Pi-PC 시간 동기화, TF·odom·IMU 시각을 검증한다. 원본 시간 정보가 맞는지 확인하기 전에 `zero_time_increment`만 해제하지 않는다. TF 조회 성공과 측정 시각 정확성을 구분한다.
- [ ] **시야 개선:** 레이저 측정면을 바퀴·지지 구조물 위로 올리는 장착을 검토한다. 하부를 유지하면 세 가림 구간을 현재 좌표 기준으로 다시 측정한다. 과거 각도 기록을 확정값으로 재사용하지 않는다.
- [ ] **자체 가림 필터:** `/scan_raw`를 보존하고 각도 또는 각도+거리 조건으로 로봇 반사를 제외한다. 스캔 배열·각도·시간 대응을 유지하고, 제외점은 NaN 등 관측 불가로 처리한다. 가려진 뒤쪽을 빈 공간으로 지우거나 측정값을 임의 보간하지 않는다. Cartographer와 Nav2 양쪽에서 검증한다.
- [ ] **IMU 활용:** 기존 Cartographer IMU 설정으로 동일 기록 데이터를 비교한다. BMI160은 현재 자세 quaternion을 제공하지 않으므로 BNO055용 절대 방향각 EKF 설정을 그대로 복사하지 않는다. EKF를 추가하면 `odom → base_footprint` TF 발행자를 하나로 유지한다.
- [ ] **SLAM 비교:** 입력 좌표·시간·가림 처리를 정리한 뒤 동일한 기록으로 Cartographer와 `slam_toolbox`를 비교한다. 실물 이동·토크·런타임 실행은 별도 현장 안전 절차를 따른다.

## 오프라인 검증과 테스트 보완

2026-09-12 선택 실행한 테스트는 16개 중 15개 통과, 1개 실패였다.

- `lekiwi_description/test/test_base_urdf_layout.py`: pytest로 7개 실행, 6개 통과. `test_base_visual_front_aligns_with_ros_positive_x_axis`가 XML 속성 순서에 의존한 문자열 비교로 실패했다. XML 파싱으로 비교한 실제 xyz/rpy 값은 기대값과 일치했다.
- `lekiwi_sensors/test/test_ydlidar_params.py`, `test_scan_timing_normalizer.py`에서 `-k 'ydlidar or normalize_scan_timing'`: 5개 통과.
- `lekiwi_cartographer/test/test_cartographer_contract.py`에서 `-k 'lua or imu'`: 4개 통과.

description의 현재 CTest는 Python 파일을 직접 실행하며, 실패한 위 함수는 `__main__`에서 호출하지 않는다. 추후 XML 의미 기반 비교와 테스트 수집 경로를 함께 보완한다. 기존 테스트는 설정 계약을 확인할 뿐 실제 좌우 방향, 가림 처리의 타당성, 주행 중 지도 품질을 증명하지 않는다.

## 출처

- [참조 저장소 고정 commit](https://github.com/adityakamath/lekiwi_ros2/tree/301f98b6e121477c5b7bdb5000d45e8f878beadf)
- [작성자의 라이다 장착 설명](https://foxglove.dev/blog/upgrading-the-lekiwi-into-a-lidar-equipped-explorer)
- [작성자의 SLAM·자율주행 설명](https://kamathrobotics.com/autonomous-navigation-with-lekiwi)
- [참조 필터 설정](https://github.com/adityakamath/lekiwi_ros2/blob/301f98b6e121477c5b7bdb5000d45e8f878beadf/lekiwi_bringup/config/payloads/pantilt/laser_filter.yaml)
- [참조 EKF 설정](https://github.com/adityakamath/lekiwi_ros2/blob/301f98b6e121477c5b7bdb5000d45e8f878beadf/lekiwi_navigation/config/robot_localization/ekf.yaml)
- [Gibbard LD06 문서](https://www.gibbard.me/lidar/)
- [YDLIDAR SDK 각도 변환](https://github.com/YDLIDAR/YDLidar-SDK/blob/master/src/CYdLidar.cpp)
- [YDLIDAR ROS 2 드라이버](https://github.com/YDLIDAR/ydlidar_ros2_driver/blob/humble/src/ydlidar_ros2_driver_node.cpp)
- [Cartographer ROS 2 측정점 시간 변환](https://github.com/ros2/cartographer_ros/blob/ros2/cartographer_ros/src/msg_conversion.cpp)

고정 commit이 없는 외부 링크의 구현 설명은 2026-09-12 조회 기준이다. 실제 개발 시 설치 버전과 다시 대조한다.
