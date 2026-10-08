# 가져온 자료와 버전

## Bosch BMI160 SensorAPI

- 공식 소스: https://github.com/boschsensortec/BMI160_SensorAPI
- 고정 commit: `252ac2859ac1d2010915b7a7cfcbf501d7adfb40`
- 라이선스: BSD-3-Clause. [원문](vendor/bosch_bmi160/LICENSE)
- 범위: `bmi160.c`, `bmi160.h`, `bmi160_defs.h`, LICENSE, 원본 버전 기록을 포함합니다. 공식 소스는 수정하지 않았습니다.
- 최초 복사 출처: LeKiwi 저장소의 `lekiwi_sensors/vendor/bosch_bmi160`.
- 점검 도구 폴더만 복사해도 독립 실행하도록 실제 소스 파일을 포함합니다. ROS 센서 패키지의 경로를 런타임에 참조하지 않습니다.
- 단위 환산: ±2g는 16384LSB/g, ±250°/s는 131.2LSB/(°/s), 표준 중력 9.80665m/s²를 사용합니다.

## YDLIDAR SDK

- 공식 소스: https://github.com/YDLIDAR/YDLidar-SDK
- 고정 commit: `42a82ed10d2304094c111fc63dee8e4a229b79b7`
- 라이선스: SDK의 `LICENSE.txt` 및 개별 소스 고지를 따릅니다. 설치 후 `.deps/YDLidar-SDK`에서 확인할 수 있습니다.
- 범위: 설치 스크립트가 SDK 전체를 고정 버전으로 내려받고 정적 라이브러리로 링크합니다. 원본을 수정하지 않습니다.
- `CYdLidar` 공식 API의 초기화, 스캔 수신, stop, disconnect를 호출하는 독립 래퍼를 작성했습니다. ROS 드라이버를 가져오지 않았습니다.
- 기본 설정 비교 출처: LeKiwi의 `lekiwi_sensors/param/ydlidar.yaml`, 소스 시점 commit `acf5664`. 230400baud, 삼각 측정형, 4kHz sample rate, 10Hz scan, intensity 8bit를 적용합니다.
- 자동 재연결은 검사에서 끕니다. 연결 오류가 반복되면 실패로 기록하고 종료하기 위함입니다.

## 검수 기준 참고

- LeKiwi `docs/hardware/bmi160_imu_validation.md`의 정지 IMU 수신·중력·각속도 범위를 참고했습니다.
- 이 도구의 추가 수치 기준은 README에 구분해 기록했습니다. 소프트웨어 테스트를 실물 검수 통과로 해석하지 않습니다.
