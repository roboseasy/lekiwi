# Third-party material

The repository's original ROS 2 code is distributed under the root Apache-2.0
license. The following material retains its own origin and terms. The product
owner confirmed on 2026-09-30 that redistribution rights for the robot model
and the adapted SLAM/Nav2 configuration have been secured; the underlying
permission records should be kept with the company's release records.

| Material | Origin | Included here | License or permission record |
| --- | --- | --- | --- |
| LeKiwi body, SO101 model meshes and Xacro | `roboseasy-members/lekiwi_isaacsim` and the source workspaces recorded in `lekiwi_description/docs/IMPORT.md` and `SOURCE.md` | `lekiwi_description/meshes`, `urdf`, camera mounts | Company-controlled model; redistribution confirmed by product owner. |
| Cartographer and Nav2 configuration adaptation | `yeon-03/lekiwi-pill-pickup`, commit `9f4a00cfd9dd15e31e73f96825eb86193ef2a20a` | `lekiwi_cartographer/config`, `lekiwi_navigation2/config` | Redistribution permission confirmed by product owner; upstream snapshot did not contain a license file. See `docs/hardware/slam_navigation.md`. |
| Bosch BMI160 API | Bosch Sensortec | `lekiwi_sensors/vendor/bosch_bmi160`, `lekiwi-sensor-check/vendor/bosch_bmi160` | BSD-3-Clause; original copyright and license are retained in each directory's `LICENSE`. |
| YDLIDAR SDK and ROS 2 driver | YDLIDAR | Fetched separately by `lekiwi.repos`; the sensor checker also fetches the SDK through `lekiwi-sensor-check/setup.sh` | See each upstream repository's license at the pinned commits. |

ROS 없는 센서 점검 도구의 가져온 범위와 고정 버전은
[센서 점검 도구의 출처 기록](lekiwi-sensor-check/THIRD_PARTY_NOTICES.md)을 참고한다.

Before publishing an archive or binary that contains the model or adapted
configuration, the release owner should retain the actual permission records
and check that the shipped notices match those permissions.
