#!/usr/bin/env python3
"""Export the current model and verify it before replacing a portable ZIP."""
import argparse
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path, PurePosixPath
import shutil
import subprocess
import tempfile
import xml.etree.ElementTree as ET
from zipfile import ZIP_DEFLATED, ZipFile


REPO = Path(__file__).resolve().parents[1]
DESCRIPTION = REPO / 'lekiwi_description'
BUNDLE_NAME = 'lekiwi_urdf_bundle'
parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--output', type=Path, default=REPO / 'exports' / f'{BUNDLE_NAME}.zip')
parser.add_argument('--hardware-sensors', action='store_true', help='Include the existing lidar and IMU mounts.')
args = parser.parse_args()
TARGET = args.output.resolve()
TARGET.parent.mkdir(parents=True, exist_ok=True)
XACRO = shutil.which('xacro')
if XACRO is None:
    parser.error('xacro is unavailable; source the ROS environment first')


def sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def render(path):
    return subprocess.check_output([
        XACRO, str(path),
        'base_mesh_dir:=meshes/base',
        'soarm_mesh_dir:=meshes/soarm',
        f'use_hardware_sensors:={str(args.hardware_sensors).lower()}',
    ], text=True)


def canonical(xml):
    root = ET.fromstring(xml)
    return ET.canonicalize(ET.tostring(root, encoding='unicode'), strip_text=True)


old_sha256 = sha256(TARGET) if TARGET.exists() else None
created_at = datetime.now(timezone.utc)
source_branch = subprocess.check_output(
    ['git', 'branch', '--show-current'], cwd=REPO, text=True).strip()
source_hashes = {}
with tempfile.TemporaryDirectory(prefix='lekiwi-latest-bundle-') as temporary:
    stage = Path(temporary)
    bundle = stage / BUNDLE_NAME
    bundle.mkdir()

    def copy(source, destination):
        destination.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(source, destination)
        assert sha256(source) == sha256(destination)
        source_hashes[str(source.relative_to(REPO))] = sha256(source)

    for source in sorted((DESCRIPTION / 'meshes').rglob('*.stl')):
        copy(source, bundle / 'meshes' / source.relative_to(DESCRIPTION / 'meshes'))
    for source in sorted((DESCRIPTION / 'urdf').rglob('*.xacro')):
        copy(source, bundle / 'source_xacro/urdf' / source.relative_to(DESCRIPTION / 'urdf'))
    for source in sorted((DESCRIPTION / 'config').iterdir()):
        if source.is_file():
            copy(source, bundle / 'source_xacro/config' / source.name)
    for filename in ['SOURCE.md', 'IMPORT.md', 'FRAME_MIGRATION.md', 'import_manifest.json']:
        copy(DESCRIPTION / 'docs' / filename, bundle / 'provenance' / filename)
    for filename in ['LICENSE', 'THIRD_PARTY_NOTICES.md']:
        copy(REPO / filename, bundle / filename)

    expected_xml = render(DESCRIPTION / 'urdf/lekiwi.urdf.xacro')
    robot = ET.fromstring(expected_xml)
    ET.indent(robot, space='  ')
    ET.ElementTree(robot).write(bundle / 'lekiwi_soarm.urdf', encoding='utf-8', xml_declaration=True)

    readme = '''# LeKiwi + SO101 최신 모델 번들

__SNAPSHOT_DATE__ 기준 `lekiwi/__SOURCE_BRANCH__` 작업본의 교육용 모델과 카메라 좌표계 스냅샷입니다.

## 열기

압축을 풀고 `lekiwi_soarm.urdf`를 엽니다. `meshes/`는 URDF와 같은 폴더에 둡니다.
모든 mesh 참조는 상대 경로이며 ROS package 경로나 원래 PC의 절대 경로가 필요하지 않습니다.

## 포함한 모델

- 교육용 `lekiwi_body_soarm_mount.stl` 몸체, omni wheel 3개, SO101 팔과 교육용 팔 장착 위치.
- 전방 `base_link → front_camera_optical_frame`, 손목 `wrist_link → wrist_camera_optical_frame`.
- 교육용 카메라 상자와 렌즈 외형, 원본 `camera_mounts.json` 설정.
- `base_footprint → base_link`: (0, 0, 0.075) m. 기준 높이는 바퀴 접지면입니다.
- 기본 내보내기는 18개 link, 17개 joint입니다. `--hardware-sensors` 내보내기는 lidar/imu를 포함한 20개 link, 19개 joint입니다. 실제 구성은 `MANIFEST.json`을 확인합니다.

RViz에서 확인한 기본 자세는 wrist_roll=-π/2 rad, 나머지 가동 관절=0 rad입니다.
URDF 자체는 초기 관절각을 지정하지 않으므로 다른 프로그램에서 이 자세를 보려면 별도로 설정합니다.

## 적용 범위

이 번들은 현재 ROS/RViz 모델을 고정 URDF로 내보낸 것입니다.
Isaac 전용 USD, 수동 롤러 충돌 구조, 추가 base 질량·관성 보정은 포함하지 않습니다.
Isaac Sim에서 물리 주행을 검증한 번들이 아닙니다.
카메라는 광학 좌표계와 외형이며 영상 센서를 생성하는 코드는 포함하지 않습니다.
장착값의 원본 `calibrated: false`와 손목 카메라의 wrist_roll 이전 장착 기준을 유지합니다.

## 재생성

Xacro를 사용할 수 있는 환경에서 이 폴더를 작업 경로로 실행합니다.

```bash
xacro source_xacro/urdf/lekiwi.urdf.xacro base_mesh_dir:=meshes/base soarm_mesh_dir:=meshes/soarm use_hardware_sensors:=false > lekiwi_soarm.urdf
```

하드웨어 프레임을 포함한 번들을 재생성할 때는 위 인자를 `use_hardware_sensors:=true`로 설정합니다.

현재 패키지의 Xacro·설정과 STL을 바이트 그대로 복사했습니다. 내보낸 URDF에서 mesh 경로만 상대 경로로 지정했습니다.
원본 교육용 프로젝트는 수정하지 않았습니다. 출처와 권리는 `provenance/` 및 `THIRD_PARTY_NOTICES.md`, 파일 해시는 `MANIFEST.json`에 있습니다.
'''
    readme = readme.replace('__SNAPSHOT_DATE__', created_at.date().isoformat())
    readme = readme.replace('__SOURCE_BRANCH__', source_branch or 'detached HEAD')
    (bundle / 'README.md').write_text(readme, encoding='utf-8')
    manifest = {
        'created_utc': created_at.isoformat(),
        'source_project': 'lekiwi',
        'source_branch': source_branch,
        'source_head': subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=REPO, text=True).strip(),
        'source_is_working_tree_snapshot': True,
        'classroom_source_commit': json.loads((DESCRIPTION / 'docs/import_manifest.json').read_text())['source_commit'],
        'model_entrypoint': 'lekiwi_soarm.urdf',
        'hardware_sensors': args.hardware_sensors,
        'links': len(robot.findall('link')),
        'joints': len(robot.findall('joint')),
        'source_sha256': source_hashes,
        'files_sha256': {str(p.relative_to(bundle)): sha256(p) for p in sorted(bundle.rglob('*')) if p.is_file()},
    }
    (bundle / 'MANIFEST.json').write_text(json.dumps(manifest, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')

    archive = stage / f'{BUNDLE_NAME}.zip'
    with ZipFile(archive, 'w', compression=ZIP_DEFLATED, compresslevel=6) as zipped:
        for p in sorted(bundle.rglob('*')):
            if p.is_file():
                zipped.write(p, arcname=str(p.relative_to(stage)))

    # Validate the independent extracted archive, not the staging source tree.
    extracted = stage / 'validation'
    with ZipFile(archive) as zipped:
        assert zipped.testzip() is None
        assert len(zipped.namelist()) == len(set(zipped.namelist()))
        for name in zipped.namelist():
            parts = PurePosixPath(name)
            assert not parts.is_absolute() and '..' not in parts.parts
            assert parts.parts[0] == BUNDLE_NAME
        zipped.extractall(extracted)
    checked = extracted / BUNDLE_NAME
    imported_manifest = json.loads((checked / 'MANIFEST.json').read_text())
    actual_files = {str(p.relative_to(checked)) for p in checked.rglob('*') if p.is_file()}
    assert actual_files == set(imported_manifest['files_sha256']) | {'MANIFEST.json'}
    for relative, digest in imported_manifest['files_sha256'].items():
        assert sha256(checked / relative) == digest
    exported = ET.parse(checked / 'lekiwi_soarm.urdf').getroot()
    assert canonical(ET.tostring(exported, encoding='unicode')) == canonical(expected_xml)
    assert canonical(render(checked / 'source_xacro/urdf/lekiwi.urdf.xacro')) == canonical(expected_xml)
    links = {link.get('name') for link in exported.findall('link')}
    joints = exported.findall('joint')
    expected_links = 20 if args.hardware_sensors else 18
    assert len(links) == expected_links and len(joints) == expected_links - 1
    assert links - {joint.find('child').get('link') for joint in joints} == {'base_footprint'}
    mesh_paths = set()
    for mesh in exported.iter('mesh'):
        path = PurePosixPath(mesh.get('filename'))
        assert not path.is_absolute() and '..' not in path.parts and '://' not in str(path)
        assert path.parts[0] == 'meshes'
        assert (checked / str(path)).is_file(), path
        mesh_paths.add(str(path))
    for camera, parent in [('front', 'base_link'), ('wrist', 'wrist_link')]:
        joint = exported.find(f"joint[@name='{camera}_camera_joint']")
        assert joint.get('type') == 'fixed' and joint.find('parent').get('link') == parent
        assert joint.find('child').get('link') == f'{camera}_camera_optical_frame'
    assert ('lidar_link' in links) == args.hardware_sensors
    assert ('imu_link' in links) == args.hardware_sensors
    assert not (checked / 'meshes/base/lekiwi_assem.stl').exists()

    # Guard against concurrent source edits before replacing the authorized ZIP.
    for relative, digest in source_hashes.items():
        assert sha256(REPO / relative) == digest, relative
    assert (sha256(TARGET) if TARGET.exists() else None) == old_sha256
    new_sha256 = sha256(archive)
    descriptor, name = tempfile.mkstemp(prefix='.lekiwi-bundle-', suffix='.zip.tmp', dir=TARGET.parent)
    os.close(descriptor)
    atomic_candidate = Path(name)
    try:
        shutil.copyfile(archive, atomic_candidate)
        atomic_candidate.chmod(0o664)
        assert sha256(atomic_candidate) == new_sha256
        os.replace(atomic_candidate, TARGET)
    finally:
        if atomic_candidate.exists():
            atomic_candidate.unlink()
    assert sha256(TARGET) == new_sha256
    print(json.dumps({
        'zip': str(TARGET),
        'size_bytes': TARGET.stat().st_size,
        'sha256': new_sha256,
        'files': len(actual_files),
        'mesh_files': len(list((checked / 'meshes').rglob('*.stl'))),
        'referenced_mesh_files': len(mesh_paths),
        'links': len(links),
        'joints': len(joints),
        'checks': ['ZIP CRC', 'extracted file hashes', 'relative mesh resolution', 'current-model equivalence', 'bundled Xacro regeneration', 'camera parent frames'],
        'old_zip_replaced': old_sha256 is not None,
    }, ensure_ascii=False, indent=2))
