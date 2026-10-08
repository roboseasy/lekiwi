"""Check model resources and the old-to-current frame registration."""
import hashlib
import json
import math
from pathlib import Path
import subprocess
import unittest
import xml.etree.ElementTree as ET

import yaml


PACKAGE = Path(__file__).resolve().parents[1]


def rotation(rpy):
    r, p, y = rpy
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return [[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
            [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr], [-sp, cp*sr, cp*cr]]


def quaternion_rotation(q):
    x, y, z, w = q
    return [[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
            [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
            [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]]


def origin(joint):
    element = joint.find('origin')
    xyz = [float(v) for v in element.get('xyz', '0 0 0').split()]
    rpy = [float(v) for v in element.get('rpy', '0 0 0').split()]
    return xyz, rotation(rpy)


class DefaultModelTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.preview = ET.fromstring(subprocess.check_output(
            ['xacro', str(PACKAGE / 'urdf/lekiwi.urdf.xacro')], text=True))
        cls.hardware = ET.fromstring(subprocess.check_output(
            ['xacro', str(PACKAGE / 'urdf/lekiwi.urdf.xacro'), 'use_hardware_sensors:=true'], text=True))
        cls.legacy = ET.parse(PACKAGE / 'test/fixtures/legacy_base.urdf').getroot()

    def assert_vector_equal(self, actual, expected, tolerance=1e-10):
        for a, b in zip(actual, expected, strict=True):
            self.assertAlmostEqual(a, b, delta=tolerance)

    def test_complete_tree_and_local_mesh_resources(self):
        for model, count in [(self.preview, 18), (self.hardware, 20)]:
            links = {e.get('name') for e in model.findall('link')}
            joints = model.findall('joint')
            self.assertEqual(len(links), count)
            self.assertEqual(len(joints), count - 1)
            children = {j.find('child').get('link') for j in joints}
            self.assertEqual(links - children, {'base_footprint'})
            self.assertEqual(len(children), count - 1)
            reached = {'base_footprint'}
            for _ in links:
                reached.update(j.find('child').get('link') for j in joints
                               if j.find('parent').get('link') in reached)
            self.assertEqual(reached, links)
            paths = set()
            for mesh in model.iter('mesh'):
                uri = mesh.get('filename')
                self.assertTrue(uri.startswith('package://lekiwi_description/meshes/'))
                path = PACKAGE / uri.removeprefix('package://lekiwi_description/')
                self.assertTrue(path.is_file(), path)
                paths.add(path)
            self.assertEqual(len(paths), 15)

    def test_original_model_assets_remain_identical(self):
        manifest = json.loads((PACKAGE / 'docs/import_manifest.json').read_text())
        for entry in manifest['files']:
            self.assertEqual(hashlib.sha256((PACKAGE / entry['destination']).read_bytes()).hexdigest(),
                             entry['sha256'], entry['destination'])

    def test_ground_reference_and_arm_mount(self):
        xyz, _ = origin(self.preview.find("joint[@name='base_footprint_joint']"))
        self.assert_vector_equal(xyz, [0, 0, 0.075])
        xyz, _ = origin(self.preview.find("joint[@name='soarm_mount_joint']"))
        self.assert_vector_equal(xyz, [0.019988279335, 0.001703025019, 0.052884])

    def test_cameras_preserve_parent_optical_pose(self):
        config = json.loads((PACKAGE / 'config/camera_mounts.json').read_text())
        for name, spec in config['cameras'].items():
            joint = self.preview.find(f"joint[@name='{name}_camera_joint']")
            self.assertEqual(joint.get('type'), 'fixed')
            self.assertEqual(joint.find('parent').get('link'), spec['parent_link'])
            self.assertEqual(joint.find('child').get('link'), f'{name}_camera_optical_frame')
            xyz, matrix = origin(joint)
            self.assert_vector_equal(xyz, spec['translation'])
            for actual, expected in zip(matrix, quaternion_rotation(spec['quaternion_xyzw']), strict=True):
                self.assert_vector_equal(actual, expected)

    def test_wheel_transforms_use_same_rigid_registration(self):
        params = yaml.safe_load((PACKAGE / 'config/base_frame.yaml').read_text())['lekiwi_node']['ros__parameters']
        R = rotation([0, 0, params['kinematics_frame_yaw']])
        t = [params['kinematics_frame_x'], params['kinematics_frame_y'], 0]
        names = [f'{wheel}_wheel_mount_joint' for wheel in ['left', 'back', 'right']]
        for name in names:
            old_xyz, old_R = origin(self.legacy.find(f"joint[@name='{name}']"))
            xyz, new_R = origin(self.hardware.find(f"joint[@name='{name}']"))
            expected_xyz = [sum(R[i][j]*old_xyz[j] for j in range(3)) + t[i] for i in range(3)]
            self.assert_vector_equal(xyz, expected_xyz)
            for i in range(3):
                self.assert_vector_equal(new_R[i], [sum(R[i][k]*old_R[k][j] for k in range(3)) for j in range(3)])

    def test_lidar_uses_measured_offset_and_aligns_x_while_inverting_yz(self):
        joint = self.hardware.find("joint[@name='lidar_joint']")
        self.assertEqual(joint.get('type'), 'fixed')
        self.assertEqual(joint.find('parent').get('link'), 'soarm_base_link')
        self.assertEqual(joint.find('child').get('link'), 'lidar_link')
        xyz, lidar_R = origin(joint)
        self.assert_vector_equal(xyz, [-0.02067, 0, -0.10250])
        mount_xyz, mount_R = origin(self.hardware.find("joint[@name='soarm_mount_joint']"))
        base_xyz = [mount_xyz[i] + sum(mount_R[i][j]*xyz[j] for j in range(3)) for i in range(3)]
        self.assert_vector_equal(base_xyz, [-0.000681720665, 0.001703025019, -0.049616])
        ground_xyz, _ = origin(self.hardware.find("joint[@name='base_footprint_joint']"))
        self.assertAlmostEqual(ground_xyz[2] + base_xyz[2], 0.025384)
        # Physical axis directions supplied by the user, independently of URDF RPY.
        expected_R = [[1, 0, 0], [0, -1, 0], [0, 0, -1]]
        for i in range(3):
            self.assert_vector_equal(
                [sum(mount_R[i][k]*lidar_R[k][j] for k in range(3)) for j in range(3)], expected_R[i])

    def test_temporary_imu_mount_is_above_lidar_in_base_axes(self):
        imu = self.hardware.find("joint[@name='imu_joint']")
        lidar = self.hardware.find("joint[@name='lidar_joint']")
        self.assertEqual(imu.get('type'), 'fixed')
        self.assertEqual(imu.find('parent').get('link'), 'soarm_base_link')
        self.assertEqual(imu.find('child').get('link'), 'imu_link')
        imu_xyz, _ = origin(imu)
        lidar_xyz, _ = origin(lidar)
        self.assert_vector_equal(imu_xyz, [-0.02067, 0, -0.05250])
        mount_xyz, mount_R = origin(self.hardware.find("joint[@name='soarm_mount_joint']"))
        displacement = [sum(mount_R[i][j] * (imu_xyz[j] - lidar_xyz[j]) for j in range(3))
                        for i in range(3)]
        # Above the lidar along robot +Z, not along the upside-down lidar's +Z.
        self.assert_vector_equal(displacement, [0, 0, 0.05])
        base_xyz = [mount_xyz[i] + sum(mount_R[i][j]*imu_xyz[j] for j in range(3)) for i in range(3)]
        ground_xyz, _ = origin(self.hardware.find("joint[@name='base_footprint_joint']"))
        self.assertAlmostEqual(ground_xyz[2] + base_xyz[2], 0.075384)

    def test_imu_rotation_is_nominal_ninety_degrees(self):
        # 2026-09-23 user-selected mount angle; older hand-tilt observations only
        # establish approximate direction and are not absolute yaw calibration.
        # Forward lift adds +base X gravity and rotates about -base Y.
        # Left lift adds +base Y gravity and rotates about +base X.
        _, matrix = origin(self.hardware.find("joint[@name='imu_joint']"))
        for actual, expected in zip(matrix, [[0, -1, 0], [1, 0, 0], [0, 0, 1]], strict=True):
            self.assert_vector_equal(actual, expected)
        observations = [
            ([0.21221485255126935, -3.4282753136901873, -0.621915429040536], 0, 1),
            ([5.190296405187987, 0.3398030561218262, -1.426223534753415], 1, 1),
            ([-0.3485496890989731, -0.016061247791271688, 0.012654662526362742], 1, -1),
            ([0.046682583265409014, -0.5537813239742106, 0.02700763058308026], 0, 1),
        ]
        for sensor, axis, sign in observations:
            base = [sum(matrix[i][j] * sensor[j] for j in range(3)) for i in range(3)]
            self.assertGreater(sign * base[axis], 0)
            self.assertLess(abs(base[1 - axis]), 0.10 * abs(base[axis]))
        self.assert_vector_equal([matrix[i][2] for i in range(3)], [0, 0, 1])

    def test_preview_and_hardware_share_the_entire_robot(self):
        hardware = ET.fromstring(ET.tostring(self.hardware))
        for element in list(hardware):
            if element.get('name') in {'lidar_link', 'lidar_joint', 'imu_link', 'imu_joint', 'lekiwi_lidar'}:
                hardware.remove(element)
        def canonical(model):
            return ET.canonicalize(ET.tostring(model, encoding='unicode'), strip_text=True)
        self.assertEqual(canonical(hardware), canonical(self.preview))


if __name__ == '__main__':
    unittest.main()
