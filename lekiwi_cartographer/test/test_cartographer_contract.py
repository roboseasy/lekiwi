from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parents[1]


class CartographerContractTest(unittest.TestCase):
    def test_standard_launch_selects_imu_profile(self):
        launch = (ROOT / 'launch/cartographer.launch.py').read_text()
        self.assertIn("default_value='lekiwi_2d_imu.lua'", launch)
        self.assertIn("default_value='/imu/data_raw'", launch)

    def test_odom_has_single_owner_and_imu_is_optional(self):
        for name, frame, imu in [('lekiwi_2d.lua', 'base_footprint', 'false'),
                                  ('lekiwi_2d_imu.lua', 'imu_link', 'true')]:
            lua = (ROOT / 'config' / name).read_text()
            self.assertIn('published_frame = "odom"', lua)
            self.assertIn('provide_odom_frame = false', lua)
            self.assertIn('use_odometry = true', lua)
            self.assertIn(f'tracking_frame = "{frame}"', lua)
            self.assertIn(f'use_imu_data = {imu}', lua)
            for setting in ['max_range = 9.0', 'occupied_space_weight = 20.0',
                            'num_range_data = 45', 'num_threads = 4']:
                self.assertIn(setting, lua)


if __name__ == '__main__':
    unittest.main()
