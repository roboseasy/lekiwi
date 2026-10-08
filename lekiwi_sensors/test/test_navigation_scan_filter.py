import importlib.machinery
import math
from pathlib import Path
from types import SimpleNamespace
import unittest

module = importlib.machinery.SourceFileLoader(
    'navigation_filter', str(Path(__file__).resolve().parents[1] / 'scripts/navigation_scan_filter')).load_module()


class ScanFilterTest(unittest.TestCase):
    def scan(self, ranges, intensities=()):
        return SimpleNamespace(ranges=list(ranges), intensities=list(intensities),
                               range_min=0.05, range_max=12.0, angle_min=0.0,
                               angle_increment=math.pi/2, time_increment=0.001,
                               scan_time=0.16, header='unchanged')

    def test_rejected_beams_never_clear_obstacles_and_input_is_unchanged(self):
        scan = self.scan([0.1, 1.0, float('inf'), 13.0])
        out = module.filter_scan(scan, 0.21, 0.0)
        self.assertEqual(out.ranges[1], 1.0)
        self.assertTrue(all(math.isnan(out.ranges[i]) for i in (0, 2, 3)))
        self.assertEqual(scan.ranges[0], 0.1)
        for key in ('header', 'angle_min', 'angle_increment', 'time_increment', 'scan_time'):
            self.assertEqual(getattr(scan, key), getattr(out, key))

    def test_upside_down_mount_masks_opposite_sensor_bearing(self):
        scan = self.scan([1.0]*4)
        sectors = module.parse_sectors(['-90:10'])
        out = module.filter_scan(scan, 0.21, 0.0, sectors, (1, 0, 0, 0))
        self.assertTrue(math.isnan(out.ranges[1]))
        self.assertEqual(out.ranges[3], 1.0)

    def test_sector_wraps_at_pi(self):
        out = module.filter_scan(self.scan([1.0]*4), 0.21, 0.0,
                                 module.parse_sectors(['-179:10']), (0, 0, 0, 1))
        self.assertTrue(math.isnan(out.ranges[2]))

    def test_intensity_requires_actual_measurements(self):
        with self.assertRaises(ValueError):
            module.filter_scan(self.scan([1.0]), 0.21, 25.0)
        out = module.filter_scan(self.scan([1.0]*3, [24, 25, float('nan')]), 0.21, 25.0)
        self.assertTrue(math.isnan(out.ranges[0]))
        self.assertEqual(out.ranges[1], 1.0)
        self.assertTrue(math.isnan(out.ranges[2]))

    def test_invalid_configuration_is_rejected(self):
        for value in ['nan:20', '0:360', '0:-1']:
            with self.assertRaises(ValueError):
                module.parse_sectors([value])
        with self.assertRaises(ValueError):
            module.filter_scan(self.scan([1]), float('nan'), 0)
        with self.assertRaises(ValueError):
            module.filter_scan(self.scan([1]), 0.21, 0, [(0, 0.1)])


if __name__ == '__main__':
    unittest.main()
