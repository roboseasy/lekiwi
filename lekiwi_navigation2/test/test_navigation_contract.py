from pathlib import Path
import math
import unittest
import yaml

ROOT = Path(__file__).resolve().parents[2]
PARAMS = yaml.safe_load((ROOT / 'lekiwi_navigation2/config/nav2_params.yaml').read_text())


class NavigationContractTest(unittest.TestCase):
    def test_standard_launch_uses_saved_map_and_waits_for_localization(self):
        launch = (ROOT / 'lekiwi_navigation2/launch/navigation.launch.py').read_text()
        self.assertIn("Path.home() / 'maps' / 'lekiwi' / 'map.yaml'", launch)
        self.assertIn("'use_rviz',\n        default_value='true'", launch)
        self.assertIn("'autostart',\n        default_value='true'", launch)
        self.assertIn("'auto_arm_motors',\n        default_value='true'", launch)
        self.assertIn("IfCondition(AndSubstitution(use_rviz, auto_arm_motors))", launch)
        self.assertIn("executable='lekiwi_nav2_motor_guard'", launch)
        self.assertIn("target_action=rviz_node", launch)

    def test_omni_controller_and_localization(self):
        amcl = PARAMS['amcl']['ros__parameters']
        self.assertEqual(amcl['robot_model_type'], 'nav2_amcl::OmniMotionModel')
        self.assertFalse(amcl['set_initial_pose'])
        controller = PARAMS['controller_server']['ros__parameters']
        mppi = controller['FollowPath']
        self.assertEqual(mppi['motion_model'], 'Omni')
        self.assertEqual(mppi['plugin'], 'nav2_mppi_controller::MPPIController')
        self.assertAlmostEqual(controller['controller_frequency'] * mppi['model_dt'], 1)
        self.assertNotIn('PreferForwardCritic', mppi['critics'])

    def test_limits_fit_new_base_and_keep_lateral_motion(self):
        base = yaml.safe_load((ROOT / 'lekiwi_bringup/param/base.yaml').read_text())['lekiwi_node']['ros__parameters']
        limits = PARAMS['velocity_smoother']['ros__parameters']['max_velocity']
        self.assertGreater(limits[1], 0)
        for value, name in zip(limits, ['max_linear_x', 'max_linear_y', 'max_angular_z']):
            self.assertLessEqual(value, base[name])
        self.assertLessEqual(math.hypot(*limits[:2]), base['max_linear_speed'])
        self.assertAlmostEqual(base['max_wheel_speed'] * base['speed_tick_scale'], base['speed_tick_limit'])

    def test_collision_monitor_is_final_command_publisher(self):
        p = PARAMS['collision_monitor']['ros__parameters']
        self.assertEqual(p['cmd_vel_in_topic'], 'cmd_vel_smoothed')
        self.assertEqual(p['cmd_vel_out_topic'], 'cmd_vel')
        launch = (ROOT / 'lekiwi_navigation2/launch/navigation.launch.py').read_text()
        self.assertIn("'collision_monitor',", launch)
        self.assertIn("package='nav2_collision_monitor'", launch)
        self.assertNotIn("('cmd_vel_smoothed', 'cmd_vel')", launch)
        for name in ['controller_server', 'behavior_server', 'velocity_smoother', 'collision_monitor']:
            self.assertFalse(PARAMS[name]['ros__parameters']['enable_stamped_cmd_vel'])

    def test_costmaps_keep_current_body_size_and_scan_loss_detection(self):
        scan_filter = yaml.safe_load(
            (ROOT / 'lekiwi_sensors/param/navigation_scan.yaml').read_text()
        )['navigation_scan_filter']['ros__parameters']
        min_range = scan_filter['self_radius']
        self.assertFalse(scan_filter['mask_wheels'])
        self.assertAlmostEqual(PARAMS['amcl']['ros__parameters']['laser_min_range'], min_range)
        for name in ['local_costmap', 'global_costmap']:
            p = PARAMS[name][name]['ros__parameters']
            self.assertEqual(p['robot_base_frame'], 'base_footprint')
            self.assertGreaterEqual(p['robot_radius'], 0.22)
            scan = p['obstacle_layer']['scan']
            self.assertEqual(scan['topic'], '/scan_navigation')
            self.assertAlmostEqual(scan['obstacle_min_range'], min_range)
            self.assertAlmostEqual(scan['raytrace_min_range'], min_range)
            self.assertLessEqual(scan['expected_update_rate'], 0.25)
            self.assertFalse(scan['inf_is_valid'])


if __name__ == '__main__':
    unittest.main()
