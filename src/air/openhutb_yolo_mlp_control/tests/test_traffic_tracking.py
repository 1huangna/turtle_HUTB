"""Coordinate and stale-data checks independent of simulator availability."""

import importlib.util
import math
import json
from pathlib import Path
import sys
import tempfile
import time
from types import ModuleType, SimpleNamespace
import unittest
from unittest.mock import patch

SPEC = importlib.util.spec_from_file_location(
    'traffic_common', Path(__file__).parents[1] / 'tools' / 'traffic' / 'common.py')
common = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(common)


class TrafficTrackingTests(unittest.TestCase):
    def test_follow_preserves_orientation_and_releases_without_hover(self):
        tools = str(Path(__file__).parents[1] / 'tools' / 'traffic')
        with patch.object(sys, 'path', [tools] + sys.path):
            import follow_vehicle
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            orientation = object()
            pose = SimpleNamespace(position=SimpleNamespace(x_val=0, y_val=0, z_val=25),
                                   orientation=orientation)
            calls = []

            class Client:
                def __init__(self, **kwargs):
                    pass

                def ping(self):
                    return True

                def simListSceneObjects(self):
                    return [str(i) for i in range(5)]

                def simGetObjectPose(self, name):
                    return SimpleNamespace(position=SimpleNamespace(x_val=0, y_val=0, z_val=50))

                def simGetVehiclePose(self):
                    return pose

                def getMultirotorState(self):
                    return SimpleNamespace(collision=SimpleNamespace(has_collided=False))

                def cancelLastTask(self):
                    calls.append('cancel')

                def enableApiControl(self, enabled):
                    calls.append(('api', enabled))

                def simSetVehiclePose(self, updated, ignore_collision):
                    self_test.assertIs(updated.orientation, orientation)
                    self_test.assertFalse(ignore_collision)
                    calls.append('pose')
                    (root / 'follow_exit').touch()

            self_test = self
            vendor = ModuleType('openhutb_yolo_mlp_control.vendor')
            vendor.airsim = SimpleNamespace(
                MultirotorClient=Client, Vector3r=lambda *v: v,
                Pose=lambda position, orientation: SimpleNamespace(position=position, orientation=orientation))
            common.write_json(root / 'references.json', {'map': 'test', 'references':
                              [{'name': str(i), 'carla': [0, 0, 0]} for i in range(5)]})
            common.write_json(root / 'live.json', {'time': time.time(), 'map': 'test', 'vehicles':
                              [dict(actor_id=1, x=0, y=0, z=0, yaw_deg=0, vx=2, vy=0)]})
            args = SimpleNamespace(data=root, host='fake', port=41451, target_id=None,
                                   distance=12, height=25, speed=12, duration=90)
            with patch.dict(sys.modules, {'openhutb_yolo_mlp_control.vendor': vendor}):
                with patch.object(follow_vehicle.time, 'sleep'):
                    follow_vehicle.follow(args)
            self.assertEqual(calls, ['cancel', 'pose', 'cancel', ('api', False)])
            self.assertTrue(json.loads((root / 'follow_status.json').read_text())['control_released'])

    def test_calibration_flips_z_and_fits_translation(self):
        pairs = [([i, i * 2, i * 3], [i - 36, i * 2 + 7, 50 - i * 3]) for i in range(6)]
        offset, residual = common.calibration_offset(pairs)
        self.assertEqual(offset, [-36, 7, 50])
        self.assertEqual(residual, 0)

    def test_wrong_reference_and_nonfinite_are_rejected(self):
        pairs = [([0, 0, 0], [1, 2, 3]) for _ in range(5)]
        pairs[-1] = ([0, 0, 0], [9, 2, 3])
        with self.assertRaises(ValueError):
            common.calibration_offset(pairs)
        pairs[-1] = ([0, 0, 0], [math.nan, 2, 3])
        with self.assertRaises(ValueError):
            common.calibration_offset(pairs)

    def test_following_height_and_trailing_distance(self):
        point = common.following_point(dict(x=10, y=20, z=1, yaw_deg=90), [-36, 7, 50], 12, 25)
        for actual, expected in zip(point, [-26, 15, 24]):
            self.assertAlmostEqual(actual, expected)

    def test_step_is_limited_and_does_not_overshoot(self):
        result = common.step_towards([0, 0, 0], [3, 4, 0], 2)
        for actual, expected in zip(result, [1.2, 1.6, 0]):
            self.assertAlmostEqual(actual, expected)
        self.assertEqual(common.step_towards([0, 0, 0], [3, 4, 0], 20), [3, 4, 0])

    def test_stale_and_future_telemetry_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'live.json'
            for age in (3, -3):
                common.write_json(path, {'time': time.time() - age})
                with self.assertRaises(RuntimeError):
                    common.read_live(directory)
            common.write_json(path, {'time': time.time(), 'vehicles': []})
            self.assertEqual(common.read_live(directory)['vehicles'], [])


if __name__ == '__main__':
    unittest.main()
