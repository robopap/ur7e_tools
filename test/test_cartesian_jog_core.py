
import sys
from pathlib import Path
import unittest
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

try:
    from ur7e_tools.cartesian_jog_core import (
        validate_axis_delta,
        translated_target_pose,
        validate_pose_name,
    )
except ImportError:
    validate_axis_delta = None
    translated_target_pose = None
    validate_pose_name = None


class CartesianJogCoreTests(unittest.TestCase):
    def test_core_api_exists(self):
        self.assertIsNotNone(validate_axis_delta)
        self.assertIsNotNone(translated_target_pose)
        self.assertIsNotNone(validate_pose_name)

    def test_target_translation_preserves_orientation(self):
        self.assertIsNotNone(validate_axis_delta)
        R = np.eye(3)
        p = np.array([0.4, -0.2, 0.5])
        d = np.array([0.0, 0.020, 0.0])
        R2, p2 = translated_target_pose(R, p, d)
        np.testing.assert_allclose(R2, R)
        np.testing.assert_allclose(p2, p + d)

    def test_one_axis_and_20mm_limit(self):
        self.assertIsNotNone(validate_axis_delta)
        np.testing.assert_allclose(
            validate_axis_delta(0.0, 0.020, 0.0),
            [0.0, 0.020, 0.0],
        )
        with self.assertRaises(ValueError):
            validate_axis_delta(0.005, 0.005, 0.0)
        with self.assertRaises(ValueError):
            validate_axis_delta(0.0, 0.021, 0.0)
        with self.assertRaises(ValueError):
            validate_axis_delta(0.0, 0.0, 0.0)

    def test_pose_name_validation(self):
        self.assertEqual(validate_pose_name("jewelry_approach"), "jewelry_approach")
        for bad in ("../x", "bad/name", "", "space name"):
            with self.assertRaises(ValueError):
                validate_pose_name(bad)


if __name__ == "__main__":
    unittest.main()
