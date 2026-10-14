"""Offline numeric tests for the single-UR3 differential jog backend."""
import numpy as np
import pytest

from ur7e_tools.ur3_jog_math import axis_twist, joint_step


def test_jog_axes_and_speed_bounds():
    assert np.allclose(axis_twist('X', -1, .003, .03), [-.003, 0, 0, 0, 0, 0])
    assert np.allclose(axis_twist('RZ', 1, .003, .03), [0, 0, 0, 0, 0, .03])
    with pytest.raises(ValueError):
        axis_twist('X', 1, .3, .03)


def test_velocity_saturation_and_singular_jacobian():
    assert np.max(abs(joint_step(np.eye(6), np.ones(6) * 3))) <= .20000001
    assert np.allclose(joint_step(np.zeros((6, 6)), np.ones(6)), 0)


def test_nan_input_rejected():
    with pytest.raises(ValueError):
        joint_step(np.eye(6), [np.nan] * 6)
