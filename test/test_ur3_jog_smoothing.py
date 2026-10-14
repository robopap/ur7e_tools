"""Offline tests for speed limits and command acceleration."""
import numpy as np
import pytest
from ur7e_tools.ur3_jog_math import axis_twist, slew_joint_velocity, joint_step

def test_speed_range():
    assert axis_twist('X',1,0.01,0.03)[0] == pytest.approx(.01)
    assert axis_twist('X',1,0.05,0.15)[0] == pytest.approx(.05)
    with pytest.raises(ValueError):
        axis_twist('X',1,0.051,0.15)
    assert np.max(abs(joint_step(np.eye(6), np.ones(6)))) <= .20000001

def test_smoothing_bounds():
    prev=np.zeros(6)
    target=np.full(6,.12)
    x=slew_joint_velocity(prev,target,.02,max_joint_accel=.35)
    assert np.allclose(x,.007)
    assert np.allclose(slew_joint_velocity(x, -target,.02), np.zeros(6))

def test_invalid_input():
    with pytest.raises(ValueError):
        slew_joint_velocity(np.zeros(6),np.ones(6),-1)
    with pytest.raises(ValueError):
        slew_joint_velocity(np.zeros(6),np.array([np.nan]*6),.02)
