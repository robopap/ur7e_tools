import numpy as np
import pytest
from ur7e_tools.ur3_jog_math import enforce_joint_limits, controller_states_match


def test_blocks_outward_only():
    q = np.array([-1., 1., 0., 0., 0., 0.])
    v = np.array([-.2, .2, -.1, .1, -.1, .1])
    lo = np.full(6, -1.)
    hi = np.full(6, 1.)
    result = enforce_joint_limits(q, v, lo, hi)
    np.testing.assert_allclose(result, [0, 0, -.1, .1, -.1, .1])
    np.testing.assert_allclose(enforce_joint_limits(q, -v, lo, hi), -v)


def test_continuous_unbounded():
    q = np.zeros(6)
    v = np.full(6, .1)
    np.testing.assert_allclose(enforce_joint_limits(q, v, np.full(6, -np.inf), np.full(6, np.inf)), v)


def test_invalid_inputs():
    with pytest.raises(ValueError):
        enforce_joint_limits(np.zeros(6), np.zeros(6), np.ones(6), -np.ones(6))
    with pytest.raises(ValueError):
        enforce_joint_limits(np.zeros(5), np.zeros(6), np.zeros(6), np.ones(6))


def test_controller_states_confirmed():
    states = {'joint_trajectory_controller': 'active',
              'forward_velocity_controller': 'inactive',
              'forward_position_controller': 'inactive'}
    assert controller_states_match(states, ['joint_trajectory_controller'],
                                   ['forward_velocity_controller', 'forward_position_controller'])
    assert not controller_states_match(states, ['forward_velocity_controller'],
                                       ['joint_trajectory_controller'])
    assert not controller_states_match({}, ['joint_trajectory_controller'], [])
    states['forward_velocity_controller'] = 'active'
    assert not controller_states_match(states, ['joint_trajectory_controller'],
                                       ['forward_velocity_controller'])
