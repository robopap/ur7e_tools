"""Bounded Cartesian differential IK for single-UR3 jogging."""
import numpy as np


def joint_step(jacobian, twist, damping=0.08, max_joint_speed=0.20):
    j = np.asarray(jacobian, dtype=float)
    v = np.asarray(twist, dtype=float)
    if j.shape != (6, 6) or v.shape != (6,) or not np.isfinite(j).all() or not np.isfinite(v).all():
        raise ValueError('Expected finite 6x6 Jacobian and 6D twist')
    if damping <= 0 or max_joint_speed <= 0:
        raise ValueError('Invalid safety limits')
    qd = j.T @ np.linalg.solve(j @ j.T + damping**2 * np.eye(6), v)
    scale = min(1., max_joint_speed / max(float(np.max(np.abs(qd))), 1e-12))
    return qd * scale


def axis_twist(axis, sign, linear_speed_m_s, angular_speed_rad_s):
    if axis not in ('X', 'Y', 'Z', 'RX', 'RY', 'RZ') or sign not in (-1, 1):
        raise ValueError('Invalid axis/direction')
    if not 0 < linear_speed_m_s <= 0.05 or not 0 < angular_speed_rad_s <= 0.15:
        raise ValueError('Speed out of bounds')
    v = np.zeros(6)
    v[('X', 'Y', 'Z', 'RX', 'RY', 'RZ').index(axis)] = sign * (linear_speed_m_s if len(axis) == 1 else angular_speed_rad_s)
    return v


def slew_joint_velocity(previous, target, dt, max_joint_accel=0.35):
    """Bound acceleration of a running jog (rad/s^2); STOP bypasses smoothing."""
    old = np.asarray(previous, dtype=float)
    goal = np.asarray(target, dtype=float)
    if old.shape != (6,) or goal.shape != (6,) or not np.isfinite(old).all() or not np.isfinite(goal).all():
        raise ValueError('Expected finite 6D joint velocity vectors')
    if not np.isfinite(dt) or dt < 0 or not np.isfinite(max_joint_accel) or max_joint_accel <= 0:
        raise ValueError('Invalid acceleration parameters')
    delta = max_joint_accel * min(float(dt), 0.04)
    return old + np.clip(goal - old, -delta, delta)


def enforce_joint_limits(position, velocity, lower, upper, margin=0.03):
    """Prevent outward velocity at bounded joint limits (continuous: +/-inf)."""
    q, qd, lo, hi = (np.asarray(x, dtype=float) for x in (position, velocity, lower, upper))
    if any(x.shape != (6,) for x in (q, qd, lo, hi)) or not np.isfinite(q).all() or not np.isfinite(qd).all():
        raise ValueError('Expected finite 6D positions and velocities')
    if np.isnan(lo).any() or np.isnan(hi).any() or np.any(lo >= hi):
        raise ValueError('Invalid joint bounds')
    if not np.isfinite(margin) or margin < 0 or np.any((hi - lo) <= 2 * margin):
        raise ValueError('Invalid joint limit margin')
    result = qd.copy()
    result[(q <= lo + margin) & (result < 0)] = 0.
    result[(q >= hi - margin) & (result > 0)] = 0.
    return result


def controller_states_match(states, active, inactive):
    """Require named controllers active and all others not active."""
    return (all(states.get(name) == 'active' for name in active)
            and all(states.get(name) != 'active' for name in inactive))
