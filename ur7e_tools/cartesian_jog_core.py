
from __future__ import annotations
import re
import numpy as np

MAX_JOG_M = 0.020
_POSE_NAME_RE = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_-]*$")


def validate_axis_delta(dx, dy, dz, max_step_m=MAX_JOG_M):
    delta = np.asarray([dx, dy, dz], dtype=float)
    if delta.shape != (3,) or not np.all(np.isfinite(delta)):
        raise ValueError("Cartesian jog delta must be three finite values.")
    nonzero = np.flatnonzero(np.abs(delta) > 1e-12)
    if len(nonzero) != 1:
        raise ValueError("Exactly one Cartesian axis must move per jog.")
    if abs(float(delta[nonzero[0]])) > float(max_step_m) + 1e-12:
        raise ValueError(
            f"One jog is limited to {1000.0 * float(max_step_m):.1f} mm."
        )
    return delta


def translated_target_pose(rotation, translation, delta_base):
    rotation = np.asarray(rotation, dtype=float)
    translation = np.asarray(translation, dtype=float)
    delta_base = np.asarray(delta_base, dtype=float)
    if rotation.shape != (3, 3):
        raise ValueError("rotation must have shape (3, 3).")
    if translation.shape != (3,) or delta_base.shape != (3,):
        raise ValueError("translation and delta_base must have shape (3,).")
    return rotation.copy(), translation + delta_base


def validate_pose_name(name):
    name = str(name).strip()
    if not _POSE_NAME_RE.fullmatch(name):
        raise ValueError(
            "Pose name must start with a letter/number and contain only "
            "letters, numbers, '_' or '-'."
        )
    return name
