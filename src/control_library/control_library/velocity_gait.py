"""Planar body-twist to foot-space gait primitives.

Coordinates use the robot base frame: +X forward, +Y left, +Z up.
During stance, a planted foot moves backward relative to a translating or
rotating body. Swing returns the foot to its next foothold.
"""

import math


def _require_finite(*values):
    if not all(math.isfinite(float(value)) for value in values):
        raise ValueError("Gait inputs must be finite numbers")


def stance_displacement(foot_xy, velocity_x, velocity_y, yaw_rate, stance_duration):
    """Foot displacement relative to the body while its world position is planted.

    For a stationary world-frame foot, its velocity in the rotating body frame
    is -v_body - omega_z x p_foot:
      dx/dt = -vx + omega*y
      dy/dt = -vy - omega*x
    """
    if len(foot_xy) != 2:
        raise ValueError("foot_xy must contain x and y")
    x, y = (float(value) for value in foot_xy)
    vx, vy, wz, duration = map(float, (velocity_x, velocity_y, yaw_rate, stance_duration))
    _require_finite(x, y, vx, vy, wz, duration)
    if duration <= 0.0:
        raise ValueError("stance_duration must be positive")
    return ((-vx + wz * y) * duration, (-vy - wz * x) * duration)


def sample_foot_path(phase, dx, dy, stance_ratio=0.65, swing_height=0.02):
    """Sample one closed stance/swing cycle as x/y offsets and z lift."""
    phase, dx, dy, stance_ratio, swing_height = map(
        float, (phase, dx, dy, stance_ratio, swing_height)
    )
    _require_finite(phase, dx, dy, stance_ratio, swing_height)
    if not 0.0 < stance_ratio < 1.0:
        raise ValueError("stance_ratio must be between zero and one")
    if swing_height < 0.0:
        raise ValueError("swing_height cannot be negative")

    phase %= 1.0
    if phase < stance_ratio:
        progress = phase / stance_ratio
        return (-0.5 * dx + dx * progress,
                -0.5 * dy + dy * progress,
                0.0)

    progress = (phase - stance_ratio) / (1.0 - stance_ratio)
    smooth = progress * progress * (3.0 - 2.0 * progress)
    lift = swing_height * math.sin(math.pi * progress)
    return (0.5 * dx - dx * smooth,
            0.5 * dy - dy * smooth,
            lift)


def relative_foot_path(global_phase, phase_offset, dx, dy,
                       stance_ratio=0.65, swing_height=0.02):
    """Return a phase-shifted foot path whose first sample is exactly zero."""
    phase = (float(global_phase) + float(phase_offset)) % 1.0
    initial_phase = float(phase_offset) % 1.0
    current = sample_foot_path(phase, dx, dy, stance_ratio, swing_height)
    initial = sample_foot_path(initial_phase, dx, dy, stance_ratio, swing_height)
    return tuple(current[i] - initial[i] for i in range(3))


def in_diagonal_double_support(phase, stance_ratio=0.65):
    """Return true while both diagonal pairs are in stance."""
    phase = float(phase) % 1.0
    _require_finite(phase, stance_ratio)
    other_phase = (phase + 0.5) % 1.0
    return phase < stance_ratio and other_phase < stance_ratio
