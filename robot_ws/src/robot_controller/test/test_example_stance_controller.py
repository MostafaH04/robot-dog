"""Tests for the deterministic example stance controller."""

import numpy as np

from robot_controller.example_stance_controller import StancePattern


def test_stance_pattern_is_periodic_finite_and_within_adapter_limits():
    pattern = StancePattern(steps_per_cycle=80)

    first_command = pattern.command(0)
    assert first_command == pattern.command(80)

    for step in range(80):
        command = np.asarray(pattern.command(step)).reshape(4, 3)
        assert np.all(np.isfinite(command))
        assert np.all(np.abs(command[:, 0]) <= np.pi / 2)
        assert np.all(command[:, 1] >= -np.pi / 6)
        assert np.all(command[:, 1] <= np.pi / 2)
        assert np.all(command[:, 2] >= -7 * np.pi / 18)
        assert np.all(command[:, 2] <= 7 * np.pi / 18)
        converted = [
            pattern.kinematics.leg_control_conversion(*leg_angles)
            for leg_angles in command
        ]
        assert np.all(np.isfinite(converted))
