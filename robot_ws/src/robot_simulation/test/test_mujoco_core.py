"""Exercise the ROS-independent deterministic MuJoCo core."""

from dataclasses import fields

import numpy as np
import pytest

from robot_simulation.mujoco_core import (
    FOOT_NAMES,
    GroundTruth,
    JOINT_NAMES,
    JointCommand,
    MujocoSimulator,
    SensorData,
    trajectory_fingerprint,
)


def test_reset_and_fixed_trajectory_are_exactly_repeatable():
    """Reset and replay should produce exactly equal public data."""
    simulator = MujocoSimulator(settle_steps=80)
    command = {'Revolute_25': 0.47, 'Revolute_31': 0.47}

    first = [simulator.reset()]
    first.extend(simulator.step(command) for _ in range(20))
    second = [simulator.reset()]
    second.extend(simulator.step(command) for _ in range(20))

    assert np.array_equal(
        trajectory_fingerprint(first),
        trajectory_fingerprint(second),
    )
    assert first[-1].time == pytest.approx(20 * simulator.timestep)


def test_safe_stance_exposes_expected_teaching_interface():
    """The core should expose mock sensors without embedded truth."""
    result = MujocoSimulator(settle_steps=120).reset()

    assert result.sensors.joint_states.names == JOINT_NAMES
    assert tuple(result.sensors.foot_contacts) == FOOT_NAMES
    assert all(
        contact.normal_force >= 0.0
        for contact in result.sensors.foot_contacts.values()
    )
    assert np.all(np.isfinite(result.sensors.imu.specific_force_body))
    assert 'ground_truth' not in {field.name for field in fields(SensorData)}
    assert {field.name for field in fields(GroundTruth)} == {
        'position_world',
        'orientation_wxyz',
        'linear_velocity_body',
        'angular_velocity_body',
    }


def test_named_commands_hold_omitted_targets_and_reject_bad_input():
    """Command updates should be persistent, named, and strictly checked."""
    simulator = MujocoSimulator(settle_steps=0)
    original_targets = simulator.joint_targets
    simulator.step({'Revolute_25': 0.50})

    assert simulator.joint_targets['Revolute_25'] == 0.50
    for name in set(JOINT_NAMES) - {'Revolute_25'}:
        assert simulator.joint_targets[name] == original_targets[name]
    with pytest.raises(ValueError, match='unknown joint'):
        simulator.step({'not_a_joint': 0.0})
    with pytest.raises(ValueError, match='outside'):
        simulator.step({'Revolute_1': 10.0})
    with pytest.raises(ValueError, match='unique'):
        JointCommand(('Revolute_1', 'Revolute_1'), (0.0, 0.1))
