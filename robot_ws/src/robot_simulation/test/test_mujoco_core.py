"""Exercise the ROS-independent deterministic MuJoCo core."""

from dataclasses import fields
import xml.etree.ElementTree as ET

import mujoco

import numpy as np

import pytest

from robot_simulation.mujoco_core import (
    FOOT_NAMES,
    GroundTruth,
    JOINT_NAMES,
    JointCommand,
    MODEL_PATHS,
    MODEL_VARIANTS,
    MujocoSimulator,
    SensorData,
    trajectory_fingerprint,
)


@pytest.mark.parametrize('model_variant', MODEL_VARIANTS)
def test_reset_and_fixed_trajectory_are_exactly_repeatable(model_variant):
    """Reset and replay should produce exactly equal public data."""
    simulator = MujocoSimulator(
        settle_steps=80,
        model_variant=model_variant,
    )
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


@pytest.mark.parametrize('model_variant', MODEL_VARIANTS)
def test_safe_stance_exposes_expected_teaching_interface(model_variant):
    """The core should expose mock sensors without embedded truth."""
    simulator = MujocoSimulator(
        settle_steps=120,
        model_variant=model_variant,
    )
    result = simulator.reset()

    assert result.sensors.joint_states.names == JOINT_NAMES
    assert tuple(result.sensors.foot_contacts) == FOOT_NAMES
    assert all(
        contact.normal_force >= 0.0
        for contact in result.sensors.foot_contacts.values()
    )
    assert np.all(np.isfinite(result.sensors.imu.specific_force_body))
    assert result.time == 0.0
    assert simulator.step().time == pytest.approx(simulator.timestep)
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


def test_rejected_multi_joint_command_does_not_change_next_step():
    """A late validation failure must not leave an earlier target applied."""
    simulator = MujocoSimulator(settle_steps=0)
    reference = MujocoSimulator(settle_steps=0)
    original_targets = simulator.joint_targets

    with pytest.raises(ValueError, match='outside'):
        simulator.step({
            'Revolute_25': 0.50,
            'Revolute_31': 10.0,
        })

    assert simulator.joint_targets == original_targets
    assert np.array_equal(
        trajectory_fingerprint([simulator.step()]),
        trajectory_fingerprint([reference.step()]),
    )


def test_rotated_base_angular_velocity_matches_gyro_body_frame():
    """Free-joint angular qvel is already local, like the gyro reading."""
    simulator = MujocoSimulator(settle_steps=0)
    half_angle = np.pi / 4.0
    simulator.data.qpos[3:7] = (
        np.cos(half_angle),
        0.0,
        0.0,
        np.sin(half_angle),
    )
    simulator.data.qvel[3:6] = (0.3, -0.4, 0.5)
    mujoco.mj_forward(simulator.model, simulator.data)

    result = simulator.observe()

    assert np.allclose(
        result.ground_truth.angular_velocity_body,
        result.sensors.imu.angular_velocity_body,
        rtol=0.0,
        atol=1e-12,
    )


def test_model_variants_are_explicit_and_primitive_remains_default():
    """Selection should be explicit without breaking custom model loading."""
    default = MujocoSimulator(settle_steps=0)
    enhanced = MujocoSimulator(
        settle_steps=0,
        model_variant='enhanced',
    )
    custom = MujocoSimulator(
        model_path=MODEL_PATHS['primitive'],
        settle_steps=0,
    )

    assert MODEL_VARIANTS == ('primitive', 'enhanced')
    assert default.model_variant == 'primitive'
    assert default.model_path == MODEL_PATHS['primitive']
    assert enhanced.model_variant == 'enhanced'
    assert custom.model_variant == 'custom'
    with pytest.raises(ValueError, match='unknown model variant'):
        MujocoSimulator(model_variant='digital_twin')
    with pytest.raises(ValueError, match='mutually exclusive'):
        MujocoSimulator(
            model_path=MODEL_PATHS['primitive'],
            model_variant='enhanced',
        )


def test_enhanced_model_uses_exported_mass_and_geometry_invariants():
    """Repository-backed fidelity changes should remain machine-checkable."""
    primitive = MujocoSimulator(settle_steps=0)
    enhanced = MujocoSimulator(
        settle_steps=0,
        model_variant='enhanced',
    )

    assert float(primitive.model.body('base').mass[0]) == pytest.approx(6.0)
    assert float(enhanced.model.body('base').mass[0]) == pytest.approx(
        10.123728045116188,
    )
    source_position = np.array(
        (-0.008382264, -0.000014798, 0.030113658),
    )
    source_inertia = np.array((
        (0.03254, -0.000031, 0.001839),
        (-0.000031, 0.280159, -0.000001),
        (0.001839, -0.000001, 0.3037),
    ))
    reflect_y = np.diag((1.0, -1.0, 1.0))
    expected_position = reflect_y @ source_position
    expected_inertia = reflect_y @ source_inertia @ reflect_y

    xml_root = ET.parse(MODEL_PATHS['enhanced']).getroot()
    base_inertial = xml_root.find(
        "./worldbody/body[@name='base']/inertial",
    )
    assert base_inertial is not None
    xml_position = np.fromstring(base_inertial.attrib['pos'], sep=' ')
    full_inertia = np.fromstring(
        base_inertial.attrib['fullinertia'],
        sep=' ',
    )
    xml_inertia = np.array((
        (full_inertia[0], full_inertia[3], full_inertia[4]),
        (full_inertia[3], full_inertia[1], full_inertia[5]),
        (full_inertia[4], full_inertia[5], full_inertia[2]),
    ))

    assert np.allclose(xml_position, expected_position, rtol=0.0, atol=1e-9)
    assert np.allclose(xml_inertia, expected_inertia, rtol=0.0, atol=1e-12)
    assert np.allclose(
        enhanced.model.body('base').ipos,
        expected_position,
        rtol=0.0,
        atol=1e-9,
    )
    assert np.allclose(
        enhanced.model.body('front_right_hip').pos,
        (0.1184, -0.076, 0.0196),
        rtol=0.0,
        atol=1e-12,
    )
    assert enhanced.model.geom('front_right_foot_geom').type[0] == (
        mujoco.mjtGeom.mjGEOM_CAPSULE
    )
    assert primitive.model.geom('front_right_foot_geom').type[0] == (
        mujoco.mjtGeom.mjGEOM_SPHERE
    )
    assert sum(enhanced.model.body_mass) > sum(primitive.model.body_mass)


@pytest.mark.parametrize('model_variant', MODEL_VARIANTS)
def test_variants_preserve_joint_actuator_and_contact_names(model_variant):
    """A model swap must not change any public addressing semantics."""
    simulator = MujocoSimulator(
        settle_steps=80,
        model_variant=model_variant,
    )

    for name in JOINT_NAMES:
        actuator = simulator.model.actuator(f'act_{name}')
        assert actuator.id >= 0
        assert np.array_equal(
            actuator.ctrlrange,
            simulator.model.joint(name).range,
        )
    assert tuple(simulator.observe().sensors.foot_contacts) == FOOT_NAMES
    assert max(
        contact.normal_force
        for contact in simulator.observe().sensors.foot_contacts.values()
    ) > 0.0
