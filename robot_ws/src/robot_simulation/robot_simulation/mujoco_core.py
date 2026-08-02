"""Provide a deterministic, ROS-independent MuJoCo teaching core."""

from dataclasses import dataclass
from pathlib import Path
from types import MappingProxyType
from typing import Mapping, Sequence

import mujoco

import numpy as np


JOINT_NAMES_BY_LEG = (
    ('Revolute_1', 'Revolute_25', 'Revolute_40'),
    ('Revolute_3', 'Revolute_20', 'Revolute_23'),
    ('Revolute_4', 'Revolute_35', 'Revolute_45'),
    ('Revolute_5', 'Revolute_31', 'Revolute_48'),
)
JOINT_NAMES = tuple(name for leg in JOINT_NAMES_BY_LEG for name in leg)
FOOT_NAMES = ('front_left', 'front_right', 'rear_left', 'rear_right')
SAFE_STANCE_POSITIONS = (0.0, 0.45, -0.90) * 4
MODEL_PATHS = MappingProxyType({
    'primitive': (
        Path(__file__).with_name('models') / 'minimal_quadruped.xml'
    ),
    'enhanced': (
        Path(__file__).with_name('models') / 'enhanced_quadruped.xml'
    ),
})
MODEL_VARIANTS = tuple(MODEL_PATHS)
DEFAULT_MODEL_PATH = MODEL_PATHS['primitive']


def _readonly(values):
    """Copy numeric values into a read-only float array."""
    result = np.asarray(values, dtype=np.float64).copy()
    result.setflags(write=False)
    return result


@dataclass(frozen=True)
class JointCommand:
    """Named position targets; omitted joints hold their last target."""

    names: tuple[str, ...]
    positions: tuple[float, ...]

    def __post_init__(self):
        """Reject ambiguous or numerically invalid commands."""
        if len(self.names) != len(self.positions):
            raise ValueError(
                'joint command names and positions must have equal lengths'
            )
        if len(set(self.names)) != len(self.names):
            raise ValueError('joint command names must be unique')
        if not np.all(np.isfinite(self.positions)):
            raise ValueError('joint command positions must be finite')

    @classmethod
    def from_mapping(cls, positions: Mapping[str, float]):
        """Construct a command while retaining the mapping iteration order."""
        return cls(
            tuple(positions),
            tuple(float(value) for value in positions.values()),
        )


@dataclass(frozen=True)
class ImuSample:
    """Noise-free mock IMU data in the base frame."""

    orientation_wxyz: np.ndarray
    angular_velocity_body: np.ndarray
    specific_force_body: np.ndarray


@dataclass(frozen=True)
class JointStateSample:
    """Noise-free simulated joint state in canonical command order."""

    names: tuple[str, ...]
    positions: np.ndarray
    velocities: np.ndarray
    efforts: np.ndarray


@dataclass(frozen=True)
class FootContactSample:
    """Idealized contact and simulated wrench about a foot site."""

    in_contact: bool
    normal_force: float
    force_foot: np.ndarray
    torque_foot: np.ndarray


@dataclass(frozen=True)
class SensorData:
    """Experiment inputs that do not include simulator ground truth."""

    imu: ImuSample
    joint_states: JointStateSample
    foot_contacts: Mapping[str, FootContactSample]


@dataclass(frozen=True)
class GroundTruth:
    """Exact simulator base state for evaluation, never an estimator input."""

    position_world: np.ndarray
    orientation_wxyz: np.ndarray
    linear_velocity_body: np.ndarray
    angular_velocity_body: np.ndarray


@dataclass(frozen=True)
class StepResult:
    """One timestamped result with sensors and evaluation truth separated."""

    time: float
    sensors: SensorData
    ground_truth: GroundTruth


class MujocoSimulator:
    """Own one deterministic MuJoCo model/data pair with fixed-step control."""

    def __init__(
        self,
        model_path=None,
        settle_steps=250,
        model_variant='primitive',
    ):
        """Load and validate one model, then enter its safe stance."""
        if settle_steps < 0:
            raise ValueError('settle_steps must be non-negative')
        if model_variant not in MODEL_VARIANTS:
            raise ValueError(
                f'unknown model variant {model_variant!r}; '
                f'expected one of {MODEL_VARIANTS}'
            )
        if model_path is not None and model_variant != 'primitive':
            raise ValueError(
                'model_path and a non-default model_variant are mutually '
                'exclusive'
            )
        path = Path(model_path) if model_path is not None else MODEL_PATHS[
            model_variant
        ]
        self.model_variant = (
            'custom' if model_path is not None else model_variant
        )
        self.model_path = path
        self.model = mujoco.MjModel.from_xml_path(str(path))
        self.data = mujoco.MjData(self.model)
        self.settle_steps = settle_steps
        self._validate_model()
        self._joint_ids = {
            name: self.model.joint(name).id for name in JOINT_NAMES
        }
        self._actuator_ids = {
            name: self.model.actuator(f'act_{name}').id for name in JOINT_NAMES
        }
        self._foot_geom_ids = {
            foot: self.model.geom(f'{foot}_foot_geom').id
            for foot in FOOT_NAMES
        }
        self._foot_site_ids = {
            foot: self.model.site(f'{foot}_foot_site').id
            for foot in FOOT_NAMES
        }
        self._ground_geom_id = self.model.geom('ground').id
        self._base_body_id = self.model.body('base').id
        self._targets = np.asarray(SAFE_STANCE_POSITIONS, dtype=np.float64)
        self.reset()

    @property
    def timestep(self):
        """Return the immutable physics step size in seconds."""
        return float(self.model.opt.timestep)

    @property
    def joint_targets(self):
        """Return a read-only copy of the persistent named position targets."""
        return MappingProxyType({
            name: float(target)
            for name, target in zip(JOINT_NAMES, self._targets)
        })

    @staticmethod
    def safe_stance_command():
        """Return the conservative full-joint command used during reset."""
        return JointCommand(JOINT_NAMES, SAFE_STANCE_POSITIONS)

    def reset(self):
        """Return the same settled safe stance on every call."""
        mujoco.mj_resetDataKeyframe(self.model, self.data, 0)
        self._targets = np.asarray(SAFE_STANCE_POSITIONS, dtype=np.float64)
        self._write_controls()
        mujoco.mj_forward(self.model, self.data)
        for _ in range(self.settle_steps):
            mujoco.mj_step(self.model, self.data)
        self.data.time = 0.0
        mujoco.mj_forward(self.model, self.data)
        return self.observe()

    def step(self, command=None):
        """Apply an optional command and advance exactly one model tick."""
        if command is not None:
            if isinstance(command, Mapping):
                command = JointCommand.from_mapping(command)
            if not isinstance(command, JointCommand):
                raise TypeError(
                    'command must be JointCommand, a mapping, or None'
                )
            self._apply_command(command)
        self._write_controls()
        mujoco.mj_step(self.model, self.data)
        return self.observe()

    def observe(self):
        """Copy the current state into explicit sensor and truth containers."""
        orientation = _readonly(self.data.xquat[self._base_body_id])
        rotation_world_from_body = np.asarray(
            self.data.xmat[self._base_body_id]
        ).reshape(3, 3)
        base_velocity = np.asarray(self.data.qvel[:6])
        linear_velocity_body = rotation_world_from_body.T @ base_velocity[:3]
        angular_velocity_body = base_velocity[3:]

        joint_positions = []
        joint_velocities = []
        joint_efforts = []
        for name in JOINT_NAMES:
            joint_id = self._joint_ids[name]
            actuator_id = self._actuator_ids[name]
            position_address = self.model.jnt_qposadr[joint_id]
            velocity_address = self.model.jnt_dofadr[joint_id]
            joint_positions.append(self.data.qpos[position_address])
            joint_velocities.append(self.data.qvel[velocity_address])
            joint_efforts.append(self.data.actuator_force[actuator_id])

        contacts = {
            foot: self._foot_contact(foot)
            for foot in FOOT_NAMES
        }
        sensors = SensorData(
            imu=ImuSample(
                orientation_wxyz=_readonly(orientation),
                angular_velocity_body=_readonly(
                    self.data.sensor('imu_gyro').data
                ),
                specific_force_body=_readonly(
                    self.data.sensor('imu_specific_force').data
                ),
            ),
            joint_states=JointStateSample(
                names=JOINT_NAMES,
                positions=_readonly(joint_positions),
                velocities=_readonly(joint_velocities),
                efforts=_readonly(joint_efforts),
            ),
            foot_contacts=MappingProxyType(contacts),
        )
        truth = GroundTruth(
            position_world=_readonly(self.data.xpos[self._base_body_id]),
            orientation_wxyz=orientation,
            linear_velocity_body=_readonly(linear_velocity_body),
            angular_velocity_body=_readonly(angular_velocity_body),
        )
        return StepResult(float(self.data.time), sensors, truth)

    def _validate_model(self):
        names = {
            self.model.joint(index).name
            for index in range(self.model.njnt)
        }
        missing = set(JOINT_NAMES) - names
        if missing:
            raise ValueError(
                f'MuJoCo model is missing command joints: {sorted(missing)}'
            )
        if self.model.nkey < 1:
            raise ValueError('MuJoCo model must define a safe_stance keyframe')

    def _apply_command(self, command):
        target_by_name = dict(zip(command.names, command.positions))
        unknown = set(target_by_name) - set(JOINT_NAMES)
        if unknown:
            raise ValueError(f'unknown joint command names: {sorted(unknown)}')

        validated_targets = []
        for name, value in target_by_name.items():
            actuator_id = self._actuator_ids[name]
            low, high = self.model.actuator_ctrlrange[actuator_id]
            if not low <= value <= high:
                raise ValueError(
                    f'joint target {name}={value} outside [{low}, {high}]'
                )
            validated_targets.append((JOINT_NAMES.index(name), value))

        for target_index, value in validated_targets:
            self._targets[target_index] = value

    def _write_controls(self):
        for target, name in zip(self._targets, JOINT_NAMES):
            self.data.ctrl[self._actuator_ids[name]] = target

    def _foot_contact(self, foot):
        foot_geom_id = self._foot_geom_ids[foot]
        site_id = self._foot_site_ids[foot]
        site_position_world = np.asarray(self.data.site_xpos[site_id])
        rotation_world_from_foot = np.asarray(
            self.data.site_xmat[site_id]
        ).reshape(3, 3)
        force_world = np.zeros(3)
        torque_world = np.zeros(3)
        normal_force = 0.0

        for contact_id in range(self.data.ncon):
            contact = self.data.contact[contact_id]
            geom_ids = (contact.geom[0], contact.geom[1])
            if (
                foot_geom_id not in geom_ids
                or self._ground_geom_id not in geom_ids
            ):
                continue
            wrench_contact = np.zeros(6)
            mujoco.mj_contactForce(
                self.model,
                self.data,
                contact_id,
                wrench_contact,
            )
            rotation_world_from_contact = np.asarray(
                contact.frame
            ).reshape(3, 3).T
            sign_for_foot = 1.0 if contact.geom[1] == foot_geom_id else -1.0
            contact_force_world = (
                sign_for_foot
                * rotation_world_from_contact
                @ wrench_contact[:3]
            )
            contact_torque_world = (
                sign_for_foot
                * rotation_world_from_contact
                @ wrench_contact[3:]
            )
            force_world += contact_force_world
            torque_world += contact_torque_world + np.cross(
                np.asarray(contact.pos) - site_position_world,
                contact_force_world,
            )
            normal_force += max(0.0, float(wrench_contact[0]))

        return FootContactSample(
            in_contact=normal_force > 1e-9,
            normal_force=normal_force,
            force_foot=_readonly(rotation_world_from_foot.T @ force_world),
            torque_foot=_readonly(rotation_world_from_foot.T @ torque_world),
        )


def trajectory_fingerprint(results: Sequence[StepResult]):
    """Flatten stable public fields for deterministic smoke comparisons."""
    values = []
    for result in results:
        values.append(result.time)
        values.extend(result.sensors.imu.orientation_wxyz)
        values.extend(result.sensors.imu.angular_velocity_body)
        values.extend(result.sensors.imu.specific_force_body)
        values.extend(result.sensors.joint_states.positions)
        values.extend(result.sensors.joint_states.velocities)
        values.extend(result.sensors.joint_states.efforts)
        for foot in FOOT_NAMES:
            contact = result.sensors.foot_contacts[foot]
            values.extend((float(contact.in_contact), contact.normal_force))
            values.extend(contact.force_foot)
            values.extend(contact.torque_foot)
        values.extend(result.ground_truth.position_world)
        values.extend(result.ground_truth.orientation_wxyz)
        values.extend(result.ground_truth.linear_velocity_body)
        values.extend(result.ground_truth.angular_velocity_body)
    return np.asarray(values, dtype=np.float64)
