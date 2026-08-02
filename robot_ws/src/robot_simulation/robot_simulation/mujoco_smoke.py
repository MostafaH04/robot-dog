"""Run a short headless determinism check for the MuJoCo core."""

from math import sin

import numpy as np

from robot_simulation.mujoco_core import (
    JOINT_NAMES_BY_LEG,
    MODEL_VARIANTS,
    MujocoSimulator,
    trajectory_fingerprint,
)


def _run_trajectory(model_variant):
    simulator = MujocoSimulator(model_variant=model_variant)
    results = [simulator.reset()]
    for step in range(120):
        offset = 0.02 * sin(2.0 * np.pi * step / 120.0)
        command = {
            joint_name: 0.45 + offset
            for _hip, joint_name, _calf in JOINT_NAMES_BY_LEG
        }
        results.append(simulator.step(command))
    return results


def main():
    """Require repeatable, physically valid signals from every model."""
    for model_variant in MODEL_VARIANTS:
        first = _run_trajectory(model_variant)
        second = _run_trajectory(model_variant)
        first_fingerprint = trajectory_fingerprint(first)
        second_fingerprint = trajectory_fingerprint(second)

        if not np.array_equal(first_fingerprint, second_fingerprint):
            raise RuntimeError(
                f'{model_variant} trajectories were not exactly repeatable'
            )
        if not np.all(np.isfinite(first_fingerprint)):
            raise RuntimeError(
                f'{model_variant} trajectory contains non-finite values'
            )
        contact_samples = [
            contact.normal_force
            for result in first
            for contact in result.sensors.foot_contacts.values()
        ]
        if max(contact_samples) <= 0.0:
            raise RuntimeError(
                f'{model_variant} trajectory never produced foot contact'
            )
    print(
        'MuJoCo smoke test passed for primitive and enhanced models: '
        'fixed-step reset/step is repeatable, mock sensors are finite, '
        'and foot forces are nonzero.'
    )


if __name__ == '__main__':
    main()
