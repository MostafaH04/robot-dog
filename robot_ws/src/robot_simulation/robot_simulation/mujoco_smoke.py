"""Run a short headless determinism check for the MuJoCo core."""

from math import sin

import numpy as np

from robot_simulation.mujoco_core import (
    JOINT_NAMES_BY_LEG,
    MujocoSimulator,
    trajectory_fingerprint,
)


def _run_trajectory():
    simulator = MujocoSimulator()
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
    """Require exact repeatability and physically valid teaching signals."""
    first = _run_trajectory()
    second = _run_trajectory()
    first_fingerprint = trajectory_fingerprint(first)
    second_fingerprint = trajectory_fingerprint(second)

    if not np.array_equal(first_fingerprint, second_fingerprint):
        raise RuntimeError('MuJoCo trajectories were not exactly repeatable')
    if not np.all(np.isfinite(first_fingerprint)):
        raise RuntimeError('MuJoCo trajectory contains non-finite values')
    contact_samples = [
        contact.normal_force
        for result in first
        for contact in result.sensors.foot_contacts.values()
    ]
    if max(contact_samples) <= 0.0:
        raise RuntimeError(
            'MuJoCo smoke trajectory never produced foot contact'
        )
    print(
        'MuJoCo smoke test passed: fixed-step reset/step is repeatable, '
        'mock sensors are finite, and foot forces are nonzero.'
    )


if __name__ == '__main__':
    main()
