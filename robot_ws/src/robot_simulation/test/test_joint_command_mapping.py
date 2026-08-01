"""Validate simulation command, time, and frame invariants."""

from pathlib import Path
from xml.etree import ElementTree

from robot_controller.quad_joint_controller import SIM_JOINT_NAMES_BY_LEG
from robot_simulation.pybullet_sim import (
    GROUND_TRUTH_BASE_FRAME,
    GROUND_TRUTH_WORLD_FRAME,
    JOINT_DIRECTION_BY_NAME,
    simulation_timestamp,
)


ROBOT_DESCRIPTION = Path(__file__).parents[1] / 'urdf' / 'robot_core.xacro'


def test_each_command_group_is_a_base_to_foot_chain():
    root = ElementTree.parse(ROBOT_DESCRIPTION).getroot()
    joints = {
        joint.attrib['name']: (
            joint.find('parent').attrib['link'],
            joint.find('child').attrib['link'],
        )
        for joint in root.findall('joint')
    }

    commanded_names = [name for leg in SIM_JOINT_NAMES_BY_LEG for name in leg]
    assert len(commanded_names) == 12
    assert len(set(commanded_names)) == 12

    foot_links = set()
    for hip_name, upper_name, foot_name in SIM_JOINT_NAMES_BY_LEG:
        hip_parent, hip_child = joints[hip_name]
        upper_parent, upper_child = joints[upper_name]
        foot_parent, foot_child = joints[foot_name]
        assert hip_parent == 'base_link'
        assert hip_child == upper_parent
        assert upper_child == foot_parent
        foot_links.add(foot_child)

    assert len(foot_links) == 4


def test_joint_directions_are_semantic_and_cover_the_urdf():
    root = ElementTree.parse(ROBOT_DESCRIPTION).getroot()
    urdf_joint_names = {
        joint.attrib['name']
        for joint in root.findall('joint')
    }

    assert set(JOINT_DIRECTION_BY_NAME) == urdf_joint_names
    assert set(JOINT_DIRECTION_BY_NAME.values()) == {-1.0, 1.0}
    assert {
        name
        for name, direction in JOINT_DIRECTION_BY_NAME.items()
        if direction < 0.0
    } == {
        'Revolute_3',
        'Revolute_5',
        'Revolute_20',
        'Revolute_35',
        'Revolute_40',
    }


def test_simulation_timestamps_are_exact_step_multiples():
    assert simulation_timestamp(0).sec == 0
    assert simulation_timestamp(0).nanosec == 0
    assert simulation_timestamp(1).nanosec == 10_000_000

    timestamp = simulation_timestamp(123)
    assert timestamp.sec == 1
    assert timestamp.nanosec == 230_000_000


def test_ground_truth_frames_are_distinct_from_operational_frames():
    assert GROUND_TRUTH_WORLD_FRAME != 'world'
    assert GROUND_TRUTH_BASE_FRAME != 'base_link'
