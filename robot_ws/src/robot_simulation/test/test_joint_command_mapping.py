"""Validate that semantic command groups follow complete URDF leg chains."""

from pathlib import Path
from xml.etree import ElementTree

from robot_controller.quad_joint_controller import SIM_JOINT_NAMES_BY_LEG


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
