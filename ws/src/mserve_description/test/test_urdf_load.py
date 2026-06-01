from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET


def test_xacro_expands_to_expected_links_and_joints():
    xacro_path = Path(__file__).resolve().parent.parent / 'urdf' / 'mserve.urdf.xacro'
    assert xacro_path.exists()

    result = subprocess.run(
        ['xacro', str(xacro_path)],
        check=True,
        capture_output=True,
        text=True,
    )
    robot = ET.fromstring(result.stdout)

    assert robot.tag == 'robot'
    assert robot.attrib['name'] == 'mserve_robot'

    links = {link.attrib['name'] for link in robot.findall('link')}
    joints = {joint.attrib['name']: joint.attrib['type'] for joint in robot.findall('joint')}

    assert {
        'base_link',
        'chassis',
        'left_wheel',
        'right_wheel',
        'caster_wheel',
        'lidar_link',
        'camera_link',
        'display_link',
        'arm_mount_link',
    }.issubset(links)

    assert joints['left_wheel_joint'] == 'continuous'
    assert joints['right_wheel_joint'] == 'continuous'
    assert joints['chassis_joint'] == 'fixed'
