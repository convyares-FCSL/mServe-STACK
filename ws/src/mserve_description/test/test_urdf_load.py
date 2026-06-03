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
    lidar_sensor = robot.find("./gazebo[@reference='lidar_link']/sensor[@name='rplidar_c1']")
    camera_sensor = robot.find("./gazebo[@reference='camera_link']/sensor[@name='pi_camera_module_3']")

    assert {
        'base_link',
        'chassis',
        'left_wheel',
        'right_wheel',
        'caster_wheel',
        'camera_link_optical',
        'lidar_riser_link',
        'lidar_link',
        'camera_link',
        'display_link',
        'arm_mount_link',
    }.issubset(links)

    assert joints['left_wheel_joint'] == 'continuous'
    assert joints['right_wheel_joint'] == 'continuous'
    assert joints['chassis_joint'] == 'fixed'
    assert joints['camera_joint'] == 'fixed'
    assert joints['camera_optical_joint'] == 'fixed'
    assert joints['lidar_riser_joint'] == 'fixed'
    assert joints['lidar_joint'] == 'fixed'
    assert camera_sensor is not None
    assert camera_sensor.attrib['type'] == 'camera'
    assert camera_sensor.findtext('topic') == 'camera/image_raw'
    assert lidar_sensor is not None
    assert lidar_sensor.attrib['type'] == 'gpu_lidar'
    assert lidar_sensor.findtext('topic') == 'scan'
