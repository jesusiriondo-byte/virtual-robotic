import os
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_controller import WebotsController


def generate_launch_description():
    package_dir = get_package_share_directory('panda_controller')

    panda_driver = WebotsController(
        robot_name='Panda',
        port='1234',
        parameters=[
            {'robot_description': os.path.join(package_dir, 'resource', 'panda_webots.urdf')},
        ]
    )

    overhead_cam_driver = WebotsController(
        robot_name='OverheadCam',
        parameters=[
            {'robot_description': os.path.join(package_dir, 'resource', 'overhead_camera.urdf')},
        ]
    )

    return LaunchDescription([
        panda_driver,
        overhead_cam_driver,
    ])
