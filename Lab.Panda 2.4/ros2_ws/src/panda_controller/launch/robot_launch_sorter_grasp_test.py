"""Lanza SOLO el Sorter y su camara cenital (worlds/panda_sorter_grasp_test.wbt)
-- mundo de prueba aislado con el cubo ya colocado junto a la pared, sin
Loader ni cinta, para iterar el agarre rapido (sesion 2026-08-27).

    ros2 launch panda_controller robot_launch_sorter_grasp_test.py
"""

import os
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_controller import WebotsController


def generate_launch_description():
    package_dir = get_package_share_directory('panda_controller')
    urdf = os.path.join(package_dir, 'resource', 'panda_webots.urdf')
    cam_sorter_urdf = os.path.join(package_dir, 'resource', 'overhead_camera_sorter.urdf')

    sorter_driver = WebotsController(
        robot_name='Sorter',
        namespace='sorter',
        port='1234',
        parameters=[{'robot_description': urdf}],
    )

    cam_sorter_driver = WebotsController(
        robot_name='OverheadCamSorter',
        port='1234',
        parameters=[{'robot_description': cam_sorter_urdf}],
    )

    return LaunchDescription([
        sorter_driver,
        cam_sorter_driver,
    ])
