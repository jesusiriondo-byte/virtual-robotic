"""Lanza SOLO el Sorter y su camara cenital para el EXPERIMENTO de base
girada (sesion 2026-09-08) -- abrir antes en Webots
worlds/panda_sorter_base_rotation_test.wbt (misma escena que
panda_sorter_grasp_test.wbt, pero con la base del Sorter girada 26.6 grados
hacia el punto de recogida). Identico a robot_launch_sorter_grasp_test.py,
solo cambia el mundo que hay que tener abierto en Webots.

    ros2 launch panda_controller robot_launch_sorter_base_rotation_test.py

Luego, en otra terminal, con robot_base_yaw:=0.4636 (26.6 grados en
radianes) para que la cinematica sepa que la base esta girada:

    ros2 run panda_controller grasp_yaw_test --ros-args \\
        -p robot_base_yaw:=0.4636
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
