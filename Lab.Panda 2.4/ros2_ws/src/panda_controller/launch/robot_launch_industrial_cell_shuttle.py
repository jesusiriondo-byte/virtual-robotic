"""Copia de robot_launch_industrial_cell.py + el controller de la bandeja
del Sorter (sesion 2026-09-06, prueba de integracion completa de
worlds/panda_industrial_cell_shuttle.wbt -- Loader+cinta reales, no el
mundo aislado panda_shuttle_test.wbt). Si tras validar en vivo se decide
promover la bandeja a produccion, este fichero sustituye a
robot_launch_industrial_cell.py (y panda_industrial_cell_shuttle.wbt a
panda_industrial_cell.wbt).

    ros2 launch panda_controller robot_launch_industrial_cell_shuttle.py
"""

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_controller import WebotsController


def generate_launch_description():
    package_dir = get_package_share_directory('panda_controller')
    urdf = os.path.join(package_dir, 'resource', 'panda_webots.urdf')
    cam_loader_urdf = os.path.join(package_dir, 'resource', 'overhead_camera.urdf')
    cam_sorter_urdf = os.path.join(package_dir, 'resource', 'overhead_camera_sorter.urdf')
    warehouse_urdf = os.path.join(package_dir, 'resource', 'warehouse_supervisor.urdf')
    shuttle_urdf = os.path.join(package_dir, 'resource', 'sorter_shuttle_supervisor.urdf')

    loader_driver = WebotsController(
        robot_name='Loader',
        namespace='loader',
        port='1234',
        parameters=[{'robot_description': urdf}],
    )

    sorter_driver = WebotsController(
        robot_name='Sorter',
        namespace='sorter',
        port='1234',
        parameters=[{'robot_description': urdf}],
    )

    cam_loader_driver = WebotsController(
        robot_name='OverheadCamLoader',
        port='1234',
        parameters=[{'robot_description': cam_loader_urdf}],
    )

    cam_sorter_driver = WebotsController(
        robot_name='OverheadCamSorter',
        port='1234',
        parameters=[{'robot_description': cam_sorter_urdf}],
    )

    warehouse_driver = WebotsController(
        robot_name='WarehouseSupervisor',
        port='1234',
        parameters=[{'robot_description': warehouse_urdf}],
    )

    # Bandeja/transbordador del Sorter (sesion 2026-09-06) -- mismo patron
    # que warehouse_driver, sin cuerpo fisico.
    shuttle_driver = WebotsController(
        robot_name='SorterShuttleSupervisor',
        port='1234',
        parameters=[{'robot_description': shuttle_urdf}],
    )

    button_listener_node = Node(
        package='panda_controller',
        executable='button_listener',
    )

    return LaunchDescription([
        loader_driver,
        sorter_driver,
        cam_loader_driver,
        cam_sorter_driver,
        warehouse_driver,
        shuttle_driver,
        button_listener_node,
    ])
