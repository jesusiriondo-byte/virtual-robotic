"""Lanza los dos brazos de la celda industrial (worlds/panda_industrial_cell.wbt):
Loader y Sorter, cada uno con su propio namespace ROS2 (ver my_robot_driver.py
-- namespacea via self.__robot.getName(), no via el namespace= de aqui, que
solo afecta al nodo nativo del driver y al bridge de camara).

    ros2 launch panda_controller robot_launch_industrial_cell.py
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

    # Camaras cenitales: mismo topicName absoluto de siempre
    # (/overhead_camera, /overhead_camera_sorter) -- no van namespaceadas
    # por robot porque cada una ya tiene un nombre de topic distinto (ver
    # los .urdf), no hace falta el truco de self.__robot.getName() que si
    # necesitan los brazos (my_robot_driver.py).
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

    # Almacen reciclador (sesion 2026-08-28): sin cuerpo fisico, solo
    # necesita el protocolo extern para llegar a warehouse_supervisor_driver.py.
    warehouse_driver = WebotsController(
        robot_name='WarehouseSupervisor',
        port='1234',
        parameters=[{'robot_description': warehouse_urdf}],
    )

    # Bandeja/transbordador del punto de recogida del Sorter (prototipo de la
    # sesion 2026-09-06, promovido a produccion el 2026-09-10): encaja en pose
    # exacta el cubo que llega descentrado, que era el cuello de botella real
    # -- medido en un lote de 15 min sin bandeja, 62 rechazos por "no bajo"
    # contra 15 clasificados (1,01/min frente a los ~2,0 de referencia).
    # Sin cuerpo fisico, mismo patron que warehouse_driver.
    shuttle_driver = WebotsController(
        robot_name='SorterShuttleSupervisor',
        port='1234',
        parameters=[{'robot_description': shuttle_urdf}],
    )

    # Puente del boton fisico de parada de emergencia de la Pico W (sesion
    # 2026-08-31, bug real: el usuario reporto que el boton "no funciona"
    # -- no era la Pico ni la red, era que este nodo casi nunca se lanzaba
    # a mano por ser un paso aparte, facil de olvidar, en LANZAR_PROYECTO.md.
    # Un nodo puente de seguridad no deberia depender de que alguien se
    # acuerde de escribir el comando -- va aqui, arranca siempre con el
    # resto de la celda. No es un WebotsController (no habla con Webots,
    # solo escucha TCP del puerto 5002 y publica /emergency_stop), un
    # Node normal de ROS2 basta.
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
