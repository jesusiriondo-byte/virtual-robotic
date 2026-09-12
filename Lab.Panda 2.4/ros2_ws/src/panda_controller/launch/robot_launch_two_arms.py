"""Prueba de humo del cableado multi-robot (sesion 2026-08-27, celda
industrial de dos robots, Fase 1 del plan).

NO toca robot_launch.py (el de siempre, para las demos de un solo robot) --
este launch es exclusivo para 'worlds/panda_two_arms_smoke_test.wbt', que
tiene un segundo Panda ("Panda2"). Los dos usan el MISMO panda_webots.urdf
(no hace falta duplicarlo): my_robot_driver.py namespacea su propio
nodo/topics leyendo el nombre real del robot en Webots
(self.__robot.getName()), no el namespace= de aqui abajo -- ese namespace=
solo namespacea el nodo NATIVO del driver y el bridge de camara, nunca le
llega al plugin Python embebido (comprobado en vivo, sesion 2026-08-27:
sys.argv y 'properties' llegan vacios dentro del plugin pase lo que pase).
Se deja namespace= igual que en robot_launch.py por si acaso el bridge de
camara si lo necesita (ya confirmado que si: /loader/panda_camera/...).

Uso: apuntar temporalmente docker-compose.yml al mundo de prueba, luego:
    ros2 launch panda_controller robot_launch_two_arms.py

Verificacion: 'ros2 topic list' debe mostrar /loader/joint_positions y
/sorter/joint_positions (NINGUN /joint_positions a secas), y un
'ros2 topic pub' a cada uno debe mover solo su brazo en Webots.
"""

import os
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_controller import WebotsController


def generate_launch_description():
    package_dir = get_package_share_directory('panda_controller')
    urdf = os.path.join(package_dir, 'resource', 'panda_webots.urdf')

    loader_driver = WebotsController(
        robot_name='Panda',
        namespace='loader',
        port='1234',
        parameters=[{'robot_description': urdf}],
    )

    sorter_driver = WebotsController(
        robot_name='Panda2',
        namespace='sorter',
        # MISMO puerto que el loader (sesion 2026-08-27, hallazgo real): un
        # Webots corriendo escucha en UN solo puerto para todos sus
        # controladores externos, y distingue el robot por 'robot_name' en
        # el handshake, no por puerto. Con port='1235' aqui, este
        # controlador se quedaba reintentando conectar para siempre porque
        # Webots nunca escuchaba en ese puerto (confirmado en el log de
        # Webots: "Waiting for ... connection on port 1234 targeting robot
        # named 'Panda2'").
        port='1234',
        parameters=[{'robot_description': urdf}],
    )

    return LaunchDescription([
        loader_driver,
        sorter_driver,
    ])
