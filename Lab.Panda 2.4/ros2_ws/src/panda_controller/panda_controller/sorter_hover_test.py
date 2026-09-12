#!/usr/bin/env python3
"""Prueba AISLADA solo del Sorter (sesion 2026-08-27): localiza el cubo en
el tope de la cinta con su propia camara cenital (igual que sorter_demo.py),
baja SOLO hasta STOP_Z (30cm sobre la mesa) y se queda ahi QUIETO, sin
girar la muneca, sin bajar mas y sin tocar la pinza.

Se crea para poder entregar el control manual (teleop_gui, remapeado al
namespace 'sorter') justo desde ahi, sin depender de acertar el momento
exacto con una parada de emergencia externa (demasiado dificil de
sincronizar a mano -- varios intentos previos llegaron tarde, con la pinza
ya cerrada).

Al llegar, lanza el PROPIO teleop_gui automaticamente (sesion 2026-08-27, a
peticion del usuario: "quiero el control manual activado cuando pares"),
pasandole la pose articular COMANDADA real como punto de partida -- sin
esto la ventana arrancaria creyendo que el brazo esta en HOME y el primer
"orientar pinza abajo" automatico le haria dar un salto grande e indeseado
nada mas abrirse.

    ros2 run panda_controller sorter_hover_test

Ctrl+C para salir en cualquier momento antes de llegar (no suelta nada
porque no ha cogido nada).
"""

import subprocess
import sys

import rclpy

from panda_controller.cube_shuttle_demo import CubeShuttleDemo, SAFE_Z, HOME_POSITIONS, CUBE_TABLE_Z
from panda_controller.panda_ikpy_kinematics import PandaIkpyKinematics
from panda_controller.overhead_vision import OverheadLocator, COLOR_NAMES
from panda_controller.sorter_demo import (
    SORTER_BASE, SORTER_CAM_TOPIC, SORTER_CAM_X, SORTER_CAM_Y, SORTER_CAM_Z,
    SORTER_TABLE_X_RANGE, SORTER_TABLE_Y_RANGE, PICKUP_X, PICKUP_Y,
    DETECT_RETRIES, DETECT_RETRY_WAIT,
)

WALL_TOP_Z = 0.77   # BELT_END_STOP (sesion 2026-08-28, pared bajada a 3cm): translation.z=0.755 + size.z/2=0.015
# STOP_Z referenciado a la MESA/cinta (CUBE_TABLE_Z=0.77), no a la pared
# (sesion 2026-08-27, a peticion del usuario). 20cm (z=0.97) golpeaba el
# cubo en la propia bajada (aviso real del usuario, sesion 2026-08-28) --
# probado a 30cm (z=1.07), ahora en 25cm (z=1.02) a peticion del usuario.
STOP_Z = CUBE_TABLE_Z + 0.25


class SorterHoverTest(CubeShuttleDemo):
    def __init__(self):
        super().__init__()
        self.kin = PandaIkpyKinematics(base=SORTER_BASE)
        self.cur = self.kin.seed_from_real(self.real_theta)
        self.locator = OverheadLocator(
            self, topic=SORTER_CAM_TOPIC,
            cam_x=SORTER_CAM_X, cam_y=SORTER_CAM_Y, cam_z=SORTER_CAM_Z,
            table_x_range=SORTER_TABLE_X_RANGE, table_y_range=SORTER_TABLE_Y_RANGE)

    def _esperar_color(self):
        for intento in range(1, DETECT_RETRIES + 1):
            color = self.locator.detect_color()
            if color is not None:
                return color
            self.get_logger().warn(
                f'sin cubo detectado (intento {intento}/{DETECT_RETRIES}), '
                f'reintento en {DETECT_RETRY_WAIT:.0f}s...')
            self.spin_for(DETECT_RETRY_WAIT)
        return None

    def run(self):
        color = self._esperar_color()
        x, y = PICKUP_X, PICKUP_Y
        if color is None:
            self.get_logger().warn(
                'No se ha detectado ningun cubo; voy al punto de recogida asumido.')
        else:
            located = self.locator.locate_with_yaw(0.77, color=color)
            if located is not None:
                x, y, yaw_offset = located
                name = COLOR_NAMES.get(color, color)
                self.get_logger().info(
                    f'Cubo {name} localizado en ({x:.4f},{y:.4f}), '
                    f'giro real detectado={__import__("numpy").degrees(yaw_offset):.1f} grados.')
        self.cur = self.kin.seed_from_real(HOME_POSITIONS)
        if not self.ramp(x, y, SAFE_Z, STOP_Z, 0.0, 0.0, 10, 'bajada hasta 20cm sobre la mesa'):
            self.get_logger().error(f'No se pudo llegar ni a STOP_Z={STOP_Z:.2f} (limite articular).')
            return False
        self.get_logger().info(
            f'QUIETO 20cm por encima de la mesa/cinta (z={STOP_Z:.2f}, mesa a {CUBE_TABLE_Z:.2f}, '
            f'pared a {WALL_TOP_Z:.2f}) en ({x:.3f},{y:.3f}). Lanzando control manual...')
        joints_str = '[' + ','.join(f'{v:.6f}' for v in self.real_theta) + ']'
        subprocess.Popen([
            sys.executable, '-m', 'panda_controller.teleop_gui',
            '--ros-args',
            '-r', '/joint_positions:=/sorter/joint_positions',
            '-r', '/gripper_position:=/sorter/gripper_position',
            '-p', f'robot_base_x:={SORTER_BASE[0]}',
            '-p', f'robot_base_y:={SORTER_BASE[1]}',
            '-p', f'robot_base_z:={SORTER_BASE[2]}',
            '-p', f'initial_commanded_joints:={joints_str}',
            '-p', f'camera_topic:={SORTER_CAM_TOPIC}',
            '-p', f'camera_x:={SORTER_CAM_X}',
            '-p', f'camera_y:={SORTER_CAM_Y}',
            '-p', f'camera_z:={SORTER_CAM_Z}',
            '-p', f'camera_table_x_min:={SORTER_TABLE_X_RANGE[0]}',
            '-p', f'camera_table_x_max:={SORTER_TABLE_X_RANGE[1]}',
            '-p', f'camera_table_y_min:={SORTER_TABLE_Y_RANGE[0]}',
            '-p', f'camera_table_y_max:={SORTER_TABLE_Y_RANGE[1]}',
        ])
        self.get_logger().info('Control manual lanzado. Este nodo se cierra.')
        return True


def main(args=None):
    rclpy.init(args=args)
    node = SorterHoverTest()
    node.get_logger().info('Esperando a que el driver se suscriba...')
    if not node.wait_for_subscribers():
        node.get_logger().error(
            f'Nadie se ha suscrito tras {node.max_wait_seconds:.0f}s.')
    else:
        try:
            node.run()
        except KeyboardInterrupt:
            pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
