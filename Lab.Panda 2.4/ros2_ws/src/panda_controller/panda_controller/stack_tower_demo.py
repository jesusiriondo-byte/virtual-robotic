#!/usr/bin/env python3
"""Apila una torre de cubos, de abajo a arriba, reusando leg()/lift_shift_place()
de CubeShuttleDemo con el parametro place_z (sesion 2026-08-26).

Antes esto se hacia con scripts sueltos fuera del paquete (uno por par de
cubos). Localiza cada cubo por vision cenital justo antes de moverlo -- el
de abajo de cada par no en su posicion de diseno, sino donde de verdad haya
quedado tras el paso anterior (los cubos SI se desplazan un poco al recibir
uno encima, ver resumen_proyecto_panda.md).

Necesita 'ros2 launch panda_controller robot_launch.py' ya corriendo, y que
el primer color de 'order' (la base) este realmente apoyado en la mesa.

    ros2 run panda_controller stack_tower_demo
    ros2 run panda_controller stack_tower_demo --ros-args -p order:="['R','G','B']"
"""

import rclpy

from panda_controller.cube_shuttle_demo import CubeShuttleDemo, CUBE_TABLE_Z, CUBE_VISION_Z

CUBE_SIZE = 0.06
TCP_TO_CUBE_CENTER = 0.10  # mismo offset que HOVER_GRASP = CUBE_TABLE_Z + 0.10


class StackTowerDemo(CubeShuttleDemo):
    def __init__(self):
        super().__init__()
        self.declare_parameter('order', ['R', 'G', 'B'])
        self.order = list(self.get_parameter('order').value)

    def run(self):
        if len(self.order) < 2:
            self.get_logger().error('order necesita al menos 2 colores.')
            return False
        base_color = self.order[0]
        base_center_z = CUBE_TABLE_Z
        base_pos = self.locator.locate(base_center_z, color=base_color)
        if base_pos is None:
            self.get_logger().error(f'No localizo la base ({base_color}).')
            return False
        self.get_logger().info(
            f'Base de la torre: {base_color} en {tuple(round(v, 4) for v in base_pos)}')

        for level, color in enumerate(self.order[1:], start=1):
            src_pos = self.locator.locate(CUBE_VISION_Z, color=color)
            if src_pos is None:
                self.get_logger().error(f'No localizo el cubo {color} (nivel {level}); abortando.')
                return False
            dest_center_z = base_center_z + level * CUBE_SIZE
            place_z = dest_center_z + TCP_TO_CUBE_CENTER
            self.get_logger().info(
                f'--- Nivel {level}: {color} sobre {base_color} '
                f'(coger en {tuple(round(v, 4) for v in src_pos)}, '
                f'dejar en {tuple(round(v, 4) for v in base_pos)}, place_z={place_z:.3f}) ---')
            if not self.leg(src_pos, base_pos, f'apilar_{color}_nivel{level}',
                             color=color, place_z=place_z):
                self.get_logger().error(f'Abortando torre en el nivel {level} ({color}).')
                return False
        self.get_logger().info(f'TORRE COMPLETA: {"-".join(self.order)} (de abajo a arriba).')
        return True


def main(args=None):
    rclpy.init(args=args)
    node = StackTowerDemo()
    node.get_logger().info('Esperando a que el driver se suscriba...')
    if not node.wait_for_subscribers():
        node.get_logger().error(
            f'Nadie se ha suscrito tras {node.max_wait_seconds:.0f}s '
            "(revisa que 'ros2 launch panda_controller robot_launch.py' este corriendo).")
    else:
        node.run()
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
