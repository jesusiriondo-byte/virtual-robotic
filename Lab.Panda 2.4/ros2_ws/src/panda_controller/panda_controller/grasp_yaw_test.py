#!/usr/bin/env python3
"""Prueba AISLADA del agarre real del Sorter (sesion 2026-08-27, version 2):
localiza el cubo con la propia camara del Sorter (posicion + giro real,
igual que sorter_demo.py), aplica el MISMO ajuste de giro por colision con
la pared (SorterDemo._adjust_grasp_yaw_for_obstacles) y clasifica los 3
cubos de panda_sorter_base_rotation_test.wbt en sus cajas (sesion
2026-09-08: ya no se queda en un solo agarre + desplazamiento de prueba,
ver run()).

    ros2 run panda_controller grasp_yaw_test

Ctrl+C para soltar la pinza y salir en cualquier momento.
"""

import rclpy

from panda_controller.cube_shuttle_demo import CubeShuttleDemo
from panda_controller.panda_ikpy_kinematics import PandaIkpyKinematics
from panda_controller.overhead_vision import OverheadLocator, COLOR_NAMES
from panda_controller.sorter_demo import (
    SORTER_BASE, SORTER_CAM_TOPIC, SORTER_CAM_X, SORTER_CAM_Y, SORTER_CAM_Z,
    SORTER_TABLE_X_RANGE, SORTER_TABLE_Y_RANGE, PICKUP_X, PICKUP_Y, BOXES,
    DETECT_RETRIES, DETECT_RETRY_WAIT, SorterDemo,
)


class GraspYawTest(CubeShuttleDemo):
    def __init__(self):
        super().__init__()
        # base_yaw (sesion 2026-09-08): 0.0 salvo que se pase por ROS param
        # robot_base_yaw -- ver panda_ikpy_kinematics.PandaIkpyKinematics.
        base_yaw = float(self.get_parameter('robot_base_yaw').value)
        self.kin = PandaIkpyKinematics(base=SORTER_BASE, base_yaw=base_yaw)
        # Girar la muneca ANTES de bajar, con margen extra sobre HOVER_HIGH
        # (sesion 2026-09-08, aviso real del usuario mirando Webots: "sigues
        # girando antes de cerrar la pinza y mueve los cubos... con cinco cm
        # mas alto"). Probado con 5cm (3/3 clasificados sin fallos), bajado
        # a 3cm despues a peticion del usuario para ganar velocidad -- si
        # esto vuelve a rozar cubos vecinos, subir otra vez a 0.05.
        # SOLO aqui, en este test AISLADO sin cinta -- sorter_demo.py
        # se queda en False a proposito (rotate_before_descend=True se probo
        # dos veces en produccion, sesion 2026-09-04, y fallo por una causa
        # nunca identificada; ver el historial en sorter_demo.py). Sin cinta
        # de por medio en este mundo, es la ocasion de probarlo con cuidado.
        self.rotate_before_descend = True
        self.rotate_clearance_extra = 0.03
        self.cur = self.kin.seed_from_real(self.real_theta)
        self.locator = OverheadLocator(
            self, topic=SORTER_CAM_TOPIC,
            cam_x=SORTER_CAM_X, cam_y=SORTER_CAM_Y, cam_z=SORTER_CAM_Z,
            table_x_range=SORTER_TABLE_X_RANGE, table_y_range=SORTER_TABLE_Y_RANGE)
        # Reutilizar el ajuste de colision del Sorter tal cual (no
        # reimplementarlo aqui) -- se llama sin instanciar SorterDemo entero
        # via el metodo "unbound", pasandole self (que ya tiene
        # current_grasp_yaw y get_logger, todo lo que necesita).
        self._adjust_grasp_yaw_for_obstacles = SorterDemo._adjust_grasp_yaw_for_obstacles.__get__(self)
        self._adjust_grasp_position_for_obstacles = SorterDemo._adjust_grasp_position_for_obstacles.__get__(self)
        # EXPERIMENTO (sesion 2026-08-28): ya no se fuerza grasp_z ni
        # base_grasp_yaw propios -- se dejan los valores por defecto de
        # CubeShuttleDemo (HOVER_GRASP, YAW), los mismos que usa
        # loader_demo.py sin fallos. Ver sorter_demo.py para el porque.

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
        """CICLO COMPLETO con los 3 cubos (sesion 2026-09-08, a peticion
        real del usuario tras validar el agarre suelto: 'termina el ciclo
        con los tres') -- ya no se queda en un solo agarre + desplazamiento
        de prueba, clasifica los 3 cubos de panda_sorter_base_rotation_test.wbt
        en sus cajas, igual que hace SorterDemo.run() en produccion pero
        acotado a 3 intentos (aqui no hay cinta reponiendo cubos). Reutiliza
        leg() (deteccion + agarre + deposito + reintentos), el mismo metodo
        de alto nivel que sorter_demo.py, para no duplicar esa logica ya
        validada."""
        self.park_at_home()
        clasificados = 0
        for _ in range(3):
            color = self._esperar_color()
            if color is None:
                self.get_logger().warn('No se ha detectado ningun cubo mas -- termino el ciclo.')
                break
            name = COLOR_NAMES.get(color, color)
            dest = BOXES.get(color, BOXES['R'])
            self.get_logger().info(f'--- Cubo {name} detectado -> caja {name} {dest} ---')
            if not self.leg((PICKUP_X, PICKUP_Y), dest, f'clasificar_{color}',
                             color=color, place_yaw=self.base_grasp_yaw):
                self.get_logger().error(
                    f'[{name}] fallo recogiendo/clasificando tras agotar reintentos -- '
                    f'{clasificados} clasificado(s) hasta ahora, sigo con el siguiente.')
                self.park_at_home()
                continue
            clasificados += 1
            self.get_logger().info(f'CUBO {name} CLASIFICADO. Total: {clasificados}/3.')
            self.park_at_home()
        self.get_logger().info(f'Ciclo de prueba terminado: {clasificados}/3 clasificados.')
        return clasificados == 3


def main(args=None):
    rclpy.init(args=args)
    node = GraspYawTest()
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
