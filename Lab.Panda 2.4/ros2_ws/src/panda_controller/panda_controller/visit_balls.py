#!/usr/bin/env python3
"""
Nodo ROS2 que hace un recorrido por los 3 cubos: para cada uno, el TCP se
coloca encima (hover), BAJA hasta el cubo, se queda ahi parado un par de
segundos, SUBE de vuelta al hover, y pasa al siguiente. Orden: verde ->
azul -> rojo. Al terminar con el rojo, sube y vuelve a la posicion inicial
del brazo (la misma pose de reposo con la que arranca
`my_robot_driver.py`, es decir todas las articulaciones a 0 en espacio
comandado).

Igual que en el proyecto hermano del UR5e (`Lad.Claude 2.2`), este nodo NO
agarra ningun cubo -- es solo bajar a tocar cada uno y esperar (el agarre
de verdad, con cierre/apertura de pinza, vive en `pick_and_place.py`). Por
eso NO publica en `/gripper_position`: solo usa `/joint_positions`, igual
que `move_above_ball.py`.

Ademas, mientras esta abajo tocando cada cubo, publica en `/comando_led`
(std_msgs/String: "R"/"G"/"B") el color de ese cubo, y lo apaga ("0") justo
antes de subir -- mismo patron que `visit_balls.py` del UR5e, para que lo
recoja un futuro nodo `led_publisher` (pendiente de portar a este paquete,
ver `resumen_proyecto_panda.md`). Si no hay ningun `led_publisher`
corriendo, el recorrido del brazo sigue igual: publicar en el topic no
bloquea nada.

Cinematica: igual modelo que `move_above_ball.py` (DH MODIFICADOS,
convencion de Craig, 7 articulaciones, Jacobiano 6x7 resuelto por minimos
cuadrados amortiguados). Las funciones de cinematica estan DUPLICADAS aqui a
proposito, en vez de importarlas de otro fichero, para no arriesgar tocar
codigo ya validado (mismo patron que `pick_and_place.py` y que el UR5e).

Como lanzarlo (con Webots en PLAY y `ros2 launch panda_controller
robot_launch.py` corriendo en otra terminal):

    ros2 run panda_controller visit_balls
"""

import math

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, String

JOINT_NAMES = [
    'panda_joint1',
    'panda_joint2',
    'panda_joint3',
    'panda_joint4',
    'panda_joint5',
    'panda_joint6',
    'panda_joint7',
]

# Parametros DH MODIFICADOS (convencion de Craig) del Panda, duplicados de
# move_above_ball.py a proposito (ver docstring del modulo).
PANDA_MDH = [
    (0.0,     0.0,        0.333),
    (0.0,    -math.pi/2,  0.0),
    (0.0,     math.pi/2,  0.316),
    (0.0825,  math.pi/2,  0.0),
    (-0.0825, -math.pi/2, 0.384),
    (0.0,     math.pi/2,  0.0),
    (0.088,   math.pi/2,  0.0),
]

FLANGE_TO_TCP_Z = 0.107 + 0.1034  # brida (0.107) + mano/dedos (~0.1034)

# Postura de reposo segura, igual que HOME_POSITIONS en my_robot_driver.py:
# joint4 del Panda NO puede ser 0 (su rango real es aprox. -3.14 a -0.4 rad,
# el codo viene "pre-doblado" de fabrica), asi que el reposo real no es
# "todo ceros". Debe coincidir con la postura que fija el driver al
# arrancar para que la duracion del primer segmento se estime bien y para
# que el ultimo paso ("home") no le pida a joint4 una posicion fuera de
# rango.
HOME_POSITIONS = [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]

# Debe coincidir con COMMANDED_VELOCITY de my_robot_driver.py. Se usa solo
# para ESTIMAR cuanto tarda el brazo en llegar a cada objetivo antes de
# pasar a la siguiente fase.
JOINT_VELOCITY_RAD_S = 1.0
MIN_SEGMENT_DURATION_S = 1.5
SEGMENT_MARGIN_S = 0.8

TICK_PERIOD_S = 0.05  # 20 Hz


def mdh_transform(a_prev: float, alpha_prev: float, d: float, theta: float) -> np.ndarray:
    ca, sa = math.cos(alpha_prev), math.sin(alpha_prev)
    ct, st = math.cos(theta), math.sin(theta)
    return np.array([
        [ct, -st, 0.0, a_prev],
        [st * ca, ct * ca, -sa, -sa * d],
        [st * sa, ct * sa, ca, ca * d],
        [0.0, 0.0, 0.0, 1.0],
    ])


def forward_kinematics(thetas: np.ndarray) -> np.ndarray:
    T = np.eye(4)
    for theta, (a_prev, alpha_prev, d) in zip(thetas, PANDA_MDH):
        T = T @ mdh_transform(a_prev, alpha_prev, d, theta)
    T_flange = np.eye(4)
    T_flange[2, 3] = FLANGE_TO_TCP_Z
    return T @ T_flange


def rotation_error(r_current: np.ndarray, r_target: np.ndarray) -> np.ndarray:
    r_err = r_target @ r_current.T
    cos_theta = np.clip((np.trace(r_err) - 1.0) / 2.0, -1.0, 1.0)
    theta = math.acos(cos_theta)
    if abs(theta) < 1e-8:
        return np.zeros(3)
    axis = np.array([
        r_err[2, 1] - r_err[1, 2],
        r_err[0, 2] - r_err[2, 0],
        r_err[1, 0] - r_err[0, 1],
    ]) / (2.0 * math.sin(theta))
    return axis * theta


def numeric_jacobian(thetas: np.ndarray, eps: float = 1e-6) -> np.ndarray:
    n = len(thetas)
    T0 = forward_kinematics(thetas)
    p0, r0 = T0[:3, 3], T0[:3, :3]
    J = np.zeros((6, n))
    for i in range(n):
        dthetas = thetas.copy()
        dthetas[i] += eps
        Ti = forward_kinematics(dthetas)
        J[:3, i] = (Ti[:3, 3] - p0) / eps
        dR = Ti[:3, :3] @ r0.T
        J[3:, i] = np.array([
            dR[2, 1] - dR[1, 2],
            dR[0, 2] - dR[2, 0],
            dR[1, 0] - dR[0, 1],
        ]) / (2.0 * eps)
    return J


def inverse_kinematics(target_pos, target_r, theta_init, max_iters=300,
                        tol=1e-4, damping=0.05):
    thetas = np.array(theta_init, dtype=float)
    err = np.zeros(6)
    for iteration in range(max_iters):
        T = forward_kinematics(thetas)
        pos_err = target_pos - T[:3, 3]
        rot_err = rotation_error(T[:3, :3], target_r)
        err = np.concatenate([pos_err, rot_err])
        if np.linalg.norm(err) < tol:
            return thetas, True, iteration, float(np.linalg.norm(err))
        J = numeric_jacobian(thetas)
        JJt = J @ J.T + (damping ** 2) * np.eye(6)
        dtheta = J.T @ np.linalg.solve(JJt, err)
        thetas = thetas + dtheta
    return thetas, False, max_iters, float(np.linalg.norm(err))


def solve_with_restarts(target_pos, target_r, heuristic_seed=None,
                         n_random_restarts=6, rng_seed=0, n_joints=7):
    rng = np.random.default_rng(rng_seed)
    seeds = []
    if heuristic_seed is not None:
        seeds.append(np.array(heuristic_seed, dtype=float))
    seeds += [rng.uniform(-math.pi, math.pi, size=n_joints) for _ in range(n_random_restarts)]

    best = None
    for seed in seeds:
        thetas, converged, iters, err = inverse_kinematics(target_pos, target_r, seed)
        if best is None or err < best[3]:
            best = (thetas, converged, iters, err)
        if converged:
            return best
    return best


# Orientacion deseada del TCP en todos los waypoints: mirando hacia abajo.
TARGET_R = np.array([
    [1.0, 0.0, 0.0],
    [0.0, -1.0, 0.0],
    [0.0, 0.0, -1.0],
])


class VisitBalls(Node):
    def __init__(self):
        super().__init__('visit_balls')

        self.declare_parameter('robot_base_x', 0.5)
        self.declare_parameter('robot_base_y', -0.3)
        self.declare_parameter('robot_base_z', 0.74)

        self.declare_parameter('cube_verde_x', 0.4)
        self.declare_parameter('cube_verde_y', 0.05)
        self.declare_parameter('cube_verde_z', 0.77)

        self.declare_parameter('cube_azul_x', 0.5)
        self.declare_parameter('cube_azul_y', 0.15)
        self.declare_parameter('cube_azul_z', 0.77)

        self.declare_parameter('cube_rojo_x', 0.3)
        self.declare_parameter('cube_rojo_y', -0.05)
        self.declare_parameter('cube_rojo_z', 0.77)

        self.declare_parameter('hover_height', 0.15)
        # Altura del TCP al "bajar hasta el cubo": 0.0 = TCP (punto medio
        # entre dedos) en el centro del cubo. No hay agarre aqui, es solo
        # llegar hasta el.
        self.declare_parameter('touch_height', 0.0)
        # Segundos que se queda parado abajo, tocando cada cubo.
        self.declare_parameter('hold_seconds', 2.0)

        self.declare_parameter('joint_offsets_deg', [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -90.0])
        self.declare_parameter('max_wait_seconds', 30.0)

        base = np.array([
            self.get_parameter('robot_base_x').value,
            self.get_parameter('robot_base_y').value,
            self.get_parameter('robot_base_z').value,
        ])
        cube_verde = np.array([
            self.get_parameter('cube_verde_x').value,
            self.get_parameter('cube_verde_y').value,
            self.get_parameter('cube_verde_z').value,
        ])
        cube_azul = np.array([
            self.get_parameter('cube_azul_x').value,
            self.get_parameter('cube_azul_y').value,
            self.get_parameter('cube_azul_z').value,
        ])
        cube_rojo = np.array([
            self.get_parameter('cube_rojo_x').value,
            self.get_parameter('cube_rojo_y').value,
            self.get_parameter('cube_rojo_z').value,
        ])

        hover = float(self.get_parameter('hover_height').value)
        touch = float(self.get_parameter('touch_height').value)
        self.hold_seconds = float(self.get_parameter('hold_seconds').value)

        offsets_deg = list(self.get_parameter('joint_offsets_deg').value)
        self.offsets_rad = np.radians(offsets_deg)
        self.max_wait_seconds = float(self.get_parameter('max_wait_seconds').value)

        # Pose fisica de partida / de vuelta al final: my_robot_driver.py
        # fija las 7 articulaciones del brazo a HOME_POSITIONS en su
        # init() (no a todo ceros, ver comentario de HOME_POSITIONS arriba).
        # Esa es la pose COMANDADA de reposo. En espacio "raw" (antes del
        # offset de calibracion) eso es raw = comandado - offset.
        self.home_raw = np.array(HOME_POSITIONS) - self.offsets_rad

        # Orden pedido: verde -> azul -> rojo -> posicion inicial. Por cada
        # cubo: hover encima -> bajar hasta tocarlo (con espera) -> subir de
        # vuelta al hover -> (al siguiente cubo, o a "home" si era el
        # ultimo). Color LED (comando que entenderia un futuro
        # led_publisher) por cubo.
        cubes_in_order = [
            ('verde', cube_verde, 'G'),
            ('azul', cube_azul, 'B'),
            ('rojo', cube_rojo, 'R'),
        ]
        targets_world = []
        for cube_name, cube_pos, led_color in cubes_in_order:
            targets_world.append((f'hover_{cube_name}', cube_pos + np.array([0.0, 0.0, hover]), 0.0, None))
            targets_world.append((f'baja_{cube_name}', cube_pos + np.array([0.0, 0.0, touch]), self.hold_seconds, led_color))
            targets_world.append((f'sube_{cube_name}', cube_pos + np.array([0.0, 0.0, hover]), 0.0, None))

        seed_raw = self.home_raw
        self.plan = []
        for i, (name, target_world, hold, led_color) in enumerate(targets_world):
            target_base = target_world - base
            if i == 0:
                pan0 = math.atan2(target_base[1], target_base[0])
                heuristic = [pan0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]
                thetas, converged, iters, err = inverse_kinematics(target_base, TARGET_R, heuristic)
                if not converged:
                    thetas, converged, iters, err = solve_with_restarts(
                        target_base, TARGET_R, heuristic_seed=heuristic
                    )
            else:
                thetas, converged, iters, err = inverse_kinematics(target_base, TARGET_R, seed_raw)
                if not converged:
                    thetas, converged, iters, err = solve_with_restarts(
                        target_base, TARGET_R, heuristic_seed=seed_raw
                    )

            if not converged:
                self.get_logger().warn(
                    f'[{name}] IK no convergio del todo (error={err:.5f}). '
                    'Se usa la mejor solucion encontrada.'
                )
            else:
                self.get_logger().info(f'[{name}] IK convergida en {iters} iter (error={err:.6f}).')

            self.plan.append({'name': name, 'raw': thetas, 'hold': hold, 'led': led_color})
            seed_raw = thetas

        # Ultimo paso: volver a la pose inicial (no hace falta IK, es una
        # pose conocida de antemano).
        self.plan.append({'name': 'home', 'raw': self.home_raw, 'hold': 0.0, 'led': None})

        self.pub_joint = self.create_publisher(Float64MultiArray, '/joint_positions', 10)
        self.pub_led = self.create_publisher(String, '/comando_led', 10)

        self.state = 'WAIT_SUBS'
        self.plan_index = -1
        self.prev_raw = self.home_raw
        self.seg_start_time = None
        self.seg_duration = None
        self.hold_until = None
        self.elapsed_waiting = 0.0

        self.timer = self.create_timer(TICK_PERIOD_S, self._tick)

    def _publish_joint_target(self, raw_thetas):
        commanded = raw_thetas + self.offsets_rad
        wrapped = (commanded + math.pi) % (2 * math.pi) - math.pi
        msg = Float64MultiArray()
        msg.data = wrapped.tolist()
        self.pub_joint.publish(msg)
        return wrapped

    def _start_segment(self, index):
        step = self.plan[index]
        start_raw = self.prev_raw
        target_raw = step['raw']
        delta = float(np.max(np.abs(target_raw - start_raw)))
        self.seg_duration = max(MIN_SEGMENT_DURATION_S, delta / JOINT_VELOCITY_RAD_S + SEGMENT_MARGIN_S)
        self.seg_start_time = self.get_clock().now().nanoseconds / 1e9
        wrapped = self._publish_joint_target(target_raw)
        self.get_logger().info(
            f"[{step['name']}] Publicado /joint_positions (deg)=" +
            ', '.join(f'{math.degrees(v):.1f}' for v in wrapped) +
            f' | duracion estimada {self.seg_duration:.2f}s, espera posterior {step["hold"]:.1f}s'
        )
        self.plan_index = index
        self.state = 'MOVING'

    def _tick(self):
        if self.state == 'WAIT_SUBS':
            if self.pub_joint.get_subscription_count() > 0:
                self.get_logger().info('Driver suscrito. Empezando recorrido.')
                self._start_segment(0)
                return
            self.elapsed_waiting += TICK_PERIOD_S
            if self.elapsed_waiting >= self.max_wait_seconds:
                self.get_logger().error(
                    f'Nadie se ha suscrito a /joint_positions tras '
                    f'{self.max_wait_seconds:.0f}s. ¿Esta corriendo '
                    "'ros2 launch panda_controller robot_launch.py'? Abandonando."
                )
                self.state = 'DONE'
            return

        if self.state == 'MOVING':
            now = self.get_clock().now().nanoseconds / 1e9
            if now - self.seg_start_time >= self.seg_duration:
                step = self.plan[self.plan_index]
                self.prev_raw = step['raw']
                if step['led']:
                    self.pub_led.publish(String(data=step['led']))
                    self.get_logger().info(f"[{step['name']}] LED -> {step['led']} (encima del cubo)")
                if step['hold'] > 0.0:
                    self.hold_until = now + step['hold']
                    self.state = 'HOLDING'
                    self.get_logger().info(f"[{step['name']}] Llegada. Esperando {step['hold']:.1f}s...")
                else:
                    self._advance()
            return

        if self.state == 'HOLDING':
            now = self.get_clock().now().nanoseconds / 1e9
            if now >= self.hold_until:
                step = self.plan[self.plan_index]
                if step['led']:
                    self.pub_led.publish(String(data='0'))
                    self.get_logger().info(f"[{step['name']}] LED -> apagado (dejando el cubo)")
                self._advance()
            return

    def _advance(self):
        next_index = self.plan_index + 1
        if next_index >= len(self.plan):
            self.state = 'DONE'
            self.get_logger().info('Recorrido completo. Brazo de vuelta a la posicion inicial.')
        else:
            self._start_segment(next_index)


def main(args=None):
    rclpy.init(args=args)
    node = VisitBalls()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
