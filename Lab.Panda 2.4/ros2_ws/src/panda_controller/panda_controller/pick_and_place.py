#!/usr/bin/env python3
"""
Nodo ROS2 de secuencia completa de pick-and-place: coge el cubo verde y lo
deja a la derecha del cubo azul.

A diferencia del proyecto hermano del UR5e (`Lad.Claude 2.2`), el Panda SI
tiene pinza real (`panda_finger_joint1/2`) en su PROTO oficial de Webots, asi
que aqui NO hace falta el truco de Supervisor que teletransportaba la bola:
basta con publicar en `/gripper_position` (recogido por `my_robot_driver.py`,
que mueve los dos dedos en espejo) para cerrar los dedos sobre el cubo de
verdad, dejar que la fisica de Webots haga su trabajo (friccion + masa baja
del cubo, 0.05 kg), y confiar en que se queda agarrado mientras el brazo
se mueve.

Secuencia (igual estructura que `pick_and_place.py` del UR5e, adaptada):
  hover sobre cubo verde -> bajar -> CERRAR pinza -> esperar a que se
  asiente el agarre -> subir -> hover sobre destino -> bajar -> ABRIR pinza
  -> esperar a que se suelte -> subir (retreat).

Destino = cubo_azul + offset. Misma asuncion sin confirmar con el usuario
que en el UR5e: "a la derecha del azul" se interpreta como continuar la
fila en +Y (mismo X/Z que el cubo azul, Y = cubo_azul.y + place_offset_y =
0.1 por defecto). Ajustable con `place_offset_x/y/z` si no era eso.

Cinematica: igual modelo que `move_above_ball.py` (DH MODIFICADOS,
convencion de Craig, 7 articulaciones, Jacobiano 6x7 resuelto por minimos
cuadrados amortiguados). Las funciones de cinematica estan DUPLICADAS aqui a
proposito, en vez de importarlas de `move_above_ball.py`, para no arriesgar
tocar ese fichero ya validado (mismo patron que uso el UR5e).

Objetos: originalmente bolas (esferas de radio 0.03 m), cambiadas a cubos de
0.06 m de lado (semi-lado 0.03 m, mismo tamano que el diametro/radio de la
bola que sustituyen) para que la geometria de agarre no cambiara: cada dedo
sigue cerrando en torno a esa misma distancia media desde el centro del
objeto.

Apertura/cierre de dedos: aproximado, pendiente de calibracion empirica
igual que `joint_offsets_deg`. Cada dedo se mueve en espejo desde 0 (cerrado)
hasta ~0.04 m (abierto), asi que para agarrar el cubo cada dedo deberia
quedarse en torno al semi-lado (0.03 m) o un poco menos para dar margen de
compresion/agarre; el valor por defecto de `gripper_closed_position`
(0.025 m) es una primera aproximacion, ajustable si el cubo se escapa o el
agarre queda demasiado apretado.

LED: al cerrar la pinza sobre el cubo verde se publica "G" en
`/comando_led` (mismo topic y convencion que `visit_balls.py`), y al
abrirla en el destino se publica "0" (apagar). Lo recoge `led_publisher.py`
(tambien en este paquete) y lo reenvia por Wi-Fi a la Raspberry Pi Pico W;
si `led_publisher` no esta corriendo o la Pico no esta accesible, la
secuencia de agarre sigue igual, publicar en el topic no bloquea nada.

Como lanzarlo (con Webots en PLAY y `ros2 launch panda_controller
robot_launch.py` corriendo en otra terminal):

    ros2 run panda_controller pick_and_place
"""

import math

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Float64, String

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
# arrancar para que la duracion del primer segmento se estime bien.
HOME_POSITIONS = [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]

# Debe coincidir con COMMANDED_VELOCITY de my_robot_driver.py. Se usa solo
# para ESTIMAR cuanto tarda el brazo en llegar a cada objetivo.
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


# Orientacion deseada del TCP en todos los waypoints: mirando hacia abajo,
# igual que en move_above_ball.py.
TARGET_R = np.array([
    [1.0, 0.0, 0.0],
    [0.0, -1.0, 0.0],
    [0.0, 0.0, -1.0],
])


class PickAndPlace(Node):
    def __init__(self):
        super().__init__('pick_and_place')

        self.declare_parameter('robot_base_x', 0.5)
        self.declare_parameter('robot_base_y', -0.3)
        self.declare_parameter('robot_base_z', 0.74)

        self.declare_parameter('cube_verde_x', 0.4)
        self.declare_parameter('cube_verde_y', 0.05)
        self.declare_parameter('cube_verde_z', 0.77)

        self.declare_parameter('cube_azul_x', 0.5)
        self.declare_parameter('cube_azul_y', 0.15)
        self.declare_parameter('cube_azul_z', 0.77)

        # Destino = cubo_azul + este offset. Misma asuncion que el UR5e:
        # seguir la fila en +Y. Ajustable si "a la derecha" era otra cosa.
        self.declare_parameter('place_offset_x', 0.0)
        self.declare_parameter('place_offset_y', 0.1)
        self.declare_parameter('place_offset_z', 0.0)

        self.declare_parameter('hover_height', 0.15)
        # Altura del TCP al agarrar/soltar: 0.0 = TCP (punto medio entre
        # dedos) exactamente en el centro del cubo.
        self.declare_parameter('grasp_height', 0.0)

        # Posiciones de dedo (metros, por dedo, en espejo). Ver docstring
        # del modulo: aproximadas, pendientes de calibracion empirica.
        self.declare_parameter('gripper_open_position', 0.04)
        self.declare_parameter('gripper_closed_position', 0.025)

        # Tiempo de espera tras publicar el cierre/apertura de la pinza,
        # para dar margen a que los dedos lleguen a su posicion y la fisica
        # de Webots asiente el contacto antes de mover el brazo otra vez.
        self.declare_parameter('grasp_settle_seconds', 1.0)
        self.declare_parameter('release_settle_seconds', 0.5)

        self.declare_parameter('joint_offsets_deg', [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -90.0])
        self.declare_parameter('max_wait_seconds', 45.0)

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
        place_offset = np.array([
            self.get_parameter('place_offset_x').value,
            self.get_parameter('place_offset_y').value,
            self.get_parameter('place_offset_z').value,
        ])
        destino = cube_azul + place_offset

        hover = float(self.get_parameter('hover_height').value)
        grasp = float(self.get_parameter('grasp_height').value)

        self.gripper_open = float(self.get_parameter('gripper_open_position').value)
        self.gripper_closed = float(self.get_parameter('gripper_closed_position').value)
        self.grasp_settle_seconds = float(self.get_parameter('grasp_settle_seconds').value)
        self.release_settle_seconds = float(self.get_parameter('release_settle_seconds').value)

        self.base = base
        offsets_deg = list(self.get_parameter('joint_offsets_deg').value)
        self.offsets_rad = np.radians(offsets_deg)
        self.max_wait_seconds = float(self.get_parameter('max_wait_seconds').value)

        waypoints_world = [
            ('hover_verde', cube_verde + np.array([0.0, 0.0, hover]), None),
            ('grasp_verde', cube_verde + np.array([0.0, 0.0, grasp]), 'close'),
            ('lift_verde', cube_verde + np.array([0.0, 0.0, hover]), None),
            ('hover_destino', destino + np.array([0.0, 0.0, hover]), None),
            ('place_destino', destino + np.array([0.0, 0.0, grasp]), 'open'),
            ('retreat', destino + np.array([0.0, 0.0, hover]), None),
        ]

        self.get_logger().info(
            f'Destino calculado (asuncion "a la derecha" = +Y): {destino.tolist()} '
            f'(cubo_azul={cube_azul.tolist()} + offset={place_offset.tolist()})'
        )

        # Pose fisica de partida: my_robot_driver.py fija las 7 articulaciones
        # del brazo a HOME_POSITIONS en su init() (no a todo ceros, ver
        # comentario de HOME_POSITIONS arriba). Esa es la pose COMANDADA
        # inicial. En espacio "raw" (antes del offset de calibracion) eso
        # equivale a raw = comandado - offset.
        home_raw = np.array(HOME_POSITIONS) - self.offsets_rad
        seed_raw = home_raw

        self.plan = []
        for name, target_world, gripper_action in waypoints_world:
            target_base = target_world - base
            if name == 'hover_verde':
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

            self.plan.append({
                'name': name,
                'raw': thetas,
                'gripper_action': gripper_action,
            })
            seed_raw = thetas

        self.pub_joint = self.create_publisher(Float64MultiArray, '/joint_positions', 10)
        self.pub_gripper = self.create_publisher(Float64, '/gripper_position', 10)
        self.pub_led = self.create_publisher(String, '/comando_led', 10)

        self.state = 'WAIT_SUBS'
        self.plan_index = -1
        self.prev_raw = home_raw
        self.seg_start_time = None
        self.seg_duration = None
        self.settle_until = None
        self.elapsed_waiting = 0.0

        self.timer = self.create_timer(TICK_PERIOD_S, self._tick)

    def _publish_joint_target(self, raw_thetas):
        commanded = raw_thetas + self.offsets_rad
        wrapped = (commanded + math.pi) % (2 * math.pi) - math.pi
        msg = Float64MultiArray()
        msg.data = wrapped.tolist()
        self.pub_joint.publish(msg)
        return wrapped

    def _publish_gripper(self, position):
        self.pub_gripper.publish(Float64(data=position))

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
            f' | duracion estimada {self.seg_duration:.2f}s'
        )
        self.plan_index = index
        self.state = 'MOVING'

    def _tick(self):
        if self.state == 'WAIT_SUBS':
            ready = (
                self.pub_joint.get_subscription_count() > 0
                and self.pub_gripper.get_subscription_count() > 0
            )
            if ready:
                self.get_logger().info('Driver suscrito. Abriendo pinza y empezando secuencia.')
                self._publish_gripper(self.gripper_open)
                self._start_segment(0)
                return
            self.elapsed_waiting += TICK_PERIOD_S
            if self.elapsed_waiting >= self.max_wait_seconds:
                self.get_logger().error(
                    f'Nadie se ha suscrito tras {self.max_wait_seconds:.0f}s '
                    "(revisa que 'ros2 launch panda_controller robot_launch.py' "
                    'este corriendo). Abandonando.'
                )
                self.state = 'DONE'
            return

        if self.state == 'MOVING':
            now = self.get_clock().now().nanoseconds / 1e9
            if now - self.seg_start_time >= self.seg_duration:
                step = self.plan[self.plan_index]
                self.prev_raw = step['raw']
                if step['gripper_action'] == 'close':
                    self._publish_gripper(self.gripper_closed)
                    self.get_logger().info(
                        f'[{step["name"]}] Pinza: CERRAR a {self.gripper_closed:.4f} m '
                        '(agarrando cubo_verde).'
                    )
                    self.pub_led.publish(String(data='G'))
                    self.get_logger().info(f"[{step['name']}] LED -> G (agarrando cubo_verde)")
                    self.settle_until = now + self.grasp_settle_seconds
                    self.state = 'GRIP_SETTLE'
                elif step['gripper_action'] == 'open':
                    self._publish_gripper(self.gripper_open)
                    self.get_logger().info(
                        f'[{step["name"]}] Pinza: ABRIR a {self.gripper_open:.4f} m '
                        '(soltando en destino).'
                    )
                    self.pub_led.publish(String(data='0'))
                    self.get_logger().info(f"[{step['name']}] LED -> apagado (soltando el cubo)")
                    self.settle_until = now + self.release_settle_seconds
                    self.state = 'GRIP_SETTLE'
                else:
                    self._advance()
            return

        if self.state == 'GRIP_SETTLE':
            now = self.get_clock().now().nanoseconds / 1e9
            if now >= self.settle_until:
                self._advance()
            return

    def _advance(self):
        next_index = self.plan_index + 1
        if next_index >= len(self.plan):
            self.state = 'DONE'
            self.get_logger().info('Secuencia de pick-and-place completada.')
        else:
            self._start_segment(next_index)


def main(args=None):
    rclpy.init(args=args)
    node = PickAndPlace()
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
