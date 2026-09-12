#!/usr/bin/env python3
"""
Nodo ROS2 que calcula (por cinematica inversa numerica) la configuracion de
articulaciones del Franka Emika Panda necesaria para situar el TCP (punto
central entre los dedos de la pinza) encima del cubo verde del mundo
`panda_bolas.wbt`, y publica esa configuracion en `/joint_positions` para
que la recoja `my_robot_driver.py`.

Cinematica:
- Parametros DH MODIFICADOS (convencion de Craig), distintos de los DH
  estandar usados en el proyecto del UR5e. El Panda tiene 7 articulaciones
  (redundante para una tarea de 6 GDL), asi que el solver numerico trabaja
  con un Jacobiano de 6x7 (no cuadrado) resuelto por minimos cuadrados
  amortiguados (damped least squares), que ya funciona igual de bien con
  matrices no cuadradas.
- Tabla de parametros (a_{i-1}, alpha_{i-1}, d_i) tomada de la tabla DH
  modificada del Panda ampliamente publicada en la documentacion/ecosistema
  de Franka Emika. NO se ha podido verificar directamente contra el PROTO
  de Webots (GitHub bloqueado para fetch, docs de Cyberbotics ilegibles por
  ser una SPA), asi que, exactamente igual que paso con el UR5e (desfase de
  180 grados descubierto probando en Webots), es MUY probable que haga
  falta al menos una ronda de calibracion empirica con `joint_offsets_deg`
  una vez se vea el comportamiento real en el simulador.
- Se ignora deliberadamente el giro de 45 grados que algunas fuentes
  describen entre la brida (flange) del joint7 y la mano/pinza: si al
  probar en Webots la pinza queda rotada, se corrige sumando el offset
  correspondiente a `joint_offsets_deg[6]`, sin tocar la geometria/solver
  (mismo patron de correccion que en el UR5e).

Coordenadas del Panda y del cubo verde se han sacado directamente de
`worlds/panda_bolas.wbt`:
  Panda:      translation 0.5  -0.3 0.74   (sin rotacion)
  cubo_verde: translation 0.4   0.05 0.77

Posiciones re-ajustadas (acercadas a la base y repartidas tambien en X, no
solo en Y) tras comprobar en Webots que las 3 posiciones originales (todas
alineadas en X=0.5, solo separadas en Y desde 0.4 a 0.6 m de la base) no
eran alcanzables de forma fiable en la practica, aunque la IK numerica
convergiera "en el papel" (el solver no modela colisiones de los propios
eslabones del brazo contra la mesa/base, solo la cinematica del TCP).
"""

import math

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

JOINT_NAMES = [
    'panda_joint1',
    'panda_joint2',
    'panda_joint3',
    'panda_joint4',
    'panda_joint5',
    'panda_joint6',
    'panda_joint7',
]

# Parametros DH MODIFICADOS (convencion de Craig) del Panda: (a_{i-1}
# [m], alpha_{i-1} [rad], d_i [m]) para cada uno de los 7 joints, tomados de
# la tabla estandar publicada por Franka Emika.
PANDA_MDH = [
    (0.0,     0.0,        0.333),
    (0.0,    -math.pi/2,  0.0),
    (0.0,     math.pi/2,  0.316),
    (0.0825,  math.pi/2,  0.0),
    (-0.0825, -math.pi/2, 0.384),
    (0.0,     math.pi/2,  0.0),
    (0.088,   math.pi/2,  0.0),
]

# Transformacion fija adicional desde el joint7 hasta el TCP (brida +
# longitud de la pinza cerrada, aproximada). No es una articulacion: se
# aplica siempre igual, despues de resolver los 7 angulos.
FLANGE_TO_TCP_Z = 0.107 + 0.1034  # brida (0.107) + mano/dedos (~0.1034)


def mdh_transform(a_prev: float, alpha_prev: float, d: float, theta: float) -> np.ndarray:
    """Transformacion DH MODIFICADA: Rx(alpha_prev) * Tx(a_prev) * Rz(theta) * Tz(d)."""
    ca, sa = math.cos(alpha_prev), math.sin(alpha_prev)
    ct, st = math.cos(theta), math.sin(theta)
    return np.array([
        [ct, -st, 0.0, a_prev],
        [st * ca, ct * ca, -sa, -sa * d],
        [st * sa, ct * sa, ca, ca * d],
        [0.0, 0.0, 0.0, 1.0],
    ])


def forward_kinematics(thetas: np.ndarray) -> np.ndarray:
    """Devuelve la transformacion 4x4 del TCP respecto a la base del Panda."""
    T = np.eye(4)
    for theta, (a_prev, alpha_prev, d) in zip(thetas, PANDA_MDH):
        T = T @ mdh_transform(a_prev, alpha_prev, d, theta)
    # Offset fijo brida -> TCP a lo largo del eje Z local del joint7.
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
        # J es 6x7 (redundante): igual formula de minimos cuadrados
        # amortiguados, que funciona igual de bien con matrices no
        # cuadradas (pseudo-inversa amortiguada).
        JJt = J @ J.T + (damping ** 2) * np.eye(6)
        dtheta = J.T @ np.linalg.solve(JJt, err)
        thetas = thetas + dtheta
    return thetas, False, max_iters, float(np.linalg.norm(err))


def solve_with_restarts(target_pos, target_r, heuristic_seed=None,
                         n_random_restarts=6, rng_seed=0, n_joints=7):
    """Intenta primero la semilla heuristica (postura 'ready' tipica del
    Panda orientada al objetivo); si no converge, cae a semillas
    aleatorias."""
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


class MoveAboveBall(Node):
    def __init__(self):
        super().__init__('move_above_ball')

        # Traslacion del Panda en el mundo (worlds/panda_bolas.wbt), sin rotacion.
        self.declare_parameter('robot_base_x', 0.5)
        self.declare_parameter('robot_base_y', -0.3)
        self.declare_parameter('robot_base_z', 0.74)

        # Posicion del cubo verde en el mundo (worlds/panda_bolas.wbt).
        self.declare_parameter('cube_x', 0.4)
        self.declare_parameter('cube_y', 0.05)
        self.declare_parameter('cube_z', 0.77)

        # Altura por encima del centro del cubo a la que situar el TCP.
        self.declare_parameter('hover_height', 0.15)

        self.declare_parameter('publish_repeats', 5)
        self.declare_parameter('publish_period', 1.0)
        self.declare_parameter('max_wait_seconds', 30.0)

        # Offset de calibracion (grados), uno por cada uno de los 7 joints.
        # Se aplica DESPUES de resolver la geometria, solo para traducir a
        # lo que hay que mandarle al motor real. Se espera, por lo visto en
        # el UR5e, tener que ajustar al menos uno de estos valores tras la
        # primera prueba en Webots.
        self.declare_parameter('joint_offsets_deg', [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -90.0])

        base = np.array([
            self.get_parameter('robot_base_x').value,
            self.get_parameter('robot_base_y').value,
            self.get_parameter('robot_base_z').value,
        ])
        cube = np.array([
            self.get_parameter('cube_x').value,
            self.get_parameter('cube_y').value,
            self.get_parameter('cube_z').value,
        ])
        hover = float(self.get_parameter('hover_height').value)

        target_world = cube + np.array([0.0, 0.0, hover])
        target_base = target_world - base

        # Orientacion deseada del TCP: mirando hacia abajo (misma
        # convencion que en el UR5e; puede necesitar un giro adicional en
        # torno a Z segun como quede la pinza real al probar).
        target_r = np.array([
            [1.0, 0.0, 0.0],
            [0.0, -1.0, 0.0],
            [0.0, 0.0, -1.0],
        ])

        # Semilla heuristica: postura "ready" tipica del Panda (elbow-up,
        # comoda y lejos de singularidades), con joint1 apuntado hacia el
        # objetivo.
        pan0 = math.atan2(target_base[1], target_base[0])
        heuristic_seed = [pan0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]

        self.get_logger().info(
            f'Objetivo (frame base Panda): {target_base.tolist()} '
            f'[cubo en {cube.tolist()} + hover {hover} m, base en {base.tolist()}], '
            f'pan heuristico={math.degrees(pan0):.1f} deg'
        )

        thetas, converged, iters, err = solve_with_restarts(
            target_base, target_r, heuristic_seed=heuristic_seed
        )

        if not converged:
            self.get_logger().warn(
                f'IK no convergio del todo (error residual={err:.5f} m/rad tras '
                f'{iters} iteraciones). Se publicara la mejor solucion encontrada; '
                'revisa si el objetivo esta dentro del alcance del Panda.'
            )
        else:
            self.get_logger().info(
                f'IK convergida en {iters} iteraciones (error={err:.6f}).'
            )

        offsets_deg = list(self.get_parameter('joint_offsets_deg').value)
        offsets_rad = np.radians(offsets_deg)
        thetas_commanded = thetas + offsets_rad

        thetas_wrapped = (thetas_commanded + math.pi) % (2 * math.pi) - math.pi
        self.get_logger().info(
            'Angulos resueltos, sin calibrar (rad): ' +
            ', '.join(f'{n}={v:.4f}' for n, v in zip(JOINT_NAMES, thetas))
        )
        self.get_logger().info(
            'Angulos a enviar, con offset de calibracion (deg): ' +
            ', '.join(f'{n}={math.degrees(v):.1f}' for n, v in zip(JOINT_NAMES, thetas_wrapped))
        )

        self.target_msg = Float64MultiArray()
        self.target_msg.data = thetas_wrapped.tolist()

        self.publisher = self.create_publisher(Float64MultiArray, '/joint_positions', 10)

        self.repeats_left = int(self.get_parameter('publish_repeats').value)
        self.period = float(self.get_parameter('publish_period').value)
        self.max_wait_seconds = float(self.get_parameter('max_wait_seconds').value)
        self.elapsed_waiting = 0.0
        self.warned_waiting = False
        self.timer = self.create_timer(self.period, self._tick)

    def _tick(self):
        if self.publisher.get_subscription_count() == 0:
            self.elapsed_waiting += self.period
            if self.elapsed_waiting >= self.max_wait_seconds:
                self.get_logger().error(
                    f'Nadie se ha suscrito a /joint_positions tras '
                    f'{self.max_wait_seconds:.0f} s. ¿Esta corriendo '
                    "'ros2 launch panda_controller robot_launch.py' en otra terminal? "
                    'Abandonando sin publicar.'
                )
                self.timer.cancel()
                return
            if not self.warned_waiting:
                self.get_logger().info(
                    'Esperando a que el driver (my_robot_driver) se suscriba a '
                    '/joint_positions antes de publicar...'
                )
                self.warned_waiting = True
            return

        self.publisher.publish(self.target_msg)
        self.repeats_left -= 1
        self.get_logger().info(f'Publicado en /joint_positions (quedan {self.repeats_left} envios).')
        if self.repeats_left <= 0:
            self.timer.cancel()


def main(args=None):
    rclpy.init(args=args)
    node = MoveAboveBall()
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
