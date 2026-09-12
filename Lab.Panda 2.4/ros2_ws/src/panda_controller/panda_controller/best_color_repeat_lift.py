#!/usr/bin/env python3
"""
Nodo ROS2: decide por vision cual de los 3 colores de cubo (rojo/verde/azul)
se distingue mejor en la imagen de la wrist_camera, y repite 20 veces sobre
ESE color la secuencia de `vision_lift_cube.py` (localizar por vision ->
hover -> bajar -> CERRAR pinza, LED del color -> subir un poco -> bajar ->
ABRIR pinza, LED apagado -> subir) seguida de un tramo explicito de vuelta a
HOME_POSITIONS antes de la siguiente repeticion.

Criterio de "mejor se distingue": mismo patron HSV de `vision_lift_cube.py`
(ver COLOR_RANGES, saturacion minima alta para no confundir el cubo rojo con
el suelo ajedrezado). Para cada color se busca el contorno mas grande en una
foto tomada desde un punto de observacion cenital (misma orientacion
CAMERA_DOWN_R que `vision_lift_cube.py`) y se calcula:

    score = area_del_contorno * (saturacion_media_del_contorno / 255)

Gana el color con score mas alto; si un color no produce ningun contorno
valido su score es 0.

Por que RELOCALIZAR el cubo con la camara EN CADA repeticion (a diferencia
de una primera version de este nodo que usaba la posicion fija calibrada del
cubo y solo localizaba una vez): se probo en Webots real y, tras usar
siempre la MISMA posicion fija para las 20 repeticiones, el agarre no
quedaba centrado sobre el cubo -- cada cierre/apertura de pinza puede
desplazar ligeramente el cubo por contacto/friccion, y al repetir 20 veces
sobre una coordenada fija ese pequeno error se va acumulando (el cubo
"camina" por la mesa) hasta que la pinza termina agarrando muy descentrada o
en el aire. Relocalizando por vision antes de cada repeticion (mismo
servoing con reestimacion del Jacobiano pixel/mundo de `vision_lift_cube.py`,
ver Hito 6 de `resumen_proyecto_panda.md`) el agarre se corrige solo aunque
el cubo se haya movido un poco en la repeticion anterior.

Como lanzarlo (con Webots en PLAY y `ros2 launch panda_controller
robot_launch.py` corriendo en otra terminal):

    ros2 run panda_controller best_color_repeat_lift
"""

import math
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Float64, String
from sensor_msgs.msg import Image

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

HOME_POSITIONS = [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]

JOINT_VELOCITY_RAD_S = 1.0
MIN_SEGMENT_DURATION_S = 1.5
SEGMENT_MARGIN_S = 0.8

GRASP_R = np.array([
    [1.0, 0.0, 0.0],
    [0.0, -1.0, 0.0],
    [0.0, 0.0, -1.0],
])

# Orientacion "camara hacia abajo", igual que vision_lift_cube.py: necesaria
# para que la wrist_camera mire a la mesa (con GRASP_R mira al horizonte).
CAMERA_DOWN_R = np.array([
    [0.0, 0.0, 1.0],
    [0.0, -1.0, 0.0],
    [1.0, 0.0, 0.0],
])

# Mismos umbrales que vision_lift_cube.py (saturacion minima alta para no
# confundir el cubo rojo con el suelo ajedrezado).
COLOR_RANGES = {
    'R': ('rojo',  [((0, 200, 30), (8, 255, 255)), ((172, 200, 30), (179, 255, 255))]),
    'G': ('verde', [((40, 200, 30), (85, 255, 255))]),
    'B': ('azul',  [((95, 200, 30), (130, 255, 255))]),
}

MIN_CONTOUR_AREA = 200
CUBE_Z = 0.77  # mesa + medio cubo, igual que worlds/panda_bolas.wbt

# Mismo tablero que vision_lift_cube.py: Table { translation 0.5 0 0  size
# 0.8 1.2 0.74 } -> x en [0.1, 0.9], y en [-0.6, 0.6], acotado con margen.
TABLE_XY_MIN = np.array([0.2, -0.5])
TABLE_XY_MAX = np.array([0.8, 0.5])

# Limites reales de cada articulacion (leidos del log de my_robot_driver.py
# al arrancar), usados solo durante la fase de busqueda con CAMERA_DOWN_R
# (ver JOINT_LIMITS en vision_lift_cube.py).
JOINT_LIMITS = [
    (-2.9671, 2.9671),
    (-1.8326, 1.8326),
    (-2.9671, 2.9671),
    (-3.1416, -0.4000),
    (-2.9671, 2.9671),
    (-0.0873, 3.8223),
    (-2.9671, 2.9671),
]


def clamp_to_table(xy):
    return np.clip(xy, TABLE_XY_MIN, TABLE_XY_MAX)


def mdh_transform(a_prev, alpha_prev, d, theta):
    ca, sa = math.cos(alpha_prev), math.sin(alpha_prev)
    ct, st = math.cos(theta), math.sin(theta)
    return np.array([
        [ct, -st, 0.0, a_prev],
        [st * ca, ct * ca, -sa, -sa * d],
        [st * sa, ct * sa, ca, ca * d],
        [0.0, 0.0, 0.0, 1.0],
    ])


def forward_kinematics(thetas):
    T = np.eye(4)
    for theta, (a_prev, alpha_prev, d) in zip(thetas, PANDA_MDH):
        T = T @ mdh_transform(a_prev, alpha_prev, d, theta)
    T_flange = np.eye(4)
    T_flange[2, 3] = FLANGE_TO_TCP_Z
    return T @ T_flange


def flange_forward_kinematics(thetas):
    """Como forward_kinematics, pero SIN el offset de mano/dedos
    (FLANGE_TO_TCP_Z): devuelve la pose del flange (salida de joint7) tal
    cual, que es el frame del que cuelgan tanto PandaHand como la
    wrist_camera como hijos DIRECTOS de endEffectorSlot en
    worlds/panda_bolas.wbt (ver camera_world_position)."""
    T = np.eye(4)
    for theta, (a_prev, alpha_prev, d) in zip(thetas, PANDA_MDH):
        T = T @ mdh_transform(a_prev, alpha_prev, d, theta)
    return T


# Camera { translation 0 0 0.12 rotation 0 1 0 -1.5708 name "wrist_camera" }
# en worlds/panda_bolas.wbt, hermana de PandaHand dentro de endEffectorSlot
# (o sea, offset respecto al FLANGE, no respecto al TCP con mano/dedos).
CAMERA_LOCAL_OFFSET = np.array([0.0, 0.0, 0.12])


def camera_world_position(thetas, base):
    """Posicion mundo real de la wrist_camera (no del TCP/flange). BUG REAL
    encontrado en Webots: la localizacion por vision (heredada de
    vision_lift_cube.py) asumia que la posicion del TCP cuando la imagen
    esta centrada ES la posicion del cubo, sin tener en cuenta que la camara
    esta montada 0.12 m por delante del flange. Con la orientacion de
    busqueda (CAMERA_DOWN_R) ese offset cae sobre el eje X del mundo (no
    sobre Z), asi que el cubo localizado quedaba sistematicamente
    desplazado varios cm en +X -- justo hacia donde esta el cubo verde
    respecto al rojo -- y la pinza cerraba "por delante" del cubo real
    (visto y confirmado por el usuario en Webots real)."""
    T_flange = flange_forward_kinematics(thetas)
    cam_pos = T_flange[:3, 3] + T_flange[:3, :3] @ CAMERA_LOCAL_OFFSET
    return cam_pos + base


def rotation_error(r_current, r_target):
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


def numeric_jacobian(thetas, eps=1e-6):
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


def within_joint_limits(thetas, margin_deg=2.0):
    margin = math.radians(margin_deg)
    return all(
        (JOINT_LIMITS[i][0] + margin) <= t <= (JOINT_LIMITS[i][1] - margin)
        for i, t in enumerate(thetas)
    )


def solve_within_limits(target_pos, target_r, seed, n_random_restarts=8, rng_seed=0, n_joints=7):
    thetas, converged, _, err = inverse_kinematics(target_pos, target_r, seed)
    if converged and within_joint_limits(thetas):
        return thetas, True
    best_valid = (thetas, err) if (converged and within_joint_limits(thetas)) else None
    best_any = (thetas, err) if converged else None
    rng = np.random.default_rng(rng_seed)
    for _ in range(n_random_restarts):
        s = rng.uniform(-math.pi, math.pi, size=n_joints)
        t2, c2, _, e2 = inverse_kinematics(target_pos, target_r, s)
        if not c2:
            continue
        if best_any is None or e2 < best_any[1]:
            best_any = (t2, e2)
        if within_joint_limits(t2) and (best_valid is None or e2 < best_valid[1]):
            best_valid = (t2, e2)
    if best_valid is not None:
        return best_valid[0], True
    if best_any is not None:
        return best_any[0], False
    return thetas, False


def detect_largest_blob(bgr, only_letter=None):
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    best = None  # (area, cx, cy, letra_led, nombre_color)
    for letter, (name, ranges) in COLOR_RANGES.items():
        if only_letter is not None and letter != only_letter:
            continue
        mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
        for lo, hi in ranges:
            mask |= cv2.inRange(hsv, np.array(lo), np.array(hi))
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for c in contours:
            area = cv2.contourArea(c)
            if area < MIN_CONTOUR_AREA:
                continue
            M = cv2.moments(c)
            if M['m00'] == 0:
                continue
            cx = M['m10'] / M['m00']
            cy = M['m01'] / M['m00']
            if best is None or area > best[0]:
                best = (area, cx, cy, letter, name)
    return best


def evaluate_color_distinguishability(bgr, logger=None):
    """Para cada color de COLOR_RANGES, busca su contorno mas grande en la
    imagen y calcula score = area * (saturacion_media / 255). Devuelve un
    dict letra -> (nombre, area, saturacion_media, score)."""
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    sat_channel = hsv[:, :, 1]
    result = {}
    for letter, (name, ranges) in COLOR_RANGES.items():
        mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
        for lo, hi in ranges:
            mask |= cv2.inRange(hsv, np.array(lo), np.array(hi))
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        best_area = 0.0
        best_mean_sat = 0.0
        for c in contours:
            area = cv2.contourArea(c)
            if area < MIN_CONTOUR_AREA or area <= best_area:
                continue
            contour_mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
            cv2.drawContours(contour_mask, [c], -1, 255, thickness=cv2.FILLED)
            best_mean_sat = float(cv2.mean(sat_channel, mask=contour_mask)[0])
            best_area = area
        score = best_area * (best_mean_sat / 255.0)
        result[letter] = (name, best_area, best_mean_sat, score)
        if logger is not None:
            logger.info(
                f'[distinguir color] {name} ({letter}): area={best_area:.0f} '
                f'sat_media={best_mean_sat:.0f} score={score:.0f}'
            )
    return result


class BestColorRepeatLift(Node):
    def __init__(self):
        super().__init__('best_color_repeat_lift')

        self.declare_parameter('robot_base_x', 0.5)
        self.declare_parameter('robot_base_y', -0.3)
        self.declare_parameter('robot_base_z', 0.74)

        # Punto de observacion cenital para decidir que color se distingue
        # mejor y como punto de partida de la localizacion de cada cubo:
        # centro aproximado de los 3 cubos (ver worlds/panda_bolas.wbt).
        self.declare_parameter('search_x', 0.4)
        self.declare_parameter('search_y', 0.05)
        self.declare_parameter('search_z', 1.6)

        self.declare_parameter('hover_height', 0.15)
        self.declare_parameter('grasp_height', 0.0)
        self.declare_parameter('lift_height', 0.05)

        self.declare_parameter('gripper_open_position', 0.04)
        self.declare_parameter('gripper_closed_position', 0.025)

        self.declare_parameter('grasp_settle_seconds', 1.0)
        self.declare_parameter('release_settle_seconds', 0.5)

        self.declare_parameter('joint_offsets_deg', [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -90.0])
        self.declare_parameter('max_wait_seconds', 45.0)
        self.declare_parameter('num_repeats', 20)

        self.base = np.array([
            self.get_parameter('robot_base_x').value,
            self.get_parameter('robot_base_y').value,
            self.get_parameter('robot_base_z').value,
        ])
        self.search_xy = np.array([
            self.get_parameter('search_x').value,
            self.get_parameter('search_y').value,
        ])
        self.search_z = float(self.get_parameter('search_z').value)

        self.hover = float(self.get_parameter('hover_height').value)
        self.grasp = float(self.get_parameter('grasp_height').value)
        self.lift = float(self.get_parameter('lift_height').value)

        self.gripper_open = float(self.get_parameter('gripper_open_position').value)
        self.gripper_closed = float(self.get_parameter('gripper_closed_position').value)
        self.grasp_settle_seconds = float(self.get_parameter('grasp_settle_seconds').value)
        self.release_settle_seconds = float(self.get_parameter('release_settle_seconds').value)

        offsets_deg = list(self.get_parameter('joint_offsets_deg').value)
        self.offsets_rad = np.radians(offsets_deg)
        self.max_wait_seconds = float(self.get_parameter('max_wait_seconds').value)
        self.num_repeats = int(self.get_parameter('num_repeats').value)

        self.home_raw = np.array(HOME_POSITIONS) - self.offsets_rad

        self.pub_joint = self.create_publisher(Float64MultiArray, '/joint_positions', 10)
        self.pub_gripper = self.create_publisher(Float64, '/gripper_position', 10)
        self.pub_led = self.create_publisher(String, '/comando_led', 10)
        self.sub_img = self.create_subscription(Image, '/panda_camera/image_color', self._on_image, 10)

        self.latest_bgr = None
        self.cx_img, self.cy_img = 160.0, 120.0
        self.prev_raw = self.home_raw

        # Todo el trabajo se hace de forma bloqueante aqui en el __init__
        # (localizar + moverse + agarrar, repetido num_repeats veces), igual
        # patron que la fase de localizacion de vision_lift_cube.py, pero
        # extendido a TODA la secuencia de movimiento (no solo a la busqueda)
        # para poder relocalizar el cubo por vision antes de cada
        # repeticion sin anidar spin_once dentro de un callback de timer.
        if not self._wait_for_subscribers():
            self.state = 'DONE'
            return

        self._publish_gripper(self.gripper_open)

        chosen = self._choose_best_distinguishable_color()
        if chosen is None:
            self.get_logger().error('No se ha podido distinguir ningun color de cubo. Nada que hacer.')
            self.state = 'DONE'
            return
        self.target_letter, self.color_name = chosen
        self.get_logger().info(
            f'Cubo elegido: {self.color_name} ({self.target_letter}). '
            f'Empezando {self.num_repeats} repeticiones.'
        )

        completed = 0
        for cycle in range(1, self.num_repeats + 1):
            located = self._locate_cube()
            if located is None:
                self.get_logger().error(
                    f'[rep {cycle}/{self.num_repeats}] no se ha podido localizar el cubo '
                    f'{self.color_name}. Abortando repeticiones restantes.'
                )
                break
            xy, err_px = located
            cube_world = np.array([xy[0], xy[1], CUBE_Z])
            self.get_logger().info(
                f'[rep {cycle}/{self.num_repeats}] cubo {self.color_name} localizado en '
                f'{cube_world.tolist()} (error_px={err_px:.1f}).'
            )
            plan = self._build_cycle_plan(cube_world)
            self._execute_plan_blocking(plan, cycle)
            completed = cycle

        self.state = 'DONE'
        self.get_logger().info(
            f'Terminado: {completed}/{self.num_repeats} repeticiones completadas sobre '
            f'el cubo {getattr(self, "color_name", "?")}.'
        )

    # ---- utilidades bloqueantes de comunicacion con el driver/camara ----

    def _wait_for_subscribers(self):
        self.get_logger().info('Esperando a que el driver se suscriba...')
        elapsed = 0.0
        while rclpy.ok() and elapsed < self.max_wait_seconds:
            ready = (
                self.pub_joint.get_subscription_count() > 0
                and self.pub_gripper.get_subscription_count() > 0
            )
            if ready:
                self.get_logger().info('Driver suscrito.')
                return True
            rclpy.spin_once(self, timeout_sec=0.1)
            elapsed += 0.1
        self.get_logger().error(
            f'Nadie se ha suscrito tras {self.max_wait_seconds:.0f}s '
            "(revisa que 'ros2 launch panda_controller robot_launch.py' este corriendo)."
        )
        return False

    def _on_image(self, msg):
        arr = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, 4))
        self.latest_bgr = cv2.cvtColor(arr, cv2.COLOR_BGRA2BGR)

    def _publish_joint_target(self, raw_thetas):
        commanded = raw_thetas + self.offsets_rad
        wrapped = (commanded + math.pi) % (2 * math.pi) - math.pi
        msg = Float64MultiArray()
        msg.data = wrapped.tolist()
        self.pub_joint.publish(msg)
        return wrapped

    def _publish_gripper(self, position):
        self.pub_gripper.publish(Float64(data=position))

    def _goto_raw(self, target_base, target_r, seed, prefer_within_limits=False):
        if prefer_within_limits:
            thetas, ok = solve_within_limits(target_base, target_r, seed)
            if not ok:
                self.get_logger().warn(
                    'No se encontro ninguna solucion de IK dentro de los limites reales '
                    'de articulacion para este punto; se usa la mejor disponible.'
                )
        else:
            thetas, ok, iters, err = inverse_kinematics(target_base, target_r, seed)
            if not ok:
                thetas, ok, iters, err = solve_with_restarts(target_base, target_r, heuristic_seed=seed)
        wrapped = self._publish_joint_target(thetas)
        for _ in range(10):
            self.pub_joint.publish(Float64MultiArray(data=wrapped.tolist()))
            rclpy.spin_once(self, timeout_sec=0.1)
        return thetas

    def _grab_frame(self):
        self.latest_bgr = None
        start = time.time()
        while rclpy.ok() and self.latest_bgr is None and time.time() - start < 3.0:
            rclpy.spin_once(self, timeout_sec=0.1)
        return self.latest_bgr

    def _observe_at(self, xy, seed, z=None):
        target_base = np.array([xy[0], xy[1], z if z is not None else self.search_z]) - self.base
        thetas = self._goto_raw(target_base, CAMERA_DOWN_R, seed, prefer_within_limits=True)
        # Esperar a que el brazo haya llegado de verdad antes de capturar
        # (ver vision_lift_cube.py: un tiempo fijo no basta si el punto
        # anterior estaba lejos).
        delta = float(np.max(np.abs(np.array(thetas) - np.array(seed))))
        settle = max(0.9, delta / JOINT_VELOCITY_RAD_S + 0.5)
        time.sleep(settle)
        frame = self._grab_frame()
        blob = detect_largest_blob(frame, only_letter=self.target_letter) if frame is not None else None
        return thetas, blob

    # ---- eleccion del color mas distinguible (una sola vez) ----

    def _choose_best_distinguishable_color(self):
        target_base = np.array([self.search_xy[0], self.search_xy[1], self.search_z]) - self.base
        pan0 = math.atan2(target_base[1], target_base[0])
        seed = [pan0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]
        thetas = self._goto_raw(target_base, CAMERA_DOWN_R, seed, prefer_within_limits=True)
        delta = float(np.max(np.abs(np.array(thetas) - np.array(self.prev_raw))))
        settle = max(1.0, delta / JOINT_VELOCITY_RAD_S + 0.5)
        time.sleep(settle)
        self.prev_raw = thetas

        frame = self._grab_frame()
        if frame is None:
            self.get_logger().error('[distinguir color] no ha llegado ninguna imagen de la camara.')
            return None

        scores = evaluate_color_distinguishability(frame, logger=self.get_logger())
        best_letter = max(scores, key=lambda k: scores[k][3])
        best_name, best_area, best_sat, best_score = scores[best_letter]
        if best_score <= 0.0:
            self.get_logger().error('[distinguir color] ningun color supero el umbral minimo de deteccion.')
            return None
        self.get_logger().info(
            f'[distinguir color] mejor distinguido: {best_name} ({best_letter}) '
            f'con score={best_score:.0f} (area={best_area:.0f}, sat_media={best_sat:.0f}).'
        )
        return best_letter, best_name

    # ---- localizacion del cubo elegido (una vez por repeticion) ----

    SEARCH_FALLBACK_OFFSETS = [
        (0.0, 0.0),
        (-0.15, 0.15), (0.15, 0.15), (-0.15, -0.15), (0.15, -0.15),
        (0.0, 0.3), (0.0, -0.3), (-0.25, 0.0), (0.25, 0.0),
    ]

    def _find_initial_view(self):
        pan0 = math.atan2(self.search_xy[1] - self.base[1], self.search_xy[0] - self.base[0])
        base_seed = np.array([pan0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785])
        MIN_GOOD_AREA = 600
        best = None  # (area, xy, thetas, blob)
        for dx, dy in self.SEARCH_FALLBACK_OFFSETS:
            xy = clamp_to_table(self.search_xy + np.array([dx, dy]))
            thetas, blob = self._observe_at(xy, base_seed)
            if blob is None:
                self.get_logger().warn(f'[localizar cubo] xy={xy.tolist()} sin deteccion, probando otro punto.')
                continue
            area = blob[0]
            if best is None or area > best[0]:
                best = (area, xy, thetas, blob)
            if area >= MIN_GOOD_AREA:
                return xy, thetas, blob
            self.get_logger().warn(
                f'[localizar cubo] xy={xy.tolist()} deteccion muy pequena (area={area:.0f}), '
                'probando otro punto.'
            )
        if best is not None:
            return best[1], best[2], best[3]
        return None, None, None

    def _gauss_newton_servo(self, xy, z, seed, blob0, max_outer, threshold_px, d0=0.02):
        d = d0
        best = None
        xy = xy.copy()
        for outer in range(max_outer):
            if outer > 0:
                thetas0, blob0 = self._observe_at(xy, seed, z=z)
                seed = thetas0
            else:
                thetas0 = seed
            if blob0 is None:
                self.get_logger().warn(f'[localizar cubo] z={z} xy={xy.tolist()} sin deteccion.')
                break
            area0, u0, v0, letter, name = blob0
            e = np.array([self.cx_img - u0, self.cy_img - v0])
            enorm = float(np.linalg.norm(e))
            # Posicion real de la CAMARA (no del TCP nominal "xy" que se usa
            # solo como variable de control del servoing) en esta
            # observacion -- ver camera_world_position para el bug que
            # corrige.
            cam_xy = camera_world_position(thetas0, self.base)[:2]
            if best is None or enorm < best[1]:
                best = (cam_xy, enorm, thetas0)
            if enorm < threshold_px:
                return cam_xy, enorm, thetas0
            thetas_x, blobx = self._observe_at(clamp_to_table(xy + np.array([d, 0.0])), thetas0, z=z)
            thetas_y, bloby = self._observe_at(clamp_to_table(xy + np.array([0.0, d])), thetas0, z=z)
            if blobx is None or bloby is None:
                d *= 0.5
                seed = thetas0
                continue
            _, ux, vx, _, _ = blobx
            _, uy, vy, _, _ = bloby
            J = np.array([[(ux - u0) / d, (uy - u0) / d],
                          [(vx - v0) / d, (vy - v0) / d]])
            lam = 25.0
            delta = np.linalg.solve(J.T @ J + lam * np.eye(2), J.T @ e)
            max_step = 0.08
            dn = np.linalg.norm(delta)
            if dn > max_step:
                delta = delta / dn * max_step
            xy = clamp_to_table(xy + delta)
            seed = thetas0
        return best

    def _locate_cube(self):
        """Localizacion en dos pasadas (gruesa en alto, fina mas cerca de la
        mesa), igual patron que vision_lift_cube.py, restringida siempre a
        self.target_letter. Devuelve (xy_mundo, error_px) o None."""
        xy, seed, blob0 = self._find_initial_view()
        if blob0 is None:
            self.get_logger().error(f'[localizar cubo] no se ha visto el cubo {self.color_name} en ningun punto de busqueda.')
            return None

        coarse = self._gauss_newton_servo(xy, self.search_z, seed, blob0, max_outer=6, threshold_px=20.0)
        if coarse is None:
            return None
        xy_c, err_c, thetas_c = coarse

        refine_z = max(self.search_z - 0.5, self.base[2] + 0.45)
        thetas_r, blob_r = self._observe_at(xy_c, thetas_c, z=refine_z)
        fine = self._gauss_newton_servo(
            xy_c, refine_z, thetas_r, blob_r, max_outer=6, threshold_px=10.0, d0=0.01
        )
        if fine is None:
            self.get_logger().warn('[localizar cubo] la pasada fina no vio nada; uso el resultado de la gruesa.')
            return xy_c, err_c
        xy_f, err_f, _ = fine
        return xy_f, err_f

    # ---- construccion y ejecucion del plan de un ciclo ----

    def _build_cycle_plan(self, cube_world):
        color_name = self.color_name
        waypoints_world = [
            (f'hover_{color_name}', cube_world + np.array([0.0, 0.0, self.hover]), None),
            (f'grasp_{color_name}', cube_world + np.array([0.0, 0.0, self.grasp]), 'close'),
            (f'lift_{color_name}', cube_world + np.array([0.0, 0.0, self.lift]), None),
            (f'lower_{color_name}', cube_world + np.array([0.0, 0.0, self.grasp]), 'open'),
            ('retreat', cube_world + np.array([0.0, 0.0, self.hover]), None),
        ]
        plan = []
        seed_raw = self.home_raw
        for name, target_world, gripper_action in waypoints_world:
            target_base = target_world - self.base
            if name.startswith('hover_'):
                pan0 = math.atan2(target_base[1], target_base[0])
                heuristic = [pan0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]
                thetas, converged, iters, err = inverse_kinematics(target_base, GRASP_R, heuristic)
                if not converged:
                    thetas, converged, iters, err = solve_with_restarts(
                        target_base, GRASP_R, heuristic_seed=heuristic
                    )
            else:
                thetas, converged, iters, err = inverse_kinematics(target_base, GRASP_R, seed_raw)
                if not converged:
                    thetas, converged, iters, err = solve_with_restarts(
                        target_base, GRASP_R, heuristic_seed=seed_raw
                    )
            if not converged:
                self.get_logger().warn(f'[{name}] IK no convergio del todo (error={err:.5f}).')
            plan.append({'name': name, 'raw': thetas, 'gripper_action': gripper_action})
            seed_raw = thetas
        # Tramo final explicito: volver a la posicion inicial (HOME) antes de
        # la siguiente repeticion. No hace falta IK, HOME_POSITIONS ya esta
        # en espacio de articulaciones.
        plan.append({'name': 'home', 'raw': self.home_raw, 'gripper_action': None})
        return plan

    def _move_to_raw_blocking(self, raw_thetas, step_name, cycle):
        delta = float(np.max(np.abs(raw_thetas - self.prev_raw)))
        duration = max(MIN_SEGMENT_DURATION_S, delta / JOINT_VELOCITY_RAD_S + SEGMENT_MARGIN_S)
        wrapped = self._publish_joint_target(raw_thetas)
        self.get_logger().info(
            f"[rep {cycle}/{self.num_repeats}] [{step_name}] Publicado /joint_positions (deg)=" +
            ', '.join(f'{math.degrees(v):.1f}' for v in wrapped) +
            f' | duracion estimada {duration:.2f}s'
        )
        elapsed = 0.0
        while rclpy.ok() and elapsed < duration:
            rclpy.spin_once(self, timeout_sec=0.1)
            elapsed += 0.1
        self.prev_raw = raw_thetas

    def _execute_plan_blocking(self, plan, cycle):
        for step in plan:
            self._move_to_raw_blocking(step['raw'], step['name'], cycle)
            if step['gripper_action'] == 'close':
                self._publish_gripper(self.gripper_closed)
                self.pub_led.publish(String(data=self.target_letter))
                self.get_logger().info(
                    f"[rep {cycle}/{self.num_repeats}] [{step['name']}] "
                    f'Pinza CERRAR, LED -> {self.target_letter} (agarrando cubo {self.color_name}).'
                )
                self._sleep_spinning(self.grasp_settle_seconds)
            elif step['gripper_action'] == 'open':
                self._publish_gripper(self.gripper_open)
                self.pub_led.publish(String(data='0'))
                self.get_logger().info(
                    f"[rep {cycle}/{self.num_repeats}] [{step['name']}] "
                    'Pinza ABRIR, LED -> apagado (soltando el cubo).'
                )
                self._sleep_spinning(self.release_settle_seconds)
        self.get_logger().info(
            f'[rep {cycle}/{self.num_repeats}] completada: cubo {self.color_name} '
            'agarrado, levantado, devuelto y brazo en HOME.'
        )

    def _sleep_spinning(self, seconds):
        elapsed = 0.0
        while rclpy.ok() and elapsed < seconds:
            rclpy.spin_once(self, timeout_sec=0.1)
            elapsed += 0.1


def main(args=None):
    rclpy.init(args=args)
    node = BestColorRepeatLift()
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
