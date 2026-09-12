#!/usr/bin/env python3
"""
Nodo ROS2: localiza CUALQUIER cubo con la wrist_camera (sin asumir su
posicion, por si se ha movido de donde estaba en `worlds/panda_bolas.wbt`),
lo coge, lo sube un poco y lo vuelve a dejar en el mismo sitio. Mismo patron
de movimiento que `lift_ball.py`, pero con la posicion del cubo estimada por
vision en vez de leida de parametros fijos.

Localizacion visual (servoing, no proyeccion analitica de un solo tiro):
1. Sube el TCP muy por encima de la mesa con una orientacion especial
   (CAMERA_DOWN_R, distinta de la orientacion de agarre) para que la
   wrist_camera mire hacia abajo -- con la orientacion normal de agarre
   (pinza hacia abajo) la camara mira hacia el horizonte, no hacia la mesa
   (se comprobo en Webots real; el offset de montaje camara-brida no
   coincide con el eje de aproximacion de la pinza).
2. Detecta el cubo mas grande de color rojo/verde/azul en la imagen
   (`/panda_camera/image_color`, umbral HSV).
3. Si no esta centrado, mide el Jacobiano pixel<->mundo LOCAL con dos
   perturbaciones pequenas (mover un poco en X, un poco en Y, mirar de
   nuevo) y da un paso de Newton amortiguado hacia el centro de la imagen.
   Se re-mide el Jacobiano en cada iteracion (no se reutiliza uno lejano)
   porque se comprobo empiricamente que un Jacobiano medido lejos del punto
   actual puede estar mal condicionado y no generalizar.
4. Cuando el error esta por debajo de un umbral en pixeles, la posicion XY
   del TCP en ese instante ES la posicion XY del cubo (camara mirando
   derecha hacia abajo); Z se conoce (mesa + medio cubo = 0.77 m, igual que
   el resto de nodos del paquete).

Con la posicion asi estimada, se ejecuta la misma secuencia que
`lift_ball.py`: hover -> bajar -> CERRAR pinza (LED del color detectado) ->
subir un poco -> bajar -> ABRIR pinza (LED apagado) -> subir (retreat).

Cinematica: mismas funciones DH modificadas duplicadas que el resto del
paquete (ver docstring de `move_above_ball.py`).

Como lanzarlo (con Webots en PLAY y `ros2 launch panda_controller
robot_launch.py` corriendo en otra terminal):

    ros2 run panda_controller vision_lift_cube
"""

import math
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Float64, String
from sensor_msgs.msg import Image

JOINT_NAMES = [
    'panda_joint1',
    'panda_joint2',
    'panda_joint3',
    'panda_joint4',
    'panda_joint5',
    'panda_joint6',
    'panda_joint7',
]

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

TICK_PERIOD_S = 0.05  # 20 Hz

# Orientacion de agarre: TCP mirando hacia abajo (igual que el resto del
# paquete).
GRASP_R = np.array([
    [1.0, 0.0, 0.0],
    [0.0, -1.0, 0.0],
    [0.0, 0.0, -1.0],
])

# Orientacion "camara hacia abajo": flange X apuntando hacia +Z mundo, para
# que la wrist_camera (montada con rotacion fija -90 deg en Y respecto al
# flange, ver worlds/panda_bolas.wbt) mire hacia el suelo/mesa en vez de
# hacia el horizonte. Comprobado en Webots real: con GRASP_R la camara ve
# las montanas del fondo; con esta orientacion ve la mesa.
CAMERA_DOWN_R = np.array([
    [0.0, 0.0, 1.0],
    [0.0, -1.0, 0.0],
    [1.0, 0.0, 0.0],
])

# Umbrales HSV para los 3 colores de cubo. Saturacion minima alta (200) a
# proposito: el suelo ajedrezado tiene tonos rojizos/marrones con matiz
# (hue) parecido al cubo rojo pero MENOS saturados (~150-165 frente a 255
# del cubo), y sin ese margen la deteccion "roja" enganchaba el suelo en
# vez del cubo (bug encontrado probando en Webots real).
COLOR_RANGES = {
    'R': ('rojo',  [((0, 200, 30), (8, 255, 255)), ((172, 200, 30), (179, 255, 255))]),
    'G': ('verde', [((40, 200, 30), (85, 255, 255))]),
    'B': ('azul',  [((95, 200, 30), (130, 255, 255))]),
}

CUBE_Z = 0.77  # mesa + medio cubo, igual que worlds/panda_bolas.wbt

# Tablero de la mesa: Table { translation 0.5 0 0  size 0.8 1.2 0.74 } en
# worlds/panda_bolas.wbt -> x en [0.1, 0.9], y en [-0.6, 0.6]. Se acota la
# busqueda visual a un margen dentro de ese tablero (no al borde exacto,
# para que la camara siga viendo mesa alrededor del punto y no se salga a
# mirar el suelo) para que ni los offsets de busqueda ni los pasos del
# servoing puedan alejarse hacia el suelo o la propia base del robot, como
# paso en pruebas reales antes de acotarlo.
TABLE_XY_MIN = np.array([0.2, -0.5])
TABLE_XY_MAX = np.array([0.8, 0.5])


def clamp_to_table(xy):
    return np.clip(xy, TABLE_XY_MIN, TABLE_XY_MAX)


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


# Limites reales de cada articulacion, tal como los imprime my_robot_driver.py
# al arrancar (leidos del propio motor en Webots, no de la doc generica de
# Franka: para panda_joint4 y panda_joint6 no coinciden exactamente). El
# solver de IK no los conoce, asi que puede converger a una solucion
# matematicamente valida pero fuera de rango -- fue justo lo que paso en la
# fase de busqueda visual de este nodo (warnings de Webots "too big/too low
# requested position" vistos en pruebas reales), sin que rompiera el agarre
# porque ese SI usa GRASP_R, una orientacion con la que nunca se vio el
# problema. Se acota aqui solo la orientacion CAMERA_DOWN_R, que es la que
# lo sufria.
JOINT_LIMITS = [
    (-2.9671, 2.9671),
    (-1.8326, 1.8326),
    (-2.9671, 2.9671),
    (-3.1416, -0.4000),
    (-2.9671, 2.9671),
    (-0.0873, 3.8223),
    (-2.9671, 2.9671),
]


def within_joint_limits(thetas, margin_deg=2.0):
    margin = math.radians(margin_deg)
    return all(
        (JOINT_LIMITS[i][0] + margin) <= t <= (JOINT_LIMITS[i][1] - margin)
        for i, t in enumerate(thetas)
    )


def solve_within_limits(target_pos, target_r, seed, n_random_restarts=8, rng_seed=0, n_joints=7):
    """Como inverse_kinematics, pero prefiere una solucion que respete
    JOINT_LIMITS. Prueba primero la semilla dada (para no saltar de
    configuracion si no hace falta); solo si esa solucion converge fuera de
    rango, prueba semillas aleatorias y se queda con la de menor error entre
    las que si cumplen los limites. Devuelve (thetas, encontrada_dentro_de_limites)."""
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
    """Si only_letter no es None (p.ej. 'B'), ignora los demas colores: evita
    que el servoing salte de perseguir un cubo a perseguir otro cuando dos
    quedan con tamano parecido en la imagen (visto en pruebas reales con
    'cualquier cubo')."""
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
            if area < 200:
                continue
            M = cv2.moments(c)
            if M['m00'] == 0:
                continue
            cx = M['m10'] / M['m00']
            cy = M['m01'] / M['m00']
            if best is None or area > best[0]:
                best = (area, cx, cy, letter, name)
    return best


class VisionLiftCube(Node):
    def __init__(self):
        super().__init__('vision_lift_cube')

        self.declare_parameter('robot_base_x', 0.5)
        self.declare_parameter('robot_base_y', -0.3)
        self.declare_parameter('robot_base_z', 0.74)

        # Punto de partida para la busqueda visual: centro aproximado de la
        # zona donde suelen estar los cubos. Si no se ve ninguno desde aqui,
        # se prueban los puntos de SEARCH_FALLBACK_XY (ver _locate_cube).
        self.declare_parameter('search_x', 0.4)
        self.declare_parameter('search_y', -0.1)
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

        # 'R'/'G'/'B' para buscar solo ese color (evita el salto entre
        # cubos con tamano parecido en imagen); '' = cualquiera.
        self.declare_parameter('target_color', '')
        # Si es false, se queda posicionado (hover) encima del cubo
        # encontrado, sin agarrarlo ni moverlo -- util para comprobar
        # visualmente que la localizacion es correcta antes de fiarse del
        # agarre completo.
        self.declare_parameter('grasp_enabled', True)

        self.target_letter = (self.get_parameter('target_color').value or '').strip().upper() or None
        self.grasp_enabled = bool(self.get_parameter('grasp_enabled').value)

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

        self.pub_joint = self.create_publisher(Float64MultiArray, '/joint_positions', 10)
        self.pub_gripper = self.create_publisher(Float64, '/gripper_position', 10)
        self.pub_led = self.create_publisher(String, '/comando_led', 10)
        self.sub_img = self.create_subscription(Image, '/panda_camera/image_color', self._on_image, 10)

        self.latest_bgr = None
        self.cx_img, self.cy_img = 160.0, 120.0

        self.plan = None
        self.state = 'MOVING'
        self.plan_index = -1
        self.prev_raw = np.array(HOME_POSITIONS) - self.offsets_rad
        self.seg_start_time = None
        self.seg_duration = None
        self.settle_until = None

        # Espera a que el driver se suscriba y localiza el cubo ANTES de
        # arrancar el timer: son pasos bloqueantes con su propio bucle de
        # rclpy.spin_once(), y llamarlos desde dentro de un callback de
        # timer (que ya corre bajo un rclpy.spin() externo) rompe la
        # recepcion de imagenes (spin_once anidado/reentrante). Se
        # comprobo el fallo en Webots real: sin este cambio nunca llegaba
        # ninguna imagen a self.latest_bgr durante la busqueda.
        self._wait_for_subscribers()
        located = self._locate_cube()
        if located is None:
            self.get_logger().error('No se ha detectado ningun cubo. Nada que hacer.')
            self.state = 'DONE'
            self.timer = self.create_timer(TICK_PERIOD_S, self._tick)
            return
        xy, led_letter, color_name = located
        cube_world = np.array([xy[0], xy[1], CUBE_Z])
        self.get_logger().info(
            f'Cubo {color_name} localizado en {cube_world.tolist()}. Calculando plan de agarre...'
        )
        self.plan, home_raw, self.led_letter = self._build_grasp_plan(
            cube_world, led_letter, color_name, hover_only=not self.grasp_enabled
        )
        self.color_name = color_name
        self.prev_raw = home_raw
        if self.grasp_enabled:
            self._publish_gripper(self.gripper_open)
        else:
            self.get_logger().info(
                f'grasp_enabled=false: solo se ira encima del cubo {color_name} y se quedara ahi.'
            )

        self.timer = self.create_timer(TICK_PERIOD_S, self._tick)
        self._start_segment(0)

    def _wait_for_subscribers(self):
        self.get_logger().info('Esperando a que el driver se suscriba...')
        elapsed = 0.0
        while rclpy.ok() and elapsed < self.max_wait_seconds:
            ready = (
                self.pub_joint.get_subscription_count() > 0
                and self.pub_gripper.get_subscription_count() > 0
            )
            if ready:
                self.get_logger().info('Driver suscrito. Buscando un cubo con la camara...')
                return
            rclpy.spin_once(self, timeout_sec=0.1)
            elapsed += 0.1
        self.get_logger().error(
            f'Nadie se ha suscrito tras {self.max_wait_seconds:.0f}s '
            "(revisa que 'ros2 launch panda_controller robot_launch.py' este corriendo)."
        )

    def _on_image(self, msg):
        arr = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, 4))
        self.latest_bgr = cv2.cvtColor(arr, cv2.COLOR_BGRA2BGR)

    def _goto_raw(self, target_base, target_r, seed):
        if target_r is CAMERA_DOWN_R:
            # Fase de busqueda: preferir una solucion dentro de los limites
            # reales de articulacion (ver JOINT_LIMITS / solve_within_limits)
            # para no repetir los warnings de Webots vistos en pruebas reales.
            thetas, ok = solve_within_limits(target_base, target_r, seed)
            if not ok:
                self.get_logger().warn(
                    '[busqueda] no se encontro ninguna solucion de IK dentro de los '
                    'limites reales de articulacion para este punto; se usa la mejor '
                    'disponible (puede generar un warning de Webots).'
                )
        else:
            thetas, ok, iters, err = inverse_kinematics(target_base, target_r, seed)
            if not ok:
                thetas, ok, iters, err = solve_with_restarts(target_base, target_r, heuristic_seed=seed)
        wrapped = (thetas + self.offsets_rad + math.pi) % (2 * math.pi) - math.pi
        msg = Float64MultiArray()
        msg.data = wrapped.tolist()
        for _ in range(10):
            self.pub_joint.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.1)
        return thetas, ok

    def _grab_frame(self):
        self.latest_bgr = None
        start = time.time()
        while rclpy.ok() and self.latest_bgr is None and time.time() - start < 3.0:
            rclpy.spin_once(self, timeout_sec=0.1)
        return self.latest_bgr

    def _observe_at(self, xy, seed, z=None):
        target_base = np.array([xy[0], xy[1], z if z is not None else self.search_z]) - self.base
        thetas, converged = self._goto_raw(target_base, CAMERA_DOWN_R, seed)
        # Esperar a que el brazo haya llegado de verdad antes de capturar,
        # no un tiempo fijo: si el punto anterior estaba lejos (por ejemplo,
        # el brazo seguia posicionado sobre un cubo de una busqueda previa)
        # 0.9s no bastaba y la foto se tomaba a medio movimiento -- bug
        # visto en Webots real, la deteccion fallaba de forma intermitente
        # segun de donde viniera el brazo.
        delta = float(np.max(np.abs(np.array(thetas) - np.array(seed))))
        settle = max(0.9, delta / JOINT_VELOCITY_RAD_S + 0.5)
        time.sleep(settle)
        frame = self._grab_frame()
        blob = detect_largest_blob(frame, only_letter=self.target_letter) if frame is not None else None
        return thetas, converged, blob

    # Offsets alrededor de search_x/search_y a probar si el punto de partida
    # no ve ningun cubo (por ejemplo, si un intento previo los ha dejado
    # fuera de esa zona). Cubre una region bastante mas amplia que el campo
    # de vision de un solo punto.
    SEARCH_FALLBACK_OFFSETS = [
        (0.0, 0.0),
        (-0.15, 0.15), (0.15, 0.15), (-0.15, -0.15), (0.15, -0.15),
        (0.0, 0.3), (0.0, -0.3), (-0.25, 0.0), (0.25, 0.0),
    ]

    def _find_initial_view(self):
        """Prueba search_xy y, si no ve nada, una serie de offsets alrededor
        hasta encontrar un punto desde el que se detecte algun cubo. Devuelve
        (xy, seed_thetas, blob) o (None, None, None) si no encuentra nada."""
        pan0 = math.atan2(self.search_xy[1] - self.base[1], self.search_xy[0] - self.base[0])
        base_seed = np.array([pan0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785])
        # area minima para aceptar la vista de entrada: una deteccion muy
        # pequena suele ser solo el borde del cubo asomando en el extremo de
        # la imagen (visto en pruebas reales), y usarla como punto de partida
        # del servoing da una estimacion inicial mala -- mejor seguir
        # probando otros offsets hasta encontrar una vista mas centrada.
        MIN_GOOD_AREA = 600
        best = None  # (area, xy, thetas, blob)
        for dx, dy in self.SEARCH_FALLBACK_OFFSETS:
            xy = clamp_to_table(self.search_xy + np.array([dx, dy]))
            thetas, conv, blob = self._observe_at(xy, base_seed)
            if blob is None:
                self.get_logger().warn(f'[localizar cubo] xy={xy.tolist()} sin deteccion, probando otro punto.')
                continue
            area = blob[0]
            if best is None or area > best[0]:
                best = (area, xy, thetas, blob)
            if area >= MIN_GOOD_AREA:
                if (dx, dy) != (0.0, 0.0):
                    self.get_logger().info(f'[localizar cubo] encontrado en el offset de busqueda ({dx},{dy}).')
                return xy, thetas, blob
            self.get_logger().warn(
                f'[localizar cubo] xy={xy.tolist()} deteccion muy pequena (area={area:.0f}), '
                'probando otro punto.'
            )
        if best is not None:
            self.get_logger().warn(
                f'[localizar cubo] ninguna vista alcanzo el area minima; uso la mejor '
                f'disponible (area={best[0]:.0f}) en xy={best[1].tolist()}.'
            )
            return best[1], best[2], best[3]
        return None, None, None

    def _gauss_newton_servo(self, xy, z, seed, blob0, max_outer, threshold_px, d0=0.02):
        """Bucle de servoing (un solo nivel de altura z): re-mide el
        Jacobiano pixel/mundo LOCAL en cada paso (2 fotos extra, mover un
        poco en X y un poco en Y) y da un paso de Newton amortiguado hacia
        el centro de la imagen. Devuelve (xy, letra, nombre, error_px,
        thetas) de la mejor observacion conseguida."""
        d = d0
        best = None
        xy = xy.copy()
        for outer in range(max_outer):
            if outer > 0:
                thetas0, conv0, blob0 = self._observe_at(xy, seed, z=z)
                seed = thetas0
            else:
                thetas0 = seed
            if blob0 is None:
                self.get_logger().warn(f'[localizar cubo] z={z} xy={xy.tolist()} sin deteccion.')
                break
            area0, u0, v0, letter, name = blob0
            e = np.array([self.cx_img - u0, self.cy_img - v0])
            enorm = float(np.linalg.norm(e))
            self.get_logger().info(
                f'[localizar cubo] z={z} paso {outer} xy={xy.tolist()} color={name} area={area0:.0f} '
                f'pixel=({u0:.1f},{v0:.1f}) error_px={enorm:.1f}'
            )
            if best is None or enorm < best[3]:
                best = (xy.copy(), letter, name, enorm, thetas0)
            if enorm < threshold_px:
                self.get_logger().info(f'[localizar cubo] z={z} centrado tras {outer + 1} pasos.')
                return xy.copy(), letter, name, enorm, thetas0
            thetas_x, _, blobx = self._observe_at(clamp_to_table(xy + np.array([d, 0.0])), thetas0, z=z)
            thetas_y, _, bloby = self._observe_at(clamp_to_table(xy + np.array([0.0, d])), thetas0, z=z)
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
        """Localizacion en dos pasadas: una GRUESA, en alto (mejor campo de
        vision para encontrar el cubo, peor resolucion/precision), y una
        FINA justo despues, mas cerca de la mesa, partiendo del resultado
        de la primera -- se comprobo en Webots real que una sola pasada en
        alto se queda con demasiado error (varios cm, a veces >10cm, el
        brazo termina claramente fuera del cubo) porque el cubo ocupa pocos
        pixeles y el centroide es mas ruidoso. Devuelve (xy_mundo,
        letra_led, nombre_color) o None si no encuentra nada."""
        xy, seed, blob0 = self._find_initial_view()
        if blob0 is None:
            self.get_logger().error('[localizar cubo] no se ha visto ningun cubo en ningun punto de busqueda.')
            return None

        coarse = self._gauss_newton_servo(xy, self.search_z, seed, blob0, max_outer=6, threshold_px=20.0)
        if coarse is None:
            return None
        xy_c, letter_c, name_c, err_c, thetas_c = coarse
        self.get_logger().info(
            f'[localizar cubo] pasada gruesa: xy={xy_c.tolist()} color={name_c} error_px={err_c:.1f}'
        )

        # No bajar demasiado: se comprobo en Webots real que, muy cerca, la
        # propia muneca/mano del brazo tapa buena parte de la imagen de la
        # camara (mecanicamente inevitable, esta montada ahi) y la pasada
        # fina deja de ver el cubo.
        refine_z = max(self.search_z - 0.5, self.base[2] + 0.45)
        thetas_r, conv_r, blob_r = self._observe_at(xy_c, thetas_c, z=refine_z)
        fine = self._gauss_newton_servo(
            xy_c, refine_z, thetas_r, blob_r, max_outer=6, threshold_px=10.0, d0=0.01
        )
        if fine is None:
            self.get_logger().warn('[localizar cubo] la pasada fina no vio nada; uso el resultado de la gruesa.')
            return xy_c, letter_c, name_c
        xy_f, letter_f, name_f, err_f, _ = fine
        self.get_logger().info(
            f'[localizar cubo] pasada fina: xy={xy_f.tolist()} color={name_f} error_px={err_f:.1f}'
        )
        return xy_f, letter_f, name_f
        if best is not None:
            self.get_logger().warn(
                f'[localizar cubo] no convergio del todo, uso mejor observacion: '
                f'xy={best[1].tolist()} color={best[3]} error_px={best[0]:.1f}'
            )
            return best[1], best[2], best[3]
        return None

    def _build_grasp_plan(self, cube_world, led_letter, color_name, hover_only=False):
        home_raw = np.array(HOME_POSITIONS) - self.offsets_rad
        if hover_only:
            # Solo comprobar visualmente la localizacion: ir encima del
            # cubo y quedarse ahi, sin tocar la pinza.
            waypoints_world = [
                (f'hover_{color_name}', cube_world + np.array([0.0, 0.0, self.hover]), None),
            ]
        else:
            waypoints_world = [
                (f'hover_{color_name}', cube_world + np.array([0.0, 0.0, self.hover]), None),
                (f'grasp_{color_name}', cube_world + np.array([0.0, 0.0, self.grasp]), 'close'),
                (f'lift_{color_name}', cube_world + np.array([0.0, 0.0, self.lift]), None),
                (f'lower_{color_name}', cube_world + np.array([0.0, 0.0, self.grasp]), 'open'),
                ('retreat', cube_world + np.array([0.0, 0.0, self.hover]), None),
            ]
        plan = []
        seed_raw = home_raw
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
            else:
                self.get_logger().info(f'[{name}] IK convergida en {iters} iter (error={err:.6f}).')
            plan.append({'name': name, 'raw': thetas, 'gripper_action': gripper_action})
            seed_raw = thetas
        return plan, home_raw, led_letter

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
        if self.state == 'MOVING':
            now = self.get_clock().now().nanoseconds / 1e9
            if now - self.seg_start_time >= self.seg_duration:
                step = self.plan[self.plan_index]
                self.prev_raw = step['raw']
                if step['gripper_action'] == 'close':
                    self._publish_gripper(self.gripper_closed)
                    self.get_logger().info(
                        f'[{step["name"]}] Pinza: CERRAR a {self.gripper_closed:.4f} m '
                        f'(agarrando cubo {self.color_name}).'
                    )
                    self.pub_led.publish(String(data=self.led_letter))
                    self.get_logger().info(f"[{step['name']}] LED -> {self.led_letter}")
                    self.settle_until = now + self.grasp_settle_seconds
                    self.state = 'GRIP_SETTLE'
                elif step['gripper_action'] == 'open':
                    self._publish_gripper(self.gripper_open)
                    self.get_logger().info(
                        f'[{step["name"]}] Pinza: ABRIR a {self.gripper_open:.4f} m '
                        '(dejando el cubo otra vez en la mesa).'
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
            if self.grasp_enabled:
                self.get_logger().info(
                    f'Secuencia completada: cubo {self.color_name} localizado por vision, '
                    'agarrado, levantado y devuelto a su sitio.'
                )
            else:
                self.get_logger().info(
                    f'Secuencia completada: cubo {self.color_name} localizado por vision, '
                    'brazo posicionado encima (sin agarrar).'
                )
        else:
            self._start_segment(next_index)


def main(args=None):
    rclpy.init(args=args)
    node = VisionLiftCube()
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
