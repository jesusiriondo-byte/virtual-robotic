#!/usr/bin/env python3
"""Localizacion del cubo rojo por la wrist_camera, para corregir la deriva
de posicion en nodos que repiten agarres open-loop (ver cube_shuttle_demo.py).

Metodo: en vez de reconstruir analiticamente el offset camara->TCP (lo que
llevo una sesion entera de depuracion en best_color_repeat_lift.py contra la
cinematica DH manual, ver resumen_proyecto_panda.md), se hace un servoing
visual EMPIRICO por Newton amortiguado: observar el cubo, sondear el
jacobiano imagen/mundo moviendose +3cm en X e Y por separado, y resolver el
paso que centraria el blob en la imagen -- igual principio que
best_color_repeat_lift.py pero con la cinematica ikpy validada
(panda_ikpy_kinematics.py) en vez de la DH manual.

DOS PASADAS, altura variable (sesion 2026-08-26): una sola altura fija de
observacion resulto tener una escala pixel/metro que varia segun la zona del
espacio de trabajo -- se vieron casos con error de pixel bajo (por debajo
del umbral de convergencia) que en realidad correspondian a 3-5cm de error
real (comparado contra orientation_probe), porque a esa altura/config
concreta pocos pixeles ya representaban mucho terreno. La solucion, igual
que ya usaba best_color_repeat_lift.py: pasada GRUESA desde mas lejos (campo
de vision amplio, tolerante a partir de un guess muy malo) seguida de una
pasada FINA mas cerca del cubo (mejor resolucion pixel/metro, umbral mucho
mas estricto) -- no una altura fija unica."""

import time

import cv2
import numpy as np
import rclpy
from sensor_msgs.msg import Image

from panda_controller.panda_ikpy_kinematics import GRASP_R

RED_RANGES = [((0, 200, 30), (8, 255, 255)), ((172, 200, 30), (179, 255, 255))]
MIN_CONTOUR_AREA = 200
IMG_CX, IMG_CY = 160.0, 120.0

CUBE_TABLE_Z = 0.77
COARSE_Z = CUBE_TABLE_Z + 0.35   # campo de vision amplio, tolerante a un guess muy malo
FINE_Z = CUBE_TABLE_Z + 0.18     # mas cerca: mejor resolucion pixel/metro para el ajuste fino

PROBE_DELTA = 0.03       # paso para sondear el jacobiano imagen/mundo (m)
DAMPING = 0.7            # amortiguacion del paso de Newton (evita pasarse de largo)
COARSE_CONVERGE_PX = 25.0
FINE_CONVERGE_PX = 6.0
MAX_ITERS = 4
JOINT_VELOCITY_DEG_S = 57.3  # 1 rad/s, la velocidad forzada en my_robot_driver.py
MAX_STEP_M = 0.08        # tope al paso de Newton por iteracion (evita disparos
                          # por un jacobiano mal condicionado en un punto malo)


def detect_red_centroid(bgr):
    """Devuelve (area, cx, cy) del contorno rojo mas grande, o None."""
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
    for lo, hi in RED_RANGES:
        mask |= cv2.inRange(hsv, np.array(lo), np.array(hi))
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    best = None
    for c in contours:
        area = cv2.contourArea(c)
        if area < MIN_CONTOUR_AREA:
            continue
        M = cv2.moments(c)
        if M['m00'] == 0:
            continue
        cx, cy = M['m10'] / M['m00'], M['m01'] / M['m00']
        if best is None or area > best[0]:
            best = (area, cx, cy)
    return best


class CubeLocator:
    """Composicion, no herencia: el nodo dueño le pasa su nodo (para
    create_subscription y spin_once), su instancia de PandaIkpyKinematics y
    una funcion publish_fn(real_theta) que hace lo mismo que ya hace el
    nodo para mover el brazo (para no duplicar la logica de publicacion /
    sleep / registro de la ultima pose real)."""

    def __init__(self, node, kin, publish_fn):
        self.node = node
        self.kin = kin
        self.publish_fn = publish_fn
        self.latest_bgr = None
        node.create_subscription(Image, '/panda_camera/image_color', self._on_image, 10)

    def _on_image(self, msg):
        arr = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, 4))
        self.latest_bgr = cv2.cvtColor(arr, cv2.COLOR_BGRA2BGR)

    def _observe(self, x, y, z, seed, settle):
        th, seed, ok = self.kin.solve(x, y, z, GRASP_R, seed)
        if not ok:
            return None, seed
        # Espera proporcional al salto articular real, no un tiempo fijo:
        # la primera observacion de cada localizacion puede venir de muy
        # lejos (p.ej. justo tras "subir a home" a otra XY), y con un
        # settle fijo demasiado corto la foto se toma con el brazo todavia
        # en movimiento -- visto en la sesion 2026-08-26 (iter 0 daba un
        # error de 110px incluso apuntando exactamente al sitio correcto).
        prev = getattr(self.node, 'real_theta', th)
        delta_deg = float(np.degrees(np.max(np.abs(th - prev))))
        wait = max(settle, delta_deg / JOINT_VELOCITY_DEG_S + 0.4)
        self.publish_fn(th)
        self.latest_bgr = None
        time.sleep(wait)
        t0 = time.time()
        while self.latest_bgr is None and time.time() - t0 < 3.0:
            rclpy.spin_once(self.node, timeout_sec=0.1)
        blob = detect_red_centroid(self.latest_bgr) if self.latest_bgr is not None else None
        return blob, seed

    def _servo_pass(self, x_guess, y_guess, z, seed, converge_px, label):
        """Una pasada de servoing Newton amortiguado A UNA ALTURA FIJA.
        Devuelve (x, y, seed, err_px, encontrado_bool) -- se queda con la
        MEJOR lectura vista (menor error), no la ultima: un jacobiano mal
        condicionado en una iteracion concreta puede empeorar el resultado
        (visto en sesion 2026-08-26), el limite de paso ya lo amortigua
        pero quedarse con la mejor es una segunda red de seguridad barata."""
        x, y = x_guess, y_guess
        best = None  # (x, y, err_px)
        for it in range(MAX_ITERS):
            b0, seed = self._observe(x, y, z, seed, settle=1.2 if it == 0 else 0.6)
            if b0 is None:
                self.node.get_logger().warn(f'[vision:{label}] sin deteccion en ({x:.4f},{y:.4f}).')
                break
            _, u0, v0 = b0
            e = np.array([IMG_CX - u0, IMG_CY - v0])
            err_px = float(np.linalg.norm(e))
            self.node.get_logger().info(
                f'[vision:{label}] iter {it}: pos=({x:.4f},{y:.4f}) z={z:.2f} '
                f'blob=({u0:.1f},{v0:.1f}) error_px={err_px:.1f}')
            if best is None or err_px < best[2]:
                best = (x, y, err_px)
            if err_px < converge_px:
                break
            bx, seed = self._observe(x + PROBE_DELTA, y, z, seed, settle=0.6)
            by, seed = self._observe(x, y + PROBE_DELTA, z, seed, settle=0.6)
            if bx is None or by is None:
                self.node.get_logger().warn(f'[vision:{label}] deteccion perdida sondeando el jacobiano.')
                break
            _, ux, vx = bx
            _, uy, vy = by
            J = np.array([[(ux - u0) / PROBE_DELTA, (uy - u0) / PROBE_DELTA],
                          [(vx - v0) / PROBE_DELTA, (vy - v0) / PROBE_DELTA]])
            try:
                delta = np.linalg.solve(J.T @ J + 1e-6 * np.eye(2), J.T @ e)
            except np.linalg.LinAlgError:
                break
            dn = float(np.linalg.norm(delta))
            if dn > MAX_STEP_M:
                delta = delta / dn * MAX_STEP_M
            x, y = x + DAMPING * delta[0], y + DAMPING * delta[1]
        if best is None:
            return x_guess, y_guess, seed, None, False
        return best[0], best[1], seed, best[2], True

    def locate(self, x_guess, y_guess, seed):
        """Devuelve (x, y, seed_actualizada, encontrado_bool). Pasada
        gruesa (COARSE_Z, campo de vision amplio) para acercarse aunque el
        guess sea malo, luego pasada fina (FINE_Z, mas cerca del cubo =
        mejor resolucion pixel/metro) para afinar de verdad."""
        xc, yc, seed, err_c, found_c = self._servo_pass(
            x_guess, y_guess, COARSE_Z, seed, COARSE_CONVERGE_PX, 'gruesa')
        if not found_c:
            self.node.get_logger().warn(
                f'[vision] pasada gruesa sin deteccion cerca de ({x_guess:.3f},{y_guess:.3f}).')
            return x_guess, y_guess, seed, False

        xf, yf, seed, err_f, found_f = self._servo_pass(
            xc, yc, FINE_Z, seed, FINE_CONVERGE_PX, 'fina')
        if not found_f:
            self.node.get_logger().warn(
                '[vision] pasada fina sin deteccion; uso el resultado de la gruesa.')
            return xc, yc, seed, True
        return xf, yf, seed, True
