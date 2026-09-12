#!/usr/bin/env python3
"""Localizacion del cubo por la camara cenital fija (worlds/panda_un_cubo.wbt,
DEF OVERHEAD_CAM), en vez de la wrist_camera servoing.

Por que una camara fija en vez de la de muneca (sesion 2026-08-26): el
servoing con la camara de muneca demostro ser fragil -- un solo fallo de
deteccion durante el sondeo del jacobiano (campo de vision estrecho al
acercarse para la pasada fina) contaminaba en cascada las relocalizaciones
siguientes, con errores de hasta 9cm que ademas se "veian" como convergidos
(error de pixel bajo) porque la escala pixel/metro no es uniforme en todo
el espacio de trabajo cuando la camara se mueve con el brazo.

Con una camara FIJA la geometria es constante y conocida de una vez por
todas (proyeccion en pinhole recta hacia abajo), asi que la localizacion es
UN SOLO DISPARO -- sin mover el brazo, sin iterar, sin jacobiano: se calcula
la posicion mundo directamente a partir del pixel del centroide. Validado en
la sesion 2026-08-25: ~1.4mm de error frente a orientation_probe."""

import time

import cv2
import numpy as np
import rclpy
from sensor_msgs.msg import Image

# DEF OVERHEAD_CAM Robot { translation 0.5 0 2.27 } con Camera { rotation 0 1 0 1.5708 }.
# Subida y centrada en la mesa (antes 0.5,0.2,1.4, solo veia ~30cm de radio)
# para cubrir la mesa entera (0.8m x 1.2m) con margen -- sesion 2026-08-26.
CAM_X, CAM_Y, CAM_Z = 0.5, 0.0, 2.27
FOV = 0.9
IMG_W, IMG_H = 320, 240

# Rango mas laxo que el de la wrist_camera: bajo esta camara/iluminacion la
# cara visible del cubo aparece algo menos saturada. Verde/azul reusan los
# mismos rangos que best_color_repeat_lift.py (ya calibrados alli), solo
# ensanchados un poco en saturacion/valor como el rojo.
COLOR_RANGES = {
    'R': [((0, 150, 20), (10, 255, 255)), ((160, 150, 20), (179, 255, 255))],
    'G': [((40, 150, 20), (85, 255, 255))],
    'B': [((95, 150, 20), (130, 255, 255))],
}
COLOR_NAMES = {'R': 'rojo', 'G': 'verde', 'B': 'azul'}
MIN_CONTOUR_AREA = 80
# Techo de area (sesion 2026-08-27): sin esto, la propia textura de la
# cinta (caucho "dotted", con motas oscuras/rojizas) podia colarse por el
# rango de rojo y ganar por area a un cubo real de verdad -- se vio en
# vivo: 3 "cubos rojos" seguidos detectados en la EXACTA misma coordenada,
# imposible para 3 cubos distintos. Un cubo de 6cm visto desde la camara
# cenital nunca da un contorno mucho mayor que esto; una mancha de textura
# de toda la cinta si.
MAX_CONTOUR_AREA = 600

# Recorte de pixeles a la MESA (worlds/panda_un_cubo.wbt: Table translation
# 0.5 0 0, size 0.8 1.2 -> x:[0.1,0.9] y:[-0.6,0.6]), con margen hacia
# dentro. Bug real (sesion 2026-08-26): al subir la camara para ver la mesa
# entera, el suelo ajedrezado (baldosas granates) entro en el encuadre y
# varias baldosas caian en el mismo rango HSV que el cubo rojo, a veces con
# mas area -- "el contorno rojo mas grande" pasaba a ser suelo, no cubo.
# Recortar la busqueda al rectangulo de la mesa (calculado con la misma
# geometria de pixel_to_world, en pixeles no en mundo) elimina el suelo de
# raiz en vez de intentar afinar el rango de color.
_TABLE_X_RANGE = (0.15, 0.85)
_TABLE_Y_RANGE = (-0.55, 0.55)


class OverheadLocator:
    """Antes (sesion 2026-08-26) todo esto era constantes de modulo -- valia
    mientras solo hubiera UNA camara cenital fija. Con la celda industrial
    (sesion 2026-08-27) hay una segunda camara sobre el punto de recogida
    del Sorter, con su propia pose/topic/recorte de mesa -- se parametriza
    por instancia en vez de duplicar el fichero entero para la segunda
    camara. Los valores por defecto son EXACTAMENTE los de siempre (camara
    del Loader/panda_un_cubo.wbt), asi que las llamadas existentes
    (`OverheadLocator(self)`, sin mas argumentos) no cambian de
    comportamiento."""

    def __init__(self, node, topic='/overhead_camera/image_color',
                 cam_x=CAM_X, cam_y=CAM_Y, cam_z=CAM_Z,
                 fov=FOV, img_w=IMG_W, img_h=IMG_H,
                 table_x_range=_TABLE_X_RANGE, table_y_range=_TABLE_Y_RANGE,
                 prefer_closest_to_wall=False):
        self.node = node
        self.cam_x, self.cam_y, self.cam_z = cam_x, cam_y, cam_z
        self.fov, self.img_w, self.img_h = fov, img_w, img_h
        self.table_x_range, self.table_y_range = table_x_range, table_y_range
        # Sesion 2026-09-08, peticion real del usuario: "si sorter ve cubos
        # que intente ir siempre a por el mas pegado a la pared". Antes,
        # con varios cubos visibles a la vez (misma fila o filas distintas
        # -- "cajas con colores repetidos en varias filas", ya documentado
        # como caso real), tanto _detect_centroid como detect_color elegian
        # por AREA del contorno, sin relacion con la posicion -- podia
        # tocarle un cubo de mas atras en la cola por puro ruido de camara.
        # Con esto activado (SOLO para el Sorter, ver sorter_demo.py -- el
        # Loader no tiene pared/cola, sigue por area de siempre), se elige
        # el candidato con mayor Y de mundo: como ningun cubo puede estar
        # mas alla de BELT_END_STOP, el de mayor Y es siempre el mas cerca
        # de la pared, sin necesidad de conocer su coordenada exacta.
        self.prefer_closest_to_wall = prefer_closest_to_wall
        self.latest_bgr = None
        self.latest_msg = None
        # Guardado como atributo (sesion 2026-08-30, selector de robot en
        # teleop_gui.py): antes se creaba y se tiraba la referencia, valia
        # mientras solo hiciera falta UN locator por proceso. Para poder
        # cambiar de camara en caliente (Loader <-> Sorter) sin reiniciar
        # la ventana, quien cree un OverheadLocator nuevo debe poder cerrar
        # antes la suscripcion del anterior con
        # node.destroy_subscription(locator.sub).
        self.sub = node.create_subscription(Image, topic, self._on_image, 10)

    def _on_image(self, msg):
        # Solo se guarda el mensaje (sesion 2026-09-11): antes se convertia con
        # OpenCV CADA fotograma recibido, se fuera a usar o no. La conversion
        # se hace ahora bajo demanda en _esperar_frame(), con la misma
        # semantica de siempre: cada consulta descarta lo anterior y espera un
        # fotograma NUEVO.
        self.latest_msg = msg

    def _esperar_frame(self, timeout):
        """Espera un fotograma nuevo (hasta 'timeout' s) y lo deja convertido
        a BGR en self.latest_bgr. Devuelve ese array, o None si no llega."""
        self.latest_msg = None
        self.latest_bgr = None
        t0 = time.time()
        while self.latest_msg is None and time.time() - t0 < timeout:
            rclpy.spin_once(self.node, timeout_sec=0.1)
        msg = self.latest_msg
        if msg is None:
            return None
        arr = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, 4))
        self.latest_bgr = cv2.cvtColor(arr, cv2.COLOR_BGRA2BGR)
        return self.latest_bgr

    def _world_to_pixel(self, x, y, target_z):
        d = self.cam_z - target_z
        scale = d * np.tan(self.fov / 2.0) / (self.img_w / 2.0)
        px = self.img_w / 2.0 + (self.cam_y - y) / scale
        py = self.img_h / 2.0 + (self.cam_x - x) / scale
        return px, py

    def _table_pixel_mask_bounds(self, target_z=0.74):
        corners = [
            self._world_to_pixel(x, y, target_z)
            for x in self.table_x_range for y in self.table_y_range
        ]
        xs = [c[0] for c in corners]
        ys = [c[1] for c in corners]
        return int(min(xs)), int(max(xs)), int(min(ys)), int(max(ys))

    def _candidate_key(self, area, cx, cy):
        """Criterio de desempate entre varios contornos validos a la vez
        (mismo color en fila, o colores distintos -- ver
        prefer_closest_to_wall en __init__). Por defecto (Loader, y
        cualquiera que no lo active) sigue siendo el area, como siempre.
        Con prefer_closest_to_wall=True, gana el de mayor Y de mundo -- la
        Z usada aqui es arbitraria (0.77, altura tipica de mesa/cinta) y no
        afecta al ORDEN entre candidatos, solo compara posiciones relativas
        dentro del mismo fotograma."""
        if not self.prefer_closest_to_wall:
            return area
        _, y = self._pixel_to_world(cx, cy, 0.77)
        return y

    def _detect_centroid(self, bgr, color='R'):
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
        for lo, hi in COLOR_RANGES[color]:
            mask |= cv2.inRange(hsv, np.array(lo), np.array(hi))
        x0, x1, y0, y1 = self._table_pixel_mask_bounds()
        table_mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
        table_mask[max(0, y0):min(hsv.shape[0], y1), max(0, x0):min(hsv.shape[1], x1)] = 255
        mask &= table_mask
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        best, best_key = None, None
        for c in contours:
            area = cv2.contourArea(c)
            if area < MIN_CONTOUR_AREA or area > MAX_CONTOUR_AREA:
                continue
            M = cv2.moments(c)
            if M['m00'] == 0:
                continue
            cx, cy = M['m10'] / M['m00'], M['m01'] / M['m00']
            # Angulo del rectangulo minimo que envuelve el contorno --
            # sesion 2026-08-27: la pinza siempre cerraba asumiendo el
            # cubo perfectamente alineado con el mundo (giro fijo de
            # 45 grados), y tras muchos ciclos de prueba seguidos algun
            # cubo acabo girado de verdad (empujones, rebotes en la
            # cinta) -- la pinza cerraba "en el aire" por un lado porque
            # las caras reales ya no estaban donde el codigo asumia.
            angle_deg = cv2.minAreaRect(c)[2]
            key = self._candidate_key(area, cx, cy)
            if best is None or key > best_key:
                best = (area, cx, cy, angle_deg)
                best_key = key
        return best

    def _pixel_to_world(self, px, py, target_z):
        """Proyeccion pinhole recta hacia abajo: la camara mira exactamente
        en -Z, con "arriba" de imagen = +X mundo y "derecha" de imagen = -Y
        mundo (consecuencia de la rotacion 0 1 0 1.5708 aplicada a una
        Camera con orientacion por defecto -- misma pose en las dos camaras
        de la celda industrial, solo cambia la traslacion). d = distancia
        camara-plano objetivo."""
        d = self.cam_z - target_z
        scale = d * np.tan(self.fov / 2.0) / (self.img_w / 2.0)  # m/px
        dpx = px - self.img_w / 2.0
        dpy = py - self.img_h / 2.0
        x = self.cam_x - dpy * scale
        y = self.cam_y - dpx * scale
        return x, y

    def locate(self, target_z, color='R', timeout=3.0):
        """Devuelve (x, y) o None. Un solo disparo: no mueve el brazo."""
        if self._esperar_frame(timeout) is None:
            self.node.get_logger().warn('[overhead] sin frame de la camara cenital.')
            return None
        blob = self._detect_centroid(self.latest_bgr, color)
        if blob is None:
            name = COLOR_NAMES.get(color, color)
            self.node.get_logger().warn(f'[overhead] cubo {name} no detectado en la imagen cenital.')
            return None
        _, px, py, _ = blob
        x, y = self._pixel_to_world(px, py, target_z)
        self.node.get_logger().info(
            f'[overhead] {COLOR_NAMES.get(color, color)}: blob=({px:.1f},{py:.1f}) -> mundo=({x:.4f},{y:.4f})')
        return x, y

    def locate_with_yaw(self, target_z, color='R', timeout=3.0):
        """Igual que locate(), pero devuelve ademas el giro REAL del cubo
        respecto a su orientacion "de fabrica" (alineado con los ejes del
        mundo), como (x, y, yaw_offset_rad) o None. Sesion 2026-08-27: tras
        muchos ciclos de agarrar/soltar seguidos, un cubo puede acabar
        girado de verdad (empujones, rebotes en la cinta) -- el agarre de
        siempre asumia SIEMPRE cubo sin girar (giro fijo de la pinza), y
        cerraba en el aire por un lado cuando eso dejaba de ser cierto.
        yaw_offset_rad es el residuo dentro de +-45 grados (un cubo visto
        desde arriba es identico cada 90 grados, asi que solo importa el
        resto dentro de ese cuarto de vuelta) -- sumarlo al giro de agarre
        de siempre corrige la pinza a las caras reales."""
        if self._esperar_frame(timeout) is None:
            self.node.get_logger().warn('[overhead] sin frame de la camara cenital.')
            return None
        blob = self._detect_centroid(self.latest_bgr, color)
        if blob is None:
            name = COLOR_NAMES.get(color, color)
            self.node.get_logger().warn(f'[overhead] cubo {name} no detectado en la imagen cenital.')
            return None
        _, px, py, angle_deg = blob
        x, y = self._pixel_to_world(px, py, target_z)
        angle_rad = np.deg2rad(angle_deg) % (np.pi / 2.0)
        if angle_rad > np.pi / 4.0:
            angle_rad -= np.pi / 2.0
        self.node.get_logger().info(
            f'[overhead] {COLOR_NAMES.get(color, color)}: blob=({px:.1f},{py:.1f}) -> '
            f'mundo=({x:.4f},{y:.4f}), giro={np.degrees(angle_rad):.1f} grados')
        return x, y, angle_rad

    def detect_color(self, timeout=3.0):
        """Devuelve el color ('R'/'G'/'B') del primer blob detectado en la
        imagen actual, probando los 3 rangos y quedandose con el ganador
        segun _candidate_key (area por defecto; el mas cercano a la pared
        si prefer_closest_to_wall esta activo, sesion 2026-09-08) -- para
        cuando no se sabe de antemano que color va a llegar (p.ej. el
        Sorter recogiendo lo que sea que traiga la cinta), a diferencia de
        locate() que ya exige saber el color buscado."""
        if self._esperar_frame(timeout) is None:
            self.node.get_logger().warn('[overhead] sin frame de la camara cenital.')
            return None
        best_color, best_blob, best_key = None, None, None
        for color in COLOR_RANGES:
            blob = self._detect_centroid(self.latest_bgr, color)
            if blob is None:
                continue
            area, cx, cy, _ = blob
            key = self._candidate_key(area, cx, cy)
            if best_blob is None or key > best_key:
                best_color, best_blob, best_key = color, blob, key
        if best_color is None:
            self.node.get_logger().warn('[overhead] ningun cubo detectado en la imagen cenital.')
        return best_color
