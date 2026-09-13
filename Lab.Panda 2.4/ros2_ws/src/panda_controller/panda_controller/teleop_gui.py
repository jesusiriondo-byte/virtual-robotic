#!/usr/bin/env python3
"""
Teleoperacion manual del Panda con botones (Tkinter), en vez de teclado
(`teleop_manual.py`). Mismo motor por debajo (misma cinematica DH
modificada duplicada, mismo filtro de salto articular grande para no
"escapar" de golpe -- ver teleop_manual.py para el porque de cada uno de
estos detalles), pero controlado con clicks de raton sobre una ventana.

Necesita que el DISPLAY este reenviado al contenedor (ya lo esta: es el
mismo mecanismo con el que se ve la ventana de Webots, ver
`.devcontainer/docker-compose.yml` -- DISPLAY + /tmp/.X11-unix +
.Xauthority montados en ros2_panda_dev).

Como lanzarlo (con Webots en PLAY y `robot_launch.py` corriendo):

    ros2 run panda_controller teleop_gui
"""

import json
import math
import subprocess
import time
import urllib.error
import urllib.request

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Float64, String, Bool

from panda_controller.overhead_vision import OverheadLocator, COLOR_NAMES
from panda_controller.sorter_demo import LOTE_COLOR_QOS

import tkinter as tk
from tkinter import font as tkfont
from tkinter import messagebox
from tkinter import simpledialog
from tkinter import ttk

# Paleta industrial (sesion 2026-08-27, mismo aspecto que estop_panel.py):
# se reutiliza el panel de teleoperacion tal cual, no uno nuevo, para que
# STOP/REARME y el jog vivan en la misma ventana.
BG = '#1c1c1c'
PANEL_BG = '#2a2a2a'
YELLOW = '#f5c400'
RED = '#c81e1e'
RED_DARK = '#8f1414'
GREEN = '#2e9e3b'
GREY = '#555555'
GREY_TEXT = '#aaaaaa'
TEXT_LIGHT = '#eaeaea'

# Numero de maquina de ESTA celda (sesion 2026-09-13, ver
# Documentacion/analisis_ampliacion_taller.md): reparte que pedidos
# fabrica cada celda cuando haya mas de una -- con una sola celda (estado
# actual) no tiene efecto practico, siempre coge todo. Persistido en un
# fichero dentro de /workspace (montado del host, ver docker-compose.yml
# de Lab.Panda 2.4), para que se mantenga aunque se reinicie el
# contenedor -- a proposito NO se guarda dentro de build/install/log
# (esos se borran con cada colcon build limpio).
CONFIG_MAQUINA_PATH = '/workspace/config_maquina.json'
# Tope de este modelo de celda (peticion explicita del usuario, sesion
# 2026-09-13): el desplegable del panel no deja elegir mas de esto.
MAX_MAQUINAS = 5
# Clave maestra para poder GUARDAR el Nº Maquina (no para nada mas) --
# es un control fisico compartido, sin login como la web, asi que
# cualquiera que se acerque al panel podria cambiarlo sin querer. Mismo
# valor por defecto que TALLER_MASTER_PASSWORD en Taller_Administracion
# a proposito, para que sea la misma clave que ya conoce quien usa el
# proyecto -- no relacionado tecnicamente (viven en proyectos distintos),
# es solo una convencion para no tener dos claves distintas que recordar.
CLAVE_MAQUINA = '1111'


def _cargar_numero_maquina() -> int:
    try:
        with open(CONFIG_MAQUINA_PATH) as f:
            return int(json.load(f).get('numero_maquina', 1))
    except (OSError, ValueError, KeyError, TypeError):
        return 1  # primer arranque, o fichero corrupto: por defecto maquina 1


def _guardar_numero_maquina(numero: int) -> None:
    with open(CONFIG_MAQUINA_PATH, 'w') as f:
        json.dump({'numero_maquina': numero}, f)

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

GRASP_R = np.array([
    [1.0, 0.0, 0.0],
    [0.0, -1.0, 0.0],
    [0.0, 0.0, -1.0],
])


def rz(theta):
    """Rotacion pura alrededor del eje Z de la propia pinza (postmultiplicada
    sobre GRASP_R): gira el agarre sin tocar la orientacion "mirando hacia
    abajo". Mismo convenio que panda_ikpy_kinematics.rz (sesion 2026-08-27,
    botones de giro de la mano en el panel manual)."""
    c, s = np.cos(theta), np.sin(theta)
    return np.array([
        [c, -s, 0.0],
        [s, c, 0.0],
        [0.0, 0.0, 1.0],
    ])


STEP_DEFAULT = 0.01  # 1 cm
STEP_MIN = 0.002
STEP_MAX = 0.05
YAW_STEP_DEG = 5.0
MAX_JOINT_JUMP_DEG = 25.0
_MAX_SPLIT_DEPTH = 4  # 2**4 = 16 sub-pasos como maximo por click

# Limites reales de cada articulacion, tal como los imprime my_robot_driver.py
# al arrancar (leidos del propio motor en Webots). OJO: para panda_joint4 el
# limite inferior (-3.1416) esta pegado casi exacto al punto -pi/+pi -- ver
# _publish_joint para el bug real que esto causaba con el wrap-around
# generico por modulo.
JOINT_LIMITS = [
    (-2.9671, 2.9671),
    (-1.8326, 1.8326),
    (-2.9671, 2.9671),
    (-3.1416, -0.4000),
    (-2.9671, 2.9671),
    (-0.0873, 3.8223),
    (-2.9671, 2.9671),
]
JOINT_LIMITS_LO = np.array([lo for lo, hi in JOINT_LIMITS])
JOINT_LIMITS_HI = np.array([hi for lo, hi in JOINT_LIMITS])

# Selector de robot en el propio panel (sesion 2026-08-30): antes, para
# controlar el Sorter en vez del Loader habia que MATAR la ventana y
# relanzarla entera con otros --ros-args (ver LANZAR_PROYECTO.md 5b, ahora
# obsoleto). El preset 'loader' se sigue construyendo a partir de los
# parametros ROS declarados en __init__ (para no romper overrides ya
# existentes); el del 'sorter' se fija aqui con los mismos valores que ya
# se pasaban a mano por CLI -- si algun dia cambia la base o la camara del
# Sorter en el mundo, se actualiza SOLO aqui.
SORTER_PRESET = {
    'label': 'Sorter',
    'joint_topic': '/sorter/joint_positions',
    'gripper_topic': '/sorter/gripper_position',
    'led_topic': '/comando_led',  # Pico W de siempre, con el boton de parada
    'base': (0.0, 1.0, 0.74),
    # Giro de la base (sesion 2026-09-11): el Sorter esta girado 26.6 grados
    # en el mundo (DEF PANDA_SORTER rotation, SORTER_BASE_YAW en sorter_demo.py).
    # Sin esto el panel calculaba mal su TCP "mundo" y los botones X/Y y
    # "Centrar sobre cubo" lo movian en direcciones giradas 26.6 grados.
    'base_yaw': 0.4636,
    'camera_topic': '/overhead_camera_sorter/image_color',
    'camera': (0.25, 1.30, 2.27),
    'table_x': (0.35, 0.65),
    'table_y': (0.95, 1.35),
}

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


def _demo_vivo(nombre):
    """¿Hay ya un proceso 'sorter_demo'/'loader_demo' corriendo en este
    contenedor, lo haya lanzado este panel o no? (sesion 2026-09-11). El panel
    solo conocia los procesos que lanzaba EL MISMO (self.proc_*): al cerrarlo y
    reabrirlo, el sorter_demo anterior seguia vivo y el panel lanzaba OTRO
    encima -- dos generaciones mandando ordenes contradictorias al mismo brazo,
    fallo que ya rompio la pinza. El patron es la ruta del ejecutable real (no
    'ros2 run ...', que tras el exec ya no aparece en su linea de comandos)."""
    try:
        return subprocess.run(
            ['pgrep', '-f', f'panda_controller/lib/panda_controller/{nombre}'],
            capture_output=True, timeout=2).returncode == 0
    except (OSError, subprocess.SubprocessError):
        return False


class TeleopGuiNode(Node):
    """Solo la parte ROS2 (publishers + cinematica + estado). Sin
    dependencia de Tkinter aqui, para que sea facil de razonar por
    separado -- la ventana (TeleopApp) la usa por composicion."""

    def __init__(self):
        super().__init__('teleop_gui')

        # Selector de robot (sesion 2026-08-30): que robot se controla AL
        # ARRANCAR ('loader' o 'sorter') -- despues se puede cambiar sin
        # reiniciar la ventana con los botones LOADER/SORTER del panel
        # (ver apply_preset). Los parametros de abajo (robot_base_*,
        # joint_topic, etc.) siguen definiendo el preset 'loader' tal cual
        # (para no romper overrides ya existentes); el preset 'sorter' es
        # la constante SORTER_PRESET de arriba.
        self.declare_parameter('robot', 'loader')
        self.declare_parameter('joint_topic', '/loader/joint_positions')
        self.declare_parameter('gripper_topic', '/loader/gripper_position')
        self.declare_parameter('led_topic', '/comando_led_loader')
        self.declare_parameter('robot_base_x', 0.5)
        self.declare_parameter('robot_base_y', -0.3)
        self.declare_parameter('robot_base_z', 0.74)
        self.declare_parameter('robot_base_yaw', 0.0)
        self.declare_parameter('gripper_open_position', 0.04)
        self.declare_parameter('gripper_closed_position', 0.025)
        self.declare_parameter('joint_offsets_deg', [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -90.0])
        self.declare_parameter('max_wait_seconds', 45.0)
        # Posicion articular COMANDADA real de arranque (sesion 2026-08-27,
        # entrega de control manual desde sorter_hover_test.py): si viene
        # rellena (7 valores), la ventana arranca YA en esa pose en vez de
        # en HOME -- sin esto, al entregar el control justo encima del
        # cubo, el primer align_gripper_down() de la ventana calcularia
        # desde una pose de partida (HOME) que no es la real, y el brazo
        # daria un salto grande e indeseado nada mas abrir el panel.
        # Lista vacia como default confunde a rclpy (infiere BYTE_ARRAY y
        # luego rechaza el DOUBLE_ARRAY real que llega por -p) -- un
        # placeholder de longitud 1 (nunca 7 de verdad) fuerza el tipo
        # correcto y sigue distinguiendose de "no venia dado".
        self.declare_parameter('initial_commanded_joints', [0.0])

        # Camara cenital para "Centrar sobre cubo" (sesion 2026-08-27, a
        # peticion del usuario: entrar centrado a mano con los botones de 1cm
        # es dificil sin ver la alineacion real) -- defaults EXACTAMENTE los
        # de la camara original del Loader (overhead_vision.py), para lanzar
        # el panel con el Sorter hay que pasar sus propios valores (ver
        # sorter_demo.py: SORTER_CAM_*, SORTER_TABLE_*_RANGE).
        self.declare_parameter('camera_topic', '/overhead_camera/image_color')
        self.declare_parameter('camera_x', 0.5)
        self.declare_parameter('camera_y', 0.0)
        self.declare_parameter('camera_z', 2.27)
        self.declare_parameter('camera_table_x_min', 0.15)
        self.declare_parameter('camera_table_x_max', 0.85)
        self.declare_parameter('camera_table_y_min', -0.55)
        self.declare_parameter('camera_table_y_max', 0.55)
        self.declare_parameter('cube_table_z', 0.77)

        # Panel de pedidos (sesion 2026-08-28): Taller_Administracion vive en
        # OTRO contenedor/proyecto (docker-compose propio, red propia), no en
        # panda_ros_net -- 'taller_host' llega hasta el host real via el
        # extra_hosts añadido a ros2_app en docker-compose.yml (host-gateway),
        # y desde ahi el puerto 8000 publicado del otro proyecto. Nombre
        # deliberadamente DISTINTO del 'host.docker.internal' que ya usa el
        # servicio webots (ese apunta al propio contenedor webots, no al
        # host real -- mismo nombre con significado distinto habria sido
        # una trampa para el futuro). Parametro por si algun dia cambia la
        # URL/puerto.
        self.declare_parameter('taller_api_base', 'http://taller_host:8000')
        self.taller_api_base = str(self.get_parameter('taller_api_base').value)
        # Taller_Administracion paso a exigir sesion real en /pedidos
        # (sesion 2026-08-29, sistema de usuarios/roles) -- el panel del
        # operador necesita ver TODOS los pedidos pendientes de cualquier
        # cliente, no solo los de una empresa, asi que se loguea como
        # admin_sistema (admin/admin -- sesion 2026-09-12 renombro el admin
        # sembrado de "aladin" a "admin" con contrasena real, ver auth.py
        # de ese proyecto). Token en memoria, se renueva solo si caduca/el
        # otro servidor se reinicia (ver _taller_login).
        self.taller_token = None
        # Ver CONFIG_MAQUINA_PATH mas arriba -- se carga aqui una vez, al
        # arrancar el nodo, y se mantiene en memoria hasta que la interfaz
        # (TeleopApp.guardar_numero_maquina) lo cambie y lo vuelva a guardar.
        self.numero_maquina = _cargar_numero_maquina()
        self.cube_table_z = float(self.get_parameter('cube_table_z').value)

        self._presets = {
            'loader': {
                'label': 'Loader',
                'joint_topic': str(self.get_parameter('joint_topic').value),
                'gripper_topic': str(self.get_parameter('gripper_topic').value),
                'led_topic': str(self.get_parameter('led_topic').value),
                'base': (
                    float(self.get_parameter('robot_base_x').value),
                    float(self.get_parameter('robot_base_y').value),
                    float(self.get_parameter('robot_base_z').value),
                ),
                'base_yaw': float(self.get_parameter('robot_base_yaw').value),
                'camera_topic': str(self.get_parameter('camera_topic').value),
                'camera': (
                    float(self.get_parameter('camera_x').value),
                    float(self.get_parameter('camera_y').value),
                    float(self.get_parameter('camera_z').value),
                ),
                'table_x': (
                    float(self.get_parameter('camera_table_x_min').value),
                    float(self.get_parameter('camera_table_x_max').value),
                ),
                'table_y': (
                    float(self.get_parameter('camera_table_y_min').value),
                    float(self.get_parameter('camera_table_y_max').value),
                ),
            },
            'sorter': dict(SORTER_PRESET),
        }

        # Publishers FIJOS a las dos Picos, independientes de cual robot
        # este seleccionado en el panel (sesion 2026-09-03, bug real:
        # send_stop/send_rearm usaban self.pub_led, que apply_preset()
        # destruye y recrea segun el selector LOADER/SORTER -- con el
        # panel en modo Loader, REARME solo llegaba a la Pico del Loader
        # y la Pico W del Sorter se quedaba parpadeando en rojo para
        # siempre, aunque /emergency_stop -- el aviso de verdad que para
        # el brazo -- SI es global. El parpadeo es solo cosmetico, pero
        # el usuario lo ve como "no puedo apagarlo desde el panel"). Aqui
        # SIEMPRE se avisa a las dos Picos a la vez, venga de donde venga
        # la parada.
        self.pub_led_loader = self.create_publisher(String, self._presets['loader']['led_topic'], 10)
        self.pub_led_sorter = self.create_publisher(String, self._presets['sorter']['led_topic'], 10)

        self.gripper_open = float(self.get_parameter('gripper_open_position').value)
        self.gripper_closed = float(self.get_parameter('gripper_closed_position').value)
        offsets_deg = list(self.get_parameter('joint_offsets_deg').value)
        self.offsets_rad = np.radians(offsets_deg)
        self.max_wait_seconds = float(self.get_parameter('max_wait_seconds').value)

        # robot_name/pub_joint/pub_gripper/pub_led/base/locator: ninguno
        # existe todavia -- apply_preset() los crea desde cero (misma
        # ruta de codigo que usara luego el selector LOADER/SORTER del
        # panel para cambiar en caliente, ver TeleopApp.switch_robot).
        self.robot_name = None
        self.pub_joint = None
        self.pub_gripper = None
        self.pub_led = None
        self.locator = None
        self.apply_preset(str(self.get_parameter('robot').value))

        # STOP/REARME integrados en el mismo panel (sesion 2026-08-27, antes
        # solo existian en estop_panel.py): mismo topic /emergency_stop de
        # toda la celda, asi que un STOP desde aqui pausa tambien cualquier
        # demo automatica (loader_demo/sorter_demo) que este corriendo a la
        # vez, y viceversa -- todos escuchan el mismo topic global.
        self.stopped = False
        self._on_stop_change = None  # lo fija TeleopApp tras construir la UI
        self.pub_stop = self.create_publisher(Bool, '/emergency_stop', 10)
        self.create_subscription(Bool, '/emergency_stop', self._on_emergency_stop, 10)

        # Contador de piezas REALMENTE fabricadas, sesion 2026-08-31 --
        # correccion del usuario: el contador de progreso del panel NO debe
        # depender de si Taller_Administracion tiene el reparto automatico
        # activado o no ("si el operario le ha pedido que haga 20 piezas
        # hace 20 piezas y se va al almacen, es administracion quien decide
        # luego el reparto"). Se cuenta aqui, del lado del robot, escuchando
        # la MISMA confirmacion de entrega real que ya usa
        # warehouse_supervisor_driver.py para reciclar -- no depende en
        # nada de Taller_Administracion ni de si hay pedidos o no.
        self.entregas_color = {'R': 0, 'G': 0, 'B': 0}
        self.create_subscription(String, '/warehouse/cube_delivered', self._on_cube_delivered, 10)

        # Aviso de "lote nuevo" para el Sorter (sesion 2026-08-31, ver
        # SorterDemo._on_nuevo_lote): el Loader es un proceso nuevo en cada
        # lote y ya baila solo al arrancar, pero el Sorter persiste entre
        # lotes -- necesita que se le avise por topic.
        self.pub_nuevo_lote = self.create_publisher(Bool, '/production/nuevo_lote', 10)
        # Color objetivo del lote activo (sesion 2026-09-01, ver
        # SorterDemo._on_lote_color_objetivo): '' = sin lote de un solo
        # producto (reparto mixto), letra = todo lo entregado en este lote
        # cuenta como ese producto. QoS latched (TRANSIENT_LOCAL) para que
        # el Sorter lo reciba aunque arranque despues de este publish.
        self.pub_lote_color_objetivo = self.create_publisher(
            String, '/production/lote_color_objetivo', LOTE_COLOR_QOS)

        self.home_raw = np.array(HOME_POSITIONS) - self.offsets_rad
        initial_commanded = list(self.get_parameter('initial_commanded_joints').value)
        if len(initial_commanded) == 7:
            self.thetas = np.array(initial_commanded) - self.offsets_rad
            self.get_logger().info(
                f'Arrancando en pose entregada (no HOME): {list(np.round(initial_commanded, 4))}')
        else:
            self.thetas = self.home_raw.copy()
        self.step = STEP_DEFAULT
        # Giro de la pinza alrededor de su propio eje Z (sesion 2026-08-27,
        # botones de giro): 0.0 = orientacion base GRASP_R, sin girar. Se
        # resetea a 0.0 en HOME/Orientar-abajo porque esas acciones ya
        # reorientan directamente a GRASP_R (yaw=0) -- sin resetearlo aqui
        # el estado mostrado/usado quedaria desincronizado del giro real.
        self.yaw = 0.0
        self.yaw_step = np.radians(YAW_STEP_DEG)

        # Entrada de control por topic, ademas de los botones: permite
        # automatizar secuencias de movimientos (script externo) usando
        # exactamente el mismo motor/estado que la GUI, sin reimplementar la
        # cinematica por fuera ni perder la continuidad de self.thetas.
        # Formatos: "move dx dy dz", "open", "close", "home", "align".
        self.create_subscription(String, '/teleop_auto_cmd', self._on_auto_cmd, 10)

    def apply_preset(self, name):
        """Recrea publishers/locator/base para el robot 'name' ('loader' o
        'sorter'). Se llama una vez al arrancar y de nuevo cada vez que el
        operario cambia de robot desde el panel (sesion 2026-08-30) -- NO
        toca self.thetas/self.yaw (eso lo decide quien la llama: al
        arrancar se respeta initial_commanded_joints/HOME de siempre; al
        cambiar de robot en caliente, TeleopApp.switch_robot los resetea a
        HOME antes de reorientar, porque la pose articular de un robot no
        tiene por que ser una postura valida/segura para el otro)."""
        if name not in self._presets:
            raise ValueError(f"robot '{name}' desconocido, debe ser uno de {list(self._presets)}")
        preset = self._presets[name]
        if self.pub_joint is not None:
            self.destroy_publisher(self.pub_joint)
            self.destroy_publisher(self.pub_gripper)
            self.destroy_publisher(self.pub_led)
        if self.locator is not None:
            self.destroy_subscription(self.locator.sub)

        self.robot_name = name
        self.pub_joint = self.create_publisher(Float64MultiArray, preset['joint_topic'], 10)
        self.pub_gripper = self.create_publisher(Float64, preset['gripper_topic'], 10)
        self.pub_led = self.create_publisher(String, preset['led_topic'], 10)
        self.base = np.array(preset['base'])
        # La cinematica del panel (forward_kinematics/inverse_kinematics) trabaja
        # en el marco de la BASE del robot; el mundo se obtiene girando base_yaw
        # en Z (ver SORTER_PRESET). Con 0.0 (Loader) todo queda igual que antes.
        self.base_yaw = float(preset.get('base_yaw', 0.0))
        self.locator = OverheadLocator(
            self, topic=preset['camera_topic'],
            cam_x=preset['camera'][0], cam_y=preset['camera'][1], cam_z=preset['camera'][2],
            table_x_range=preset['table_x'], table_y_range=preset['table_y'])
        self.get_logger().info(
            f"Controlando ahora: {preset['label']} "
            f"(joints={preset['joint_topic']}, led={preset['led_topic']})")

    def _on_auto_cmd(self, msg: String):
        parts = msg.data.strip().split()
        if not parts:
            return
        cmd = parts[0]
        if cmd == 'move' and len(parts) == 4:
            dx, dy, dz = (float(x) for x in parts[1:])
            ok, text = self.move_delta(dx, dy, dz)
            p = self.tcp_world()
            self.get_logger().info(
                f'[auto] move dx={dx:.4f} dy={dy:.4f} dz={dz:.4f} -> '
                f'{"OK" if ok else "RECHAZADO"} pos=({p[0]:.4f},{p[1]:.4f},{p[2]:.4f}) {text}')
        elif cmd == 'open':
            ok = self.set_gripper(self.gripper_open)
            self.get_logger().info(f'[auto] ABRIR pinza -> {"OK" if ok else "RECHAZADO (parado)"}')
        elif cmd == 'close':
            ok = self.set_gripper(self.gripper_closed)
            self.get_logger().info(f'[auto] CERRAR pinza -> {"OK" if ok else "RECHAZADO (parado)"}')
        elif cmd == 'home':
            ok, text = self.go_home()
            self.get_logger().info(f'[auto] HOME -> {"OK" if ok else "RECHAZADO"} {text}')
        elif cmd == 'align':
            ok, text = self.align_gripper_down()
            self.get_logger().info(f'[auto] align -> {"OK" if ok else "RECHAZADO"} {text}')
        elif cmd == 'rotate' and len(parts) == 2:
            dyaw = np.radians(float(parts[1]))
            ok, text = self.rotate_delta(dyaw)
            self.get_logger().info(f'[auto] rotate {parts[1]} deg -> {"OK" if ok else "RECHAZADO"} {text}')
        else:
            self.get_logger().warn(f'[auto] comando no reconocido: {msg.data!r}')

    def wait_for_subscribers(self):
        elapsed = 0.0
        while rclpy.ok() and elapsed < self.max_wait_seconds:
            ready = (
                self.pub_joint.get_subscription_count() > 0
                and self.pub_gripper.get_subscription_count() > 0
            )
            if ready:
                return True
            rclpy.spin_once(self, timeout_sec=0.1)
            elapsed += 0.1
        return False

    def _publish_joint(self, raw_thetas):
        # NO se envuelve con modulo (x+pi)%(2pi)-pi: para jogging continuo
        # con pasos pequenos, raw_thetas ya se mantiene en un rango sensato
        # (siempre sembrado desde el valor anterior). Envolver por modulo
        # puede producir un salto de ~2pi en el angulo COMANDADO cuando el
        # valor bruto cruza por poco el borde -pi/+pi -- justo el caso de
        # panda_joint4, cuyo limite real inferior (-3.1416) esta pegado a ese
        # borde. Bug real visto en Webots: warnings "too big/too low
        # requested position" con valores ~+-3.13 y el brazo dando un salto
        # visual grande ("se ha encogido") aunque el angulo bruto solo habia
        # cambiado un poco. En vez de envolver, se recorta a los limites
        # reales de cada articulacion (ver move_delta para el rechazo previo
        # si el recorte necesario es grande).
        commanded = raw_thetas + self.offsets_rad
        clamped = np.clip(commanded, JOINT_LIMITS_LO, JOINT_LIMITS_HI)
        self.pub_joint.publish(Float64MultiArray(data=clamped.tolist()))
        # Reconciliar self.thetas con lo que REALMENTE se ha mandado (clamped),
        # no con el resultado bruto de la IK: el recorte a limites reales se
        # permite silenciosamente hasta 1 deg (ver _clamp_excess_deg), y sin
        # esto ese recorte se pierde -- self.thetas se va alejando poco a poco
        # de la pose fisica real en Webots aunque cada paso individual parezca
        # aceptado, arrastrando un error grande tras muchos pasos seguidos en
        # la misma direccion (bug real: 130 pasos automatizados via
        # /teleop_auto_cmd terminaron a >30cm de donde el propio codigo credia
        # estar, confirmado con el supervisor orientation_probe).
        self.thetas = clamped - self.offsets_rad
        return clamped

    def tcp_world(self):
        return rz(self.base_yaw) @ forward_kinematics(self.thetas)[:3, 3] + self.base

    def _clamp_excess_deg(self, thetas):
        commanded = thetas + self.offsets_rad
        clamped = np.clip(commanded, JOINT_LIMITS_LO, JOINT_LIMITS_HI)
        return float(np.degrees(np.max(np.abs(commanded - clamped))))

    # OJO (2026-09-11): el jog manual NO se bloquea durante la parada de
    # emergencia, a proposito. La parada es una PAUSA, no un abort: el
    # operario tiene que poder mover el brazo a mano (desatascar un cubo,
    # apartarlo del sensor de proximidad HC-SR04 del Loader) y al rearmar el
    # robot retoma donde lo dejo (ver _wait_while_stopped en
    # cube_shuttle_demo.py). En el proyecto replica se metieron guardas
    # "if self.stopped" aqui creyendo que era un bug, y dejaban al operario
    # sin forma de recuperar la celda. No anadirlas.
    def move_delta(self, dx, dy, dz, _depth=0, _split=False):
        """Devuelve (ok, mensaje). Mismo filtro de salto grande que
        teleop_manual.py: solo se acepta la solucion continua sembrada
        desde la pose actual, nunca una de semillas aleatorias, para que un
        click nunca haga que el brazo "salte" a otra configuracion. Ademas
        se rechaza si la solucion se sale de verdad de los limites reales de
        alguna articulacion (ver _publish_joint).

        Si el paso pedido exige un giro grande (posible cambio de rama del
        codo/singularidad), en vez de rechazarlo de golpe se subdivide en dos
        mitades y se resuelven/publican una tras otra (recursivo, hasta
        _MAX_SPLIT_DEPTH veces) -- el mismo efecto que pulsar "- paso" y
        repetir el click a mano, pero automatico. Si ni el paso mas pequeno
        cabe bajo el limite, se rechaza igual que antes.

        YA NO bloquea con la parada de emergencia activa (sesion 2026-09-10,
        peticion explicita del usuario): "cuando salta la parada de
        emergencia tiene que dejar mover el robot a mano" -- justo lo que
        hace falta para liberar a mano un dedo trabado sin esperar a que el
        rearme (que no arregla nada fisico por si solo) lo resuelva. Riesgo
        conocido y ACEPTADO por el usuario: si hay una demo automatica
        (sorter_demo/loader_demo) en pausa por esta misma parada, su
        self.real_theta (la ultima pose que ELLA comando, no una lectura de
        sensor) se queda desactualizado -- al rearmar puede pedir un
        movimiento grande de golpe para "volver" a donde ella cree que
        estaba, en vez de continuar suave desde donde lo dejaste. No hay
        proteccion automatica contra eso todavia."""
        target_world = self.tcp_world() + np.array([dx, dy, dz])
        # Mundo -> marco de la base (giro -base_yaw), en posicion Y en
        # orientacion: el giro 'yaw' de la pinza es respecto al MUNDO.
        target_base = rz(-self.base_yaw) @ (target_world - self.base)
        target_r = rz(-self.base_yaw) @ GRASP_R @ rz(self.yaw)
        thetas, converged, iters, err = inverse_kinematics(target_base, target_r, self.thetas)
        if not converged:
            return False, f'No puedo llegar ahi (IK no convergio, error={err:.4f}).'
        jump_deg = float(np.degrees(np.max(np.abs(thetas - self.thetas))))
        if jump_deg > MAX_JOINT_JUMP_DEG:
            if _depth < _MAX_SPLIT_DEPTH and max(abs(dx), abs(dy), abs(dz)) > STEP_MIN:
                half = (dx / 2.0, dy / 2.0, dz / 2.0)
                ok1, msg1 = self.move_delta(*half, _depth=_depth + 1, _split=True)
                if not ok1:
                    return False, f'Subdividido pero atascado a mitad de camino: {msg1}'
                ok2, msg2 = self.move_delta(*half, _depth=_depth + 1, _split=True)
                if not ok2:
                    return False, f'Subdividido pero atascado a mitad de camino: {msg2}'
                return True, f'OK (dividido en pasos mas pequenos, giro maximo visto {jump_deg:.1f} deg)'
            return False, (f'Ese movimiento exige un giro de {jump_deg:.1f} deg en alguna '
                            'articulacion incluso en el paso mas pequeno (posible singularidad '
                            'real); prueba otra direccion.')
        excess_deg = self._clamp_excess_deg(thetas)
        if excess_deg > 1.0:
            return False, (f'Esa posicion se sale {excess_deg:.1f} deg del limite real de '
                            'alguna articulacion (probable estas tocando el limite fisico del '
                            'brazo); prueba otra direccion. No me muevo.')
        self.thetas = thetas
        self._publish_joint(self.thetas)
        if _split:
            time.sleep(0.15)
        return True, 'OK' if not _split else 'OK (paso intermedio)'

    def go_home(self):
        # Ya no bloquea con la parada activa -- ver move_delta() para el
        # porque completo (sesion 2026-09-10).
        self.thetas = self.home_raw.copy()
        self.yaw = 0.0
        self._publish_joint(self.thetas)
        return True, 'OK'

    def align_gripper_down(self):
        """Sin filtro de salto articular (ver teleop_manual.py): reorientar
        desde HOME es una reconfiguracion grande pero deliberada. Si mantiene
        el filtro de limites reales de articulacion. Ya no bloquea con la
        parada activa -- ver move_delta() para el porque completo
        (sesion 2026-09-10)."""
        current_pos = forward_kinematics(self.thetas)[:3, 3]
        thetas, converged, iters, err = inverse_kinematics(
            current_pos, rz(-self.base_yaw) @ GRASP_R, self.thetas)
        if not converged:
            return False, f'No he podido orientar la pinza (error={err:.4f}).'
        excess_deg = self._clamp_excess_deg(thetas)
        if excess_deg > 1.0:
            return False, (f'La orientacion hacia abajo aqui se sale {excess_deg:.1f} deg del '
                            'limite real de alguna articulacion. No me muevo.')
        jump_deg = float(np.degrees(np.max(np.abs(thetas - self.thetas))))
        self.thetas = thetas
        self.yaw = 0.0
        self._publish_joint(self.thetas)
        return True, f'Pinza orientada hacia abajo (giro {jump_deg:.1f} deg).'

    def rotate_delta(self, dyaw):
        """Gira la pinza alrededor de su propio eje Z sin mover el TCP
        (misma posicion, solo orientacion) -- mismo filtro de salto grande y
        de limites reales que move_delta, para que un click nunca "salte" a
        otra configuracion del brazo. Ya no bloquea con la parada activa --
        ver move_delta() para el porque completo (sesion 2026-09-10)."""
        current_pos = forward_kinematics(self.thetas)[:3, 3]
        new_yaw = self.yaw + dyaw
        target_r = rz(-self.base_yaw) @ GRASP_R @ rz(new_yaw)
        thetas, converged, iters, err = inverse_kinematics(current_pos, target_r, self.thetas)
        if not converged:
            return False, f'No puedo girar ahi (IK no convergio, error={err:.4f}).'
        jump_deg = float(np.degrees(np.max(np.abs(thetas - self.thetas))))
        if jump_deg > MAX_JOINT_JUMP_DEG:
            return False, (f'Ese giro exige {jump_deg:.1f} deg de golpe en alguna articulacion '
                            '(posible singularidad); prueba un paso mas pequeno.')
        excess_deg = self._clamp_excess_deg(thetas)
        if excess_deg > 1.0:
            return False, (f'Ese giro se sale {excess_deg:.1f} deg del limite real de alguna '
                            'articulacion. No giro.')
        self.thetas = thetas
        self.yaw = new_yaw
        self._publish_joint(self.thetas)
        return True, f'Girado a {np.degrees(self.yaw):.1f} grados.'

    def center_on_cube(self):
        """Ajusta SOLO X/Y (misma Z) para quedar centrado sobre el cubo que
        vea la camara cenital ahora mismo, usando la misma localizacion de
        un solo disparo que el agarre automatico -- deja el descenso final
        (Z) en manos del usuario, que es la parte que de verdad quiere
        controlar el mismo. Ya no bloquea con la parada activa -- ver
        move_delta() para el porque completo (sesion 2026-09-10)."""
        color = self.locator.detect_color()
        if color is None:
            return False, 'No se ve ningun cubo en la camara cenital.'
        located = self.locator.locate_with_yaw(self.cube_table_z, color=color)
        if located is None:
            return False, 'Cubo detectado pero no se pudo localizar con precision.'
        x, y, _yaw_offset = located
        current = self.tcp_world()
        # Salvaguarda (sesion 2026-08-27, aviso real del usuario: un
        # centrado hecho demasiado bajo golpeo el cubo de lado y salio
        # disparado): el ajuste de X/Y solo se hace si la pinza esta a
        # AL MENOS 5cm por encima de la mesa/cinta -- un movimiento lateral
        # a ras del cubo es un golpe, no un centrado.
        min_safe_z = self.cube_table_z + 0.05
        if current[2] < min_safe_z:
            return False, (f'Demasiado bajo para centrar sin riesgo (z={current[2]:.3f}, '
                            f'hace falta al menos {min_safe_z:.3f}) -- sube primero con Z+.')
        dx, dy = float(x - current[0]), float(y - current[1])
        ok, msg = self.move_delta(dx, dy, 0.0)
        name = COLOR_NAMES.get(color, color)
        if ok:
            return True, f'Centrado sobre cubo {name} en ({x:.3f},{y:.3f}).'
        return False, f'Cubo {name} visto en ({x:.3f},{y:.3f}) pero el movimiento fue rechazado: {msg}'

    def set_gripper(self, position):
        # Ya no bloquea con la parada activa (sesion 2026-09-10, peticion
        # explicita del usuario) -- es justo el control que hace falta para
        # soltar a mano un dedo trabado ("se ha roto la pinza y no puedo
        # rearmar") sin esperar a que el rearme, que no arregla nada fisico
        # por si solo, lo resuelva. Ver move_delta() para el riesgo conocido
        # y aceptado (demo automatica en pausa con self.real_theta
        # desactualizado).
        self.pub_gripper.publish(Float64(data=position))
        return True

    def set_led(self, letter):
        self.pub_led.publish(String(data=letter))

    def _on_emergency_stop(self, msg):
        if bool(msg.data) != self.stopped:
            self.stopped = bool(msg.data)
            if self._on_stop_change is not None:
                self._on_stop_change(self.stopped)

    def _on_cube_delivered(self, msg):
        color = msg.data.strip().upper()
        if color in self.entregas_color:
            self.entregas_color[color] += 1

    def send_stop(self):
        self.pub_stop.publish(Bool(data=True))
        # Igual que estop_panel.py: avisa tambien a las Picos para que
        # parpadeen en rojo, mismo efecto visual venga la parada de donde
        # venga. A LAS DOS siempre (ver pub_led_loader/pub_led_sorter),
        # no solo a la del robot seleccionado en el panel -- /emergency_stop
        # es global, el aviso visual tiene que serlo tambien.
        self.pub_led_loader.publish(String(data='parada'))
        self.pub_led_sorter.publish(String(data='parada'))

    def send_rearm(self):
        self.pub_stop.publish(Bool(data=False))
        self.pub_led_loader.publish(String(data='rearme'))
        self.pub_led_sorter.publish(String(data='rearme'))

    def _taller_login(self):
        """POST /login como admin_sistema (admin/admin) -- necesario desde
        que Taller_Administracion exige sesion real en /pedidos (sesion
        2026-08-29). Devuelve True/False; no lanza."""
        url = self.taller_api_base.rstrip('/') + '/login'
        body = json.dumps({'username': 'admin', 'password': 'admin'}).encode('utf-8')
        req = urllib.request.Request(
            url, data=body, method='POST', headers={'Content-Type': 'application/json'})
        try:
            with urllib.request.urlopen(req, timeout=1.5) as resp:
                self.taller_token = json.loads(resp.read().decode('utf-8'))['token']
            return True
        except (urllib.error.URLError, TimeoutError, ValueError, KeyError):
            self.taller_token = None
            return False

    def fetch_pedidos_pendientes(self):
        """GET /pedidos en Taller_Administracion, filtrando los que no
        estan 'completado'. Devuelve None si el servidor no responde o no
        hay sesion valida -- el panel de teleop no debe romperse por
        esto, el operador simplemente ve el aviso de sin conexion.
        Se loguea solo (una vez, y de nuevo si el token caduca -- p.ej.
        el otro servidor se reinicio y perdio sus sesiones en memoria)
        para ver TODOS los pedidos de cualquier cliente, no solo uno."""
        if self.taller_token is None and not self._taller_login():
            return None
        url = self.taller_api_base.rstrip('/') + '/pedidos'
        for reintento in (1, 2):
            req = urllib.request.Request(url, headers={'X-Session-Token': self.taller_token})
            try:
                with urllib.request.urlopen(req, timeout=1.5) as resp:
                    pedidos = json.loads(resp.read().decode('utf-8'))
                # 'cancelado' (sesion 2026-09-10, nuevo en Taller_Administracion)
                # tampoco es "pendiente" para el operario -- antes solo se
                # excluia 'completado', asi que un pedido cancelado se seguia
                # ofreciendo aqui como si hubiera que fabricarlo.
                #
                # Ademas: un pedido cuyo stock_disponible YA cubre el 100% de
                # lo que falta se oculta tambien (peticion explicita del
                # usuario, sesion 2026-09-10: "un operario solo tiene que ver
                # lo que tiene pendiente, no lo que ya fabrico"). stock_disponible
                # simula el mismo reparto FIFO que haria /almacen/repartir SIN
                # tocar la BBDD (ver _con_stock_disponible en Taller_Administracion),
                # asi que funciona igual con reparto_automatico activado o no --
                # a diferencia de 'estado', que con el interruptor desactivado se
                # queda en 'pendiente' aunque ya este todo fabricado (ver memoria
                # robotica_taller_administracion.md). Esto NO cambia el estado
                # real del pedido en la BBDD ni lo reparte -- es solo un filtro
                # de la vista del operario; admin_cliente/admin_sistema lo siguen
                # viendo tal cual en el panel web de administracion.
                # numero_maquina (sesion 2026-09-13): 0 = libre, o el
                # numero de ESTA celda -- lo de otra celda ni se
                # considera, filtrado aqui (el unico sitio donde se
                # obtienen pedidos pendientes) para que herede el filtro
                # cualquier camino que lance produccion, sea el boton de
                # toda la vida o el modo Automatico (ver
                # Documentacion/analisis_ampliacion_taller.md). Con una
                # sola celda (self.numero_maquina fijo, sin otra que
                # reclame nada) esto no descarta nada en la practica.
                return [
                    p for p in pedidos
                    if p.get('estado') not in ('completado', 'cancelado')
                    and p.get('stock_disponible', 0) < (p.get('cantidad_pedida', 0) - p.get('cantidad_completada', 0))
                    and p.get('numero_maquina', 0) in (0, self.numero_maquina)
                ]
            except urllib.error.HTTPError as e:
                if e.code in (401, 422) and reintento == 1 and self._taller_login():
                    continue
                return None
            except (urllib.error.URLError, TimeoutError, ValueError):
                return None
        return None

    def reclamar_pedido(self, pedido_id, forzar=False) -> bool:
        """POST /pedidos/{id}/reclamar con self.numero_maquina -- hay que
        llamarlo ANTES de lanzar produccion para ese pedido (ver
        _lanzar_produccion y sus llamantes). True si lo consigue (estaba
        libre, o ya era mio); False si esta cogido por otra celda o hay
        cualquier fallo de red -- en los dos casos, quien llama no debe
        lanzar produccion para ese pedido."""
        if self.taller_token is None and not self._taller_login():
            return False
        url = f"{self.taller_api_base.rstrip('/')}/pedidos/{pedido_id}/reclamar"
        body = json.dumps({'numero_maquina': self.numero_maquina, 'forzar': forzar}).encode('utf-8')
        for reintento in (1, 2):
            req = urllib.request.Request(
                url, data=body, method='POST',
                headers={'X-Session-Token': self.taller_token, 'Content-Type': 'application/json'})
            try:
                with urllib.request.urlopen(req, timeout=1.5) as resp:
                    resp.read()
                return True
            except urllib.error.HTTPError as e:
                if e.code in (401, 422) and reintento == 1 and self._taller_login():
                    continue
                return False  # 409 incluido: ya es de otra celda
            except (urllib.error.URLError, TimeoutError, ValueError):
                return False
        return False

    def fetch_productos(self):
        """GET /productos -- sin sesion (endpoint publico). Se usa para
        rellenar el desplegable del lanzador de lotes (sesion 2026-08-30)
        con los productos reales del catalogo en vez de tener 'R'/'G'/'B'
        escritos a mano aqui."""
        url = self.taller_api_base.rstrip('/') + '/productos'
        try:
            with urllib.request.urlopen(url, timeout=1.5) as resp:
                return json.loads(resp.read().decode('utf-8'))
        except (urllib.error.URLError, TimeoutError, ValueError):
            return None

    def marcar_cubo_clasificado(self, color):
        """POST /taller/cubo_clasificado -- lo llama el operador a mano
        desde el panel justo despues de colocar la pieza el mismo, ya que
        de momento no hay ninguna celda automatica publicando esto por su
        cuenta (ver README de Taller_Administracion)."""
        url = self.taller_api_base.rstrip('/') + '/taller/cubo_clasificado'
        body = json.dumps({'color': color}).encode('utf-8')
        req = urllib.request.Request(
            url, data=body, method='POST',
            headers={'Content-Type': 'application/json'})
        try:
            with urllib.request.urlopen(req, timeout=1.5) as resp:
                return True, json.loads(resp.read().decode('utf-8'))
        except urllib.error.HTTPError as e:
            # Cuerpo no-JSON (p.ej. un 500 en texto plano): antes json.loads
            # lanzaba aqui mismo y la excepcion escapaba al callback de Tk.
            cuerpo = e.read().decode('utf-8', errors='replace')
            try:
                detail = json.loads(cuerpo).get('detail', cuerpo)
            except ValueError:
                detail = cuerpo or str(e)
            return False, detail
        except (urllib.error.URLError, TimeoutError, ValueError) as e:
            return False, str(e)


class TeleopApp:
    """Aspecto de cuadro de mando industrial (sesion 2026-08-27, mismo
    convenio de colores que estop_panel.py) -- STOP/REARME viven en el
    mismo panel que el jog en vez de en una ventana aparte, para poder
    parar el brazo sin soltar el ratón del control manual."""

    def __init__(self, node: TeleopGuiNode):
        self.node = node
        self.node._on_stop_change = self.on_stop_change
        self.root = tk.Tk()
        self.root.title('Panel de control manual - Panda')
        self.root.configure(bg=BG)

        title_font = tkfont.Font(family='Arial', size=15, weight='bold')
        status_font = tkfont.Font(family='Arial', size=13, weight='bold')
        big = tkfont.Font(family='Arial', size=12, weight='bold')
        mono = tkfont.Font(family='monospace', size=11)
        self.big_font, self.mono_font = big, mono

        header = tk.Label(self.root, text='PANEL DE CONTROL MANUAL',
                           font=title_font, bg=BG, fg=YELLOW)
        header.grid(row=0, column=0, columnspan=5, padx=18, pady=(16, 4))

        stripe = tk.Canvas(self.root, width=600, height=10, bg=BG, highlightthickness=0)
        stripe.grid(row=1, column=0, columnspan=5, pady=(0, 10))
        n_stripes = 30
        w = 600 / n_stripes
        for i in range(n_stripes):
            color = YELLOW if i % 2 == 0 else '#000000'
            stripe.create_rectangle(i * w, 0, (i + 1) * w, 10, fill=color, outline='')

        # --- STOP / REARME / estado, arriba del todo -----------------
        stop_panel = tk.Frame(self.root, bg=PANEL_BG, bd=4, relief='ridge')
        stop_panel.grid(row=2, column=0, columnspan=5, padx=18, pady=(0, 10), sticky='we')

        self.status_var = tk.StringVar(value='EN MARCHA')
        self.status_label = tk.Label(stop_panel, textvariable=self.status_var, font=status_font,
                                      bg=PANEL_BG, fg=GREEN, pady=8)
        self.status_label.grid(row=0, column=0, columnspan=2, sticky='we')

        self.stop_btn = tk.Button(
            stop_panel, text='PARADA\nDE EMERGENCIA', font=big, width=15, height=3,
            bg=RED, fg='white', activebackground=RED_DARK, activeforeground='white',
            relief='raised', bd=6, command=self.do_stop)
        self.stop_btn.grid(row=1, column=0, padx=14, pady=10)

        self.rearm_btn = tk.Button(
            stop_panel, text='REARME', font=big, width=15, height=3,
            relief='raised', bd=6, state='disabled', command=self.do_rearm)
        self.rearm_btn.grid(row=1, column=1, padx=14, pady=10)

        # --- selector de robot (sesion 2026-08-30): antes habia que matar
        # la ventana y relanzarla con otros --ros-args para pasar de
        # controlar el Loader al Sorter (ver LANZAR_PROYECTO.md, ahora
        # obsoleto en ese punto) -- ahora se cambia aqui mismo, sin
        # reiniciar nada. El boton del robot activo se resalta en amarillo.
        robot_row = tk.Frame(stop_panel, bg=PANEL_BG)
        robot_row.grid(row=2, column=0, columnspan=2, pady=(0, 10))
        tk.Label(robot_row, text='Robot controlado:', font=big, bg=PANEL_BG, fg=TEXT_LIGHT
                 ).grid(row=0, column=0, padx=(0, 8))
        self.robot_btns = {}
        for i, name in enumerate(('loader', 'sorter')):
            b = tk.Button(robot_row, text=name.upper(), font=big, width=10,
                          command=lambda n=name: self.switch_robot(n),
                          relief='raised', bd=4)
            b.grid(row=0, column=1 + i, padx=4)
            self.robot_btns[name] = b
        self.update_robot_buttons()

        self.pos_var = tk.StringVar()
        self.step_var = tk.StringVar()
        self.log_var = tk.StringVar(value='Listo.')

        self._jog_buttons = []

        def mkbtn(parent, text, cmd, row, col, width=8, track=True, colspan=1):
            b = tk.Button(parent, text=text, font=big, width=width, command=cmd,
                          bg=PANEL_BG, fg=TEXT_LIGHT, activebackground='#3a3a3a',
                          activeforeground=TEXT_LIGHT, relief='raised', bd=4)
            b.grid(row=row, column=col, columnspan=colspan, padx=3, pady=3, sticky='we')
            if track:
                self._jog_buttons.append(b)
            return b

        def mkframe(parent, text):
            return tk.LabelFrame(parent, text=text, font=big, bg=PANEL_BG, fg=YELLOW, bd=3, relief='groove')

        # --- pestañas "Movimiento" / "Producción" (sesion 2026-08-30,
        # rediseño pedido por el usuario: "control manual profesional...
        # que pueda acceder a todo" -- con muchos pedidos pendientes la
        # ventana de una sola columna se quedaba mas alta que la pantalla
        # y los botones de abajo quedaban inalcanzables, sin scroll. Las
        # pestañas separan lo que se usa constantemente (mover el brazo)
        # de lo que se consulta de vez en cuando (pedidos/lotes), asi cada
        # una cabe sola en pantalla. STOP/REARME y el selector de robot se
        # quedan FUERA de las pestañas, siempre visibles -- un control de
        # seguridad no deberia poder quedar escondido detras de una
        # pestaña.
        style = ttk.Style()
        style.theme_use('clam')
        style.configure('Panda.TNotebook', background=BG, borderwidth=0)
        style.configure('Panda.TNotebook.Tab', background=PANEL_BG, foreground=TEXT_LIGHT,
                         font=big, padding=[16, 8], borderwidth=0)
        style.map('Panda.TNotebook.Tab',
                  background=[('selected', YELLOW)], foreground=[('selected', 'black')])
        # Combobox del Nº Maquina (ver mas abajo) -- 'clam' es el unico tema
        # que deja recolorear el desplegable a juego con el resto del panel
        # oscuro, por eso ya se fijaba arriba para el Notebook tambien.
        style.configure('Panda.TCombobox', fieldbackground=PANEL_BG, background=PANEL_BG,
                         foreground=TEXT_LIGHT, arrowcolor=TEXT_LIGHT, borderwidth=0)
        style.map('Panda.TCombobox', fieldbackground=[('readonly', PANEL_BG)],
                  foreground=[('readonly', TEXT_LIGHT)])

        notebook = ttk.Notebook(self.root, style='Panda.TNotebook')
        notebook.grid(row=3, column=0, columnspan=5, padx=18, pady=(4, 16), sticky='nsew')
        self.root.grid_rowconfigure(3, weight=1)

        tab_movimiento = tk.Frame(notebook, bg=BG)
        tab_produccion = tk.Frame(notebook, bg=BG)
        notebook.add(tab_movimiento, text='  Movimiento  ')
        notebook.add(tab_produccion, text='  Producción  ')

        # ================= Pestaña "Movimiento" =================
        pos_label = tk.Label(tab_movimiento, textvariable=self.pos_var, font=mono, bg=BG, fg=TEXT_LIGHT, justify='left')
        pos_label.grid(row=0, column=0, columnspan=5, sticky='w', pady=(4, 2))

        step_label = tk.Label(tab_movimiento, textvariable=self.step_var, font=mono, bg=BG, fg=TEXT_LIGHT)
        step_label.grid(row=1, column=0, columnspan=5, sticky='w')

        # --- movimiento XYZ ---
        move_frame = mkframe(tab_movimiento, 'Mover TCP (mundo)')
        move_frame.grid(row=2, column=0, columnspan=2, padx=8, pady=8, sticky='n')

        mkbtn(move_frame, 'Y-', lambda: self.do_move(0, -self.node.step, 0), 1, 0)
        mkbtn(move_frame, 'X+\nadelante', lambda: self.do_move(self.node.step, 0, 0), 0, 1)
        mkbtn(move_frame, 'X-\natras', lambda: self.do_move(-self.node.step, 0, 0), 2, 1)
        mkbtn(move_frame, 'Y+', lambda: self.do_move(0, self.node.step, 0), 1, 2)
        mkbtn(move_frame, 'Z+\nsubir', lambda: self.do_move(0, 0, self.node.step), 0, 3)
        mkbtn(move_frame, 'Z-\nbajar', lambda: self.do_move(0, 0, -self.node.step), 2, 3)
        mkbtn(move_frame, '📷 Centrar sobre cubo (X/Y, vision)', self.do_center,
              3, 0, width=20, colspan=4)

        step_frame = mkframe(tab_movimiento, 'Paso')
        step_frame.grid(row=2, column=2, padx=8, pady=8, sticky='n')
        mkbtn(step_frame, '- paso', self.step_down, 0, 0, width=10, track=False)
        mkbtn(step_frame, '+ paso', self.step_up, 1, 0, width=10, track=False)

        # --- giro de la pinza (yaw) ---
        rotate_frame = mkframe(tab_movimiento, 'Girar pinza')
        rotate_frame.grid(row=2, column=4, padx=8, pady=8, sticky='n')
        mkbtn(rotate_frame, '↺\nGirar-', lambda: self.do_rotate(-self.node.yaw_step), 0, 0, width=10)
        mkbtn(rotate_frame, '↻\nGirar+', lambda: self.do_rotate(self.node.yaw_step), 1, 0, width=10)

        # --- pinza / home / orientar ---
        action_frame = mkframe(tab_movimiento, 'Pinza y postura')
        action_frame.grid(row=2, column=3, padx=8, pady=8, sticky='n')
        mkbtn(action_frame, 'ABRIR\npinza', self.do_open, 0, 0, width=10)
        mkbtn(action_frame, 'CERRAR\npinza', self.do_close, 1, 0, width=10)
        mkbtn(action_frame, 'HOME', self.do_home, 2, 0, width=10)
        mkbtn(action_frame, 'Orientar\npinza abajo', self.do_align, 3, 0, width=10)

        # --- LED ---
        led_frame = mkframe(tab_movimiento, 'LED')
        led_frame.grid(row=3, column=0, columnspan=5, padx=8, pady=(0, 8), sticky='we')
        mkbtn(led_frame, 'Rojo', lambda: self.do_led('R'), 0, 0, width=8, track=False)
        mkbtn(led_frame, 'Verde', lambda: self.do_led('G'), 0, 1, width=8, track=False)
        mkbtn(led_frame, 'Azul', lambda: self.do_led('B'), 0, 2, width=8, track=False)
        mkbtn(led_frame, 'Apagar', lambda: self.do_led('0'), 0, 3, width=8, track=False)

        self.log_label = tk.Label(tab_movimiento, textvariable=self.log_var, font=mono, bg=BG,
                                   fg=GREEN, wraplength=760, justify='left')
        self.log_label.grid(row=4, column=0, columnspan=5, sticky='w', pady=(4, 8))

        # ================= Pestaña "Producción" =================
        # --- pedidos pendientes (Taller_Administracion), sesion 2026-08-28:
        # el operador ve aqui que falta por hacer y marca cada pieza a mano
        # segun la va colocando, mientras no exista una celda automatica que
        # avise sola. Si el otro proyecto no esta levantado, se avisa sin
        # romper el resto del panel.
        # --- Resumen por producto (sesion 2026-09-01, a peticion del
        # usuario: "si hay dos lotes de tornillos, uno con 3 y otro con
        # 2, el lote seria de cinco, para hacer todos los tornillos a la
        # vez"): agrupa TODOS los pedidos pendientes por producto y
        # muestra un total + un boton "Lanzar todo" por producto -- ENCIMA
        # de la lista de pedidos individuales (que sigue debajo tal cual,
        # por si el operario quiere ver el detalle de cliente/fecha de
        # cada uno). Se decidio ASI, en la misma pestaña, en vez de una
        # pestaña nueva (el usuario pidio "plantea tu como seria mejor" --
        # una pestaña aparte solo duplicaria la misma lista sin aportar
        # nada, y obligaria a cambiar de vista para pasar del resumen al
        # detalle). Reconstruido cada vez que refresh_pedidos() vuelve a
        # pedir la lista (mismos 4s), no tiene sondeo propio.
        resumen_frame = mkframe(tab_produccion, 'Resumen por producto (varios pedidos a la vez)')
        resumen_frame.grid(row=0, column=0, padx=8, pady=(8, 4), sticky='we')
        self.resumen_producto_frame = resumen_frame

        # Boton "Lanzar todo el resumen" (sesion 2026-09-03, sustituye al
        # antiguo "Lanzar todos los pedidos pendientes" -- peticion
        # explicita del usuario: "quita [el boton] de pedidos pendientes
        # y le anades a resumen por producto, asi que si le damos al
        # boton lanzamos todo el resumen por producto"). El antiguo boton
        # lanzaba UN lote mixto de los tres colores fisicos a la vez
        # (color=None) -- no podia representar productos SIN cubo fisico
        # (Y/M/C/W, ver "modo comodin" en Taller_Administracion) ni fijar
        # el LED de producto a uno solo. Este nuevo lanza cada producto
        # del resumen, UNO DETRAS DE OTRO (misma mecanica que el "Lanzar
        # todo" de un solo producto, ver lanzar_producto_agrupado): hasta
        # que el lote actual no termina DE VERDAD (ver
        # _actualizar_estado_lote, que dispara _lanzar_siguiente_de_cola)
        # no arranca el siguiente, y el LED de producto se queda fijo
        # solo mientras dura ESE lote -- exactamente lo pedido.
        self.btn_lanzar_resumen = tk.Button(
            tab_produccion, text='Lanzar todo el resumen (uno detrás de otro)', font=big,
            bg=PANEL_BG, fg=TEXT_LIGHT, activebackground='#3a3a3a', activeforeground=TEXT_LIGHT,
            command=self.lanzar_todo_resumen)
        self.btn_lanzar_resumen.grid(row=1, column=0, padx=8, pady=(0, 8), sticky='w')

        # Interruptor "Automatico" (sesion 2026-09-13, a peticion del usuario:
        # "todo parado y si llega un pedido se arranca el taller solo sin el
        # proceso manual"). Con esto activado, cada refresh_pedidos() (cada
        # 4s) hace lo mismo que pulsar "Lanzar todo el resumen" en cuanto ve
        # pedidos pendientes y no hay ya una produccion/cola en marcha -- sin
        # dialogo de confirmacion (nadie para pulsar "Si"). Requiere que el
        # PROCESO de teleop_gui siga corriendo (no hace falta ver la ventana,
        # pero el proceso tiene que seguir vivo: es el que vigila). Declarada
        # AQUI (no mas abajo con el resto de estado) porque el propio
        # Checkbutton de justo debajo ya la necesita -- declararla despues de
        # construir la UI revienta con AttributeError (bug real, visto al
        # probarlo).
        self.auto_produccion = tk.BooleanVar(value=False)
        # selectcolor fijado a mano porque en temas oscuros de Tkinter el
        # indicador del Checkbutton se queda casi invisible con los colores
        # por defecto del sistema.
        tk.Checkbutton(
            tab_produccion, text='Automático: lanzar pedidos solo, sin tocar nada',
            variable=self.auto_produccion, font=self.mono_font,
            bg=BG, fg='#66bb6a', selectcolor=PANEL_BG,
            activebackground=BG, activeforeground='#66bb6a',
        ).grid(row=1, column=1, padx=8, pady=(0, 8), sticky='w')

        # Numero de maquina de esta celda (ver CONFIG_MAQUINA_PATH):
        # cargado ya en self.node.numero_maquina al arrancar el nodo --
        # este control solo deja CAMBIARLO y guardarlo para el proximo
        # arranque, no es la unica fuente de verdad mientras el proceso
        # sigue vivo (esa es self.node.numero_maquina en memoria).
        # Desplegable de solo lectura (no Entry libre), 1..MAX_MAQUINAS --
        # a peticion del usuario, este modelo de celda no admite mas de
        # 5 maquinas a la vez.
        maquina_frame = tk.Frame(tab_produccion, bg=BG)
        maquina_frame.grid(row=2, column=1, padx=8, pady=(0, 8), sticky='w')
        tk.Label(maquina_frame, text='Nº Máquina:', font=self.mono_font, bg=BG, fg=TEXT_LIGHT
                 ).pack(side='left')
        valor_inicial = self.node.numero_maquina if 1 <= self.node.numero_maquina <= MAX_MAQUINAS else 1
        self.numero_maquina_var = tk.IntVar(value=valor_inicial)
        ttk.Combobox(
            maquina_frame, textvariable=self.numero_maquina_var,
            values=list(range(1, MAX_MAQUINAS + 1)), state='readonly',
            style='Panda.TCombobox', width=3, font=self.mono_font,
        ).pack(side='left', padx=(4, 4))
        tk.Button(maquina_frame, text='Guardar', font=self.mono_font,
                  bg=PANEL_BG, fg=TEXT_LIGHT, activebackground='#3a3a3a', activeforeground=TEXT_LIGHT,
                  command=self.guardar_numero_maquina).pack(side='left')

        # Lista de pedidos individuales CON SCROLL (sesion 2026-09-02, a
        # peticion del usuario: con muchos pedidos a la vez, antes se
        # cortaba en MAX_PEDIDOS_VISIBLES filas con un texto "...y N mas"
        # -- se podia LANZAR todo con el boton de abajo, pero no se podia
        # VER ni tocar (+1 pieza / lanzar este pedido) el resto sin salir
        # del panel. Canvas+Scrollbar de toda la vida de Tkinter (un
        # tk.Frame no se puede desplazar solo): la rueda del raton
        # desplaza mientras el cursor esta encima de la lista.
        pedidos_label_frame = mkframe(tab_produccion, 'Pedidos pendientes (Taller_Administracion)')
        pedidos_label_frame.grid(row=2, column=0, padx=8, pady=(4, 4), sticky='we')

        pedidos_canvas = tk.Canvas(pedidos_label_frame, bg=PANEL_BG, highlightthickness=0, height=200)
        pedidos_scrollbar = tk.Scrollbar(pedidos_label_frame, orient='vertical', command=pedidos_canvas.yview)
        pedidos_frame = tk.Frame(pedidos_canvas, bg=PANEL_BG)
        pedidos_frame.bind('<Configure>', lambda e: pedidos_canvas.configure(scrollregion=pedidos_canvas.bbox('all')))
        pedidos_canvas.create_window((0, 0), window=pedidos_frame, anchor='nw')
        pedidos_canvas.configure(yscrollcommand=pedidos_scrollbar.set)
        pedidos_canvas.grid(row=0, column=0, sticky='nsew')
        pedidos_scrollbar.grid(row=0, column=1, sticky='ns')
        pedidos_label_frame.grid_columnconfigure(0, weight=1)

        def _pedidos_mousewheel(event):
            pedidos_canvas.yview_scroll(-1 if event.delta > 0 else 1, 'units')

        def _pedidos_bind_wheel(_event):
            pedidos_canvas.bind_all('<MouseWheel>', _pedidos_mousewheel)

        def _pedidos_unbind_wheel(_event):
            pedidos_canvas.unbind_all('<MouseWheel>')

        pedidos_canvas.bind('<Enter>', _pedidos_bind_wheel)
        pedidos_canvas.bind('<Leave>', _pedidos_unbind_wheel)

        self.pedidos_frame = pedidos_frame

        # --- lanzador manual de lotes (sesion 2026-08-30, a peticion del
        # usuario): "el operario pone las piezas necesarias para hacer cada
        # cosa y lo manda" -- elige producto + cantidad y lanza loader_demo
        # con only_color/cycles (un color = un lote, ver conversacion sobre
        # produccion realista), asegurandose de que sorter_demo tambien
        # este corriendo para clasificar lo que vaya llegando. Sustituye,
        # de momento, al reparto automatico por pedido mas antiguo del que
        # se hablo (mas adelante, no hecho todavia) -- esto es el envio
        # MANUAL explicito.
        lote_frame = mkframe(tab_produccion, 'Producción manual (lote)')
        lote_frame.grid(row=3, column=0, padx=8, pady=(0, 16), sticky='we')
        self.lote_frame = lote_frame
        self.lote_producto_var = tk.StringVar()
        self.lote_cantidad_var = tk.StringVar(value='3')
        self._productos_cache = []  # [(nombre, color), ...]
        self.lote_status_var = tk.StringVar(value='Cargando catálogo...')

        tk.Label(lote_frame, text='Producto:', font=big, bg=PANEL_BG, fg=TEXT_LIGHT
                 ).grid(row=0, column=0, padx=6, pady=6, sticky='w')
        self.lote_producto_menu = tk.OptionMenu(lote_frame, self.lote_producto_var, '')
        self.lote_producto_menu.config(bg=PANEL_BG, fg=TEXT_LIGHT, width=18)
        self.lote_producto_menu.grid(row=0, column=1, padx=6, pady=6)
        tk.Label(lote_frame, text='Cantidad:', font=big, bg=PANEL_BG, fg=TEXT_LIGHT
                 ).grid(row=0, column=2, padx=6, pady=6, sticky='w')
        tk.Entry(lote_frame, textvariable=self.lote_cantidad_var, width=5, font=big
                 ).grid(row=0, column=3, padx=6, pady=6)
        tk.Button(lote_frame, text='Lanzar lote', font=big, bg=PANEL_BG, fg=TEXT_LIGHT,
                  activebackground='#3a3a3a', activeforeground=TEXT_LIGHT,
                  command=self.lanzar_lote
                  ).grid(row=0, column=4, padx=6, pady=6)
        tk.Label(lote_frame, textvariable=self.lote_status_var, font=self.mono_font,
                 bg=PANEL_BG, fg=TEXT_LIGHT, justify='left'
                 ).grid(row=1, column=0, columnspan=5, padx=6, pady=(0, 6), sticky='w')

        # "Repetir último lote" (sesion 2026-09-10, peticion explicita del
        # usuario: poder repetir el ultimo lote lanzado -- cualquiera de
        # los tres tipos (manual, pedido real, o un tramo de "lanzar todo
        # el resumen"), sin tener que volver a elegir producto/cantidad a
        # mano. Guarda los mismos argumentos que recibio _lanzar_produccion
        # la ultima vez (ver alli) y los reenvia identicos. Empieza
        # deshabilitado -- no hay nada que repetir hasta el primer lanzamiento.
        self.repetir_lote_btn = tk.Button(
            lote_frame, text='🔁 Repetir último lote', font=big, bg=PANEL_BG, fg=TEXT_LIGHT,
            activebackground='#3a3a3a', activeforeground=TEXT_LIGHT,
            state='disabled', command=self.repetir_ultimo_lote,
        )
        self.repetir_lote_btn.grid(row=2, column=0, columnspan=5, padx=6, pady=(0, 6), sticky='w')

        self.proc_loader = None   # subprocess.Popen del lote en curso (o None)
        self.proc_sorter = None   # subprocess.Popen de sorter_demo (persistente)
        self.lote_inicio_entregas = None   # foto de entregas al lanzar el lote actual
        self.lote_objetivo_por_color = {}  # {color: cantidad pedida en este lote}
        self.lote_color_objetivo = None    # None = mixto, letra = lote de un solo producto
        self.pedido_id_en_curso = None  # id del pedido lanzado con "Lanzar este pedido", None si no hay uno en curso
        self.producto_en_curso = None  # color lanzado con "Lanzar todo" (resumen por producto), None si no hay uno en curso
        self.cola_lotes = []  # [(color, cantidad, nombre), ...] pendientes de "Lanzar todo el resumen", uno detras de otro
        self._ultimo_lote_args = None  # kwargs de la ultima _lanzar_produccion, para "Repetir ultimo lote"

        self.refresh_position()
        self.refresh_step()

        # Orientar la pinza hacia abajo YA, al abrir la ventana: el brazo
        # arranca en HOME_POSITIONS, cuya orientacion NO coincide con
        # GRASP_R (ver teleop_manual.py), asi que sin este paso los botones
        # de "Mover TCP" se rechazan todos de entrada (bug real reportado
        # por el usuario: "lo que esta dentro mover tcp no funciona") porque
        # exigirian ese giro grande de golpe. Asi la ventana arranca ya lista
        # para mover en XYZ sin que haga falta saber que pulsar antes.
        ok, msg = self.node.align_gripper_down()
        self.refresh_position()
        self.log('Listo (pinza orientada hacia abajo). ' + msg, ok)

        self.set_stopped_ui(self.node.stopped)
        self._spin_tick()
        self.refresh_pedidos()
        self.refresh_productos()
        self._poll_lote()

    def refresh_productos(self):
        """Rellena el desplegable de producto del lanzador de lotes.
        Reintenta cada 5s si Taller_Administracion no responde todavia,
        sin bloquear el resto del panel."""
        productos = self.node.fetch_productos()
        if productos:
            self._productos_cache = [(p['nombre'], p['color']) for p in productos]
            menu = self.lote_producto_menu['menu']
            menu.delete(0, 'end')
            for nombre, color in self._productos_cache:
                etiqueta = f'{nombre} ({color})'
                menu.add_command(label=etiqueta,
                                  command=lambda v=etiqueta: self.lote_producto_var.set(v))
            if not self.lote_producto_var.get() and self._productos_cache:
                nombre, color = self._productos_cache[0]
                self.lote_producto_var.set(f'{nombre} ({color})')
            self._actualizar_estado_lote()
        else:
            self.lote_status_var.set('Sin conexión con Taller_Administracion -- reintentando...')
            self.root.after(5000, self.refresh_productos)

    def switch_robot(self, name):
        if name == self.node.robot_name:
            return
        self._audit(f'Cambiar de robot -> {name}')
        self.node.apply_preset(name)
        # Reset a HOME (sesion 2026-08-30): la pose articular de un robot
        # no es necesariamente una postura valida/segura para el otro (cada
        # uno vive en un sitio distinto de la celda) -- no tiene sentido
        # arrastrar self.thetas del robot anterior.
        self.node.thetas = self.node.home_raw.copy()
        self.node.yaw = 0.0
        self.update_robot_buttons()
        ok, msg = self.node.align_gripper_down()
        self.refresh_position()
        self.log(f"Controlando ahora: {self.node._presets[name]['label']}. " + msg, ok)

    def update_robot_buttons(self):
        for name, btn in self.robot_btns.items():
            activo = name == self.node.robot_name
            btn.config(
                bg=YELLOW if activo else PANEL_BG,
                fg='black' if activo else TEXT_LIGHT,
                activebackground='#c9a400' if activo else '#3a3a3a',
                relief='sunken' if activo else 'raised',
            )

    def lanzar_lote(self):
        seleccion = self.lote_producto_var.get()
        color = next((c for n, c in self._productos_cache if f'{n} ({c})' == seleccion), None)
        if color is None:
            self.lote_status_var.set('Elige un producto antes de lanzar el lote.')
            return
        try:
            cantidad = int(self.lote_cantidad_var.get())
            if cantidad < 1:
                raise ValueError
        except ValueError:
            self.lote_status_var.set('Cantidad no válida (debe ser un entero >= 1).')
            return
        if self.proc_loader is not None and self.proc_loader.poll() is None:
            self.lote_status_var.set('Ya hay un lote del Loader en curso -- espera a que termine.')
            return
        if not messagebox.askyesno(
            'Confirmar lote',
            f'Esto lanza al Loader y al Sorter a fabricar {cantidad} unidad(es) de "{seleccion}".\n'
            'Si estás controlando el Loader o el Sorter a mano ahora mismo, '
            'se pelearán por el brazo con la demo automática.\n\n¿Continuar?',
        ):
            return
        self._audit(f'Lanzar lote: {seleccion} x{cantidad}')
        self._lanzar_produccion(color, cantidad, seleccion)

    def repetir_ultimo_lote(self):
        """Repite el ultimo lote lanzado, exactamente con los mismos
        argumentos que recibio _lanzar_produccion aquella vez (sesion
        2026-09-10, peticion explicita del usuario). Vale para los tres
        tipos de lote (manual, pedido real, tramo de resumen) porque
        _lanzar_produccion es el unico punto de paso comun.

        Ojo si el ultimo lote llevaba 'pedido_id' (vino de "Lanzar este
        pedido"): repetir con la MISMA cantidad puede fabricar de mas si
        ese pedido ya quedo cubierto -- no se recalcula aqui lo que falta
        de verdad (para eso, 'Lanzar este pedido' desde la lista, que si
        relee la cantidad restante). El aviso de confirmacion deja claro
        que cantidad va a repetir para que el operario lo vea antes de
        aceptar."""
        args = self._ultimo_lote_args
        if args is None:
            self.lote_status_var.set('Todavía no se ha lanzado ningún lote que repetir.')
            return
        if self.proc_loader is not None and self.proc_loader.poll() is None:
            self.lote_status_var.set('Ya hay un lote del Loader en curso -- espera a que termine.')
            return
        if self.cola_lotes:
            self.lote_status_var.set('Ya hay una cola de lotes en marcha -- espera a que termine.')
            return
        aviso_pedido = (
            f'\n\nOjo: este lote estaba atado al pedido #{args["pedido_id"]} -- si ya se '
            'completó, repetirlo fabricará piezas de más (van a Stock, no se pierden, pero '
            'no las pidió nadie).' if args['pedido_id'] is not None else ''
        )
        if not messagebox.askyesno(
            'Repetir último lote',
            f'Esto repite el último lote: {args["cantidad"]} unidad(es) de "{args["etiqueta"]}".\n'
            'Si estás controlando el Loader o el Sorter a mano ahora mismo, '
            f'se pelearán por el brazo con la demo automática.{aviso_pedido}\n\n¿Continuar?',
        ):
            return
        self._audit(f'Repetir último lote: {args["etiqueta"]} x{args["cantidad"]}')
        if args['pedido_id'] is not None:
            self.pedido_id_en_curso = args['pedido_id']
        self._lanzar_produccion(**args)

    def lanzar_pedido(self, pedido_id, color, cantidad, nombre):
        """'Lanzar este pedido' (sesion 2026-09-01): mismo mecanismo que
        lanzar_lote() pero para UN pedido real concreto de la lista, con
        su cantidad restante ya calculada -- no hace falta elegir producto
        ni escribir cantidad a mano. Al terminar, el operario tiene que
        pulsar el siguiente 'Lanzar este pedido' el mismo a proposito
        (no se encadenan solos): entre pedido y pedido el operario puede
        necesitar cambiar de produccion a mano.

        'pedido_id' (mismo dia, bug real visto en vivo): cada pieza se ata
        a ESTE pedido en concreto en Taller_Administracion (ver
        _lanzar_produccion), asi que se completa de verdad aunque el
        interruptor de reparto_automatico este desactivado -- antes se
        quedaba "pendiente" para siempre pese a que el stock ya tuviera
        piezas de sobra, porque nadie pulsaba "Repartir stock" a mano."""
        if cantidad < 1:
            self.lote_status_var.set(f'"{nombre}" ya no tiene unidades pendientes.')
            return
        if self.proc_loader is not None and self.proc_loader.poll() is None:
            self.lote_status_var.set('Ya hay un lote del Loader en curso -- espera a que termine.')
            return
        if not messagebox.askyesno(
            'Confirmar pedido',
            f'Esto lanza al Loader y al Sorter a fabricar {cantidad} unidad(es) de "{nombre}" '
            f'para el pedido #{pedido_id}.\n'
            'Si estás controlando el Loader o el Sorter a mano ahora mismo, '
            'se pelearán por el brazo con la demo automática.\n\n¿Continuar?',
        ):
            return
        if not self._reclamar_grupo([pedido_id]):
            return
        self._audit(f'Lanzar este pedido: #{pedido_id} {nombre} x{cantidad}')
        self.pedido_id_en_curso = pedido_id  # para pintar el boton en verde, ver refresh_pedidos()
        self._lanzar_produccion(color, cantidad, nombre, pedido_id=pedido_id)

    def _lanzar_produccion(self, color, cantidad, etiqueta, pedido_id=None, forzar_reparto=False):
        """Arranca sorter_demo (si no esta ya vivo, persiste entre lotes) y
        un loader_demo nuevo. 'color'=None hace que loader_demo cicle los
        TRES colores a la vez (comportamiento de por defecto de
        CRATE_CUBES, un R+un G+un B por vuelta, 'cantidad' vueltas) --
        usado por 'Lanzar todos los pedidos pendientes' (sesion
        2026-08-30, segunda vuelta: la primera version hacia un color
        detras de otro con una cola, pero el usuario lo probo y le parecio
        demasiado lento/pesado de ver -- "esto asi es un coñazo y poco
        productivo" -- así que ahora reparte YA los tres colores dentro
        del mismo lote, igual que hacia el Loader originalmente antes de
        que existiera 'only_color'). 'color'=<letra> (desde 'Lanzar lote',
        un producto elegido a mano) sigue restringiendo a un solo color --
        pero ojo, "restringir" ya NO significa que el Loader solo coja ese
        cubo fisico (ver loader_demo.py, sesion 2026-09-01: 'cubes =
        CRATE_CUBES' es fijo, SIEMPRE usa los 3 cubos reales). 'only_color'
        es solo la ETIQUETA del producto que se esta fabricando -- fija el
        LED de producto y hace que el Sorter cuente cada entrega real como
        ESTE producto (ver SorterDemo._lote_color_objetivo), sea cual sea
        el color real del cubo que la genero. Por eso vale igual para un
        producto con cubo fisico (R/G/B) que para uno "solo LED" (Y/M/C/W,
        sesion 2026-09-02, catalogo ampliable) -- no hay que distinguirlos
        aqui, loader_demo.py ya no exige que 'only_color' sea un color
        de CRATE_CUBES."""
        # Un loader_demo vivo que NO lanzo este panel (otra ventana anterior, o
        # uno lanzado a mano por terminal): no lanzar otro encima, se pelearian
        # por el brazo del Loader (ver _demo_vivo).
        if (self.proc_loader is None or self.proc_loader.poll() is not None) and _demo_vivo('loader_demo'):
            self.lote_status_var.set(
                'Ya hay un loader_demo corriendo fuera del panel -- espera a que termine o páralo.')
            self.pedido_id_en_curso = None
            self.producto_en_curso = None
            self.cola_lotes = []
            return
        # Recordar EXACTAMENTE estos argumentos para "Repetir ultimo lote"
        # (sesion 2026-09-10) -- da igual por que puerta se llego aqui
        # (Lanzar lote / Lanzar este pedido / un tramo de Lanzar todo el
        # resumen), _lanzar_produccion es el unico sitio por el que pasan
        # todos, asi que es el sitio correcto para guardarlo una sola vez.
        self._ultimo_lote_args = dict(
            color=color, cantidad=cantidad, etiqueta=etiqueta,
            pedido_id=pedido_id, forzar_reparto=forzar_reparto,
        )
        self.repetir_lote_btn.config(state='normal')
        # Salida a fichero, NO a DEVNULL (bug real, sesion 2026-08-30: con
        # DEVNULL, si un lote se queda sin hacer nada -- p.ej. un cubo
        # descolocado tras una prueba anterior, o el otro robot ocupado a
        # mano en el mismo topic -- no habia forma de ver el motivo, ni
        # desde el panel ni por fuera).
        # Si ya hay un sorter_demo vivo (de una ventana anterior del panel o
        # lanzado a mano) se reutiliza: es persistente entre lotes y escucha
        # /production/* igual. Lanzar otro encima es el fallo de _demo_vivo.
        if (self.proc_sorter is None or self.proc_sorter.poll() is not None) and not _demo_vivo('sorter_demo'):
            log_sorter = open('/tmp/lote_sorter.log', 'w')
            self.proc_sorter = subprocess.Popen([
                'ros2', 'run', 'panda_controller', 'sorter_demo', '--ros-args',
                '-r', '__ns:=/sorter',
                '-p', 'robot_base_x:=0.0', '-p', 'robot_base_y:=1.00', '-p', 'robot_base_z:=0.74',
            ], stdout=log_sorter, stderr=subprocess.STDOUT)
        loader_args = [
            'ros2', 'run', 'panda_controller', 'loader_demo', '--ros-args',
            '-r', '__ns:=/loader',
            '-p', f'cycles:={cantidad}', '-p', 'led_topic:=/comando_led_loader',
            '-p', 'led_topic_producto:=/comando_led_producto',
        ]
        if color is not None:
            # Comillas EXPLICITAS alrededor del valor (sesion 2026-09-02,
            # bug real): ROS2 interpreta '-p nombre:=valor' como YAML, y
            # en YAML 'Y'/'y'/'N'/'n'/'yes'/'no'/'on'/'off' son literales
            # BOOLEANOS -- 'only_color:=Y' (amarillo) se convertia en
            # 'True' en vez de la letra "Y", y loader_demo.py crasheaba al
            # declarar el parametro (tipo STRING esperado, BOOL recibido).
            # Forzando comillas YAML se interpreta siempre como texto, sea
            # cual sea la letra.
            loader_args += ['-p', f'only_color:="{color}"']
        log_loader = open('/tmp/lote_loader.log', 'w')
        self.proc_loader = subprocess.Popen(loader_args, stdout=log_loader, stderr=subprocess.STDOUT)
        # Contador de piezas de ESTE lote (sesion 2026-08-31): foto de las
        # entregas totales acumuladas hasta ahora (self.node.entregas_color,
        # ver TeleopGuiNode._on_cube_delivered) y el objetivo pedido -- la
        # diferencia en cada refresco (_actualizar_estado_lote) es lo
        # fabricado DE VERDAD en este lote, sin mirar para nada si
        # Taller_Administracion tiene el reparto automatico activado o no
        # (eso es una decision administrativa aparte, ver conversacion).
        self.lote_inicio_entregas = dict(self.node.entregas_color)
        self.lote_color_objetivo = color  # None = mixto, letra = un solo producto
        colores_lote = [color] if color is not None else ['R', 'G', 'B']
        self.lote_objetivo_por_color = {c: cantidad for c in colores_lote}
        # Avisa al Sorter (si ya estaba vivo de un lote anterior) para que
        # aparque y baile antes de seguir -- el Loader, al ser un proceso
        # nuevo, ya lo hace solo al arrancar (ver loader_demo.py).
        self.node.pub_nuevo_lote.publish(Bool(data=True))
        # Color objetivo para el Sorter (sesion 2026-09-01, ver
        # SorterDemo._on_lote_color_objetivo): con un solo producto, toda
        # entrega de este lote cuenta como 'color' aunque el cubo real sea
        # de otro color (el Loader ya usa los 3 cubos, ver loader_demo.py).
        # Formato 'COLOR:CANTIDAD:PEDIDO_ID', COLOR vacio en reparto mixto
        # (cada cubo sigue contando por su color real). La cantidad es
        # SIEMPRE el total de piezas de verdad -- en reparto mixto son
        # los 3 colores a la vez ('cantidad' vueltas x 3, ver loader_demo.py
        # objetivo_piezas), no las vueltas en si. Le dice al Sorter cuando
        # el lote esta REALMENTE terminado (bug real, sesion 2026-09-01:
        # antes en reparto mixto nunca se avisaba de que habia terminado,
        # el LED de producto se quedaba encendido con el ultimo color real
        # visto) para apagar entonces el LED de producto -- no cuando el
        # Loader simplemente acaba de descargar en la cinta. pedido_id
        # (relleno solo desde 'Lanzar este pedido') ata cada entrega a ESE
        # pedido concreto sin depender del interruptor de reparto_automatico.
        # 'forzar_reparto' (mismo dia, "Lanzar todo este producto": varios
        # pedidos reales del mismo producto sumados, sin un pedido_id
        # unico) hace que cada pieza se aplique igual al pedido pendiente
        # mas antiguo de ese color, tambien sin mirar el interruptor.
        total_piezas = cantidad if color is not None else cantidad * 3
        mensaje_lote = f'{color or ""}:{total_piezas}:{pedido_id or ""}:{"1" if forzar_reparto else ""}'
        self.node.pub_lote_color_objetivo.publish(String(data=mensaje_lote))
        self.lote_status_var.set(
            f'Fabricando {etiqueta} (Loader lanzado, Sorter activo). '
            'Logs en /tmp/lote_loader.log y /tmp/lote_sorter.log.')

    def _agrupar_pedidos_por_producto(self, pedidos):
        """Agrupa pedidos activos por producto (color), sumando lo que
        falta por completar de cada uno (puede haber varios pedidos del
        mismo producto, de distintos clientes). Compartido por
        _refrescar_resumen_productos (solo mostrar) y lanzar_todo_resumen
        (fabricar de verdad) -- una sola fuente para el mismo calculo.
        Ademas de 'restante' (lo que hay que fabricar), suma 'completada'
        y 'pedida' (sesion 2026-09-03, a peticion del usuario: "podemos
        saber cuantas piezas estan hechas y cuantas faltan" -- en
        'Pedidos pendientes' ya se veia por pedido individual
        (completada/pedida), pero el resumen agrupado solo mostraba lo
        pendiente, no el total hecho de ese producto entre todos sus
        pedidos). Tambien guarda 'ids' -- sesion 2026-09-13, necesario
        para reclamar (ver TeleopGuiNode.reclamar_pedido) cada pedido
        concreto antes de lanzar produccion para el grupo entero."""
        por_producto = {}
        for p in pedidos:
            color = p['producto']['color']
            restante = p['cantidad_pedida'] - p['cantidad_completada']
            if restante <= 0:
                continue
            fila = por_producto.setdefault(
                color, {'nombre': p['producto']['nombre'], 'restante': 0, 'n_pedidos': 0,
                        'completada': 0, 'pedida': 0, 'ids': []})
            fila['restante'] += restante
            fila['n_pedidos'] += 1
            fila['completada'] += p['cantidad_completada']
            fila['pedida'] += p['cantidad_pedida']
            fila['ids'].append(p['id'])
        return por_producto

    def _reclamar_grupo(self, ids) -> bool:
        """Reclama TODOS los pedidos de 'ids' para esta máquina (ver
        TeleopGuiNode.reclamar_pedido) antes de lanzar producción -- si
        alguno falla (otra celda se lo quedó primero, o fallo de red), no
        se lanza nada: mejor reintentar en el siguiente refresco con los
        datos ya al día que fabricar de menos sin que el operario se
        entere. Los que sí se reclamaron en un intento fallido se quedan
        reclamados -- no hace daño, ya los tiene esta máquina para la
        próxima vez."""
        for pid in ids:
            if not self.node.reclamar_pedido(pid):
                self.log(
                    f'No se ha podido reclamar el pedido #{pid} para esta máquina '
                    '(ya asignado a otra, o sin conexión) -- lote cancelado, se '
                    'reintentará solo en el próximo ciclo.', False)
                return False
        return True

    def lanzar_todo_resumen(self):
        """'Lanzar todo el resumen' (sesion 2026-09-03, sustituye a
        'Lanzar todos los pedidos pendientes' -- ver comentario junto al
        boton, mas arriba, para el porque del cambio). Encola cada
        producto del 'Resumen por producto' y los va lanzando UNO DETRAS
        DE OTRO (ver _lanzar_siguiente_de_cola / _actualizar_estado_lote):
        mismo mecanismo que 'Lanzar todo' de un solo producto
        (lanzar_producto_agrupado, LED fijo, forzar_reparto=True), solo
        que encadenado automaticamente sin pulsar boton por boton."""
        if self.proc_loader is not None and self.proc_loader.poll() is None:
            self.lote_status_var.set('Ya hay una producción en curso -- espera a que termine.')
            return
        if self.cola_lotes:
            self.lote_status_var.set('Ya hay una cola de lotes en marcha -- espera a que termine.')
            return
        pedidos = self.node.fetch_pedidos_pendientes()
        if pedidos is None:
            self.lote_status_var.set('Sin conexión con Taller_Administracion.')
            return
        por_producto = self._agrupar_pedidos_por_producto(pedidos)
        if not por_producto:
            self.lote_status_var.set('No hay pedidos pendientes que fabricar.')
            return
        items = [(color, info['restante'], info['nombre'], info['ids'])
                 for color, info in sorted(por_producto.items())]
        resumen = ', '.join(f'{nombre} ({color}) x{cantidad}' for color, cantidad, nombre, _ids in items)
        if not messagebox.askyesno(
            'Confirmar producción de todo el resumen',
            f'Esto va a fabricar, UN PRODUCTO DETRAS DE OTRO (el siguiente no empieza '
            f'hasta que el anterior termine de verdad, LED de producto fijo por lote):'
            f'\n{resumen}\n\n'
            'Si estás controlando el Loader o el Sorter a mano ahora mismo, '
            'se pelearán por el brazo con la demo automática.\n\n¿Continuar?',
        ):
            return
        self._encolar_resumen(items, resumen, 'Lanzar todo el resumen (uno detrás de otro)')

    def _encolar_resumen(self, items, resumen, motivo):
        """Parte comun de lanzar_todo_resumen() (boton, con dialogo de
        confirmacion) y el modo Automatico en refresh_pedidos() (sin
        dialogo -- no hay nadie para pulsar "Si" cada 4s)."""
        self._audit(f'{motivo}: {resumen}')
        self.cola_lotes = items
        self._lanzar_siguiente_de_cola()

    def _auto_lanzar_si_toca(self, pedidos):
        """Modo Automatico (checkbox, sesion 2026-09-13): mismo camino que
        el boton 'Lanzar todo el resumen', pero disparado solo desde
        refresh_pedidos() (cada 4s) en vez de a mano, y sin dialogo de
        confirmacion. Mismas guardas que el boton (nada en curso, nada en
        cola) para no lanzar un lote encima de otro. 'pedidos' viene ya
        pedido por refresh_pedidos(), no se vuelve a pedir aqui."""
        if not self.auto_produccion.get():
            return
        if pedidos is None or not pedidos:
            return
        if self.proc_loader is not None and self.proc_loader.poll() is None:
            return
        if self.cola_lotes:
            return
        por_producto = self._agrupar_pedidos_por_producto(pedidos)
        if not por_producto:
            return
        items = [(color, info['restante'], info['nombre'], info['ids'])
                 for color, info in sorted(por_producto.items())]
        resumen = ', '.join(f'{nombre} ({color}) x{cantidad}' for color, cantidad, nombre, _ids in items)
        self._encolar_resumen(items, resumen, 'Automático (sin confirmar)')

    def guardar_numero_maquina(self):
        """Boton 'Guardar' junto al desplegable Nº Máquina -- ver
        CONFIG_MAQUINA_PATH. El combobox es 'readonly' (solo elige de la
        lista 1..MAX_MAQUINAS, no se puede escribir a mano), asi que aqui
        ya no hace falta validar texto libre. Pide la clave (CLAVE_MAQUINA)
        antes de aplicar nada -- es un control fisico compartido, sin login
        como la web, cualquiera que pase por delante podria tocarlo sin
        querer. Si la clave es correcta, cambia self.node.numero_maquina en
        caliente (afecta al siguiente refresco de pedidos, sin reiniciar
        nada) Y lo persiste en disco para que se mantenga en el proximo
        arranque."""
        numero = self.numero_maquina_var.get()
        if numero == self.node.numero_maquina:
            return  # no ha cambiado nada, no hace falta ni pedir la clave
        clave = simpledialog.askstring(
            'Confirmar cambio', 'Clave para cambiar el Nº Máquina:', show='*', parent=self.root)
        if clave != CLAVE_MAQUINA:
            if clave is not None:  # None = ha pulsado Cancelar, no hace falta avisar de nada
                messagebox.showerror('Clave incorrecta', 'No se ha cambiado el Nº Máquina.')
            self.numero_maquina_var.set(self.node.numero_maquina)  # deshace la seleccion en el desplegable
            return
        self.node.numero_maquina = numero
        _guardar_numero_maquina(numero)
        self.lote_status_var.set(f'Nº Máquina guardado: {numero} (se mantiene en el próximo arranque).')

    def _lanzar_siguiente_de_cola(self):
        """Saca el siguiente producto de self.cola_lotes y lo lanza como
        un lote normal de un solo producto. Llamado una vez al confirmar
        'Lanzar todo el resumen', y despues cada vez que
        _actualizar_estado_lote detecta que el lote EN CURSO ha
        terminado de verdad (Sorter ha contado tantas entregas reales
        como pedia el lote, no solo que el Loader haya acabado de
        descargar en la cinta -- ver sorter_demo.py::run())."""
        if not self.cola_lotes:
            return
        if self.proc_loader is not None and self.proc_loader.poll() is None:
            # No deberia pasar (solo se llama cuando el lote anterior ya
            # ha terminado de verdad), pero por seguridad no lanzar dos
            # loader_demo a la vez -- reintentar en el siguiente sondeo.
            self.root.after(2000, self._lanzar_siguiente_de_cola)
            return
        color, cantidad, nombre, ids = self.cola_lotes.pop(0)
        if not self._reclamar_grupo(ids):
            # No se ha podido reclamar (ver _reclamar_grupo) -- se salta
            # este producto y se sigue con el siguiente de la cola, en vez
            # de dejar la cola entera colgada esperando a este.
            self.root.after(500, self._lanzar_siguiente_de_cola)
            return
        self.producto_en_curso = color
        self._lanzar_produccion(color, cantidad, nombre, forzar_reparto=True)

    def _actualizar_estado_lote(self):
        if self.proc_loader is None:
            estado_loader = 'sin lotes lanzados todavía'
        elif self.proc_loader.poll() is None:
            estado_loader = 'EN CURSO'
        else:
            estado_loader = f'terminado (código {self.proc_loader.returncode})'
            # El Loader terminar de descargar en la cinta no es el mismo
            # instante en que el Sorter termina de clasificar (ver
            # sorter_demo.py), pero para el boton "Lanzar este pedido" en
            # verde basta con esto: en cuanto el Loader ya no esta vivo,
            # ya no hay riesgo de mandar dos lotes a la vez, que es lo que
            # este color intenta evitar visualmente.
            self.pedido_id_en_curso = None
            self.producto_en_curso = None
        if self.proc_sorter is not None and self.proc_sorter.poll() is None:
            estado_sorter = 'activo'
        elif _demo_vivo('sorter_demo'):
            estado_sorter = 'activo (externo)'
        else:
            estado_sorter = 'sin arrancar' if self.proc_sorter is None else 'parado'
        progreso = ''
        if self.lote_inicio_entregas is not None and self.lote_objetivo_por_color:
            # Piezas de verdad fabricadas en ESTE lote (ver _lanzar_produccion):
            # diferencia entre las entregas totales acumuladas ahora y la foto
            # de cuando se lanzo, contadas por el propio robot al confirmar
            # cada entrega real -- no depende de Taller_Administracion ni de
            # si su reparto automatico esta activado.
            if self.lote_color_objetivo is not None:
                # Lote de un solo producto (sesion 2026-09-01, correccion
                # real del usuario): el Loader usa los 3 cubos, asi que una
                # entrega de CUALQUIER color cuenta para este producto --
                # sumar los tres en vez de exigir que coincida el color
                # real (ver loader_demo.py y SorterDemo._lote_color_objetivo).
                objetivo = self.lote_objetivo_por_color[self.lote_color_objetivo]
                hechas_total = sum(
                    self.node.entregas_color.get(c, 0) - self.lote_inicio_entregas.get(c, 0)
                    for c in ('R', 'G', 'B'))
                progreso = f'  |  Fabricadas: {hechas_total}/{objetivo} ({self.lote_color_objetivo})'
                # Lote REALMENTE terminado (sesion 2026-09-03): el Sorter
                # ya ha contado tantas entregas reales como pedia el lote
                # (mismo criterio que apaga el LED de producto en
                # sorter_demo.py::run(), no solo que el Loader haya
                # acabado de descargar en la cinta) -- si viene de
                # "Lanzar todo el resumen" (self.cola_lotes no vacio),
                # este es el momento de encadenar el siguiente producto.
                if hechas_total >= objetivo and self.cola_lotes:
                    self._lanzar_siguiente_de_cola()
            else:
                partes = []
                hechas_total = 0
                objetivo_total = 0
                for c, objetivo in self.lote_objetivo_por_color.items():
                    hechas = self.node.entregas_color.get(c, 0) - self.lote_inicio_entregas.get(c, 0)
                    partes.append(f'{c}:{hechas}/{objetivo}')
                    hechas_total += hechas
                    objetivo_total += objetivo
                progreso = f'  |  Fabricadas: {hechas_total}/{objetivo_total} ({" ".join(partes)})'
        self.lote_status_var.set(f'Lote Loader: {estado_loader}  |  Sorter: {estado_sorter}{progreso}')

    def _poll_lote(self):
        self._actualizar_estado_lote()
        self.root.after(2000, self._poll_lote)

    def _refrescar_resumen_productos(self, pedidos):
        """Bloque 'Resumen por producto' (sesion 2026-09-01): agrupa todos
        los pedidos pendientes por producto y muestra el total + un boton
        'Lanzar todo' por producto. Reconstruido junto con refresh_pedidos()
        (mismos 4s), no tiene sondeo propio."""
        for child in self.resumen_producto_frame.winfo_children():
            child.destroy()
        if not pedidos:
            tk.Label(self.resumen_producto_frame,
                     text='Sin pedidos pendientes.' if pedidos == [] else
                     f'Sin conexión con Taller_Administracion ({self.node.taller_api_base}).',
                     font=self.mono_font, bg=PANEL_BG, fg=TEXT_LIGHT if pedidos == [] else '#ff6b6b'
                     ).grid(row=0, column=0, sticky='w', padx=6, pady=4)
            return
        por_producto = self._agrupar_pedidos_por_producto(pedidos)
        for i, (color, info) in enumerate(sorted(por_producto.items())):
            plural = 'pedido' if info['n_pedidos'] == 1 else 'pedidos'
            texto = (f"{info['nombre']} ({color}): {info['completada']}/{info['pedida']} hechas "
                     f"-- {info['restante']} pendiente(s) ({info['n_pedidos']} {plural})")
            tk.Label(self.resumen_producto_frame, text=texto, font=self.mono_font,
                     bg=PANEL_BG, fg=TEXT_LIGHT).grid(row=i, column=0, sticky='w', padx=6, pady=2)
            en_curso = (self.producto_en_curso == color)
            color_boton = '#2e7d32' if en_curso else PANEL_BG
            tk.Button(self.resumen_producto_frame,
                      text='Lanzando...' if en_curso else 'Lanzar todo', font=self.big_font,
                      bg=color_boton, fg=TEXT_LIGHT,
                      activebackground='#2e7d32' if en_curso else '#3a3a3a', activeforeground=TEXT_LIGHT,
                      command=lambda c=color, n=info['nombre'], r=info['restante'], ids=info['ids']:
                      self.lanzar_producto_agrupado(c, r, n, ids)
                      ).grid(row=i, column=1, padx=6, pady=2)

    def lanzar_producto_agrupado(self, color, cantidad, nombre, ids):
        """'Lanzar todo' del resumen por producto (sesion 2026-09-01, a
        peticion del usuario: "si hay dos lotes de tornillos, uno con 3 y
        otro con 2, el lote seria de cinco, para hacer todos los tornillos
        a la vez"). A diferencia de lanzar_pedido() no hay un pedido_id
        unico al que atar cada pieza (pueden ser varios pedidos distintos)
        -- se manda forzar_reparto=True para que cada pieza se aplique
        igual al pedido pendiente MAS ANTIGUO de ese color, sin depender
        del interruptor reparto_automatico (mismo problema real que
        lanzar_pedido, ver _lanzar_produccion). 'ids' (sesion 2026-09-13):
        los pedidos concretos que forman este grupo, para reclamarlos
        antes de lanzar -- ver _reclamar_grupo."""
        if cantidad < 1:
            self.lote_status_var.set(f'"{nombre}" ya no tiene unidades pendientes.')
            return
        if self.proc_loader is not None and self.proc_loader.poll() is None:
            self.lote_status_var.set('Ya hay un lote del Loader en curso -- espera a que termine.')
            return
        if not messagebox.askyesno(
            'Confirmar producto',
            f'Esto lanza al Loader y al Sorter a fabricar {cantidad} unidad(es) de "{nombre}", '
            'sumando todos los pedidos pendientes de ese producto.\n'
            'Si estás controlando el Loader o el Sorter a mano ahora mismo, '
            'se pelearán por el brazo con la demo automática.\n\n¿Continuar?',
        ):
            return
        if not self._reclamar_grupo(ids):
            return
        self._audit(f'Lanzar todo el producto: {nombre} x{cantidad}')
        self.producto_en_curso = color  # para pintar el boton en verde, ver _refrescar_resumen_productos()
        self._lanzar_produccion(color, cantidad, nombre, forzar_reparto=True)

    def refresh_pedidos(self):
        for child in self.pedidos_frame.winfo_children():
            child.destroy()
        pedidos = self.node.fetch_pedidos_pendientes()
        self._refrescar_resumen_productos(pedidos)
        self._auto_lanzar_si_toca(pedidos)
        if pedidos is None:
            tk.Label(self.pedidos_frame, text=f'Sin conexion con Taller_Administracion ({self.node.taller_api_base}).',
                     font=self.mono_font, bg=PANEL_BG, fg='#ff6b6b'
                     ).grid(row=0, column=0, sticky='w', padx=6, pady=4)
        elif not pedidos:
            tk.Label(self.pedidos_frame, text='No hay pedidos pendientes.',
                     font=self.mono_font, bg=PANEL_BG, fg=TEXT_LIGHT
                     ).grid(row=0, column=0, sticky='w', padx=6, pady=4)
        else:
            # Antes se cortaba en MAX_PEDIDOS_VISIBLES filas con un texto
            # "...y N mas" (sesion 2026-08-30) -- ahora que la lista tiene
            # su propio scroll (sesion 2026-09-02, ver mas arriba), se
            # muestran TODOS, ya vienen ordenados por fecha desde el
            # backend.
            for i, p in enumerate(pedidos):
                producto = p['producto']
                texto = (f"{producto['nombre']} ({producto['color']}): "
                         f"{p['cantidad_completada']}/{p['cantidad_pedida']} -- {p['estado']}")
                tk.Label(self.pedidos_frame, text=texto, font=self.mono_font, bg=PANEL_BG, fg=TEXT_LIGHT
                         ).grid(row=i, column=0, sticky='w', padx=6, pady=2)
                tk.Button(self.pedidos_frame, text='+1 pieza', font=self.big_font,
                          bg=PANEL_BG, fg=TEXT_LIGHT, activebackground='#3a3a3a', activeforeground=TEXT_LIGHT,
                          command=lambda c=producto['color']: self.do_marcar_pieza(c)
                          ).grid(row=i, column=1, padx=6, pady=2)
                # "Lanzar este pedido" (sesion 2026-09-01, a peticion del
                # usuario: "mira si puedo seleccionar el lote que quiero
                # lanzar" -- entre pedido y pedido el operario cambia de
                # produccion a mano, no quiere que se enlacen solos.
                # Lanza SOLO este pedido como lote de un solo producto
                # (mismo mecanismo que 'Lanzar lote': LED fijo, usa los 3
                # cubos como material, ver _lanzar_produccion), con la
                # cantidad que falta de ESTE pedido en concreto.
                restante = p['cantidad_pedida'] - p['cantidad_completada']
                # Verde mientras ESTE pedido este en curso de verdad
                # (sesion 2026-09-01, a peticion del usuario: "pon el
                # boton en verde para saber que se ha lanzado") --
                # self.pedido_id_en_curso se fija en lanzar_pedido() y se
                # limpia en _actualizar_estado_lote() en cuanto el Loader
                # termina (vivo o no, ver ese metodo). Como este frame se
                # destruye y recrea entero cada 4s (ver arriba), no basta
                # con cambiar el color del boton ya creado -- hay que
                # volver a pintarlo en verde cada vez que se reconstruye.
                en_curso = (self.pedido_id_en_curso == p['id'])
                color_boton = '#2e7d32' if en_curso else PANEL_BG
                tk.Button(self.pedidos_frame,
                          text='Lanzando...' if en_curso else 'Lanzar este pedido', font=self.big_font,
                          bg=color_boton, fg=TEXT_LIGHT,
                          activebackground='#2e7d32' if en_curso else '#3a3a3a', activeforeground=TEXT_LIGHT,
                          command=lambda pid=p['id'], c=producto['color'], n=producto['nombre'], r=restante:
                          self.lanzar_pedido(pid, c, r, n)
                          ).grid(row=i, column=2, padx=6, pady=2)
        self.root.after(4000, self.refresh_pedidos)

    def do_marcar_pieza(self, color):
        ok, info = self.node.marcar_cubo_clasificado(color)
        if ok:
            pedido = info.get('pedido')
            if pedido is not None:
                self.log(f'Pieza {color} registrada en el pedido #{pedido["id"]} '
                          f'({pedido["cantidad_completada"]}/{pedido["cantidad_pedida"]}).', True)
            else:
                self.log(f'Pieza {color} guardada en stock (sin pedido pendiente, o reparto '
                          f'manual activo) -- {info.get("stock_actual", "?")} unidades.', True)
        else:
            self.log(f'No se pudo registrar la pieza {color}: {info}', False)
        self.refresh_pedidos()

    def _spin_tick(self):
        rclpy.spin_once(self.node, timeout_sec=0.0)
        self.root.after(50, self._spin_tick)

    def refresh_position(self):
        p = self.node.tcp_world()
        self.pos_var.set(
            f'TCP mundo:  x={p[0]:.4f}   y={p[1]:.4f}   z={p[2]:.4f}   '
            f'giro={np.degrees(self.node.yaw):.1f} deg')

    def refresh_step(self):
        self.step_var.set(f'Paso actual: {self.node.step * 100:.2f} cm')

    def log(self, msg, ok=True):
        self.log_var.set(msg)
        self.log_label.config(fg=GREEN if ok else '#ff6b6b')

    def do_stop(self):
        self.node.send_stop()
        self.log('PARADA manual enviada desde el panel.', ok=False)

    def do_rearm(self):
        self.node.send_rearm()
        self.log('Rearme enviado -- puedes seguir moviendo el brazo.')

    def on_stop_change(self, stopped):
        self.set_stopped_ui(stopped)
        if stopped:
            self.log('PARADA DE EMERGENCIA ACTIVA (fisica, manual o de otra demo).', ok=False)

    def set_stopped_ui(self, stopped):
        if stopped:
            self.status_var.set('PARADO')
            self.status_label.config(fg=RED)
            self.rearm_btn.config(state='normal', bg=YELLOW, fg='black',
                                   activebackground='#c9a400', activeforeground='black')
        else:
            self.status_var.set('EN MARCHA')
            self.status_label.config(fg=GREEN)
            self.rearm_btn.config(state='disabled', bg=GREY, fg=GREY_TEXT,
                                   activebackground=GREY, activeforeground=GREY_TEXT)
        # Los botones de jog/pinza/postura YA NO se deshabilitan durante la
        # parada (sesion 2026-09-10, cambio de rumbo explicito del usuario:
        # "cuando salta la parada de emergencia tiene que dejar mover el
        # robot a mano" -- necesita poder liberar a mano un dedo trabado en
        # vez de solo esperar a REARME, que no arregla nada fisico por si
        # solo). node.move_delta/set_gripper/etc. ya no rechazan la orden
        # tampoco -- ver esos metodos para el riesgo conocido y aceptado
        # (self.real_theta de una demo automatica en pausa puede quedar
        # desactualizado). Se deja _jog_buttons y este metodo tal cual por
        # si se quiere volver a deshabilitar algo especifico en el futuro,
        # simplemente no se tocan aqui.

    def _audit(self, msg):
        # Registro de CADA clic con marca de tiempo en el log del nodo (que
        # va a /tmp/teleop_gui.log si se lanzo con stdout redirigido) -- para
        # poder saber con certeza que boton se pulso de verdad ante un
        # comportamiento inesperado, en vez de tener que adivinarlo (bug de
        # diagnostico real: un salto a HOME que no se pudo explicar sin
        # saber si fue un click en el boton HOME o el resultado de otro
        # boton).
        self.node.get_logger().info(f'[click] {msg}')

    def do_move(self, dx, dy, dz):
        ok, msg = self.node.move_delta(dx, dy, dz)
        p = self.node.tcp_world()
        # Registrar el RESULTADO (no solo la intencion) con la posicion TCP
        # resultante: sin esto, un click rechazado (IK no convergio / fuera
        # de limites) queda indistinguible en el log de uno aceptado, y
        # reconstruir a posteriori donde estaba realmente la pinza en un
        # momento dado (p.ej. al pulsar CERRAR) se vuelve una suma a ciegas
        # que puede arrastrar error si algun click intermedio se rechazo.
        self._audit(f'Mover TCP dx={dx:.4f} dy={dy:.4f} dz={dz:.4f} -> '
                     f'{"OK" if ok else "RECHAZADO"} pos=({p[0]:.4f},{p[1]:.4f},{p[2]:.4f})')
        self.refresh_position()
        self.log(msg if not ok else 'Movido.', ok)

    def step_down(self):
        self.node.step = max(STEP_MIN, self.node.step / 2.0)
        self.refresh_step()

    def step_up(self):
        self.node.step = min(STEP_MAX, self.node.step * 2.0)
        self.refresh_step()

    def do_center(self):
        ok, msg = self.node.center_on_cube()
        self._audit(f'Centrar sobre cubo -> {"OK" if ok else "RECHAZADO"} {msg}')
        self.refresh_position()
        self.log(msg, ok)

    def do_rotate(self, dyaw):
        ok, msg = self.node.rotate_delta(dyaw)
        self._audit(f'Girar pinza dyaw={np.degrees(dyaw):.1f} deg -> '
                     f'{"OK" if ok else "RECHAZADO"} {msg}')
        self.refresh_position()
        self.log(msg, ok)

    def do_open(self):
        p = self.node.tcp_world()
        self._audit(f'ABRIR pinza pos=({p[0]:.4f},{p[1]:.4f},{p[2]:.4f})')
        ok = self.node.set_gripper(self.node.gripper_open)
        self.log('Pinza -> ABIERTA' if ok else 'PARADA activa -- rearma antes de abrir.', ok)

    def do_close(self):
        p = self.node.tcp_world()
        self._audit(f'CERRAR pinza pos=({p[0]:.4f},{p[1]:.4f},{p[2]:.4f})')
        ok = self.node.set_gripper(self.node.gripper_closed)
        self.log('Pinza -> CERRADA' if ok else 'PARADA activa -- rearma antes de cerrar.', ok)

    def do_home(self):
        # HOME esta pegado a otros botones de uso frecuente (Abrir/Cerrar
        # pinza, Orientar pinza abajo) en la misma columna -- un misclick ahi
        # manda el brazo a HOME sin querer y se ve como un "salto" raro
        # (reportado por el usuario). Se pide confirmacion porque es la
        # unica accion de este panel que reposiciona el brazo entero de
        # golpe a un sitio lejos de donde este trabajando.
        if not messagebox.askyesno(
            'Confirmar HOME',
            'Esto mueve el brazo entero a la posicion de reposo (HOME).\n'
            'Si estabas cerca de un cubo, se alejara de el.\n\n'
            'Continuar?',
        ):
            self._audit('HOME cancelado por el usuario (dialogo de confirmacion)')
            return
        self._audit('HOME confirmado')
        ok_home, msg_home = self.node.go_home()
        if not ok_home:
            self.refresh_position()
            self.log(msg_home, False)
            return
        # Encadenar la reorientacion aqui tambien: igual que al arrancar la
        # ventana, HOME_POSITIONS no deja la pinza mirando hacia abajo, y
        # sin este paso los botones de "Mover TCP" volverian a rechazarse
        # todos tras pulsar HOME.
        ok, msg = self.node.align_gripper_down()
        self.refresh_position()
        self.log('Vuelto a HOME y pinza reorientada hacia abajo. ' + msg, ok)

    def do_align(self):
        self._audit('Orientar pinza abajo')
        ok, msg = self.node.align_gripper_down()
        self.refresh_position()
        self.log(msg, ok)

    def do_led(self, letter):
        names = {'R': 'rojo', 'G': 'verde', 'B': 'azul', '0': 'apagado'}
        self._audit(f'LED -> {names[letter]}')
        self.node.set_led(letter)
        self.log(f'LED -> {names[letter]}')

    def run(self):
        self.root.mainloop()


def main(args=None):
    rclpy.init(args=args)
    node = TeleopGuiNode()
    node.get_logger().info('Esperando a que el driver se suscriba...')
    if not node.wait_for_subscribers():
        node.get_logger().error(
            f'Nadie se ha suscrito tras {node.max_wait_seconds:.0f}s '
            "(revisa que 'ros2 launch panda_controller robot_launch.py' este corriendo)."
        )
        node.destroy_node()
        rclpy.shutdown()
        return
    node.get_logger().info('Driver suscrito. Abriendo ventana...')

    app = TeleopApp(node)
    try:
        app.run()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
