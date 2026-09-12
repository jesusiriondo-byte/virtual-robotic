#!/usr/bin/env python3
"""
Teleoperacion manual del Panda por teclado: mueve el TCP en linea recta
(X/Y/Z del mundo, con la pinza siempre mirando hacia abajo, misma
orientacion GRASP_R que el resto del paquete) y controla la pinza y el LED
a mano, para poder verificar visualmente donde esta REALMENTE cada cubo
(la localizacion automatica por vision, en `vision_lift_cube.py` /
`best_color_repeat_lift.py`, ha dado problemas de calibracion en pruebas
reales -- ver `resumen_proyecto_panda.md`).

Necesita una terminal INTERACTIVA (lee teclas sueltas del teclado sin
Intro). No se puede lanzar en segundo plano / con `docker exec -d`.

Controles:
  w / s   : +X / -X  (adelante / atras, alejarse/acercarse a la base)
  a / d   : +Y / -Y  (izquierda / derecha)
  q / e   : +Z / -Z  (subir / bajar)
  [ / ]   : paso mas pequeno / paso mas grande (empieza en 1 cm)
  c       : CERRAR pinza
  o       : ABRIR pinza
  r/g/b   : LED rojo/verde/azul (/comando_led)
  0       : LED apagado
  h       : ir a HOME_POSITIONS (posicion inicial de reposo; la pinza NO
            queda orientada hacia abajo todavia, es una postura distinta)
  v       : orientar la pinza hacia abajo AQUI MISMO, sin moverse en XYZ
            (pulsalo despues de 'h' y antes de usar wasd/qe)
  p       : imprimir la posicion actual del TCP (mundo) sin moverse
  x       : salir

Como lanzarlo (con Webots en PLAY, `robot_launch.py` corriendo, en una
terminal INTERACTIVA -- por ejemplo `docker exec -it ros2_panda_dev bash`):

    ros2 run panda_controller teleop_manual
"""

import math
import sys
import termios
import tty

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Float64, String

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

STEP_DEFAULT = 0.01  # 1 cm
STEP_MIN = 0.002
STEP_MAX = 0.05

# Limites reales de cada articulacion (ver teleop_gui.py: _publish_joint
# para el porque de recortar en vez de envolver por modulo).
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


def read_key():
    """Lee un solo caracter del teclado sin esperar Intro (modo raw de
    termios), como hace teleop_twist_keyboard en el resto del ecosistema
    ROS2."""
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch


HELP_TEXT = """
Teleoperacion manual del Panda -- controles:
  w/s : +X / -X   a/d : +Y / -Y   q/e : +Z / -Z
  [ / ] : paso mas pequeno / mas grande (paso actual se imprime)
  c : CERRAR pinza   o : ABRIR pinza
  r/g/b : LED rojo/verde/azul   0 : LED apagado
  h : ir a HOME (posicion inicial, pinza AUN NO orientada hacia abajo)
  v : orientar la pinza hacia abajo aqui mismo (pulsalo tras 'h')
  p : imprimir posicion TCP actual
  x : salir
"""


class TeleopManual(Node):
    def __init__(self):
        super().__init__('teleop_manual')

        self.declare_parameter('robot_base_x', 0.5)
        self.declare_parameter('robot_base_y', -0.3)
        self.declare_parameter('robot_base_z', 0.74)
        self.declare_parameter('gripper_open_position', 0.04)
        self.declare_parameter('gripper_closed_position', 0.025)
        self.declare_parameter('joint_offsets_deg', [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -90.0])
        self.declare_parameter('max_wait_seconds', 45.0)

        self.base = np.array([
            self.get_parameter('robot_base_x').value,
            self.get_parameter('robot_base_y').value,
            self.get_parameter('robot_base_z').value,
        ])
        self.gripper_open = float(self.get_parameter('gripper_open_position').value)
        self.gripper_closed = float(self.get_parameter('gripper_closed_position').value)
        offsets_deg = list(self.get_parameter('joint_offsets_deg').value)
        self.offsets_rad = np.radians(offsets_deg)
        self.max_wait_seconds = float(self.get_parameter('max_wait_seconds').value)

        self.pub_joint = self.create_publisher(Float64MultiArray, '/joint_positions', 10)
        self.pub_gripper = self.create_publisher(Float64, '/gripper_position', 10)
        self.pub_led = self.create_publisher(String, '/comando_led', 10)

        self.home_raw = np.array(HOME_POSITIONS) - self.offsets_rad
        self.thetas = self.home_raw.copy()
        self.step = STEP_DEFAULT

    def wait_for_subscribers(self):
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

    def _publish_joint(self, raw_thetas):
        # NO se envuelve por modulo (ver teleop_gui.py: mismo comentario) --
        # puede producir un salto de ~2pi en el angulo COMANDADO cuando el
        # bruto cruza por poco el borde -pi/+pi, justo el caso de
        # panda_joint4 (limite real -3.1416, pegado a ese borde). Se recorta
        # a los limites reales en su lugar.
        commanded = raw_thetas + self.offsets_rad
        clamped = np.clip(commanded, JOINT_LIMITS_LO, JOINT_LIMITS_HI)
        self.pub_joint.publish(Float64MultiArray(data=clamped.tolist()))
        # Reconciliar self.thetas con lo REALMENTE mandado -- ver teleop_gui.py
        # _publish_joint para el bug real que esto evita (deriva silenciosa
        # acumulada tras muchos pasos si algun joint se recorta poco a poco).
        self.thetas = clamped - self.offsets_rad
        return clamped

    def _clamp_excess_deg(self, thetas):
        commanded = thetas + self.offsets_rad
        clamped = np.clip(commanded, JOINT_LIMITS_LO, JOINT_LIMITS_HI)
        return float(np.degrees(np.max(np.abs(commanded - clamped))))

    def tcp_world(self):
        return forward_kinematics(self.thetas)[:3, 3] + self.base

    def print_position(self):
        p = self.tcp_world()
        print(f'\r\nTCP mundo actual: x={p[0]:.4f}  y={p[1]:.4f}  z={p[2]:.4f}  '
              f'(paso={self.step*100:.1f} cm)\r')

    def move_delta(self, dx, dy, dz):
        # OJO: en teleoperacion NUNCA se debe caer a solve_with_restarts
        # (semillas aleatorias) -- puede devolver una solucion
        # matematicamente valida pero en una configuracion del brazo muy
        # distinta a la actual (codo/muneca en otra rama), lo que se ve como
        # el brazo "escapando" de golpe (bug real visto en Webots: paso a
        # paso normal y de repente un salto brusco). Solo se acepta la
        # solucion continua sembrada desde la pose actual; si no converge o
        # el salto articular es demasiado grande, se rechaza el movimiento
        # y se avisa, en vez de arriesgarse a un salto.
        target_world = self.tcp_world() + np.array([dx, dy, dz])
        target_base = target_world - self.base
        thetas, converged, iters, err = inverse_kinematics(target_base, GRASP_R, self.thetas)
        if not converged:
            print(f'\r\nAVISO: no puedo llegar ahi (IK no convergio, error={err:.4f}); '
                  'prueba un paso mas pequeno u otra direccion. No me muevo.\r')
            return
        max_jump_deg = float(np.degrees(np.max(np.abs(thetas - self.thetas))))
        if max_jump_deg > 25.0:
            print(f'\r\nAVISO: esa solucion de IK exige un salto de {max_jump_deg:.1f} deg '
                  'en alguna articulacion (probable cambio de configuracion del brazo, no un '
                  'movimiento continuo); prueba un paso mas pequeno. No me muevo.\r')
            return
        excess_deg = self._clamp_excess_deg(thetas)
        if excess_deg > 1.0:
            print(f'\r\nAVISO: esa posicion se sale {excess_deg:.1f} deg del limite real de '
                  'alguna articulacion; prueba otra direccion. No me muevo.\r')
            return
        self.thetas = thetas
        self._publish_joint(self.thetas)
        self.print_position()

    def go_home(self):
        # HOME_POSITIONS literal (la postura de reposo "segura" que usa todo
        # el paquete, con joint4 pre-doblado). OJO: su orientacion real de la
        # pinza NO coincide con GRASP_R (esta girada ~38 grados, verificado
        # por FK) -- eso es geometria real del brazo, no algo que se pueda
        # evitar recalculando. Por eso este comando manda la postura literal
        # tal cual (igual que el resto de nodos del paquete) y NO intenta
        # reorientar aqui: alinear la pinza hacia abajo es una accion
        # deliberada aparte, ver align_gripper_down() / tecla 'v'.
        self.thetas = self.home_raw.copy()
        self._publish_joint(self.thetas)
        print('\r\nMovido a HOME (postura de reposo; la pinza NO esta orientada '
              "hacia abajo todavia -- pulsa 'v' antes de usar wasd).\r")
        self.print_position()

    def align_gripper_down(self):
        # Reorienta la pinza a GRASP_R (mirando hacia abajo) SIN mover la
        # posicion XYZ actual del TCP. A diferencia de move_delta(), aqui NO
        # se aplica el filtro de salto articular grande: alinear desde una
        # orientacion muy distinta (p.ej. justo despues de 'h') es una
        # reconfiguracion grande pero DELIBERADA y esperada, no un efecto
        # colateral de un paso pequeno -- bloquearla con el mismo filtro deja
        # al usuario atascado (bug real visto: tras 'h', cualquier tecla
        # quedaba rechazada porque el primer movimiento SIEMPRE necesitaba
        # esta reorientacion).
        current_pos = forward_kinematics(self.thetas)[:3, 3]
        thetas, converged, iters, err = inverse_kinematics(current_pos, GRASP_R, self.thetas)
        if not converged:
            print(f'\r\nAVISO: no he podido orientar la pinza hacia abajo aqui '
                  f'(IK no convergio, error={err:.4f}). No me muevo.\r')
            return
        excess_deg = self._clamp_excess_deg(thetas)
        if excess_deg > 1.0:
            print(f'\r\nAVISO: esa orientacion se sale {excess_deg:.1f} deg del limite real de '
                  'alguna articulacion. No me muevo.\r')
            return
        jump_deg = float(np.degrees(np.max(np.abs(thetas - self.thetas))))
        self.thetas = thetas
        self._publish_joint(self.thetas)
        print(f'\r\nPinza orientada hacia abajo (giro articular maximo: {jump_deg:.1f} deg). '
              'Ya puedes usar wasd/qe con pasos pequenos.\r')
        self.print_position()

    def set_gripper(self, position, label):
        self.pub_gripper.publish(Float64(data=position))
        print(f'\r\nPinza -> {label} ({position:.4f} m)\r')

    def set_led(self, letter, label):
        self.pub_led.publish(String(data=letter))
        print(f'\r\nLED -> {label}\r')

    def run(self):
        print(HELP_TEXT)
        self.print_position()
        while rclpy.ok():
            ch = read_key()
            if ch == 'w':
                self.move_delta(self.step, 0.0, 0.0)
            elif ch == 's':
                self.move_delta(-self.step, 0.0, 0.0)
            elif ch == 'a':
                self.move_delta(0.0, self.step, 0.0)
            elif ch == 'd':
                self.move_delta(0.0, -self.step, 0.0)
            elif ch == 'q':
                self.move_delta(0.0, 0.0, self.step)
            elif ch == 'e':
                self.move_delta(0.0, 0.0, -self.step)
            elif ch == '[':
                self.step = max(STEP_MIN, self.step / 2.0)
                print(f'\r\nPaso -> {self.step*100:.2f} cm\r')
            elif ch == ']':
                self.step = min(STEP_MAX, self.step * 2.0)
                print(f'\r\nPaso -> {self.step*100:.2f} cm\r')
            elif ch == 'c':
                self.set_gripper(self.gripper_closed, 'CERRADA')
            elif ch == 'o':
                self.set_gripper(self.gripper_open, 'ABIERTA')
            elif ch == 'r':
                self.set_led('R', 'rojo')
            elif ch == 'g':
                self.set_led('G', 'verde')
            elif ch == 'b':
                self.set_led('B', 'azul')
            elif ch == '0':
                self.set_led('0', 'apagado')
            elif ch == 'h':
                self.go_home()
            elif ch == 'v':
                self.align_gripper_down()
            elif ch == 'p':
                self.print_position()
            elif ch == 'x' or ch == '\x03':  # x o Ctrl-C
                print('\r\nSaliendo.\r')
                break
            # cualquier otra tecla se ignora


def main(args=None):
    rclpy.init(args=args)
    node = TeleopManual()
    try:
        if not node.wait_for_subscribers():
            return
        if not sys.stdin.isatty():
            node.get_logger().error(
                'Esto necesita una terminal interactiva (no se puede lanzar con '
                "docker exec -d / en segundo plano). Usa 'docker exec -it ...'."
            )
            return
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
