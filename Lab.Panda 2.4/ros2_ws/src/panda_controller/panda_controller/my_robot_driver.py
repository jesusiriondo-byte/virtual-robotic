import rclpy
from std_msgs.msg import Float64MultiArray, Float64

JOINT_NAMES = [
    'panda_joint1',
    'panda_joint2',
    'panda_joint3',
    'panda_joint4',
    'panda_joint5',
    'panda_joint6',
    'panda_joint7',
]

FINGER_JOINT_NAMES = [
    'panda_finger::right',
    'panda_finger::left',
]

# Sensores de posicion REAL de cada dedo (no lo que se pidio, lo que de
# verdad alcanzo el motor). Sirven para verificar el agarre sin fuerza real:
# si tras cerrar los dedos llegan casi hasta la posicion de cierre PEDIDA,
# es que no habia nada bloqueandolos (agarre en el aire); si se quedan muy
# por encima, es que chocaron con el cubo (agarre real). Ver cube_shuttle_demo.py.
FINGER_SENSOR_NAMES = [
    'panda_finger::right_sensor',
    'panda_finger::left_sensor',
]

# Postura de reposo segura para arrancar el brazo. OJO: NO se puede usar
# 0.0 para todos los joints -- el joint4 real del Panda tiene el codo
# "pre-doblado" de fabrica y su rango NO incluye el 0 (limite maximo en
# torno a -0.4 rad segun el propio Webots). Se descubrio por el aviso
# "too big requested position: 0 > -0.4" al arrancar. Se usa la misma
# postura "ready" que ya sirve de semilla heuristica al solver de IK en
# move_above_ball.py, que si respeta los limites reales de cada joint.
HOME_POSITIONS = [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]

# Velocidad (rad/s) que se fuerza explicitamente en cada motor del brazo.
# Misma leccion aprendida en el proyecto del UR5e: setPosition() por si solo
# NO mueve el motor si el campo "velocity" del PROTO esta a 0 o es muy bajo,
# asi que la fijamos nosotros con setVelocity().
#
# 1.0 -> 1.4 VALIDADO; 1.4 -> 2.0 PROBADO Y REVERTIDO (sesion 2026-09-09).
# El escalon de 1.4 (con step_sleep 0.15) se midio con un lote real: 11
# cubos, 0 dislocaciones, 0 agarres falsos, 0 frenos de salto IK, y el ritmo
# bajo de 31s a 25s por cubo. ESTE ES EL TECHO CONOCIDO BUENO.
# El siguiente escalon (2.0 con step_sleep 0.11) NO aguanto, aunque la
# cuenta de tiempos cuadraba: aparecieron fallos que a 1.4 no existian --
# un cubo se solto durante el TRASLADO y cayo sobre el marco de cabecera de
# la cinta (y=0.305 en vez de 0.70), otro salio despedido fuera del mundo
# (ground truth: -396cm) y la pinza volvio a dislocarse. La causa no es el
# desfase comandado/real (eso se cubrio con LIFT_SETTLE_S) sino la inercia:
# a esa velocidad el traslado lateral arranca con aceleracion suficiente
# para vencer un agarre que a 1.4 aguantaba. Si se vuelve a intentar, hay
# que atacar primero el agarre (fuerza/cierre), no los tiempos.
# VA EMPAREJADA con step_sleep en
# cube_shuttle_demo.py: las dos suben ~25-40% a la vez A PROPOSITO. El
# intento anterior de acelerar (2026-09-03, step_sleep 0.2 -> 0.1 a secas)
# hubo que revertirlo al dia siguiente porque bajaba SOLO el tiempo de
# espera sin tocar la velocidad real del motor -- el brazo fisico se
# quedaba atras (self.real_theta guarda lo COMANDADO, no una lectura de
# sensor) y la pinza llego a cerrarse con el brazo aun moviendose,
# lanzando un cubo. Cuenta concreta: el giro de muneca son 10 pasos de
# hasta 9 grados, y a 1.0 rad/s un paso de 9 grados tarda 0.157s -- mas
# que los 0.1s que se esperaban. Manteniendo la proporcion entre las dos
# constantes, el margen de asentamiento por paso se queda igual que ahora.
# Si se vuelve a tocar, TOCAR LAS DOS.
COMMANDED_VELOCITY = 1.4

# Velocidad (m/s) para los dedos de la pinza (motor lineal, carrera pequena
# de pocos centimetros, no hace falta que sea rapida). Bajada de 0.05 a 0.02
# (sesion 2026-08-28, a peticion del usuario: "suaviza el golpe de agarre")
# -- el Sorter a veces tira al suelo al cubo vecino en una fila al cerrar la
# pinza sobre el objetivo, con fuerza suficiente para sacarlo de la cinta por
# completo (confirmado en directo, ver warehouse_supervisor_driver.py: el
# rescate automatico ya lo recupera, pero mejor que no se caiga). Mismo
# principio que ya funciono para la bajada final (12->30 pasos): menos
# velocidad de contacto, no menos fuerza de agarre -- _gripper() ya espera
# 2.0s tras pedir el cierre, de sobra para un cierre 2.5x mas lento.
FINGER_VELOCITY = 0.02

# Fuerza maxima real de los dedos (sesion 2026-09-04, aviso real del
# usuario visto varias veces: "aprieta el cubo con fuerza y sale
# disparado", con distintos colores -- no un color concreto). El PROTO
# PandaHand.proto declara maxForce=100N POR DEDO (limite del hardware
# real, pensado para agarrar piezas industriales pesadas) -- un cubo de
# esta celda pesa ~50g. Si el cierre no queda perfectamente centrado
# (agarre asimetrico, ya visto en /gripper_state con diferencias de
# varios mm entre dedos), el motor de posicion sigue empujando hasta
# 100N por lado tratando de alcanzar el cierre pedido contra el cubo --
# de sobra para lanzarlo por los aires en el mismo instante en que el
# contacto se resuelve mal, incluso con la velocidad ya rebajada
# (FINGER_VELOCITY solo limita la velocidad de aproximacion, no la
# fuerza sostenida una vez en contacto). Rebajada aqui a algo mas que de
# sobra para sujetar un cubo tan ligero sin que se resbale, pero muy por
# debajo de lo que hace falta para lanzarlo. Afecta a los DOS robots por
# igual (constante del driver compartido, no hace falta aislar por
# robot -- una fuerza sensata para agarrar beneficia a cualquiera).
# PRIMER INTENTO (5.0N) DEMASIADO BAJO -- confirmado en vivo por el
# usuario: "en 90 grados no tiene fuerza" y "el cubo rojo lo ha soltado"
# -- justo el giro de agarre ya identificado como el mas debil (ver
# _adjust_grasp_yaw_for_obstacles en sorter_demo.py) dejo de poder
# sujetar el cubo al levantar con tan poca fuerza disponible. Subido a
# un punto intermedio -- sigue muy por debajo de los 100N del PROTO
# (de sobra para lanzar un cubo de 50g), pero por encima del umbral que
# resulto insuficiente para el agarre mas debil.
FINGER_FORCE = 20.0

# Los nombres de los joints del brazo (panda_jointN) coinciden con la
# convencion estandar del ecosistema Franka/MoveIt/URDF, confirmado en
# Webots. Los de los dedos NO: el PROTO real expone LinearMotors llamados
# "panda_finger::right" / "panda_finger::left" (no "panda_finger_jointN"
# como se habia asumido sin poder verificar contra el PROTO). Se corrigio
# tras verlo en el listado de devices que imprime este driver al arrancar
# -- mismo patron que resolvio el desfase de 180 grados del UR5e. Ese
# listado se deja tal cual (se imprime siempre) por si algun otro nombre
# cambia en una version distinta del PROTO.


class MyRobotDriver:
    def init(self, webots_node, properties):
        self.__robot = webots_node.robot

        self.__log_all_devices()

        self.__motors = []
        for name, home in zip(JOINT_NAMES, HOME_POSITIONS):
            motor = self.__robot.getDevice(name)
            if motor is None:
                print(f'[panda_driver] AVISO: no se encontro el device "{name}". '
                      'Revisa el listado de devices de arriba y corrige JOINT_NAMES.',
                      flush=True)
                self.__motors.append(None)
                continue
            motor.setPosition(home)
            try:
                motor.setVelocity(COMMANDED_VELOCITY)
            except Exception:
                pass
            self.__motors.append(motor)

        self.__fingers = []
        for name in FINGER_JOINT_NAMES:
            finger = self.__robot.getDevice(name)
            if finger is None:
                print(f'[panda_driver] AVISO: no se encontro el device "{name}". '
                      'Revisa el listado de devices de arriba y corrige FINGER_JOINT_NAMES.',
                      flush=True)
                self.__fingers.append(None)
                continue
            try:
                finger.setVelocity(FINGER_VELOCITY)
            except Exception:
                pass
            try:
                finger.setAvailableForce(FINGER_FORCE)
            except Exception:
                pass
            self.__fingers.append(finger)

        timestep = int(self.__robot.getBasicTimeStep())
        self.__finger_sensors = []
        for name in FINGER_SENSOR_NAMES:
            sensor = self.__robot.getDevice(name)
            if sensor is None:
                print(f'[panda_driver] AVISO: no se encontro el device "{name}". '
                      'Sin verificacion de agarre por posicion real.', flush=True)
                self.__finger_sensors.append(None)
                continue
            sensor.enable(timestep)
            self.__finger_sensors.append(sensor)

        # joint_positions/gripper_position/gripper_state SIN barra inicial a
        # proposito (sesion 2026-08-27): con barra son nombres ABSOLUTOS, y
        # un namespace de ROS (-r __ns:=/xxx, ver WebotsController(...,
        # namespace=...) en robot_launch.py) no los alcanza -- asi es como
        # un segundo Panda chocaria con el primero en los tres topics. En
        # relativo, cada instancia de este driver cae bajo el namespace con
        # el que se lance su proceso (root si no se pasa ninguno -> mismo
        # comportamiento de siempre para el robot unico).
        # El namespace de ROS con el que se lanza este proceso (-r __ns:=...
        # en robot_launch*.py) NO le llega a este plugin embebido: sys.argv
        # aqui dentro es [''] y 'properties' llega SIEMPRE vacio (comprobado
        # empiricamente, sesion 2026-08-27 -- probado tanto con sys.argv
        # como con un atributo custom en <plugin>, ninguno de los dos llega).
        # La unica fuente fiable de "quien soy" dentro del plugin es el
        # propio nombre del robot en Webots (self.__robot.getName(), el
        # mismo "name" del DEF en el .wbt) -- eso SI esta disponible siempre,
        # sin depender de mecanismos de ROS que este plugin no puede ver.
        # "Panda" (el robot unico de siempre) se deja SIN namespace a
        # proposito, para no romper las demos de un solo robot que aun
        # publican en topics absolutos (teleop_gui.py, pick_and_place.py,
        # etc.).
        robot_name = self.__robot.getName()
        namespace = None if robot_name == 'Panda' else robot_name.lower()
        rclpy.init(args=None)
        self.__node = rclpy.create_node('panda_driver', namespace=namespace)
        self.__node.create_subscription(
            Float64MultiArray, 'joint_positions', self.__cmd_callback, 10
        )
        self.__node.create_subscription(
            Float64, 'gripper_position', self.__gripper_callback, 10
        )
        self.__pub_gripper_state = self.__node.create_publisher(
            Float64MultiArray, 'gripper_state', 10
        )
        self.__node.get_logger().info(
            'Driver Panda iniciado, esperando joint_positions y gripper_position'
        )

        for name, motor in zip(JOINT_NAMES, self.__motors):
            if motor is None:
                continue
            try:
                self.__node.get_logger().info(
                    f'Limites {name}: min={motor.getMinPosition():.4f} '
                    f'max={motor.getMaxPosition():.4f} rad, '
                    f'velocidad forzada a {COMMANDED_VELOCITY} rad/s'
                )
            except Exception as exc:
                self.__node.get_logger().warn(f'No se pudieron leer limites de {name}: {exc}')

        for name, finger in zip(FINGER_JOINT_NAMES, self.__fingers):
            if finger is None:
                continue
            try:
                self.__node.get_logger().info(
                    f'Limites {name}: min={finger.getMinPosition():.4f} '
                    f'max={finger.getMaxPosition():.4f} m, '
                    f'velocidad forzada a {FINGER_VELOCITY} m/s, '
                    f'fuerza limitada a {FINGER_FORCE}N (PROTO permite hasta 100N)'
                )
            except Exception as exc:
                self.__node.get_logger().warn(f'No se pudieron leer limites de {name}: {exc}')

    def __log_all_devices(self):
        """Diagnostico: lista TODOS los devices que expone el robot en
        Webots, tal y como los ve realmente el PROTO, para poder comparar
        contra JOINT_NAMES/FINGER_JOINT_NAMES sin tener que adivinar."""
        try:
            n = self.__robot.getNumberOfDevices()
        except Exception as exc:
            print(f'[panda_driver] No se pudo enumerar devices: {exc}', flush=True)
            return
        print(f'[panda_driver] El robot expone {n} devices en Webots:', flush=True)
        for i in range(n):
            try:
                device = self.__robot.getDeviceByIndex(i)
                print(f'  [{i}] {device.getName()} (tipo Webots node type={device.getNodeType()})',
                      flush=True)
            except Exception as exc:
                print(f'  [{i}] <error leyendo device: {exc}>', flush=True)

    def __cmd_callback(self, msg):
        # DEBUG, no INFO (sesion 2026-09-11): 8 lineas por comando llenaban el
        # log de la celda (>70.000 lineas en media hora de produccion).
        self.__node.get_logger().debug(f'Comando recibido en /joint_positions: {list(msg.data)}')
        for motor, name, position in zip(self.__motors, JOINT_NAMES, msg.data):
            if motor is None:
                continue
            motor.setPosition(position)
            self.__node.get_logger().debug(f'  -> {name} setPosition({position:.4f})')

    def __gripper_callback(self, msg):
        # Mismo valor de apertura (metros) a los dos dedos, en espejo.
        position = float(msg.data)
        self.__node.get_logger().debug(f'Comando recibido en /gripper_position: {position:.4f} m')
        for finger, name in zip(self.__fingers, FINGER_JOINT_NAMES):
            if finger is None:
                continue
            finger.setPosition(position)
            self.__node.get_logger().debug(f'  -> {name} setPosition({position:.4f})')

    def step(self):
        rclpy.spin_once(self.__node, timeout_sec=0)
        values = []
        for sensor in self.__finger_sensors:
            values.append(sensor.getValue() if sensor is not None else float('nan'))
        self.__pub_gripper_state.publish(Float64MultiArray(data=values))
