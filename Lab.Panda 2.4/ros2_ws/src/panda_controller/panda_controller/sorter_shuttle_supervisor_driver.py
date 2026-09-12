"""Plugin de Webots para el robot sin cuerpo fisico
DEF SORTER_SHUTTLE_SUPERVISOR (worlds/panda_industrial_cell_shuttle.wbt y
worlds/panda_shuttle_test.wbt, supervisor TRUE).

"Bandeja/transbordador" del punto de recogida del Sorter (sesion
2026-09-06, a peticion del usuario): el cubo llega deslizando por la
cinta y se frena fisicamente contra BELT_END_STOP, pero la pose final
tras deslizar/frenar varia un poco cada vez (friccion, fisica) -- a
diferencia del Loader, que coge de una mesa fija. Esa variacion de pose
es la que se sospecha contribuye a los fallos de agarre y al choque
contra la cinta sin resolver (ver memoria
robotica_carril_rampa_agarre_abierto). Este Supervisor detecta cuando un
cubo ha llegado y quedado quieto junto al punto de recogida y lo "encaja"
en una pose EXACTA (mismo mecanismo de teletransporte ya probado en
warehouse_supervisor_driver.py.__recycle: setSFVec3f + setSFRotation +
resetPhysics) -- PICKUP_X/PICKUP_Y de sorter_demo.py NO cambian (mismo
punto ya validado con el solver), solo se fija la pose exacta del cubo
ahi.

Lee la posicion real de los 3 cubos directamente via Supervisor
(getFromDef), igual que warehouse_supervisor_driver.py, sin depender de que
otro nodo publique posiciones. De ROS solo usa UNA suscripcion,
'/warehouse/belt_pause' (ver el BUG REAL de mas abajo).

Cadencia y tolerancia de "cubo quieto" copiadas tal cual de
sorter_demo.py (STILL_MOVING_WAIT=0.5, STILL_MOVING_TOLERANCE=0.01,
confirma con dos lecturas seguidas dentro de tolerancia) para no inventar
numeros nuevos sin verificar -- ese mecanismo ya esta probado en
produccion, solo cambia la fuente (aqui Supervisor, alli camara).

BUG REAL encontrado en la prueba de integracion completa (sesion
2026-09-06, con cinta+Loader reales): sin mas guardas, el encaje puede
disparar justo cuando el Sorter YA tiene la pinza bajando/cerrando sobre
el cubo (su propio chequeo de "quieto" por camara y el de aqui por
ground truth pueden confirmar estabilidad en momentos parecidos pero no
identicos) -- el teletransporte + resetPhysics() mueve entonces el cubo
de golpe justo debajo de una pinza que ya esta ahi, un choque de fisica
violento (visto en vivo: agarres siguientes tambien fallaban, brazo con
aspecto "roto"). Arreglo: escuchar '/warehouse/belt_pause' (el mismo
aviso que SorterDemo._pause_conveyor ya publica justo antes de la bajada
final, escuchado tambien por warehouse_supervisor_driver.py para la
cinta) -- mientras este a True, el brazo ya se ha comprometido con ESTE
cubo, y la bandeja no debe tocar nada hasta que se libere.
"""

import math

import rclpy
from std_msgs.msg import Bool

PICKUP_X, PICKUP_Y, DOCK_Z = 0.5, 1.25, 0.77

# Giro con el que se encaja el cubo, en vez de dejarlo cuadrado con los
# carriles (peticion del usuario, sesion 2026-09-10: "coloca en el amarillo
# pero con un pequeno angulo con la pared 5 en vez de dejar pegado").
#
# Cuadrado (0 grados) el cubo apoya una CARA ENTERA contra el carril y no
# deja por donde entrar; girado apoya solo una esquina, dejando un hueco en
# cuna para el dedo. Ademas el eje de cierre gira lo mismo, asi que los
# dedos quedan a 0.04*cos(angulo) del centro en X en vez de 0.04 -- unas
# decimas de mm mas de aire contra el carril, en la direccion buena.
#
# 10 grados es pequeno a proposito: el cubo de 6cm pasa a ocupar 6*(cos+sin)
# = 6.95cm de ancho, que en el canal de 11cm centrado en 0.500 sigue dejando
# 2cm libres a cada lado. Subirlo mucho se come ese margen (a 45 grados
# serian 8.49cm y solo 1.3cm por lado).
#
# El signo decide hacia que carril apunta la esquina; si queda del lado
# equivocado, cambiarlo por -10.0 y volver a medir.
DOCK_YAW_DEG = 10.0
DOCK_YAW_RAD = math.radians(DOCK_YAW_DEG)

CUBE_DEF_NAMES = {
    'R': 'CUBO_ROJO',
    'G': 'CUBO_VERDE',
    'B': 'CUBO_AZUL',
}

# A partir de aqui se considera "ha llegado" al punto de recogida. Con
# 0.035 (3.5cm) queda muy por debajo de los 6cm de separacion entre
# cubos en cola tocandose (ver CUBO_VERDE/CUBO_AZUL en
# panda_industrial_cell.wbt) -- solo debe enganchar al cubo que ya esta
# de verdad contra el tope, nunca al siguiente de la fila. Solo hay 3
# cubos reales en todo el mundo (uno por color, ver loader_demo.py /
# warehouse_supervisor_driver.py), asi que el "siguiente en la fila"
# siempre es de OTRO color -- si el radio fuera mayor podria encajarse
# encima del que ya esta docked, solapando dos cubos en el mismo punto.
CATCH_RADIUS = 0.035
# Mas alla de aqui se considera "el Sorter ya se lo ha llevado" -- libre
# para volver a encajar el siguiente cubo de ese color cuando llegue.
RELEASE_RADIUS = 0.15
# Misma cadencia/tolerancia que _esperar_cubo_quieto en sorter_demo.py.
CHECK_PERIOD_S = 0.5
STABLE_TOLERANCE = 0.01

# Que la esquina del cubo GIRADO no entre en el tope (sesion 2026-09-11,
# medido). El cubo llega y se para tocando BELT_END_STOP con la cara en
# y=1.300 (centro en 1.2700). Girado DOCK_YAW_DEG, su semiancho en Y pasa de
# 0.03 a 0.03*(cos+sin) = 0.0347, asi que encajarlo en su Y real metia la
# esquina 4.7mm DENTRO del tope y la fisica lo expulsaba de golpe
# (1.2700 -> 1.2653), justo en el punto donde agarra el Sorter. Se encaja como
# mucho en y_max (~6mm mas atras, sin interpenetracion) y la cinta lo arrima
# despues suavemente hasta el contacto. OJO: no confundir con el intento
# descartado de encajar en PICKUP_Y (16-20mm atras). Para volver al
# comportamiento anterior: quitar el min() de __dock.
STOP_FACE_Y = 1.300
CUBE_HALF = 0.03
DOCK_HOLGURA_TOPE = 0.001
DOCK_Y_MAX = STOP_FACE_Y - CUBE_HALF * (math.cos(DOCK_YAW_RAD) + math.sin(DOCK_YAW_RAD)) - DOCK_HOLGURA_TOPE


class SorterShuttleSupervisorDriver:
    def init(self, webots_node, properties):
        self.__robot = webots_node.robot

        self.__cubes = {}
        for color, def_name in CUBE_DEF_NAMES.items():
            node = self.__robot.getFromDef(def_name)
            if node is None:
                print(f'[sorter_shuttle_supervisor] AVISO: no se encontro DEF {def_name} '
                      'en el mundo -- ese color nunca se encajara en la bandeja.', flush=True)
                continue
            self.__cubes[color] = node

        self.__docked = set()
        self.__last_xy = {}
        self.__last_check = 0.0
        self.__paused = False

        rclpy.init(args=None)
        self.__node = rclpy.create_node('sorter_shuttle_supervisor')
        self.__node.create_subscription(
            Bool, '/warehouse/belt_pause', self.__on_belt_pause, 10)

        print(
            f'[sorter_shuttle_supervisor] Bandeja lista -- {len(self.__cubes)}/3 cubos '
            f'localizados, punto de recogida ({PICKUP_X},{PICKUP_Y}).', flush=True
        )

    def __on_belt_pause(self, msg):
        self.__paused = bool(msg.data)

    def __check_color(self, color, node):
        x, y, _z = node.getField('translation').getSFVec3f()
        dist = ((x - PICKUP_X) ** 2 + (y - PICKUP_Y) ** 2) ** 0.5

        if color in self.__docked:
            if dist > RELEASE_RADIUS:
                self.__docked.discard(color)
                self.__last_xy.pop(color, None)
            return

        if dist > CATCH_RADIUS:
            self.__last_xy.pop(color, None)
            return

        last = self.__last_xy.get(color)
        if last is not None and abs(x - last[0]) < STABLE_TOLERANCE and abs(y - last[1]) < STABLE_TOLERANCE:
            self.__dock(color, node)
            return
        self.__last_xy[color] = (x, y)

    def __dock(self, color, node):
        # Se respeta la Y REAL del cubo y solo se corrige X y giro (sesion
        # 2026-09-10, dos fallos medidos con la primera version, que encajaba
        # tambien en PICKUP_Y):
        #
        # 1. PICKUP_Y (1.25) es donde APUNTA el Sorter, no donde el cubo
        #    descansa de verdad tras frenar contra el tope (y~1.266). Encajarlo
        #    en 1.25 lo mandaba 1,6cm hacia ATRAS, la cinta lo volvia a empujar
        #    contra el tope y en ese segundo empujon se descentraba otra vez --
        #    justo lo que la bandeja venia a evitar. Medido: la vision lo veia
        #    luego en 0.5205/0.5157/0.5257, el agarre se acotaba a 0.511 y
        #    cerraba ~1cm descentrado. Dislocaciones de 1 a 9.
        # 2. Igualar la Y de todos los cubos al mismo valor es lo que permitia
        #    encajar DOS en el mismo punto (el riesgo que ya avisaba el
        #    comentario de CATCH_RADIUS), y con dos cubos superpuestos la
        #    camara puede leer el color del que no es -- se vio un azul
        #    depositado en la caja verde y contado como producto M.
        #
        # Dejando la Y intacta el cubo no se mueve de donde ya esta parado: la
        # bandeja solo lo desliza de lado al centro del canal y le fija el
        # giro, que es todo lo que hacia falta. Dos cubos en cola conservan su
        # separacion en Y aunque compartan X.
        _x, y, _z = node.getField('translation').getSFVec3f()
        y = min(y, DOCK_Y_MAX)   # ver DOCK_Y_MAX: la esquina girada no debe entrar en el tope
        node.getField('translation').setSFVec3f([PICKUP_X, y, DOCK_Z])
        node.getField('rotation').setSFRotation([0.0, 0.0, 1.0, DOCK_YAW_RAD])
        node.resetPhysics()
        self.__docked.add(color)
        self.__last_xy.pop(color, None)
        print(
            f'[sorter_shuttle_supervisor] Cubo {color} encajado en la bandeja '
            f'({PICKUP_X},{y:.4f},{DOCK_Z}) girado {DOCK_YAW_DEG} grados.', flush=True
        )

    def step(self):
        rclpy.spin_once(self.__node, timeout_sec=0)
        if self.__paused:
            # El Sorter ya se ha comprometido con un cubo (bajada final en
            # curso) -- no tocar nada hasta que libere la cinta.
            return
        now = self.__robot.getTime()
        if now - self.__last_check < CHECK_PERIOD_S:
            return
        self.__last_check = now
        for color, node in self.__cubes.items():
            self.__check_color(color, node)
