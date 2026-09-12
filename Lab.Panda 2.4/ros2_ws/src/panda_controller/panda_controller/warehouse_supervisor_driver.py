"""Plugin de Webots (sesion 2026-08-28) para el robot sin cuerpo fisico
DEF WAREHOUSE_SUPERVISOR (worlds/panda_industrial_cell.wbt, supervisor TRUE).

Recicla los 3 cubos de siempre en vez de anadir mas objetos fisicos:
en cuanto llega un mensaje String a '/warehouse/cube_delivered' con el
color ('R'/'G'/'B'), teletransporta ESE cubo de vuelta al mismo punto de
la caja del Loader, listo para que loader_demo.py lo vuelva a coger como
si fuera materia prima nueva -- "reciclar un almacen fuera de la vista",
la opcion mas simple pedida por el usuario. cube_shuttle_demo.py publica
ahi SOLO tras la verificacion real por camara (nunca solo por el sensor
de dedos), asi que un agarre falso no dispara un reciclaje de mentira.

Mismo patron init()/step() que my_robot_driver.py -- 'webots_node.robot'
da acceso al robot de Webots por el protocolo extern; como este robot
tiene 'supervisor TRUE' en el .wbt, expone tambien los metodos de
Supervisor (getFromDef, resetPhysics, etc.), no solo los de Robot normal.

Ademas (sesion 2026-08-28, arreglo del bloqueo real visto en la prueba de
6 ciclos) vigila la posicion de los 3 cubos en cada step(): si uno cae por
debajo de FLOOR_Z de forma sostenida -- se ha caido de una mesa/cinta por
un empujon o un choque durante un agarre -- lo rescata solo de vuelta a la
caja, sin esperar a que nadie se de cuenta. Antes, un cubo perdido asi
dejaba a Loader y Sorter esperandose mutuamente para siempre sin que se
enterara nadie.

Y (sesion 2026-08-29, aviso real del usuario: "si una pieza no la pilla y
cree que ha dejado en su cesto, luego desaparece de la cinta y se carga
en Loader") verifica la posicion REAL del cubo antes de reciclarlo, no
solo se fia del aviso. La verificacion por camara que ya tiene
sorter_demo.py (¿sigue el cubo en el punto de recogida?) es fiable
cuando el cubo esta solo en la cinta, pero NO cubre el caso de que se
suelte a medio camino (ni en el origen ni en el destino) -- y probar a
verificar por camara que de verdad llego a SU caja resulto NO ser fiable
en la practica (confirmado en vivo: con varias piezas ya amontonadas en
una caja, la camara ni detecta el verde ni ubica bien el rojo). En vez de
eso, se usa la MISMA fuente de verdad que ya usa el rescate de caidos:
la posicion real de Webots. Si el cubo sigue cerca del punto de recogida
del Sorter cuando llega el aviso de entrega, la entrega es FALSA -- no se
recicla (para no hacerlo desaparecer de donde de verdad esta) y se avisa
fuerte en el log.
"""

import rclpy
from std_msgs.msg import Bool, Float64MultiArray, String

# Mismas posiciones que CRATE_CUBES en loader_demo.py (fila unica,
# y=0.16 -- sesion 2026-08-28, movida 1cm hacia la cinta desde 0.15 a
# peticion del usuario) -- si esa fila cambia alguna vez, cambiar tambien
# aqui.
CRATE_POSITIONS = {
    'R': (0.5, 0.16, 0.77),
    'G': (0.35, 0.16, 0.77),
    'B': (0.65, 0.16, 0.77),
}

CUBE_DEF_NAMES = {
    'R': 'CUBO_ROJO',
    'G': 'CUBO_VERDE',
    'B': 'CUBO_AZUL',
}

# Red de seguridad (sesion 2026-08-28, arreglo del bloqueo real visto en la
# prueba de 6 ciclos): cualquier cubo de verdad, en mesa/cinta/caja, esta
# muy por encima de esta Z -- si uno cae por debajo es que literalmente se
# ha caido de una superficie (empujon de un agarre fallido, choque con el
# vecino en fila, etc.). CHECK_PERIOD_S evita consultar la posicion en cada
# paso de fisica (no hace falta, y son 3 getField por robot). GRACE_S exige
# que la caida sea sostenida, no un rebote de un instante, antes de rescatar
# -- para no interferir con un agarre real en curso.
FLOOR_Z = 0.5
CHECK_PERIOD_S = 1.0
GRACE_S = 3.0

# Punto de recogida del Sorter (mismo PICKUP_X/PICKUP_Y que sorter_demo.py
# -- si ese punto cambia alguna vez, cambiar tambien aqui). Si el cubo
# SIGUE aqui cuando llega su aviso de "entregado", la entrega es FALSA
# (nunca se levanto de verdad, aunque el Sorter creyera que si).
SORTER_PICKUP_X, SORTER_PICKUP_Y = 0.5, 1.25
PICKUP_CHECK_RADIUS = 0.20

# Rescate de cubos perdidos/atascados (sesion 2026-09-08, a peticion real
# del usuario tras una prueba de 25 vueltas donde tuve que rescatar dos
# cubos A MANO por terminal, sin poder mirar Webots): el rescate de
# "caidos" de arriba (FLOOR_Z) NUNCA los pillaba porque los dos seguian a
# la altura normal de mesa/cinta (z~0.77) -- uno aparecio muy lejos de
# todo (0.107,-0.634), el otro se quedo parado a mitad de cinta
# (0.389,0.518) sin que ninguna camara lo viera. Dos comprobaciones
# nuevas, generosas a proposito (durante el transporte normal el cubo
# SIEMPRE esta dentro de alguna de estas zonas, nunca fuera):
#
# - CRATE_ZONE / BOXES_ZONE: zonas de REPOSO -- normal quedarse quieto
#   ahi esperando su turno (no se comprueba movimiento).
# - BELT_ZONE: zona de TRANSITO -- el cubo deberia estar avanzando (lo
#   lleva un robot o la propia cinta), nunca quieto mucho rato.
CRATE_ZONE = ((0.15, 0.85), (-0.15, 0.35))
BELT_ZONE = ((0.20, 0.80), (0.35, 1.40))
BOXES_ZONE = ((-0.35, 0.35), (1.30, 1.60))
LEGIT_ZONES = (CRATE_ZONE, BELT_ZONE, BOXES_ZONE)

GRACE_LOST_S = 30.0    # fuera de TODAS las zonas, sostenido -> perdido de verdad
# 90s, no 60s (sesion 2026-09-08, ojo antes de bajarlo): un cubo esperando
# su turno EN el punto de recogida, quieto de verdad, es normal si el
# Sorter esta ocupado con otro (un leg() con sus reintentos internos puede
# tardar 45-60s de sobra) -- demasiado corto aqui recogeria a mano un cubo
# que solo estaba esperando, no atascado.
GRACE_STUCK_S = 90.0   # DENTRO de BELT_ZONE pero casi sin moverse -> atascado
STUCK_MOVE_EPS = 0.02  # 2cm -- por debajo de esto cuenta como "sin moverse"


def _in_zone(x, y, zone):
    (x0, x1), (y0, y1) = zone
    return x0 <= x <= x1 and y0 <= y <= y1


class WarehouseSupervisorDriver:
    def init(self, webots_node, properties):
        self.__robot = webots_node.robot

        self.__cubes = {}
        for color, def_name in CUBE_DEF_NAMES.items():
            node = self.__robot.getFromDef(def_name)
            if node is None:
                print(f'[warehouse_supervisor] AVISO: no se encontro DEF {def_name} '
                      'en el mundo -- ese color no se podra reciclar.', flush=True)
                continue
            self.__cubes[color] = node

        # Pausa de la cinta durante el agarre (sesion 2026-09-03): DEF BELT
        # (ConveyorBelt) corre a velocidad fija desde que arranca Webots --
        # el PROTO no expone 'controller' ni forma de pararla en caliente
        # salvo por Supervisor sobre su campo 'speed' (ver comentario junto
        # a DEF BELT en el .wbt). Este Supervisor SI tiene ese acceso.
        # SorterDemo publica True justo antes de la bajada final de un
        # agarre y False justo despues de la primera subida (ver
        # CubeShuttleDemo._pause_conveyor / SorterDemo._pause_conveyor) --
        # el Loader nunca publica aqui, su mesa no tiene cinta.
        self.__belt = self.__robot.getFromDef('BELT')
        self.__belt_speed_field = None
        self.__belt_normal_speed = 0.15
        if self.__belt is not None:
            self.__belt_speed_field = self.__belt.getField('speed')
            self.__belt_normal_speed = self.__belt_speed_field.getSFFloat()
        else:
            print('[warehouse_supervisor] AVISO: no se encontro DEF BELT -- '
                  'la pausa de cinta durante el agarre no tendra efecto.', flush=True)

        rclpy.init(args=None)
        self.__node = rclpy.create_node('warehouse_supervisor')
        self.__node.create_subscription(
            String, '/warehouse/cube_delivered', self.__on_cube_delivered, 10
        )
        self.__node.create_subscription(
            Bool, '/warehouse/belt_pause', self.__on_belt_pause, 10
        )
        # Reubicar un cubo a mano (sesion 2026-09-04, peticion real del
        # usuario: parada de emergencia activa impedia mover los brazos, y
        # tenia un cubo verde mal colocado desde el principio que queria
        # poner el mismo en la cinta sin depender de ningun robot).
        # Formato 'COLOR:X:Y' (Z fijo, altura de la cinta) -- mismo patron
        # que __recycle, pero a una posicion cualquiera en vez de siempre
        # la caja del Loader.
        self.__node.create_subscription(
            String, '/warehouse/mover_cubo_manual', self.__on_mover_cubo_manual, 10
        )
        # Posicion REAL de los 3 cubos, sesion 2026-08-30 (a peticion del
        # usuario: "el tema de agarrar el cubo no esta fino... ver si se
        # puede usar mejor la vision" + el bug real de "agarra otro color
        # y cree que es el rojo"). Investigado: sensores de POSICION de la
        # pinza (lo unico que se usaba hasta ahora) no pueden distinguir un
        # agarre real de uno "de refilon" -- hace falta info de contacto/
        # posicion real del objeto, no solo de la pinza (confirmado
        # buscando articulos sobre deteccion de agarre: sensores de fuerza
        # o ground-truth, no solo posicion). Esta Supervisor YA tiene
        # acceso privilegiado a los 3 DEF de los cubos (lo usa para el
        # rescate de caidos) -- en vez de anadir sensores nuevos al
        # gripper (intentado en la comunidad de Webots sin exito claro,
        # ver notas de investigacion), se publica aqui la posicion REAL de
        # cada cubo a alta frecuencia (cada step(), no solo cada
        # CHECK_PERIOD_S) para que Loader/Sorter puedan comprobar de
        # verdad "¿subio el cubo QUE YO CREO QUE TENGO, lo que sea que
        # tenga en la pinza?" en vez de fiarse solo del sensor de dedos o
        # de si la camara ve o no ve algo cerca de un punto.
        self.__pub_positions = self.__node.create_publisher(
            Float64MultiArray, '/warehouse/cube_positions', 10)
        self.__node.get_logger().info(
            f'Almacen listo -- {len(self.__cubes)}/3 cubos localizados, '
            'esperando confirmaciones de entrega en /warehouse/cube_delivered. '
            f'Control de cinta: {"OK" if self.__belt_speed_field else "NO DISPONIBLE"} '
            f'(velocidad normal {self.__belt_normal_speed:.2f} m/s, '
            'escuchando /warehouse/belt_pause).'
        )

        self.__fallen_since = {}
        self.__last_check = 0.0
        # Estado del rescate de perdidos/atascados (ver LEGIT_ZONES arriba).
        self.__outside_since = {}
        self.__stuck_since = {}
        self.__stuck_ref_pos = {}

    def __on_mover_cubo_manual(self, msg):
        try:
            color, x, y = msg.data.strip().split(':')
            color = color.strip().upper()
            x, y = float(x), float(y)
        except ValueError:
            self.__node.get_logger().warn(
                f'[mover_cubo_manual] formato invalido "{msg.data}" -- se espera "COLOR:X:Y".')
            return
        if color not in self.__cubes:
            self.__node.get_logger().warn(
                f'[mover_cubo_manual] color "{color}" desconocido o sin cubo asociado.')
            return
        z = CRATE_POSITIONS['R'][2]  # misma altura de mesa/cinta que el resto de posiciones fijas
        node = self.__cubes[color]
        node.getField('translation').setSFVec3f([x, y, z])
        node.getField('rotation').setSFRotation([0.0, 0.0, 1.0, 0.0])
        node.resetPhysics()
        self.__node.get_logger().info(
            f'Cubo {color} movido a mano a ({x:.3f},{y:.3f}) via /warehouse/mover_cubo_manual.')

    def __on_belt_pause(self, msg):
        if self.__belt_speed_field is None:
            return
        new_speed = 0.0 if msg.data else self.__belt_normal_speed
        self.__belt_speed_field.setSFFloat(new_speed)

    def __on_cube_delivered(self, msg):
        color = msg.data.strip().upper()
        if color not in self.__cubes:
            self.__node.get_logger().warn(
                f'Color "{color}" desconocido o sin cubo asociado -- ignorado.')
            return
        node = self.__cubes[color]
        x, y, _ = node.getField('translation').getSFVec3f()
        dist = ((x - SORTER_PICKUP_X) ** 2 + (y - SORTER_PICKUP_Y) ** 2) ** 0.5
        if dist < PICKUP_CHECK_RADIUS:
            self.__node.get_logger().error(
                f'Cubo {color}: aviso de entrega recibido pero SIGUE junto al punto '
                f'de recogida del Sorter ({x:.3f},{y:.3f}), a {dist * 100:.1f}cm -- '
                'entrega FALSA, NO se recicla (revisar el Sorter: cree que lo dejo '
                'en su caja pero el cubo real nunca llego a moverse de ahi).')
            return
        self.__recycle(color, motivo='entrega confirmada por el Sorter')

    def __recycle(self, color, motivo):
        node = self.__cubes[color]
        pos = CRATE_POSITIONS[color]
        node.getField('translation').setSFVec3f(list(pos))
        node.getField('rotation').setSFRotation([0.0, 0.0, 1.0, 0.0])
        node.resetPhysics()
        self.__node.get_logger().info(
            f'Cubo {color} reciclado de vuelta a la caja del Loader en {pos} ({motivo}).')

    def __check_fallen_cubes(self, now):
        """Red de seguridad (ver comentario junto a FLOOR_Z): rescata solo
        un cubo cuya Z lleva GRACE_S segundos SEGUIDOS por debajo de
        FLOOR_Z. Si vuelve a verse en una posicion normal antes de cumplir
        la gracia, se olvida sin rescatar -- para no interferir con nada
        que este en curso."""
        for color, node in self.__cubes.items():
            z = node.getField('translation').getSFVec3f()[2]
            if z < FLOOR_Z:
                since = self.__fallen_since.get(color)
                if since is None:
                    self.__fallen_since[color] = now
                elif now - since >= GRACE_S:
                    self.__node.get_logger().warn(
                        f'Cubo {color} encontrado fuera de cualquier superficie '
                        f'(z={z:.3f}) durante {GRACE_S:.0f}s seguidos.')
                    self.__recycle(color, motivo='rescate automatico, cubo caido')
                    self.__fallen_since.pop(color, None)
            else:
                self.__fallen_since.pop(color, None)

    def __check_stray_cubes(self, now):
        """Ver comentario junto a LEGIT_ZONES. Dos casos independientes,
        ambos vistos de verdad en la sesion 2026-09-08 y ninguno cubierto
        por __check_fallen_cubes (z normal en los dos)."""
        for color, node in self.__cubes.items():
            x, y, z = node.getField('translation').getSFVec3f()
            if z < FLOOR_Z:
                # Ya lo gestiona __check_fallen_cubes -- no acumular estado
                # de perdido/atascado a la vez sobre el mismo cubo.
                self.__outside_since.pop(color, None)
                self.__stuck_since.pop(color, None)
                continue

            if not any(_in_zone(x, y, zone) for zone in LEGIT_ZONES):
                since = self.__outside_since.get(color)
                if since is None:
                    self.__outside_since[color] = now
                elif now - since >= GRACE_LOST_S:
                    self.__node.get_logger().warn(
                        f'Cubo {color} encontrado fuera de las zonas de trabajo '
                        f'conocidas ({x:.3f},{y:.3f}) durante {GRACE_LOST_S:.0f}s '
                        'seguidos.')
                    self.__recycle(color, motivo='rescate automatico, cubo perdido lejos de las zonas de trabajo')
                    self.__outside_since.pop(color, None)
                    self.__stuck_since.pop(color, None)
                continue
            self.__outside_since.pop(color, None)

            if not _in_zone(x, y, BELT_ZONE):
                # Zona de reposo (caja del Loader o cajas de clasificacion)
                # -- quedarse quieto ahi es normal, no comprobar atasco.
                self.__stuck_since.pop(color, None)
                continue
            ref = self.__stuck_ref_pos.get(color)
            if ref is None or ((x - ref[0]) ** 2 + (y - ref[1]) ** 2) ** 0.5 > STUCK_MOVE_EPS:
                self.__stuck_ref_pos[color] = (x, y)
                self.__stuck_since[color] = now
                continue
            if now - self.__stuck_since[color] >= GRACE_STUCK_S:
                self.__node.get_logger().warn(
                    f'Cubo {color} atascado en la cinta, practicamente sin '
                    f'moverse de ({x:.3f},{y:.3f}) durante {GRACE_STUCK_S:.0f}s '
                    'seguidos.')
                self.__recycle(color, motivo='rescate automatico, cubo atascado en la cinta')
                self.__stuck_since.pop(color, None)
                self.__stuck_ref_pos.pop(color, None)

    def __publish_positions(self):
        """Orden FIJO R,G,B, x,y,z cada uno -- 9 floats. Se publica cada
        step() (no throttled como el rescate de caidos): un agarre real
        dura un par de segundos, y a la frecuencia de paso de Webots
        (~30Hz) hace falta esa resolucion para pillar la subida real del
        cubo justo despues de 'levantar', no una muestra de hace 1s."""
        data = []
        for color in ('R', 'G', 'B'):
            node = self.__cubes.get(color)
            if node is None:
                data.extend([0.0, 0.0, 0.0])
                continue
            data.extend(node.getField('translation').getSFVec3f())
        self.__pub_positions.publish(Float64MultiArray(data=data))

    def step(self):
        rclpy.spin_once(self.__node, timeout_sec=0)
        self.__publish_positions()
        now = self.__robot.getTime()
        if now - self.__last_check >= CHECK_PERIOD_S:
            self.__last_check = now
            self.__check_fallen_cubes(now)
            self.__check_stray_cubes(now)
