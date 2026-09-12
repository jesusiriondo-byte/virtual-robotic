#!/usr/bin/env python3
"""Robot "Loader" de la celda industrial (sesion 2026-08-27): vacia la caja
de la mesa 1 (3 cubos de pie, posiciones fijas y conocidas -- no hace falta
vision) dejando cada uno en el mismo punto de la cinta transportadora.

Reutiliza leg()/lift_shift_place() de CubeShuttleDemo sin cambios (mismo
patron que StackTowerDemo). El punto de destino en la cinta (0.5, 0.30) fue
verificado con panda_ikpy_kinematics.solve(..., check_convergence=True)
antes de fijarlo en worlds/panda_industrial_cell.wbt -- ver el propio mundo
para el porque de estos numeros.

Necesita 'ros2 launch panda_controller robot_launch_industrial_cell.py' ya
corriendo, y lanzarse en el namespace 'loader':

    ros2 run panda_controller loader_demo --ros-args -r __ns:=/loader -p use_vision:=false

LED propio (sesion 2026-08-30): este robot enciende la Pico del Loader
(sin wifi, por USB -- ver Rasberry_Pi_Pico_USB_Loader/main.py y
led_publisher_usb.py), NO la Pico W del Sorter. Hace falta el parametro
'led_topic' para que apunte al topic correcto (por defecto CubeShuttleDemo
usa '/comando_led', que es el de la Pico W):

    ros2 run panda_controller loader_demo --ros-args -r __ns:=/loader -p led_topic:=/comando_led_loader -p led_topic_producto:=/comando_led_producto

y tener corriendo aparte 'ros2 run panda_controller led_publisher_usb'.

LED "producto" (sesion 2026-08-31): segundo LED fisico de 4 patas en la
misma Pico del Loader, con colores continuos por PWM (ver
Rasberry_Pi_Pico_USB_Loader/main.py y la tabla COLOR_A_RGB en
led_publisher_usb.py) -- muestra el color del producto que se esta
fabricando ahora mismo, sin sustituir al LED de agarre de arriba (ese
sigue igual). Solo el Loader lo tiene cableado, por eso 'led_topic_producto'
esta vacio por defecto en CubeShuttleDemo (Sorter no lo activa).

Parametro 'only_color' (sesion 2026-08-27): para pruebas limpias con un
solo cubo en vez de vaciar la caja entera -- asi no se amontonan varios
cubos en la cinta a la vez, que liaba la deteccion de color del Sorter.

    ros2 run panda_controller loader_demo --ros-args -r __ns:=/loader -p only_color:=R

Parametro 'cycles' (sesion 2026-08-28, almacen reciclador): repite la
fila entera N veces. Antes de cada cubo (salvo el primerisimo), espera
con reintentos a que la camara lo vea de nuevo en la caja -- el
WarehouseSupervisor tarda unos segundos en reciclarlo tras la entrega
real del Sorter, no esta ahi al instante.

    ros2 run panda_controller loader_demo --ros-args -r __ns:=/loader -p cycles:=5
"""

import rclpy
from std_msgs.msg import Bool

from panda_controller.cube_shuttle_demo import CubeShuttleDemo, CUBE_TABLE_Z, CUBE_VISION_Z

RECYCLE_WAIT_RETRIES = 40
RECYCLE_WAIT_SECONDS = 1.0

# Caja de la mesa 1: misma fila que panda_un_cubo.wbt (dy=0.45 desde la
# base, probado). Vuelta a 3 cubos (sesion 2026-08-28): en vez de anadir
# mas cubos fisicos para "no quedarse sin material", se reciclan estos
# mismos 3 -- ver warehouse_supervisor_driver.py (WAREHOUSE_SUPERVISOR
# los teletransporta de vuelta aqui en cuanto el Sorter confirma una
# entrega real). run() puede repetir la fila varias veces (parametro
# 'cycles') precisamente para demostrar que nunca se agotan.
CRATE_CUBES = [
    ('R', 0.5, 0.16),
    ('G', 0.35, 0.16),
    ('B', 0.65, 0.16),
]

# Punto unico de deposito en la cinta. Movido de 0.30 a 0.40 (sesion
# 2026-08-30, aviso real del usuario: "el cubo verde a veces falla, los
# deja en el borde de la cinta") -- la cinta (DEF BELT ConveyorBelt,
# worlds/panda_industrial_cell.wbt) esta centrada en y=0.85 con 1.1m de
# largo tras su rotacion de 90 grados en Z, o sea que su borde mas
# cercano al Loader cae justo en y=0.30 (0.85 - 1.1/2).
#
# Volvio a pasar (sesion 2026-08-31, aviso real del usuario: "al dejar
# el cubo rojo no lo hace bien, se ha ido fuera de la cinta"): con
# y=0.40 solo quedaban 10cm de margen hasta ese borde -- confirmado en
# el log, el cubo rojo aparecio despues rotado -38.7 grados en
# (0.66,0.31), justo en la esquina de entrada de la cinta (borde
# cercano en Y a la vez que borde lateral en X, la cinta mide 0.3m de
# ancho centrada en x=0.5) -- se sale de la cinta tras el rebote de la
# caida, no de golpe. Subido a y=0.55 para casi triplicar el margen
# (25cm) -- el Loader ya demostro alcance de sobra hasta ahi (y mas
# alla: el respaldo de ground truth de leg() lo ha visto llegar incluso
# a y=1.26 cuando ha hecho falta ir a rescatar un cubo descolocado).
#
# Volvio a pasar UNA TERCERA vez (sesion 2026-08-31, mismo dia, aviso
# real del usuario: "sigo diciendo que lo deja en el borde") -- esta vez
# no era el borde de la cinta, sino el borde del carril/embudo nuevo
# (ver BELT_RAIL_* en panda_industrial_cell.wbt): la zona ancha del
# embudo empieza justo en y=0.55, el mismo punto exacto de este
# deposito, asi que el cubo caia justo en la boca de entrada del carril
# en vez de con margen dentro de el. Subido a y=0.70 -- sigue dentro de
# la zona ancha del embudo (que llega hasta y=1.00 antes de empezar a
# estrechar), con margen real a los dos lados esta vez.
BELT_DROP_X = 0.5
# 0.70 -> 0.28 (sesion 2026-09-09). El 0.70 de arriba NUNCA fue alcanzable:
# medido con el propio solver, desde la base del Loader (y=-0.3) el error
# entre lo pedido y lo que de verdad alcanza el brazo es de 0.1cm a y=0.30,
# 4.9cm a y=0.35 y 38.5cm a y=0.70. Como translate() no exige convergencia
# (a diferencia del agarre, ver strict_final en ramp()), el codigo cantaba
# "desplazar a destino fijo: OK (y=0.700)" mientras el brazo se quedaba a
# 38cm -- y los cubos aparecian en y~0.315, que es exactamente 0.70-0.385.
# Todo el rato se estuvo depositando en la cabecera de la cinta, medio
# encima de su marco, y la cinta se los llevaba: por eso "funcionaba".
# Ahora se pide un punto que el brazo alcanza de verdad, y la cinta se ha
# acercado 6cm (ver DEF BELT en el .wbt) para que ese punto caiga DENTRO
# de la banda (que ahora empieza en y=0.24) y no en el borde.
BELT_DROP_Y = 0.28

# Hueco libre en el punto de deposito (sesion 2026-09-04, aviso real del
# usuario tras arreglar el resbalon del Sorter con la pausa de cinta:
# "los cubos chocan en cola"): el Loader es un proceso independiente que
# no sabe nada de /warehouse/belt_pause (ver _pause_conveyor en
# cube_shuttle_demo.py/sorter_demo.py) -- mientras la cinta esta parada
# durante un agarre del Sorter, el ultimo cubo depositado se queda quieto
# justo en (o cerca de) BELT_DROP_X/Y, y el Loader seguia soltando el
# siguiente ENCIMA sin comprobar nada, produciendo el choque/amontonamiento
# real que se ve en Webots. En vez de acoplar el Loader a ese topic (los
# dos robots deben seguir pudiendose ejecutar sin saber nada el uno del
# otro, ver [[robotica_shared_code_por_robot]]), se usa la MISMA posicion
# real de los cubos (self.cube_pos_real, ya publicada por
# WarehouseSupervisor) para esperar a que el punto de deposito este libre
# de cualquier cubo antes de soltar el siguiente -- funciona igual si la
# cinta esta parada por el Sorter o simplemente va cargada de trabajo.
DROP_CLEAR_RADIUS = 0.10
DROP_CLEAR_RETRIES = 40
DROP_CLEAR_WAIT = 0.5


class LoaderDemo(CubeShuttleDemo):
    def __init__(self):
        super().__init__()
        self.declare_parameter('only_color', '')
        self.declare_parameter('cycles', 1)
        # Cierre mas firme, propio del Loader (sesion 2026-09-09). Se le
        # caian los cubos DURANTE el traslado caja->cinta, siempre en el
        # mismo sitio (y entre 0.279 y 0.313, justo al arrancar el
        # desplazamiento, donde la aceleracion lateral es maxima) y con
        # cuatro colores distintos: R en (0.492,0.295), G en (0.441,0.279),
        # G en (0.508,0.305), B en (0.492,0.305). No era nuevo -- el aviso
        # del usuario "loader deja el cubo gran parte en la cinta y otra
        # parte en el marco y luego se va por la cinta" era esto mismo: el
        # cubo caia al arranque de la cinta, la cinta se lo llevaba y el
        # Sorter lo clasificaba, asi que la produccion tapaba el fallo.
        # Solo se hizo visible al anadir la verificacion de transporte.
        # El Sorter, con el MISMO brazo y cubos, no los pierde -- y su
        # unica diferencia es que cierra a 0.014 (ajuste que se le puso
        # justo por resbalones, ver self.grasp_close en sorter_demo.py).
        # Se copia aqui como atributo de INSTANCIA, nunca tocando la
        # constante global GRIPPER_CLOSED: hacerlo global ya rompio una vez
        # al robot que no tocaba (ver [[robotica_shared_code_por_robot]]).
        self.grasp_close = 0.014
        # Suscrito a /warehouse/belt_pause (sesion 2026-09-08, peticion
        # directa del usuario: "si la cinta esta parada, para el Loader
        # tambien"). Revisa la decision original de arriba (DROP_CLEAR_
        # RADIUS, sesion 2026-09-04) de mantener los dos robots
        # desacoplados -- esa distancia ya evitaba soltar ENCIMA de un
        # cubo parado, pero el Loader seguia moviendose libremente
        # mientras el Sorter tenia la cinta parada para agarrar. Solo
        # LEE el aviso (nunca lo publica, eso sigue siendo cosa del
        # Sorter/WarehouseSupervisor) -- ver _esperar_hueco_en_cinta.
        self.belt_paused = False
        self.create_subscription(Bool, '/warehouse/belt_pause', self._on_belt_pause, 10)

    def _on_belt_pause(self, msg):
        self.belt_paused = bool(msg.data)

    def _esperar_reciclado(self, color, x, y):
        """Espera INDEFINIDAMENTE a que la camara vea de nuevo un cubo de
        este color en la caja -- solo hace falta a partir del 2o ciclo,
        cuando el cubo ya se entrego antes y el WarehouseSupervisor tarda
        unos segundos (recorrido Sorter + confirmacion) en reciclarlo de
        vuelta aqui. Con un solo cubo real por color, cualquier deteccion
        de ese color YA es el correcto -- no hace falta comprobar
        distancia como en el Sorter (ahi puede haber varios en fila).

        Antes se rendia tras RECYCLE_WAIT_RETRIES intentos y abortaba TODO
        el resto de la carga (ciclos y colores siguientes incluidos) --
        bloqueo real visto en la prueba de 6 ciclos: un cubo perdido en el
        Sorter dejaba a este color sin reciclar para siempre, y el Loader
        tiraba la toalla entera por un solo color. Ahora espera para
        siempre (con avisos fuertes periodicos) y deja que el nuevo
        vigilante del almacen (warehouse_supervisor_driver.py) o una
        intervencion manual lo resuelvan sin perder el resto del trabajo."""
        intento = 0
        while True:
            intento += 1
            located = self.locator.locate_with_yaw(CUBE_VISION_Z, color=color)
            if located is not None:
                return True
            if intento % RECYCLE_WAIT_RETRIES == 0:
                self.get_logger().warn(
                    f'[reciclado] el cubo {color} lleva {intento * RECYCLE_WAIT_SECONDS:.0f}s '
                    'sin volver a la caja. Puede que se haya perdido fisicamente en el '
                    'Sorter -- revisa Webots, o usa el boton "+1 pieza" del panel manual '
                    'mientras se recupera.')
            self.spin_for(RECYCLE_WAIT_SECONDS)

    def _esperar_hueco_en_cinta(self):
        """Bloquea (sin abandonar, ver docstring de _esperar_reciclado)
        mientras la cinta este pausada (self.belt_paused, ver
        _on_belt_pause) o mientras algun cubo real (de cualquier color)
        siga a menos de DROP_CLEAR_RADIUS del punto de deposito -- evita
        soltar un cubo nuevo encima de uno que la cinta aun no ha alejado
        (parada por el Sorter durante un agarre, o simplemente atascada
        por trabajo acumulado)."""
        intento = 0
        while True:
            if self.belt_paused:
                libre = False
            else:
                libre = True
                for pos in self.cube_pos_real.values():
                    dist = ((pos[0] - BELT_DROP_X) ** 2 + (pos[1] - BELT_DROP_Y) ** 2) ** 0.5
                    if dist < DROP_CLEAR_RADIUS:
                        libre = False
                        break
            if libre:
                return
            intento += 1
            if intento % DROP_CLEAR_RETRIES == 0:
                motivo = 'cinta parada por el Sorter' if self.belt_paused else \
                    f'punto de deposito ({BELT_DROP_X:.2f},{BELT_DROP_Y:.2f}) ocupado'
                self.get_logger().warn(
                    f'[cinta] {motivo} tras {intento * DROP_CLEAR_WAIT:.0f}s -- esperando '
                    'antes de soltar el siguiente cubo.')
            self.spin_for(DROP_CLEAR_WAIT)

    def run(self):
        only_color = str(self.get_parameter('only_color').value).strip().upper()
        cycles = int(self.get_parameter('cycles').value)
        # Bug real, sesion 2026-08-30: lanzar cycles>1 con use_vision:=false
        # (copiado sin querer de una prueba de un solo cubo) deja _esperar_
        # reciclado() sin hacer nada (esta bajo "if ... self.use_vision" mas
        # abajo) -- el Loader va entonces a ciegas a la posicion fija de la
        # caja SIN comprobar que el cubo reciclado ya este ahi ni corregir su
        # posicion exacta tras el teletransporte, lo que produjo agarres en
        # el aire "verificados" por el sensor (near-miss con contacto
        # parcial) y ademas confundio a la camara del Sorter con un cubo ya
        # clasificado. cycles>1 sin vision no es una demo valida -- se
        # rechaza aqui en vez de dejarlo fallar en silencio mas adelante.
        if cycles > 1 and not self.use_vision:
            self.get_logger().error(
                "cycles>1 requiere use_vision:=true (por defecto) -- sin vision, "
                "la espera de reciclado no puede confirmar que el cubo ya volvio a "
                "la caja ni corregir su posicion real. Relanza sin -p use_vision:=false."
            )
            return False
        # NO se valida contra CRATE_CUBES (sesion 2026-09-02, bug real:
        # "el led de producto no enciende el amarillo, enciende el color
        # de la bola que pilla") -- only_color YA NO selecciona que cubo
        # fisico coger (ver 'cubes = CRATE_CUBES' mas abajo, sesion
        # 2026-09-01: siempre se usan los 3), es SOLO la etiqueta del
        # producto que se esta fabricando (LED fijo + Taller_Administracion,
        # ver _lote_color_objetivo en sorter_demo.py) -- por eso tiene que
        # aceptar tambien colores "solo LED" (Y/M/C/W) que no tienen cubo
        # fisico en CRATE_CUBES. Solo se descarta basura evidente (vacio ya
        # esta cubierto por 'if only_color', esto es un guarda-tonterias).
        if only_color and not only_color.isalpha():
            self.get_logger().error(f"only_color='{only_color}' no es una letra de color valida.")
            return False
        # Aviso visual de "lote nuevo" (sesion 2026-08-31, peticion del
        # usuario): cada lote es un proceso loader_demo nuevo, asi que basta
        # con bailar una vez aqui, al principio, antes de empezar a
        # trabajar de verdad. Ver _dance() en cube_shuttle_demo.py.
        self.park_at_home()
        self._dance()
        # LED de producto encendido DESDE YA si el lote es de un solo color
        # (sesion 2026-09-01, aviso real del usuario -- "todo el lote es de
        # arandelas, el led de producto tiene que estar siempre encendido
        # para saber que estamos haciendo arandelas"): no esperar al primer
        # agarre, que se sepa desde el principio que producto se esta
        # fabricando. Con only_color vacio (lote mixto de los 3 colores) se
        # deja como antes -- se enciende con el primer agarre de cada color.
        if only_color:
            self._led_producto_fijo = True
            self._set_led_producto(only_color)
        # SIEMPRE los 3 cubos de la caja, aunque el lote sea de un solo
        # producto (sesion 2026-09-01, correccion real del usuario: "si
        # vamos a hacer cinco arandelas, tenemos que utilizar todos los
        # cubos hasta hacer cinco piezas" -- antes only_color filtraba la
        # caja a un solo cubo y cada pieza esperaba su reciclado entero
        # (~30-50s parado), cuando los otros 2 cubos estaban libres para
        # trabajar mientras tanto. El color del CUBO es solo material de
        # trabajo -- el producto que se cuenta como fabricado es el del
        # LED encendido arriba, no el color real de cada pieza (ver
        # sorter_demo.py: _lote_color_objetivo hace que Taller_Administracion
        # cuente CADA entrega de este lote como 'only_color', sea cual sea
        # el color real del cubo que la genero).
        cubes = CRATE_CUBES
        # Con only_color, 'cycles' pasa a ser la CANTIDAD DE PIEZAS a
        # entregar en total (lo que pide el operario), no vueltas a la fila
        # entera -- con los 3 cubos en juego, una "pasada" completa son 3
        # piezas, asi que el objetivo puede terminar a mitad de pasada.
        # Sin only_color (reparto mixto de pedidos pendientes), se queda
        # exactamente como antes: 'cycles' vueltas a la fila completa.
        objetivo_piezas = cycles if only_color else cycles * len(cubes)
        entregados = 0
        usos_por_color = {c: 0 for c, _, _ in cubes}
        while entregados < objetivo_piezas:
            for color, x, y in cubes:
                if entregados >= objetivo_piezas:
                    break
                usos_por_color[color] += 1
                if usos_por_color[color] > 1 and self.use_vision:
                    self._esperar_reciclado(color, x, y)
                self._esperar_hueco_en_cinta()
                self.get_logger().info(
                    f'--- [{entregados + 1}/{objetivo_piezas}] Descargando cubo {color} en '
                    f'({x:.2f},{y:.2f}) -> cinta ({BELT_DROP_X:.2f},{BELT_DROP_Y:.2f}) ---')
                if not self.leg((x, y), (BELT_DROP_X, BELT_DROP_Y),
                                f'descargar_{color}_{entregados + 1}', color=color):
                    # Sesion 2026-08-30: antes esto abortaba TODA la carga
                    # (ciclos y colores siguientes incluidos) por un solo
                    # cubo dificil -- bloqueo real visto tras anadir la
                    # tercera verificacion de agarre (mas estricta, detecta
                    # mas fallos reales de los que se veian antes). Ahora
                    # se salta SOLO esta unidad y se sigue con el resto,
                    # igual que ya hacia _esperar_reciclado con la espera
                    # indefinida -- un cubo problematico no debe tirar
                    # abajo el resto de la produccion.
                    self.get_logger().error(
                        f'[{color}] fallo agarrando/descargando tras agotar reintentos -- '
                        'salto esta unidad y sigo con el resto (no abandono toda la carga).')
                    # Mismo bug real que en sorter_demo.py (sesion 2026-08-31):
                    # leg() al agotar reintentos deja el brazo colgado donde
                    # estaba el ultimo intento fallido, sin aparcar -- puede
                    # tapar la propia camara cenital para los demas cubos de
                    # la caja. Aparcar antes de seguir con el resto.
                    self.park_at_home()
                    continue
                entregados += 1
        self.get_logger().info(f'CAJA VACIADA: {entregados} cubo(s) en la cinta.')
        # NO se apaga aqui el LED de producto (bug real, sesion
        # 2026-09-01): que el Loader termine de descargar en la cinta no
        # significa que el lote este terminado de verdad -- el Sorter
        # puede tardar bastante mas en clasificar las ultimas piezas.
        # Apagarlo aqui dejaba el LED apagado con piezas del lote aun en
        # camino. Ahora es el propio Sorter quien lo apaga, contando de
        # verdad las entregas confirmadas (ver sorter_demo.py, run()).
        return True


def main(args=None):
    rclpy.init(args=args)
    node = LoaderDemo()
    node.get_logger().info('Esperando a que el driver se suscriba...')
    if not node.wait_for_subscribers():
        node.get_logger().error(
            f'Nadie se ha suscrito tras {node.max_wait_seconds:.0f}s '
            "(revisa que 'ros2 launch panda_controller robot_launch_industrial_cell.py' este corriendo).")
    else:
        node.run()
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
