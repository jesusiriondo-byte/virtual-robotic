#!/usr/bin/env python3
"""Robot "Sorter" de la celda industrial (sesion 2026-08-27): recoge un
cubo del punto donde la cinta lo deja (el tope fisico) y lo deposita en la
caja de su color, detectado por su PROPIA camara cenital
(DEF OVERHEAD_CAM_SORTER en worlds/panda_industrial_cell.wbt) -- ya no hace
falta pasar el color por parametro (version anterior, sesion 2026-08-27
temprano, probada sin camara para validar solo el alcance).

Layout en ELE (sesion 2026-08-27, a peticion del usuario): el Sorter esta
AL LADO de la cinta (base en x=0.0), no en linea recta al final de ella --
alcanza la cinta de lado (dx=+0.5) en vez de de frente. Las cajas quedan
hacia adelante del Sorter, lejos de la cinta en X -- cero ambiguedad de
camara entre "cubo en la cinta" y "cubo ya clasificado" (bug real de la
version anterior, con todo en la misma X).

Base GIRADA 26.6 grados hacia el punto de recogida (sesion 2026-09-08,
peticion real del usuario: "podriamos mover la base del robot 45 grados
que mire directamente al cubo"). La cinematica antes solo admitia
traslacion de la base (ver base_yaw en PandaIkpyKinematics, sesion
2026-09-08) -- ya soporta rotacion, con 0.0 de base_yaw en TODOS los demas
sitios (Loader incluido) para no cambiarles nada. Validado en mundos
aislados (panda_sorter_base_rotation_test.wbt, 3/3 cubos clasificados dos
veces) y en una copia REAL de panda_industrial_cell.wbt con la cinta y los
carriles fisicos de verdad (panda_industrial_cell_base_rotation_test.wbt,
sin choques contra BELT_RAIL_L/R_RECTO) antes de aplicarlo aqui. El angulo
real es atan2(dy,dx) del punto de recogida respecto a la base (dx=+0.50,
dy=+0.25) = 0.4636 rad = 26.6 grados, NO 45 (45 apuntaria mas alla del
punto real). Ver SORTER_BASE_YAW. rotate_before_descend esta en True (ver su
historial completo en __init__: se reactivo el 2026-09-09 con la causa real de
las roturas de pinza ya identificada -- geometria de carriles y pared, no el
giro previo).

Necesita 'ros2 launch panda_controller robot_launch_industrial_cell.py' ya
corriendo (incluye la camara del Sorter), y lanzarse en el namespace
'sorter' CON la base real del Sorter (bug real, sesion 2026-08-27: sin
esto usa por defecto la base del Loader y todo el calculo sale mal, aunque
a veces "funcione" por casualidad si algo real coincide con el punto
desplazado):

    ros2 run panda_controller sorter_demo --ros-args -r __ns:=/sorter \\
        -p robot_base_x:=0.0 -p robot_base_y:=1.00 -p robot_base_z:=0.74
"""

import json
import urllib.error
import urllib.request

import numpy as np
import rclpy
from rclpy.qos import QoSDurabilityPolicy, QoSProfile
from std_msgs.msg import String, Bool

# QoS "latched" para el color objetivo del lote activo (sesion 2026-09-01):
# el Sorter es un proceso persistente que puede arrancar DESPUES de que el
# panel ya haya publicado el color del lote -- TRANSIENT_LOCAL hace que un
# suscriptor tardio reciba igualmente el ultimo valor publicado, en vez de
# perderselo por pura carrera de arranque (mismo problema que ya se evito
# con QoS por defecto en /production/nuevo_lote, pero ahi no importaba
# perderse el primer aviso porque el Sorter ya baila solo al arrancar).
LOTE_COLOR_QOS = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)

from panda_controller.cube_shuttle_demo import CubeShuttleDemo, YAW, CUBE_TABLE_Z, CUBE_VISION_Z
from panda_controller.overhead_vision import OverheadLocator, COLOR_NAMES
from panda_controller.panda_ikpy_kinematics import PandaIkpyKinematics, HOME_POSITIONS

# Base real del Sorter en el mundo (DEF PANDA_SORTER translation 0.0 1.00
# 0.74 -- alejada del Loader a peticion del usuario, sesion 2026-08-27) --
# se fuerza aqui en vez de fiarse de que quien lo lance recuerde pasar los
# ROS params robot_base_x/y/z (bug real, misma sesion).
SORTER_BASE = (0.0, 1.00, 0.74)
# Giro de la base en radianes, eje Z (sesion 2026-09-08) -- ver el HISTORIAL
# de base girada en el docstring de arriba. DEBE coincidir con el campo
# "rotation" de DEF PANDA_SORTER en worlds/panda_industrial_cell.wbt (si se
# cambia uno, cambiar el otro).
SORTER_BASE_YAW = 0.4636

# Punto de recogida: el tope fisico de la cinta, a dx=+0.50 dy=+0.25 de la
# base del Sorter -- verificado con el solver. dy=+0.25 en vez del +0.15
# original (sesion 2026-08-27, aviso real del usuario): el tope estaba
# demasiado pegado al punto de agarre y la pinza (girada 45 grados) no
# tenia hueco para cerrar sin chocar con la propia pared -- se alejo el
# tope 15cm mas para dar hueco real.
PICKUP_X = 0.5
PICKUP_Y = 1.25

# Caras internas de los carriles del tramo recto, en X de mundo. Se usan en
# _adjust_grasp_position_for_obstacles para que los dedos nunca caigan
# dentro de un carril (sesion 2026-09-09).
#
# OJO, aqui estaba el bug (medido en vivo el 2026-09-10): NO coinciden con
# la translation del nodo, como decia esta nota antes. El Extrusion de los
# carriles CRECE HACIA FUERA desde su translation, asi que la cara interior
# queda desplazada 0.015 (el primer valor del crossSection) hacia fuera:
#
#     cara_izquierda = translation_L - 0.015
#     cara_derecha   = translation_R + 0.015
#
# Con las translations en 0.45/0.55 el codigo creia tener un canal de 10cm
# cuando el real era de 13cm. Medido empujando un cubo con
# /warehouse/mover_cubo_manual y leyendo donde acababa de verdad: se queda
# quieto hasta x=0.535 y a partir de 0.540 la fisica lo devuelve siempre a
# ~0.535 (borde del cubo en 0.565); simetrico por el otro lado en 0.435.
# Consecuencia: el acotado "de seguridad" empujaba el agarre hasta 2,2cm
# fuera del cubo real y la pinza cerraba sobre su borde -- la propia
# proteccion causaba las dislocaciones.
#
# Si se vuelven a mover los carriles: aplicar el +-0.015, y comprobarlo con
# el barrido de cubo antes de darlo por bueno.
RAIL_X_L = 0.445   # translation 0.460 - 0.015
RAIL_X_R = 0.555   # translation 0.540 + 0.015
# Margen extra sobre el calculo justo, para ruido de fisica/camara.
RAIL_SAFETY_MARGIN = 0.004

# Cajas de clasificacion: dx=-0.15/0/+0.15, dy=+0.45 desde la base del
# Sorter (mismo patron ya probado en la caja del Loader), hacia ADELANTE
# del robot, lejos de la cinta en X.
BOXES = {
    'R': (0.0, 1.45),
    'G': (-0.15, 1.45),
    'B': (0.15, 1.45),
}

# Camara propia del Sorter (DEF OVERHEAD_CAM_SORTER, translation 0.25 1.30
# 2.27, a medio camino entre el punto de recogida y las cajas).
SORTER_CAM_TOPIC = '/overhead_camera_sorter/image_color'
SORTER_CAM_X, SORTER_CAM_Y, SORTER_CAM_Z = 0.25, 1.30, 2.27
# Recorte ESTRECHO a proposito, solo la zona del tope de la cinta --
# con el layout en ele, las cajas estan en otra X por completo
# (x<=0.15 vs x>=0.35 de la cinta), asi que ya no hace falta ajustar el
# recorte al milimetro para evitar el bug real de una version anterior
# (confundir un cubo ya clasificado con uno nuevo).
SORTER_TABLE_X_RANGE = (0.35, 0.65)
SORTER_TABLE_Y_RANGE = (0.95, 1.35)  # ampliado (sesion 2026-08-28): con 3
# cubos en fila tocandose, agarrar uno puede rozar/empujar al vecino unos
# cm hacia atras (confirmado real: el cubo azul, colocado en y=1.15, quedo
# en y=1.085 tras agarrar el verde) -- margen amplio para que un cubo
# empujado varios cm siga dentro del recorte de la camara.

# Antes en 5x1s: demasiado corto para trabajo CONCURRENTE con el Loader
# (bug real, sesion 2026-08-27) -- el Loader tarda ~20s en depositar un
# cubo mas el tiempo de viaje de la cinta hasta el tope, muy por encima de
# esos 5s, asi que un sorter_demo lanzado a la vez que el loader_demo se
# rendia siempre antes de que llegara nada. Con margen de sobra para un
# ciclo completo del Loader.
DETECT_RETRIES = 40
DETECT_RETRY_WAIT = 1.0

# Comprobacion de "quieto de verdad" (sesion 2026-08-28, aviso real del
# usuario): la camara puede ver el cubo mientras la cinta AUN lo esta
# empujando hacia la pared, no solo cuando ya ha llegado -- si el Sorter
# sale hacia esa posicion de inmediato, el cubo real sigue avanzando
# mientras el brazo baja (varios segundos) y el agarre llega tarde, a un
# punto donde ya no esta. Se compara la posicion en dos lecturas seguidas
# (con una pequeña espera entre medias) y solo se considera listo para
# agarrar si coinciden dentro de STILL_MOVING_TOLERANCE.
STILL_MOVING_CHECKS = 10
STILL_MOVING_WAIT = 0.5
STILL_MOVING_TOLERANCE = 0.01

# Postura de aparcado PROPIA del Sorter (sesion 2026-08-30) -- con la
# HOME_POSITIONS generica (misma que usa el Loader), el brazo del Sorter
# se queda plegado justo ENCIMA/DELANTE del punto de recogida mientras
# espera el siguiente cubo, tapando su PROPIA camara cenital (confirmado
# con una imagen real de /overhead_camera_sorter/image_color: la sombra
# del brazo cubre toda la zona de la cinta). Esto explica bloqueos reales
# vistos en pruebas largas -- el Sorter "no ve ningun cubo" aunque el
# cubo SI este ahi (confirmado con la posicion real de
# WarehouseSupervisor). Girar solo panda_joint1 (la base) aleja todo el
# brazo de esa zona sin tocar el resto de la cinematica de agarre (leg()
# sigue partiendo de HOME_POSITIONS de siempre, solo el aparcado final
# usa esta postura alternativa).
SORTER_PARK_POSITIONS = HOME_POSITIONS.copy()
SORTER_PARK_POSITIONS[0] += 1.4


class SorterDemo(CubeShuttleDemo):
    def __init__(self):
        super().__init__()
        # Forzar la base REAL del Sorter (ver SORTER_BASE arriba) en vez
        # de la que haya calculado CubeShuttleDemo.__init__ a partir de
        # los ROS params robot_base_x/y/z (que por defecto son los del
        # Loader) -- bug real corregido en esta sesion.
        self.kin = PandaIkpyKinematics(base=SORTER_BASE, base_yaw=SORTER_BASE_YAW)
        self.cur = self.kin.seed_from_real(self.real_theta)
        # Giro de agarre PROPIO del Sorter, DISTINTO al de depositar
        # (sesion 2026-08-27, confirmado con grasp_yaw_test.py + solver):
        # en el tope de la cinta los 3 cubos llegan en fila, tocandose
        # (la cinta los empuja contra BELT_END_STOP) -- cerrar la pinza
        # con el giro de siempre (YAW) choca contra el cubo vecino en vez
        # de agarrar el objetivo (por eso el ciclo completo "parecia" ir
        # bien -- sensor de dedos verificado -- pero los 3 cubos seguian
        # intactos, comprobado mirando Webots en directo). Girando 90
        # grados mas (YAW+pi/2) SI agarra bien (confirmado levantando el
        # cubo en el aire, separado de los otros dos), pero esa misma
        # orientacion NO converge en el punto de la caja (comprobado con
        # el solver) -- por eso el giro de deposito es distinto, y
        # lift_shift_place() gira la muneca durante la subida (lejos ya
        # del resto de cubos), no en el agarre ni en el deposito.
        # EXPERIMENTO (sesion 2026-08-28): tras varios falsos positivos
        # seguidos con yaw=135/grasp_z rebajado, se vuelve a los valores
        # POR DEFECTO de CubeShuttleDemo -- los mismos que usa loader_demo.py
        # sin ningun hook, probado en vivo sin fallos (vacia 3 cubos en la
        # cinta). En el mundo aislado actual solo hay UN cubo (no hay fila
        # tocandose, la razon original para yaw=135) y la pared ahora mide
        # solo 3cm (muy por debajo de HOVER_GRASP=0.87) -- ninguna de las
        # razones que motivaron desviarse de los valores por defecto sigue
        # aplicando aqui. Si esto agarra bien, confirma que el problema
        # estaba en estos ajustes especiales, no en la geometria de base.
        self.base_grasp_yaw = YAW
        self.current_grasp_yaw = self.base_grasp_yaw
        self.place_yaw = YAW
        # HISTORIAL (sesion 2026-09-03): un dia entero probando por que el
        # cubo se resbalaba de la pinza del Sorter al levantar. En su
        # momento, la UNICA entrega de verdad de esa sesion fue con
        # rotate_before_descend=False (giro y bajada a la vez) -- CADA vez
        # que se probo con True el cubo se caia, asi que se dejo en False.
        #
        # REACTIVADO (sesion 2026-09-04, peticion real del usuario mirando
        # Webots: "procura hacer los giros por encima del cubo y luego
        # bajar y cerrar, hay veces que mueves el cubo antes de cerrar").
        # La causa real de los resbalones de la sesion 2026-09-03 ya se
        # encontro y arreglo por separado (la cinta deslizaba sin parar
        # durante el agarre, ver _pause_conveyor / robotica_carril_rampa_
        # agarre_abierto.md en memoria) -- rotate_before_descend nunca fue
        # la causa, solo estaba activo a la vez que el problema real. Con
        # la cinta ya parandose de verdad, se reactiva para conseguir lo
        # que siempre se quiso: girar la muneca a la orientacion de agarre
        # ANTES de bajar (a HOVER_HIGH, altura segura), para que la bajada
        # final sea recta, sin girar, y no desplace el cubo por el barrido
        # lateral de la pinza.
        #
        # REVERTIDO OTRA VEZ (mismo dia, minutos despues): probado en vivo
        # con 6 cubos forzando el giro +90 de obstaculos a la vez, 0/6
        # clasificados, 15 avisos de agarre falso/ground truth -- el mismo
        # fallo que en 2026-09-03, pese a tener ya arreglada la cinta y el
        # canal ampliado. La causa de que rotate_before_descend=True rompa
        # el agarre sigue sin identificar (no era la cinta, no era el
        # canal) -- queda pendiente para otra sesion, con mas cuidado. NO
        # volver a poner True sin investigar la causa real primero.
        #
        # REVERTIDO QUINTA VEZ (mismo dia, poco despues): con el freno
        # JUMP_MAX_RAD puesto, 2 ciclos salieron limpios pero el 3-4 volvio
        # a romper la pinza (dedo a 0.627, rango real 0-0.04) -- el freno
        # NO lo evito, lo que descarta (o al menos no confirma) la
        # hipotesis de salto de espacio nulo entre pasos de rampa. Nueva
        # hipotesis, mas plausible: interaccion con OTRO cambio de la misma
        # sesion -- el canal de los carriles rectos se cerro 1cm por lado
        # HOY MISMO (ver mas abajo, "cerrado 1cm hacia el centro"), y
        # _adjust_grasp_yaw_for_obstacles pasa a forzar SIEMPRE el giro +90
        # (tambien hoy) -- esa combinacion abre los dedos EN ANCHO
        # cruzando el canal ya reestrechado, el mismo patron exacto del
        # atasco de dedo de 2026-09-04 (arreglado entonces ENSANCHANDO el
        # canal 1,5cm por lado). El fallo ocurre justo en 'aproximacion
        # final + giro real', la bajada recta final -- encaja. Antes de
        # reactivar otra vez: o revertir el cierre de carriles, o probar
        # con el giro condicional (no siempre 90) en vez del forzado.
        # REACTIVADO SEXTA VEZ (sesion 2026-09-08, orden directa del
        # usuario en vivo: "coloca las pinzas antes de bajar y cerrar").
        # Los intentos anteriores con True rompian la pinza -- pero
        # coincidian con el bug del giro LARGO (yaw final normalizado a
        # 90-180 grados por el +90 SIEMPRE, nunca al representante corto
        # -90..0, ver shortest_grasp_yaw() en cube_shuttle_demo.py). Con
        # el giro ya acotado a como maximo 90 grados en vez de hasta 180,
        # el barrido de la muneca en 'giro a orientacion de agarre' es la
        # mitad de grande en el peor caso -- si esto vuelve a romper la
        # pinza, ya NO se puede culpar al giro largo, seria una causa
        # distinta de verdad.
        # REVERTIDO SEXTA VEZ (sesion 2026-09-09): volvio a romper la pinza
        # en produccion real CON el giro ya acotado por shortest_grasp_yaw
        # ("de repente ha empujado para abajo contra la cinta y se ha
        # roto") -- se cumple el criterio escrito justo arriba, asi que la
        # hipotesis del giro largo queda DESCARTADA. Balance acumulado: 6
        # roturas de pinza con True, 0 con False. Se queda en False hasta
        # que haya una causa raiz identificada de verdad, no otra
        # hipotesis: no reactivar solo porque el giro "se vea mejor".
        # REACTIVADO SEPTIMA VEZ (sesion 2026-09-09, tarde) -- esta vez con
        # la causa de las roturas YA identificada y corregida, no como
        # hipotesis. Las 6 roturas anteriores con True eran siempre lo
        # mismo, medido con el sensor: UN dedo forzado fuera de su rango
        # fisico (-0.0149, -0.062) mientras el otro seguia normal, es decir
        # un dedo trabado contra algo rigido. Ese "algo" eran los carriles
        # (la pinza abierta a 8cm cruzando un canal de 8cm) y la pared
        # trasera de la caja. Hoy: canal devuelto a 10cm, apertura de bajada
        # reducida a 6.6cm (self.approach_open) y pared eliminada -- ya no
        # hay contra que trabarse. Se reactiva porque es justo lo que pide
        # el usuario ("sorter podria hacer un cierre mas limpio, sigue
        # moviendo un poco el cubo cuando baja antes de cerrar"): girar la
        # muneca arriba, a altura segura, y bajar RECTO, sin el barrido
        # lateral que descoloca el cubo antes de cerrar.
        # Si vuelve a romper la pinza, mirar el valor de los dedos: si es
        # otra vez uno fuera de rango, queda un obstaculo por encontrar; si
        # no, es un problema distinto del de siempre.
        self.rotate_before_descend = True
        # Cierre algo mas fuerte SOLO para el Sorter (sesion 2026-09-04):
        # con la cinta ya parada durante el agarre (ver _pause_conveyor),
        # el resbalon durante la subida ha pasado de "siempre" a
        # "algunas veces" -- confirmado en vivo por el usuario. Encaja con
        # un agarre marginal (el cierre por defecto deja justo el margen
        # de GRASP_VERIFY_MARGIN=0.004 para distinguir "toco algo" de
        # "aire", no necesariamente el mas firme posible) mas que con una
        # causa nueva. Aislado en self.grasp_close (instancia, no la
        # constante global GRIPPER_CLOSED) para no repetir el bug real de
        # sesion 2026-09-03 (tocar el cierre global rompio al Loader). Si
        # esto no basta o rompe algo (p.ej. empuja el cubo en vez de
        # sujetarlo), revertir a GRIPPER_CLOSED (0.017, el valor de
        # siempre) y mirar otra via.
        self.grasp_close = 0.014
        # Apertura de bajada SOLO del Sorter (sesion 2026-09-09, causa raiz
        # medida con datos en vivo). El Sorter baja DENTRO del canal de
        # carriles y, con el giro +90 de _adjust_grasp_yaw_for_obstacles
        # (forzado siempre), los dedos se abren A LO ANCHO de ese canal:
        #   carriles en x=0.45 y x=0.55  -> canal de 10cm
        #   pinza al maximo (2 x 0.04)   -> 8cm de vano
        #   => solo 1cm de holgura por lado, con el cubo PERFECTAMENTE
        #      centrado en x=0.50.
        # Pero los cubos no llegan centrados: medidos en un lote real,
        # entre x=0.499 y x=0.518 (hasta 1.8cm de desvio). Con el cubo en
        # 0.515 un dedo cae en 0.555, DENTRO del carril derecho -- y el
        # sensor lo capturo: dedo a -0.0149 (por debajo de su tope fisico)
        # 4ms despues de llegar a la pose, con la pinza aun abierta, sin
        # haber cerrado todavia. Por eso rompia de forma intermitente:
        # cuando el cubo llegaba centrado no pasaba nada.
        # Esos 2cm de apertura de mas no hacen falta -- el cubo mide 6cm.
        # Con 0.033 por dedo (6.6cm de vano) quedan 3mm por lado para
        # entrar sobre el cubo y la holgura contra el carril sube de 1.0cm
        # a 1.7cm, cubriendo el desvio real observado.
        #
        # SUBIDO a 0.036 (sesion 2026-09-10). Aquellos 0.033 se eligieron
        # con el canal REAL en 13cm, cuando habia que alejarse de los
        # carriles a toda costa. Con el canal ya en 9cm (ver RAIL_X_L/R) ese
        # recorte se puede devolver, y hacia falta: 6.6cm de vano sobre un
        # cubo de 6cm son 3mm por lado, y un cubo girado presenta
        # 6*(cos+sen) de ancho -- con solo 5.2 GRADOS de giro ya no cabe
        # entre los dedos. Fallo real capturado asi: dedos en
        # [-0.024, 0.048], los dos fuera de rango y en sentidos OPUESTOS
        # (la pinza forzada a abrirse), 4ms despues de la aproximacion
        # final, con la pinza aun abierta y sin haber cerrado -- el brazo
        # chocaba contra el cubo y lo empujaba en vez de envolverlo
        # ("hace algo extraño al cerrar y no pilla el cubo, moviendo los
        # otros"). La camara corrige el giro cuando lo ve, pero aqui
        # informo giro=0.0 y bajo como si estuviera recto.
        #   0.036 por dedo -> 7.2cm de vano
        #   giro tolerado: 5.2 grados -> 11.5 grados
        #   holgura contra el carril: 1.2cm -> 0.9cm por lado
        # Si hay que seguir subiendo, ojo: la ventana de agarre seguro se
        # estrecha a la vez (ver _adjust_grasp_position_for_obstacles), asi
        # que pasado ~0.038 el acotado empieza a descentrar la pinza sobre
        # el cubo mas de lo que el propio vano tolera.
        #
        # DE VUELTA a 0.040 (el maximo, GRIPPER_OPEN) el 2026-09-10, junto
        # con el canal ensanchado a 11cm. Con 15mm de margen por lado contra
        # el carril ya no hace falta recortar la apertura, y cada milimetro
        # de vano es margen sobre el cubo: hueco de 6mm -> 10mm, y giro
        # tolerado de 13 -> 24.5 grados.
        self.approach_open = 0.040
        # Altura de agarre un poco mas alta SOLO para el Sorter (sesion
        # 2026-09-04, sugerencia real del usuario tras ver que aprieta el
        # cubo con fuerza contra la cinta antes de cerrar la pinza,
        # lanzandolo al suelo -- sobre todo con el azul, que arrastra un
        # fallo repetido en el giro de agarre +90). Sube 0.5cm sobre el
        # HOVER_GRASP por defecto (self.grasp_z, ver CubeShuttleDemo.
        # __init__) para dar mas margen antes de tocar la cinta/el cubo.
        # Aislado en self.grasp_z (instancia), no toca HOVER_GRASP global
        # ni al Loader.
        self.grasp_z += 0.005
        # CubeShuttleDemo.__init__ ya crea self.locator apuntando a la
        # camara del LOADER (la que usa leg() para relocalizar justo antes
        # de agarrar) -- para el Sorter hay que SUSTITUIRLO por su propia
        # camara, si no leg() intenta relocalizar mirando la mesa del
        # Loader y nunca ve nada.
        # Cerrar antes la suscripcion del locator de la clase base (camara del
        # Loader): sin esto el Sorter seguia recibiendo esa camara para nada
        # durante toda la produccion (sesion 2026-09-11).
        self.destroy_subscription(self.locator.sub)
        self.locator = OverheadLocator(
            self, topic=SORTER_CAM_TOPIC,
            cam_x=SORTER_CAM_X, cam_y=SORTER_CAM_Y, cam_z=SORTER_CAM_Z,
            table_x_range=SORTER_TABLE_X_RANGE, table_y_range=SORTER_TABLE_Y_RANGE,
            # Sesion 2026-09-08, peticion real del usuario: "si sorter ve
            # cubos que intente ir siempre a por el mas pegado a la pared"
            # -- ver el comentario junto a prefer_closest_to_wall en
            # overhead_vision.py. Solo el Sorter (tiene pared/cola real);
            # el Loader no activa esto.
            prefer_closest_to_wall=True)
        # Almacen reciclador (sesion 2026-08-28): topic ABSOLUTO (no
        # namespaceado), un unico WarehouseSupervisor global escucha a
        # cualquier robot que entregue. Solo se publica tras un leg() que
        # ya paso la verificacion real por camara (ver lift_shift_place en
        # cube_shuttle_demo.py) -- un agarre falso nunca llega aqui.
        self.pub_delivered = self.create_publisher(String, '/warehouse/cube_delivered', 10)
        # Pausa de la cinta durante el agarre (sesion 2026-09-03, ver
        # CubeShuttleDemo._pause_conveyor): el WarehouseSupervisor escucha
        # este topic y pone/quita la velocidad de DEF BELT a 0 -- topic
        # ABSOLUTO (no namespaceado), un unico Supervisor global la
        # controla, igual que /warehouse/cube_delivered.
        self.pub_belt_pause = self.create_publisher(Bool, '/warehouse/belt_pause', 10)

        # Aviso de "lote nuevo" (sesion 2026-08-31, peticion del usuario:
        # "cuando hay un cambio de lote los dos robots se van a su posicion
        # inicial y bailan"). El Loader es un proceso NUEVO en cada lote
        # (le basta con bailar una vez al arrancar, ver loader_demo.py),
        # pero el Sorter es DELIBERADAMENTE persistente entre lotes (no se
        # relanza, para no perder el hilo de un cubo a medio camino) -- por
        # eso necesita enterarse por topic de cuando teleop_gui lanza un
        # lote nuevo mientras el sigue vivo. Se comprueba en un punto de
        # espera seguro del bucle principal (ver run()), nunca a mitad de
        # un agarre real.
        self._nuevo_lote_pendiente = False
        self.create_subscription(Bool, '/production/nuevo_lote', self._on_nuevo_lote, 10)

        # Producto objetivo del lote activo (sesion 2026-09-01, aviso real
        # del usuario: "el LED de producto ya dice que producto es, el
        # resto es produccion de piezas" -- un lote de un solo producto usa
        # los 3 cubos de la caja para no esperar al reciclado de uno solo
        # -- ver loader_demo.py, run()), asi que lo que se AVISA a
        # Taller_Administracion como fabricado tiene que ser el producto
        # del lote, no el color real del cubo que paso por la cinta. Vacio
        # = sin lote de un solo producto activo, cada cubo cuenta por su
        # color real (reparto mixto de pedidos pendientes). El deposito
        # FISICO en la caja de color y el aviso de reciclado
        # (/warehouse/cube_delivered) NO cambian -- siguen siendo el color
        # real, eso es pura mecanica de la simulacion, no el producto.
        self._lote_color_objetivo = ''
        # Cantidad objetivo del lote y pedido real al que atarlo (sesion
        # 2026-09-01, dos bugs reales vistos en vivo tras el mecanismo de
        # arriba: 1) el LED de producto se apagaba en cuanto el LOADER
        # terminaba de descargar en la cinta, NO cuando el Sorter de
        # verdad terminaba de clasificar/contar -- se veia apagado con
        # piezas del lote aun en camino. 2) "Lanzar este pedido" no ataba
        # la produccion a ESE pedido en concreto, asi que si el reparto
        # automatico estaba desactivado (interruptor real en
        # Taller_Administracion) el pedido se quedaba "pendiente" para
        # siempre aunque el stock ya tuviera las piezas -- el operario
        # tenia que acordarse de pulsar "Repartir stock" a mano. Con
        # cantidad+pedido_id, el propio Sorter sabe cuando el lote esta
        # REALMENTE terminado (cuenta las entregas de verdad, no las
        # descargas del Loader) y cada pieza se aplica directamente al
        # pedido pedido, sin depender del interruptor de reparto.
        self._lote_activo = False
        self._lote_cantidad_objetivo = None
        self._lote_entregadas = 0
        self._lote_pedido_id = None
        # "Lanzar todo este producto" (sesion 2026-09-01): varios pedidos
        # reales del MISMO producto sumados en un lote, sin un pedido_id
        # unico -- ver forzar_reparto en Taller_Administracion/app/main.py.
        self._lote_forzar_reparto = False
        # Topic del LED de producto del Loader (sesion 2026-09-01): el
        # Sorter publica aqui DIRECTAMENTE el apagado cuando cuenta que el
        # lote esta completo de verdad -- topic fijo, no parametrizado,
        # porque hoy solo el Loader tiene este LED fisico (ver
        # Rasberry_Pi_Pico_USB_Loader/main.py). loader_demo.py ya NO apaga
        # este LED al terminar su propia descarga -- solo lo enciende.
        self.pub_led_producto_loader = self.create_publisher(String, '/comando_led_producto', 10)
        self.create_subscription(
            String, '/production/lote_color_objetivo', self._on_lote_color_objetivo, LOTE_COLOR_QOS)

        # Integracion automatica con Taller_Administracion (sesion
        # 2026-08-29): mismo host 'taller_host' que ya usa teleop_gui.py
        # (resuelto por el extra_hosts de docker-compose.yml, ver ese
        # fichero) -- proyecto/contenedor totalmente aparte. Antes esto
        # solo pasaba a mano con el boton "+1 pieza" del panel; ahora el
        # propio Sorter avisa solo en cuanto de verdad entrega un cubo
        # (mismo punto donde ya publica en /warehouse/cube_delivered, tras
        # la verificacion real por camara -- un agarre falso nunca llega
        # aqui tampoco). 'taller_api_base' ya lo declara y guarda
        # CubeShuttleDemo.__init__ (sesion 2026-08-30, lo necesita tambien
        # para el diagnostico de agarre) -- no volver a declararlo aqui,
        # ROS2 no permite declarar el mismo parametro dos veces.

    def _pause_conveyor(self, paused):
        """Override de CubeShuttleDemo._pause_conveyor (no-op alli, el
        Loader no lo necesita): publica en /warehouse/belt_pause, que
        escucha el WarehouseSupervisor para poner DEF BELT a velocidad 0
        (o restaurarla) -- ver warehouse_supervisor_driver.py. Analisis
        sesion 2026-09-03: la cinta corre a 0.15 m/s SIEMPRE (no hay forma
        de pararla desde el propio ConveyorBelt, solo desde un Supervisor
        sobre su campo 'speed'), asi que hasta ahora CADA agarre del
        Sorter pasaba con la superficie de la cinta deslizando bajo el
        cubo -- unica diferencia estructural real frente al Loader (mesa
        fija), nunca antes eliminada como variable."""
        self.pub_belt_pause.publish(Bool(data=bool(paused)))

    def _adjust_grasp_yaw_for_obstacles(self, x, y):
        """Collision-aware grasp planning con la unica pared que hay en
        este punto de recogida (BELT_END_STOP), sin sensor nuevo -- solo
        con la geometria ya conocida del mundo (sesion 2026-08-27, tras
        confirmar en directo que un cubo a 23 grados seguia necesitando
        elegir bien la orientacion, no solo "sumar el giro detectado").

        Un cubo tiene 2 pares de caras opuestas validas para agarrar
        (girados 90 grados entre si) -- self.current_grasp_yaw ya trae el
        giro real del cubo (base_grasp_yaw + lo detectado por camara), pero
        de esas 2 orientaciones validas para ESE giro exacto, una cierra
        los dedos en la direccion PARALELA a la fila de cubos/pared (choca)
        y la otra en la PERPENDICULAR (libre). Verificado empiricamente
        (grasp_yaw_test.py, sesion 2026-08-27): con yaw=YAW (45 grados) el
        eje de cierre real queda paralelo a la fila; con yaw=YAW+90 (135
        grados) queda perpendicular. De ahi: eje_de_cierre = yaw + 45
        grados; si ese eje cae dentro de +-45 grados de la direccion de la
        fila/pared (90 grados en este mundo, cinta a lo largo de Y),
        choca -- se gira 90 grados mas (misma pareja de caras opuestas del
        cubo, valida igual) para caer en el eje libre.

        REACTIVADO en sesion 2026-08-28 (se habia desactivado para probar
        solo con un cubo suelto, sin vecinos -- confirmado que yaw=45 +
        grasp_z=HOVER_GRASP por defecto agarran bien en ese caso). Ahora
        con los 3 cubos tocandose de verdad en fila (worlds/panda_sorter_
        grasp_test.wbt) vuelve a hacer falta este ajuste.

        MARGEN AMPLIADO Y REVERTIDO (sesion 2026-09-03): se probo subir
        el margen de 45 a 55 grados pensando que el carril nuevo (mas
        grande) necesitaba mas casos corregidos con +90 grados. Resultado
        real en Webots: en un caso limite (eje de cierre justo a 45.0
        grados, antes NO se corregia) el margen ampliado SI disparo el
        giro +90 -- y ESE agarre girado (caras 3/5) es el que se resbalaba
        al levantar (verificacion ground truth: subia 2-3cm en vez de los
        5.2cm esperados, una y otra vez). El problema no era falta de
        margen, era que el giro +90 en si mismo da un agarre mas debil
        cerca de este carril -- revertido a 45 grados (el valor
        validado). El fallo real de "necesita mas giro para las caras 3/5"
        se debia al giro simultaneo con la bajada (ver approach_and_grasp
        en cube_shuttle_demo.py, ya corregido por separado), no a este
        margen.

        CAUSA RAIZ ENCONTRADA Y ARREGLADA (sesion 2026-09-04): el giro +90
        cierra la pinza CRUZANDO el canal, hacia los carriles rectos
        (belt_rail_l/r_recto, "palos" 3 y 4 -- ver Documentacion/
        cuatro_palos_embudo.html), en vez de a lo largo de el. Diagnostico
        forzando este giro SIEMPRE (umbral a 200 en vez de 45, temporal) y
        vigilando /sorter/gripper_state en directo: un cierre mostro un
        dedo casi sin moverse (0.038, apenas por debajo de GRIPPER_OPEN)
        mientras el otro cerraba casi del todo (0.015) -- un dedo
        FISICAMENTE bloqueado contra el carril, no un fallo de posicion.
        El hueco disponible entre los dos carriles dejaba solo ~1cm de
        margen real a cada lado tras restar el alcance de los dedos
        abiertos, insuficiente frente al ruido normal de vision (~1-2cm
        visto en los logs). Arreglado ampliando el hueco del canal 1,5cm
        a cada lado en el crossSection de ambos carriles rectos (ver
        panda_industrial_cell.wbt) -- verificado con 6/6 cubos clasificados
        sin ningun fallo, forzando este giro +90 en TODOS los intentos.

        FORZADO SIEMPRE, sin condicion (sesion 2026-09-08, peticion real
        del usuario tras confirmar en vivo que a veces el calculo por
        angulo NO corregia cuando debia -- "las pinzas se rompen cuando hay
        varios cubos e intenta pillar a 45 grados"): en vez de decidir por
        el umbral de 45 grados (que puede quedar en el borde y no disparar
        por poco), aplicar SIEMPRE el giro +90 -- ya validado 6/6 sin
        fallos cuando se forzo para el diagnostico de 2026-09-04, y evita
        por completo el caso borde. OJO: el canal se ha vuelto a cerrar
        1cm por lado hoy mismo (mas cerca del ancho que causo el atasco de
        dedo de 2026-09-04) -- vigilar /sorter/gripper_state por si
        reaparece un dedo muy por debajo del cierre pedido mientras el otro
        cierra del todo (la firma de aquel bug)."""
        theta = self.current_grasp_yaw
        closing_axis_deg = (np.degrees(theta) + 45.0) % 180.0
        dist_to_unsafe_axis = abs(closing_axis_deg - 90.0)  # 90=eje de la fila/pared (Y)
        # Siempre, sin condicion (el 'if True:' que habia era el resto de cuando
        # dependia del umbral de 45 grados).
        theta = theta + np.pi / 2.0
        self.get_logger().info(
            f'[colision] giro de agarre ajustado +90 grados para no cerrar paralelo '
            f'a la pared/fila (eje de cierre estaba a {dist_to_unsafe_axis:.1f} '
            'grados del eje peligroso).')
        return theta

    def _adjust_grasp_position_for_obstacles(self, x, y):
        """Retranqueo de 3cm HISTORICO (sesion 2026-08-27), DESACTIVADO en
        sesion 2026-08-28: se calibro cuando BELT_END_STOP media 10cm de
        alto (top en z=0.84, muy por encima del cubo). Verificado con la
        pestaña Position de Webots (sesion 2026-08-28): el cubo real esta
        en (0.5,1.27,0.77), a solo 1cm de lo que ya detectaba la camara
        (0.508,1.262) -- la vision estaba bien, el retranqueo de 3cm era lo
        que hacia bajar la pinza 3.8cm por detras del cubo real, causando
        un golpe en vez de un agarre limpio. Con la pared ya bajada a 3cm
        (top en z=0.77, la MISMA altura que el agarre, ya no 10cm por
        encima) el motivo original del retranqueo ya no aplica -- se deja
        el hook en su sitio (por si hiciera falta reintroducirlo) pero sin
        modificar la posicion.

        LO QUE SI HACE AHORA (sesion 2026-09-09): acotar la X del agarre para
        que los dedos NUNCA puedan caer dentro de un carril. Es la ultima
        pieza del problema que ha roto la pinza 8 veces -- ver el historial
        en self.approach_open y en rotate_before_descend. Con la apertura ya
        reducida a 6.6cm la tolerancia subio de +-1.0cm a +-1.7cm de
        descentrado, pero se capturo un cubo localizado en x=0.536 (3.6cm
        fuera): los dedos caian en 0.503 y 0.569, y el carril derecho esta
        en 0.55 -> dedo a 0.2837, pinza destrozada.

        Acotar es mejor que rechazar el intento: un cubo FISICAMENTE no
        puede estar centrado fuera de [0.48, 0.52] (mide 6cm y el canal
        entre carriles va de 0.45 a 0.55), asi que una lectura de 0.536 es
        error de camara o un cubo montado en el carril -- en los dos casos,
        cerrar mas hacia el centro del canal es lo correcto. Y si el cubo
        estuviera de verdad ahi, el agarre fallara limpio por verificacion
        en vez de rompiendo el robot."""
        x_min = RAIL_X_L + self.approach_open + RAIL_SAFETY_MARGIN
        x_max = RAIL_X_R - self.approach_open - RAIL_SAFETY_MARGIN
        x_seguro = min(max(x, x_min), x_max)
        if abs(x_seguro - x) > 1e-6:
            self.get_logger().warn(
                f'[colision] agarre en x={x:.4f} pondria un dedo dentro de un carril '
                f'(canal {RAIL_X_L}-{RAIL_X_R}, apertura {self.approach_open * 2 * 100:.1f}cm) '
                f'-- acotado a x={x_seguro:.4f}.')
        return x_seguro, y

    def _esperar_color(self):
        """Espera INDEFINIDAMENTE a que llegue un cubo (sesion 2026-08-28).
        Antes se rendia tras DETECT_RETRIES intentos y el nodo entero se
        paraba solo -- eso causo un bloqueo real en la prueba de 6 ciclos:
        un cubo se perdio por el camino, el Loader se quedo esperando su
        reciclado (que solo llega si el Sorter lo clasifica) y mientras
        tanto el Sorter se habia apagado solo por "falta de cubos", asi
        que ese reciclado nunca iba a llegar -- los dos programas seguian
        vivos pero el sistema entero estaba parado sin que nadie avisara.
        Ahora el Sorter nunca se apaga por esto: sigue esperando para
        siempre, pero avisa fuerte cada DETECT_RETRIES intentos (por si
        hay que rescatar un cubo perdido a mano, o el nuevo vigilante del
        almacen -- warehouse_supervisor_driver.py -- ya lo esta haciendo
        solo)."""
        intento = 0
        while True:
            if self._nuevo_lote_pendiente:
                # Punto seguro para el aviso de lote nuevo (sesion
                # 2026-08-31): aqui nunca hay un cubo a medio agarrar, asi
                # que aparcar y bailar no interfiere con nada en curso.
                self._nuevo_lote_pendiente = False
                self.park_at_home()
                self._dance()
            intento += 1
            color = self.locator.detect_color()
            if color is not None:
                return color
            if intento % DETECT_RETRIES == 0:
                self.get_logger().warn(
                    f'[sorter] sin ver NINGUN cubo desde hace {intento * DETECT_RETRY_WAIT:.0f}s. '
                    'Puede que se haya perdido uno fisicamente -- revisa Webots, o usa el '
                    'boton "+1 pieza" del panel manual mientras se recupera.')
            self.spin_for(DETECT_RETRY_WAIT)

    def _esperar_cubo_quieto(self, color):
        """Compara la posicion detectada en lecturas sucesivas hasta que
        coincida dos veces seguidas (dentro de STILL_MOVING_TOLERANCE) --
        confirma que el cubo ya toca la pared y no que la cinta lo sigue
        empujando. Si deja de verse a mitad de la comprobacion (color=None),
        se reinicia la comparacion, no se da por quieto con datos viejos."""
        anterior = None
        for intento in range(1, STILL_MOVING_CHECKS + 1):
            located = self.locator.locate_with_yaw(CUBE_VISION_Z, color=color)
            if located is None:
                anterior = None
                self.spin_for(STILL_MOVING_WAIT)
                continue
            x, y, _ = located
            if anterior is not None:
                dist = ((x - anterior[0]) ** 2 + (y - anterior[1]) ** 2) ** 0.5
                if dist < STILL_MOVING_TOLERANCE:
                    return True
                self.get_logger().info(
                    f'[quieto?] cubo {color} se ha movido {dist * 100:.1f}cm entre '
                    f'lecturas -- aun en la cinta, esperando (intento {intento}/{STILL_MOVING_CHECKS})...')
            anterior = (x, y)
            self.spin_for(STILL_MOVING_WAIT)
        return False

    def _notificar_taller(self, color, pedido_id=None, forzar_reparto=False):
        """POST a Taller_Administracion tras una entrega real -- si el
        otro proyecto no esta levantado o tarda, NO debe parar al Sorter
        (timeout corto, cualquier fallo se registra y se sigue). El
        boton manual '+1 pieza' del panel de teleop sigue ahi como via de
        respaldo si esto fallara.

        'pedido_id' (sesion 2026-09-01): cuando el lote viene de "Lanzar
        este pedido", la pieza se aplica DIRECTAMENTE a ese pedido en el
        backend, sin mirar el interruptor de reparto_automatico -- antes,
        con el reparto automatico desactivado, un pedido se quedaba
        "pendiente" para siempre aunque el stock ya tuviera piezas de
        sobra (bug real visto en vivo). 'forzar_reparto' (mismo dia,
        "Lanzar todo este producto": varios pedidos reales del MISMO
        producto sumados sin un pedido_id unico) hace lo mismo pero
        aplicando al pedido pendiente mas antiguo de ese color, tambien
        sin mirar el interruptor."""
        url = self.taller_api_base.rstrip('/') + '/taller/cubo_clasificado'
        cuerpo = {'color': color, 'forzar_reparto': forzar_reparto}
        if pedido_id is not None:
            cuerpo['pedido_id'] = pedido_id
        body = json.dumps(cuerpo).encode('utf-8')
        req = urllib.request.Request(
            url, data=body, method='POST',
            headers={'Content-Type': 'application/json'})
        try:
            with urllib.request.urlopen(req, timeout=1.5) as resp:
                info = json.loads(resp.read().decode('utf-8'))
            pedido = info.get('pedido')
            if pedido is not None:
                self.get_logger().info(
                    f'[taller] pedido #{pedido["id"]} actualizado '
                    f'({pedido["cantidad_completada"]}/{pedido["cantidad_pedida"]}, {pedido["estado"]}).')
            else:
                self.get_logger().info(
                    f'[taller] sin pedido pendiente para {color} (o reparto manual activo) -- '
                    f'guardado en stock ({info.get("stock_actual", "?")} unidades).')
        except urllib.error.HTTPError as e:
            detalle = e.read().decode('utf-8', errors='replace')
            self.get_logger().warn(f'[taller] error del servidor ({e.code}): {detalle}')
        except (urllib.error.URLError, TimeoutError, ValueError) as e:
            self.get_logger().warn(f'[taller] no se pudo avisar a Taller_Administracion: {e}')

    def _on_nuevo_lote(self, msg):
        self._nuevo_lote_pendiente = True

    def _on_lote_color_objetivo(self, msg):
        """Formato 'COLOR:CANTIDAD:PEDIDO_ID:FORZAR'. COLOR vacio = lote
        MIXTO (reparto de pedidos pendientes, los 3 colores a la vez) --
        cada cubo sigue contando por su color real, pero self._lote_activo
        SIGUE haciendo falta para saber cuando el lote entero ha
        terminado de verdad y apagar el LED de producto (bug real,
        sesion 2026-09-01: en un lote mixto nunca se avisaba de que
        habia terminado, asi que el LED se quedaba encendido con el
        ultimo color real que se hubiera visto -- verde, azul, lo que
        fuera). PEDIDO_ID vacio si el lote no esta atado a un pedido real
        concreto. FORZAR='1' cuando el lote suma varios pedidos reales del
        mismo producto ("Lanzar todo este producto") -- sin un pedido_id
        unico, pero cada pieza debe aplicarse igualmente al mas antiguo de
        esos pedidos, sin mirar el interruptor reparto_automatico. Mensaje
        vacio del todo = sin ningun lote activo."""
        crudo = msg.data.strip()
        self._lote_entregadas = 0
        if not crudo:
            self._lote_activo = False
            self._lote_color_objetivo = ''
            self._lote_cantidad_objetivo = None
            self._lote_pedido_id = None
            self._lote_forzar_reparto = False
            self.get_logger().info('[lote] sin lote activo -- cada cubo cuenta por su color real.')
            return
        partes = crudo.split(':')
        color = partes[0].strip().upper()
        cantidad = int(partes[1]) if len(partes) > 1 and partes[1] else None
        pedido_id = int(partes[2]) if len(partes) > 2 and partes[2] else None
        forzar = len(partes) > 3 and partes[3] == '1'
        self._lote_activo = True
        self._lote_color_objetivo = color
        self._lote_cantidad_objetivo = cantidad
        self._lote_pedido_id = pedido_id
        self._lote_forzar_reparto = forzar
        if color:
            self.get_logger().info(
                f'[lote] producto objetivo activo: toda entrega cuenta como {color} '
                f'(objetivo {cantidad} unidad(es)'
                + (f', pedido #{pedido_id}' if pedido_id else '') + '), sea del color real que sea.')
        else:
            self.get_logger().info(
                f'[lote] reparto mixto activo (objetivo {cantidad} pieza(s) en total) -- '
                'cada cubo sigue contando por su color real, LED de producto se apaga al terminar.')

    def run(self):
        """Bucle continuo (sesion 2026-08-28, a peticion del usuario: 'que
        el Sorter en cuanto vea cubos vaya a por ellos', para trabajar de
        verdad junto al Loader sin tener que relanzar este programa a mano
        cada vez). Ya NO se para solo por falta de cubos -- _esperar_color
        espera indefinidamente (ver su docstring, arreglo del bloqueo real
        de la prueba de 6 ciclos). Solo termina si un ciclo de agarre falla
        de verdad, agotando sus propios reintentos internos."""
        # Aviso visual de "lote nuevo" al arrancar (sesion 2026-08-31) --
        # para los lotes SIGUIENTES mientras este proceso sigue vivo, ver
        # el aviso por topic dentro de _esperar_color().
        self.park_at_home()
        self._dance()
        clasificados = 0
        while True:
            color = self._esperar_color()
            name = COLOR_NAMES.get(color, color)
            if not self._esperar_cubo_quieto(color):
                self.get_logger().warn(
                    f'[{name}] no se ha quedado quieto tras {STILL_MOVING_CHECKS} '
                    'comprobaciones -- reintento el ciclo entero por si sigue en la cinta.')
                continue
            dest = BOXES.get(color, BOXES['R'])
            self.get_logger().info(
                f'--- Cubo {name} detectado en la cinta -> caja {name} {dest} ---')
            if not self.leg((PICKUP_X, PICKUP_Y), dest, f'clasificar_{color}', color=color,
                             place_yaw=self.place_yaw):
                # Sesion 2026-08-30: antes esto tiraba abajo TODO el
                # Sorter (return False -> el proceso termina) por un solo
                # cubo dificil -- bloqueo real visto tras anadir la
                # tercera verificacion de agarre (mas estricta). Si el
                # cubo problematico sigue en el punto de recogida, el
                # bucle vuelve a intentarlo solo (_esperar_color lo
                # redetecta); si de verdad desaparecio, sigue esperando
                # al siguiente sin quedarse el proceso entero muerto.
                self.get_logger().error(
                    f'[{color}] fallo recogiendo/clasificando tras agotar reintentos -- '
                    f'{clasificados} clasificado(s) hasta ahora, sigo esperando (no me caigo).')
                # Bug real (sesion 2026-08-31): leg() al agotar reintentos deja
                # el brazo colgado donde estaba el ultimo intento fallido --
                # tipicamente justo encima del punto de recogida, a la altura
                # de "levantar" (15cm), NUNCA aparcado. Como la camara cenital
                # de este Sorter mira fija a ese mismo punto, el propio brazo
                # le tapaba la vista para siempre -- _esperar_color() no
                # volvia a detectar NADA nunca mas (confirmado en vivo: un
                # cubo verde que SI se vio 3 segundos despues del fallo dejo
                # de verse en cuanto se acerco a la zona, y ya no volvio a
                # detectarse ningun color mas, con 3 cubos reales esperando en
                # la cinta). Aparcar aqui, antes de volver a esperar, despeja
                # la camara igual que ya hace park_at_home() entre ciclos
                # normales.
                self.park_at_home()
                continue
            self.get_logger().info(f'CUBO CLASIFICADO en la caja {name}.')
            # /warehouse/cube_delivered SIEMPRE con el color REAL (lo usa
            # WarehouseSupervisor para reciclar ESE cubo fisico exacto de
            # vuelta a su sitio -- no tiene nada que ver con que producto
            # cuenta como fabricado).
            self.pub_delivered.publish(String(data=color))
            color_taller = self._lote_color_objetivo or color
            if color_taller != color:
                self.get_logger().info(
                    f'[lote] cubo {name} entregado, contado como producto {color_taller} '
                    '(color objetivo del lote activo).')
            self._notificar_taller(
                color_taller, pedido_id=self._lote_pedido_id, forzar_reparto=self._lote_forzar_reparto)
            # Cierre REAL del lote (sesion 2026-09-01): solo aqui, cuando
            # el Sorter ha contado de verdad tantas entregas como pedia el
            # lote, se apaga el LED de producto -- el Loader terminar de
            # descargar en la cinta NO significa que el lote este hecho
            # (el Sorter puede tardar bastante mas en clasificar las
            # ultimas piezas, visto en vivo: el LED se apagaba minuto y
            # medio antes de que la ultima pieza llegara a su caja).
            if self._lote_activo:
                self._lote_entregadas += 1
                if (self._lote_cantidad_objetivo is not None
                        and self._lote_entregadas >= self._lote_cantidad_objetivo):
                    self.pub_led_producto_loader.publish(String(data='0'))
                    etiqueta = self._lote_color_objetivo or 'reparto mixto'
                    self.get_logger().info(
                        f'[lote] {self._lote_entregadas}/{self._lote_cantidad_objetivo} '
                        f'unidad(es) de {etiqueta} completadas de verdad -- '
                        'LED de producto apagado, lote cerrado.')
                    self._lote_activo = False
                    self._lote_color_objetivo = ''
                    self._lote_cantidad_objetivo = None
                    self._lote_pedido_id = None
                    self._lote_forzar_reparto = False
                    self._lote_entregadas = 0
            self.spin_for(0.2)
            clasificados += 1

    def park_at_home(self):
        """Sobrescribe CubeShuttleDemo.park_at_home() (sesion 2026-08-30):
        el Sorter aparca en SORTER_PARK_POSITIONS (base girada respecto a
        HOME_POSITIONS) en vez de la postura generica, para no tapar su
        propia camara cenital mientras espera el siguiente cubo. Mismo
        codigo de rampa que la version base, solo cambia el destino."""
        if not self._wait_while_stopped('antes de aparcar (postura Sorter)'):
            return False
        n = 10
        start = self.real_theta.copy()
        for i in range(1, n + 1):
            if not self._wait_while_stopped('aparcado (postura Sorter)'):
                return False
            frac = i / n
            th = start + (SORTER_PARK_POSITIONS - start) * frac
            self._publish(th)
        self.cur = self.kin.seed_from_real(SORTER_PARK_POSITIONS)
        self.get_logger().info('aparcado (postura Sorter, camara despejada)')
        return True


def main(args=None):
    rclpy.init(args=args)
    node = SorterDemo()
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
