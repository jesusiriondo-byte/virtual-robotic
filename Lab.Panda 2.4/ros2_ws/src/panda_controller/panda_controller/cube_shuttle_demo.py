#!/usr/bin/env python3
"""Demo de pick-and-place repetido, usando la cinematica ikpy validada
(panda_ikpy_kinematics.py) en vez de la tabla DH manual de los demas nodos.

Un ciclo = coger el cubo, subirlo 'lift' metros, desplazarlo 'shift' metros
hacia +Y, soltarlo, subir a una altura segura ("home" en X/Y del propio
cubo), volver a cogerlo en su nueva posicion, devolverlo 'shift' metros
hacia -Y (a su sitio original) y volver a subir. Se repite 'repetitions'
veces para comprobar que el agarre (giro de 45 deg para pillar por caras
opuestas + cierre parcial a GRIPPER_CLOSED) es reproducible sin deriva.

Con use_vision=True (por defecto): antes de CADA agarre, relocaliza el cubo
con la wrist_camera (cube_vision.py) en vez de asumir que sigue exactamente
donde la pierna anterior creia haberlo dejado -- sin esto, en 10
repeticiones (20 traspasos) la deriva open-loop acumulada llevo al cubo
~7.5cm lejos de su sitio (sesion 2026-08-26). Con vision, cada agarre parte
de la posicion REAL medida, asi que la deriva no se acumula entre
repeticiones (solo queda el error de una sola pierna, <1cm).

Extendido (sesion 2026-08-26) a los 3 cubos de worlds/panda_un_cubo.wbt: por
defecto cicla rojo, verde y azul, localizando cada uno con la camara
cenital (overhead_vision.py, ve la mesa entera de un disparo, sin mover el
brazo) para fijar su propio punto A, y repite el ciclo ida/vuelta
'reps_per_color' veces (3 por defecto) sobre CADA cubo antes de pasar al
siguiente.

Necesita 'ros2 launch panda_controller robot_launch.py' ya corriendo.

    ros2 run panda_controller cube_shuttle_demo
    ros2 run panda_controller cube_shuttle_demo --ros-args -p reps_per_color:=5
    ros2 run panda_controller cube_shuttle_demo --ros-args -p colors:="['R']"
    ros2 run panda_controller cube_shuttle_demo --ros-args -p use_vision:=false
"""

import json
import time
import urllib.error
import urllib.request

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Float64, String, Bool

from panda_controller.panda_ikpy_kinematics import (
    PandaIkpyKinematics, GRASP_R, HOME_POSITIONS, GRIPPER_OPEN, GRIPPER_CLOSED, rz,
)
from panda_controller.overhead_vision import OverheadLocator, COLOR_NAMES

CUBE_TABLE_Z = 0.77          # centro del cubo apoyado en la mesa

# Plano al que la camara cenital proyecta lo que ve (sesion 2026-09-10,
# sesgo MEDIDO contra la posicion real). CUBE_TABLE_Z es el CENTRO del
# cubo, pero una camara mirando desde arriba no ve el centro: ve la CARA
# SUPERIOR, 3cm mas alta. Proyectar al plano equivocado estira las
# coordenadas hacia fuera del eje optico de forma proporcional a la
# distancia a ese eje -- error sistematico, siempre en el mismo sentido,
# no ruido.
#
# Medido con la camara del Sorter (esta en z=2.27, el punto de recogida a
# 25cm de su eje optico): la camara decia sistematicamente entre +6 y
# +12mm de mas, media 7.7mm. Y el margen de la pinza al bajar es de solo
# 6mm por lado (vano 7.2cm sobre un cubo de 6cm), asi que el sesgo se lo
# comia entero: el brazo bajaba al lado del cubo, lo EMPUJABA en vez de
# envolverlo, y lo cogia al segundo o tercer intento ("se posiciona bien
# pero no cierra la pinza, se va y vuelve y a la tercera lo coge").
# Capturado en un ciclo: el cubo paso de 0.4991 a 0.5156 tras el intento
# fallido, desplazado 1.6cm por el propio brazo.
#
# Reproyectando las 12 lecturas medidas al plano correcto: error medio
# 7.7mm -> 3.4mm (56% menos), que ya cabe de sobra en los 6mm de margen.
# Vale para los DOS robots: es geometria de la camara, no un ajuste de uno
# de ellos. Al Loader le pasa igual, solo que en la caja tiene sitio de
# sobra y por eso no se le notaba.
CUBE_VISION_Z = CUBE_TABLE_Z + 0.03

# Lado del cubo (m). Se usa para saber cuanto margen le queda a la pinza
# cuando baja abierta sobre el: (vano - CUBE_SIDE) / 2 por lado.
CUBE_SIDE = 0.06

# Margen de seguridad (m) que se le exige a ese hueco antes de BAJAR a por
# un cubo (sesion 2026-09-10, dislocacion capturada con datos completos).
# Si el punto donde la pinza puede cerrar de verdad -- ya pasado el acotado
# anticolision de los carriles -- se queda mas lejos del cubo REAL que el
# margen fisico que hay entre los dedos y el cubo, la pinza NO lo envuelve:
# baja contra su cara lateral, lo aplasta contra la cinta y un dedo se sale
# de su articulacion.
#
# Capturado en vivo: la camara dijo x=0.5077, el acotado lo llevo a 0.5050
# (no puede pasar de ahi sin meter un dedo en el carril) y el cubo REAL
# estaba en 0.5164 -- 1.14cm de desvio contra 6mm de margen. El registro
# del ground truth muestra al cubo hundiendose (z 0.7611 -> 0.7604,
# aplastado) y los dedos separandose (0.011 y 0.036) justo antes de la
# dislocacion. El cubo estaba pegado al carril derecho (su borde en 0.546,
# con la cara del carril en 0.545): en esa posicion es GEOMETRICAMENTE
# imposible agarrarlo con los dedos cruzando el canal, hagas lo que hagas.
#
# Lo correcto ahi es no bajar. El cubo se queda donde esta y, si no se
# recoloca solo, el WarehouseSupervisor lo rescata por atasco a los 90s --
# mucho mejor que romper la pinza y dejar la celda parada.
# Margen de seguridad sobre ese hueco: 1mm. El hueco util con la apertura
# actual es de 6mm por lado, asi que esto tolera hasta 5mm de desvio -- por
# encima del error residual de la camara ya corregida (3.4mm de media, ver
# CUBE_VISION_Z), asi que un agarre bueno pasa, y el caso capturado (1.14cm)
# se aborta sin llegar a tocar el cubo. Subirlo mucho aborta agarres validos;
# bajarlo a 0 deja pasar los que rompen la pinza.
GRASP_REACH_MARGIN = 0.001

# Segundos que se mantiene la parada AUTOMATICA por pinza dislocada antes de
# rearmarse sola (sesion 2026-09-10, fallo real: "el sorter tapa la camara,
# esta loco"). La parada de emergencia es una PAUSA que espera al rearme, y
# eso es lo correcto cuando la aprieta una persona: hay alguien para
# rearmar. Pero la dislocacion salta SIEMPRE en la pose de agarre, o sea con
# el brazo bajado justo encima del punto de recogida -- que es exactamente
# lo que mira su camara cenital. El brazo se quedaba ahi congelado, tapando
# su propio sensor, esperando un rearme que no iba a llegar: la celda entera
# muerta y sin forma de verse a si misma.
#
# Un robot que se protege tapando su propio sensor no esta protegido. Al
# rearmarse solo, el flujo normal sigue: el intento falla por verificacion,
# se aparca (que despeja la camara) y se vuelve a mirar. Y con la
# comprobacion de GRASP_REACH_MARGIN ya no vuelve a bajar a por un cubo que
# no puede coger, asi que no entra en bucle.
#
# OJO: esto rearma SOLO las paradas que ha disparado este mismo nodo por
# dislocacion. Una parada que venga de fuera (los botones fisicos, el panel)
# NO se auto-rearma nunca -- ahi si hay una persona detras.
AUTO_REARME_DISLOCACION_S = 4.0

# Cuantas dislocaciones SEGUIDAS (sin ningun agarre bueno de por medio) se
# aguantan antes de dejar de rearmarse solo (sesion 2026-09-10, efecto
# secundario real del auto-rearme de arriba). Cuando un dedo se sale de
# verdad NO vuelve solo a su sitio: se queda fuera de rango, asi que en
# cuanto se rearma dispara otra vez la deteccion, y otra, y otra. Medido en
# un lote: 25 dislocaciones y 25 auto-rearmes seguidos, arrastrando una
# pinza ya rota ([-0.163, 0.034]) que no se iba a recuperar sola.
#
# El auto-rearme esta para que un roce puntual no deje la celda muerta y
# ciega, no para insistir con el robot roto. Pasado este limite la parada se
# queda firme: hace falta una persona (o reiniciar Webots, que es lo que de
# verdad recoloca la articulacion). El contador se pone a cero con cada
# agarre bueno, asi que solo cuenta rachas de fallo de verdad.
MAX_DISLOCACIONES_SEGUIDAS = 3
HOVER_HIGH = CUBE_TABLE_Z + 0.20   # altura de transito segura, lejos de la mesa
HOVER_GRASP = CUBE_TABLE_Z + 0.10  # unica altura de contacto validada (agarrar Y depositar)
SAFE_Z = 1.30                # z de referencia para la rampa hacia/desde "home"
# Radio (sesion 2026-08-28) para la recomprobacion con camara en
# lift_shift_place(): un cubo del mismo color detectado mas lejos que esto
# del punto de recogida real es OTRO cubo (cajas con colores repetidos en
# varias filas), no evidencia de agarre falso.
STILL_THERE_RADIUS = 0.05
YAW = np.pi / 4.0            # giro para pillar el cubo por caras, no aristas


def shortest_grasp_yaw(yaw):
    """La pinza es simetrica cada 180 grados (agarrar a yaw y a yaw-180
    cierra sobre las MISMAS caras del cubo, fisicamente identico) --
    normaliza al representante mas cercano a 0 para que ramp() (que
    interpola en linea recta desde el yaw de partida, sin dar la vuelta
    corta por su cuenta) gire la muneca lo minimo posible en vez de dar
    un giro largo hacia un angulo fisicamente equivalente a uno mucho mas
    cercano (aviso real del usuario, sesion 2026-09-08: tras forzar el
    giro +90 de _adjust_grasp_yaw_for_obstacles SIEMPRE, el yaw final del
    Sorter cae siempre en 90-180 grados, que normalizado aqui equivale a
    -90..0 -- sin esto, la muneca daba SIEMPRE el giro largo)."""
    return ((yaw + np.pi / 2.0) % np.pi) - np.pi / 2.0

# La camara cenital fija (DEF OVERHEAD_CAM) ahora ve la mesa entera (subida
# a 2.27, ya no un circulo de ~30cm) -- no hay "fuera de su vista" en XY.
# Tras cada pierna hay que volver a HOME real (ver park_at_home) para que el
# BRAZO deje de colgar sobre la columna vertical del cubo; sin esto, la
# siguiente relocalizacion fallaba (bug real, sesion 2026-08-26).

# Verificacion de agarre por posicion real de los dedos (my_robot_driver.py
# ahora publica /gripper_state con la lectura real de cada sensor). Mismo
# principio que la accion grasp() real de franka_gripper: si el dedo llega
# muy cerca de la posicion de cierre PEDIDA, es que no habia nada bloqueando
# (agarre en el aire); si se queda notablemente por encima, es que choco con
# el cubo. Medido empiricamente (sesion 2026-08-26): ~0.017 en vacio vs
# ~0.026 sobre el cubo -- 9mm de separacion, margen de sobra para el umbral.
GRASP_VERIFY_MARGIN = 0.004
MAX_GRASP_RETRIES = 2

# Distancia maxima (metros) entre el punto de recogida esperado y la
# posicion REAL del cubo para que leg() acepte ir a por el (sesion
# 2026-09-09) -- ver el uso en leg(). El respaldo por ground truth existe
# para corregir "esta unos cm mas alla de donde lo esperaba", no para
# perseguir un cubo por toda la celda: los desvios legitimos observados
# son de pocos cm (comparar con STILL_THERE_RADIUS=0.05 del Sorter y
# DROP_CLEAR_RADIUS=0.10 del Loader), y el caso que motivo el limite
# estaba a 1.1m, en la zona del otro robot.
GROUND_TRUTH_MAX_DIST = 0.35

# Red de seguridad final de alcance (metros, distancia horizontal del punto
# de agarre a la base de ESTE robot). No sustituye a GROUND_TRUTH_MAX_DIST
# ni a la vision: es el ultimo filtro antes de mover, para que ninguna
# fuente de coordenadas -- vision, ground truth, o cualquier camino nuevo --
# pueda mandar el brazo a un punto donde fisicamente no llega. Hace falta
# porque "ok" de la IK solo mira LIMITES ARTICULARES: pedir un punto a 1.8m
# devuelve tranquilamente una postura valida con el brazo estirado del todo,
# y el log dice "bajada a hover: OK" mientras el brazo se estira hacia la
# nada (bug real 2026-09-10, y el mismo sintoma que "va a por el cubo y se
# queda arriba quieto, no llega al cubo"). El alcance util del Panda ronda
# los 0.85m; los puntos de trabajo reales de la celda estan todos a menos de
# 0.60m de su base, asi que este limite no puede morder un agarre legitimo.
REACH_MAX_XY = 0.85

# Desvio maximo (metros) entre lo que ve la camara y la posicion REAL del
# cubo antes de descartar la lectura de la camara (sesion 2026-09-09) --
# ver el uso en leg(). Medido sobre un fotograma real con el recorte
# aplicado, la camara acierta con un error de ~0.2cm en los tres colores,
# asi que 10cm es holgadisimo para variacion normal y sigue cazando el
# caso que motivo esto (una lectura a 32cm del cubo de verdad).
VISION_GROUND_TRUTH_MAX_DESVIO = 0.10

# Deteccion de pinza dislocada (sesion 2026-09-08, peticion directa del
# usuario tras varios "se ha roto la pinza"/"ahora no tiene pinza" en
# vivo): el rango FISICO real de cada dedo es 0-0.04m (limite real del
# PROTO PandaHand, ver panda_driver -- "Limites panda_finger::right:
# min=0.0000 max=0.0400"). Cuando la pinza se disloca de verdad (choque
# fisico contra un carril, dos procesos peleandose por el brazo, etc.)
# el sensor ha llegado a marcar valores como 1.315, 0.338, -0.147, -1.01
# -- muy por fuera de ese rango, nunca alcanzables por el motor real
# (limitado por setAvailableForce/setPosition). GRIPPER_DISLOCATION_MAX
# deja margen de sobra sobre el maximo real (0.04) para el ruido normal
# del sensor sin disparar en falso.
GRIPPER_DISLOCATION_MAX = 0.05

# Freno de salto entre pasos de una misma rampa (sesion 2026-09-08) -- ver
# el docstring de ramp(). 0.6 rad (~34 grados): muy por encima de un paso
# normal (n pasos por tramo, cm/grados pequenos cada uno), muy por debajo
# de un giro de aparcado legitimo (SORTER_PARK_POSITIONS usa 1.4 rad, pero
# eso ocurre en un solo salto de park_at_home(), no dentro de una rampa).
JUMP_MAX_RAD = 0.6

# Sesion 2026-08-30: fraccion minima de 'lift' que el cubo objetivo debe
# haber subido DE VERDAD (ground truth de WarehouseSupervisor) para
# contar el levantamiento como real. Empezo en 0.6 (60%) por estimacion;
# probado en vivo (agarres reales del Sorter) dio subidas de 8.9cm y
# 8.1cm sobre 15cm de lift -- 59% y 54%, ambos POR DEBAJO del 60% pero
# casi con toda seguridad agarres BUENOS (muy cerca del 100%, no del 0%
# de un fallo real -- un fallo real midio 0.0cm en la misma tanda de
# pruebas). Bajado a 0.35: sigue distinguiendo con margen de sobra un
# fallo real (~0%) de un agarre bueno (>50% tipico), sin rechazar agarres
# buenos por variacion normal de timing/asentamiento de la fisica.
# Bajado otra vez a 0.28 (sesion 2026-09-04, tras afinar el cierre de
# pinza del Sorter -- ver self.grasp_close en SorterDemo): con el agarre
# ya mejorado, un abrazo real seguia abortando de casualidad justo en el
# borde del umbral (subida real 5.2cm contra 5.25cm esperados con
# lift=0.15 y fraccion 0.35 -- diferencia de medio milimetro). El aborto
# abre la pinza EN EL AIRE (ver mas abajo), asi que un falso rechazo asi
# se ve identico a un resbalon real aunque el agarre fuera bueno de
# verdad -- confirmado por el reintento automatico inmediato, que
# clasifico el mismo cubo sin problema. Sigue de sobra por encima de un
# fallo real (~0%, ver comentario historico arriba).
GROUND_TRUTH_LIFT_FRACTION = 0.28

# Asentamiento antes de cerrar la pinza (sesion 2026-09-04, causa raiz real
# encontrada con datos en vivo -- ver aviso del usuario: "el cubo verde ha
# salido disparado... sigue girando y bajando antes de cerrar la pinza").
# _publish() solo espera step_sleep (0.1s) entre pasos de la rampa -- eso
# es tiempo de sobra para el movimiento TIPICO de un paso (unos pocos
# grados), pero NO para el ultimo paso del giro final si el giro pedido
# es grande: el brazo REAL (posicion fisica en Webots, gobernada por
# COMMANDED_VELOCITY=1.0 rad/s) puede seguir moviendose de verdad bastante
# despues de que el codigo ya haya publicado el ultimo comando y pasado a
# lo siguiente -- self.real_theta guarda lo COMANDADO, no una lectura real
# de sensor, asi que el codigo no tiene forma de saber que el brazo aun no
# ha llegado. Confirmado en vivo (captura de /sorter/joint_positions +
# /sorter/gripper_state): joint7 (muneca) seguia cambiando de valor mas de
# 1 segundo despues de que 'aproximacion final + giro real' diera OK, y
# justo ahi un dedo llego a marcar una posicion imposible (negativa) --
# la pinza cerro sobre un brazo que todavia estaba en movimiento, lanzando
# el cubo. Con esta pausa extra (sin publicar nada nuevo) justo antes de
# cerrar, se le da tiempo al brazo real a terminar de asentarse.
GRASP_SETTLE_S = 0.4

# Asentamiento antes de MEDIR la subida con ground truth (sesion
# 2026-09-09) -- ver el uso en lift_shift_place(). Mismo problema que
# GRASP_SETTLE_S (self.real_theta es lo comandado, no lo leido) pero en la
# verificacion en vez de en el agarre: medir nada mas publicar el ultimo
# paso de la rampa pilla al cubo a medio subir y rechaza agarres buenos.
# 0.3s basta para el tramo de subida (8 pasos) a las velocidades usadas
# hasta ahora; si se vuelve a subir COMMANDED_VELOCITY y reaparecen
# rechazos con subidas reales JUSTO por debajo del umbral, subir esto
# antes que tocar GROUND_TRUTH_LIFT_FRACTION -- bajar el umbral debilita
# una comprobacion real, esperar a la medida no.
LIFT_SETTLE_S = 0.3

# Distancia maxima (metros) entre el destino recien alcanzado y la posicion
# REAL del cubo para dar por bueno que sigue en la pinza (sesion
# 2026-09-09) -- ver el uso en lift_shift_place(). Un cubo agarrado va
# practicamente bajo el TCP, asi que unos pocos cm bastarian; 0.12 deja
# margen de sobra para el desfase de la medida y para un cubo cogido algo
# descentrado, y sigue estando lejisimos del caso que motivo la
# comprobacion (un cubo caido a 0.40m del destino).
CARRIED_MAX_DIST = 0.12
# Cuanto se espera a que el cubo llegue al destino antes de darlo por
# perdido. El traslado mas largo de la celda es el del Sorter (53cm) y el
# brazo puede seguir viajando de verdad un buen rato despues del ultimo
# paso publicado -- generoso a proposito: esperar de mas solo cuesta unos
# segundos en el caso raro de perdida real, mientras que quedarse corto
# aborta entregas buenas, que es mucho peor (ver el comentario del uso).
CARRIED_TIMEOUT_S = 3.0


class CubeShuttleDemo(Node):
    def __init__(self):
        super().__init__('cube_shuttle_demo')
        self.declare_parameter('repetitions', 10)
        self.declare_parameter('reps_per_color', 3)
        self.declare_parameter('colors', ['R', 'G', 'B'])
        self.declare_parameter('cube_x', 0.5)
        self.declare_parameter('cube_y', 0.15)
        self.declare_parameter('shift', 0.15)
        self.declare_parameter('lift', 0.15)
        # Bajado de 0.2 a 0.1 (sesion 2026-09-03, a peticion del usuario:
        # "podria tener mas velocidad los robots, sin romper la precision
        # de ahora"). Es solo el tiempo MUERTO entre pasos de
        # interpolacion de _publish() -- no toca el NUMERO de pasos (n,
        # en ramp()) ni check_convergence/strict_final, que es de donde
        # sale la precision real del agarre. Si en el futuro hace falta
        # mas velocidad todavia, el siguiente sitio a mirar es
        # COMMANDED_VELOCITY/FINGER_VELOCITY (my_robot_driver.py) -- ese
        # es un limite fisico de verdad, con mas riesgo de overshoot en
        # Webots justo en el paso critico de agarre.
        #
        # REVERTIDO a 0.2 (sesion 2026-09-04, causa raiz real confirmada
        # con datos en vivo): self.real_theta guarda lo COMANDADO, no una
        # lectura de sensor -- con solo 0.1s entre pasos, el brazo REAL
        # (limitado por COMMANDED_VELOCITY=1.0 rad/s en my_robot_driver.py)
        # puede quedarse atras del todo, acumulando retraso a lo largo de
        # los 30 pasos de la rampa final. Confirmado viendo /sorter/
        # joint_positions: la muneca seguia girando de verdad mas de 1s
        # despues de que el ultimo paso ya se hubiera publicado y dado por
        # bueno -- la pinza cerro con el brazo aun en movimiento y lanzo
        # el cubo por los aires (un dedo llego a marcar una posicion
        # imposible, negativa). Prioridad: estabilidad del agarre sobre
        # velocidad. Ver tambien GRASP_SETTLE_S, pausa extra justo antes
        # de cerrar la pinza.
        #
        # 0.2 -> 0.15 -> 0.11 (sesion 2026-09-09, dos escalones): acelerar
        # vez EMPAREJADO con COMMANDED_VELOCITY 1.0 -> 1.4 en
        # my_robot_driver.py -- ver alli la cuenta completa de por que el
        # intento de 2026-09-03 (bajar esto solo, a 0.1) hubo que revertirlo.
        # La proporcion entre las dos constantes es lo que fija el margen de
        # asentamiento por paso; subir la velocidad del motor en la misma
        # medida que se acorta la espera lo deja igual que antes. Si hay que
        # tocar la velocidad otra vez, tocar LAS DOS a la vez.
        self.declare_parameter('step_sleep', 0.15)
        self.declare_parameter('max_wait_seconds', 15.0)
        self.declare_parameter('use_vision', True)
        # Donde esta plantado ESTE robot en el mundo (sesion 2026-08-27,
        # celda industrial de dos robots): antes BASE era una constante
        # compartida en panda_ikpy_kinematics.py, asi que solo podia existir
        # un Panda a la vez. Mismo patron que robot_base_x/y/z en
        # teleop_gui.py, portado aqui para que cada instancia (cargador,
        # clasificador) resuelva su propia cinematica.
        self.declare_parameter('robot_base_x', 0.5)
        self.declare_parameter('robot_base_y', -0.3)
        self.declare_parameter('robot_base_z', 0.74)
        # Giro de la base en radianes, eje Z (sesion 2026-09-08, experimento
        # angulo del Sorter hacia el punto de recogida): 0.0 por defecto, que
        # deja la cinematica exactamente como estaba (ver base_yaw en
        # PandaIkpyKinematics). NO tocar en produccion sin verificar antes en
        # panda_sorter_base_rotation_test.wbt -- ver robot_launch_sorter_
        # base_rotation_test.py.
        self.declare_parameter('robot_base_yaw', 0.0)
        # Topic de LED de ESTE robot (sesion 2026-08-30, segunda Pico por
        # USB para el Loader): por defecto el de siempre (Pico W del
        # Sorter, que ademas lleva el boton fisico de parada), pero
        # loader_demo lo sustituye por '/comando_led_loader' para que cada
        # brazo encienda su propia Pico al agarrar, no una compartida.
        self.declare_parameter('led_topic', '/comando_led')
        # LED "producto" (sesion 2026-08-31): segundo LED fisico, solo en
        # la Pico del Loader, con colores continuos por PWM en vez de los
        # 3 fijos de arriba -- ver led_publisher_usb.py. Vacio por defecto
        # (nadie publica, no pasa nada); loader_demo lo activa con
        # '/comando_led_producto'. El Sorter no tiene este LED fisico, asi
        # que se deja desactivado ahi.
        self.declare_parameter('led_topic_producto', '')
        # Diagnostico de agarre (sesion 2026-08-30, a peticion del usuario
        # tras el bug real del Loader agarrando cubos que no estaban):
        # cada vez que el sensor de dedos da un agarre por bueno (LED
        # encendido) o la camara lo desmiente poco despues (agarre falso,
        # ver lift_shift_place), se avisa a Taller_Administracion para el
        # panel "Diagnostico" -- best-effort, nunca bloquea ni aborta el
        # ciclo si el servidor no responde.
        self.declare_parameter('taller_api_base', 'http://taller_host:8000')
        self.taller_api_base = self.get_parameter('taller_api_base').value
        # Nombre de robot para el diagnostico: se deduce del namespace con
        # el que se lanzo este nodo ('/loader' -> 'loader', '/sorter' ->
        # 'sorter'); las demos de un solo robot (sin namespace) quedan
        # como 'desconocido', el backend las acepta igual.
        self.robot_name = self.get_namespace().strip('/') or 'desconocido'

        self.reps = int(self.get_parameter('repetitions').value)
        self.reps_per_color = int(self.get_parameter('reps_per_color').value)
        self.colors = list(self.get_parameter('colors').value)
        self.cube_x = float(self.get_parameter('cube_x').value)
        self.shift = float(self.get_parameter('shift').value)
        self.lift = float(self.get_parameter('lift').value)
        self.sleep = float(self.get_parameter('step_sleep').value)
        self.max_wait_seconds = float(self.get_parameter('max_wait_seconds').value)
        self.y_home = float(self.get_parameter('cube_y').value)
        self.use_vision = bool(self.get_parameter('use_vision').value)

        # joint_positions/gripper_position/gripper_state SIN barra inicial a
        # proposito (sesion 2026-08-27): son relativos para que el namespace
        # con el que se lance este nodo (ver robot_launch.py) los dirija al
        # driver de SU robot. /emergency_stop se queda absoluto porque es un
        # servicio de toda la celda, no por robot. led_topic tambien es
        # absoluto (cada Pico escucha un topic fijo, ver led_topic arriba).
        self.pub_j = self.create_publisher(Float64MultiArray, 'joint_positions', 10)
        self.pub_g = self.create_publisher(Float64, 'gripper_position', 10)
        self.pub_led = self.create_publisher(String, self.get_parameter('led_topic').value, 10)
        led_topic_producto = self.get_parameter('led_topic_producto').value
        self.pub_led_producto = (
            self.create_publisher(String, led_topic_producto, 10) if led_topic_producto else None
        )
        # Bug real (sesion 2026-09-01): con un lote de un solo producto,
        # _set_led() sigue llamandose por cada cubo agarrado con su color
        # REAL (para el LED de agarre) -- sin este guard, esa misma llamada
        # tambien pisaba el LED de producto con ese color real (p.ej. un
        # cubo R usado como material dejaba el LED en rojo a mitad de un
        # lote de arandelas). loader_demo.py lo activa cuando fija el LED
        # de producto al color del lote, para que _set_led dejar de tocarlo
        # hasta el proximo _set_led_producto explicito (fin de lote).
        self._led_producto_fijo = False

        base = [
            float(self.get_parameter('robot_base_x').value),
            float(self.get_parameter('robot_base_y').value),
            float(self.get_parameter('robot_base_z').value),
        ]
        base_yaw = float(self.get_parameter('robot_base_yaw').value)
        self.kin = PandaIkpyKinematics(base=base, base_yaw=base_yaw)
        self.cur = self.kin.seed_from_real(HOME_POSITIONS)
        self.real_theta = HOME_POSITIONS.copy()
        # Giro de agarre BASE de este robot (sesion 2026-08-27): por
        # defecto el de siempre (YAW), pero SorterDemo lo sustituye por uno
        # propio (-45 grados extra) para agarrar por el otro par de caras
        # opuestas -- cada robot puede necesitar un giro distinto, ya no es
        # una constante global compartida.
        self.base_grasp_yaw = YAW
        # Giro de agarre REAL para el cubo actual: por defecto
        # base_grasp_yaw, pero leg() lo ajusta con el angulo detectado por
        # vision si el cubo ha acabado girado de verdad tras muchos ciclos
        # de agarrar/soltar (empujones, rebotes en la cinta) -- si no, la
        # pinza cerraba en el aire por un lado.
        self.current_grasp_yaw = self.base_grasp_yaw
        # Altura TCP de contacto (agarrar Y depositar), por instancia igual
        # que base_grasp_yaw (sesion 2026-08-28): por defecto HOVER_GRASP,
        # pero SorterDemo la sustituye por la suya propia, verificada a mano
        # con el panel manual (el cierre real quedo mucho mas bajo de lo que
        # HOVER_GRASP asumia) -- sin tocar esta constante global, que ya
        # esta validada para el Loader (apilado de 3 cubos).
        self.grasp_z = HOVER_GRASP
        # Cierre de la pinza al agarrar, por instancia igual que grasp_z /
        # base_grasp_yaw (sesion 2026-09-03): por defecto GRIPPER_CLOSED
        # (validado para el Loader), pero SorterDemo puede sustituirlo por
        # el suyo propio sin tocar esta constante global -- un cambio
        # anterior a la constante global para arreglar un resbalon SOLO en
        # el Sorter afecto sin querer tambien al Loader (aviso real del
        # usuario: "yo no te he dicho que toques Loader").
        self.grasp_close = GRIPPER_CLOSED
        # Por defecto False = comportamiento de SIEMPRE (gira la muneca A
        # LA VEZ que baja el ultimo tramo, ver approach_and_grasp) -- el
        # Loader se queda asi. SorterDemo lo pone a True (ver comentario en
        # approach_and_grasp, sesion 2026-09-03).
        self.rotate_before_descend = False
        # Margen extra (metros) por encima de HOVER_HIGH para el giro de
        # muneca cuando rotate_before_descend=True (sesion 2026-09-08,
        # aviso real del usuario mirando Webots durante el experimento de
        # base girada: "sigues girando antes de cerrar la pinza y mueve los
        # cubos... con cinco cm mas alto"). Por defecto 0.0 -- comportamiento
        # identico al de siempre para quien no lo toque.
        self.rotate_clearance_extra = 0.0
        # Apertura de la pinza durante la BAJADA al cubo, por instancia
        # (sesion 2026-09-09). Por defecto GRIPPER_OPEN (0.04 por dedo =
        # 8cm de vano, el maximo), que es lo de siempre para el Loader --
        # coge de una mesa abierta, sin nada a los lados. El Sorter baja
        # DENTRO del canal de carriles y ahi esos 8cm no caben con holgura:
        # ver self.approach_open en SorterDemo. Solo afecta a la bajada;
        # soltar en la caja sigue abriendo del todo (alli no hay carriles).
        self.approach_open = GRIPPER_OPEN
        self.locator = OverheadLocator(self)
        self.gripper_state = None
        self.create_subscription(Float64MultiArray, 'gripper_state', self._on_gripper_state, 10)

        # Posicion XYZ real de los 3 cubos, sesion 2026-08-30 -- ground
        # truth publicada por warehouse_supervisor_driver.py (Supervisor,
        # no inferida de sensores/camara). Topic ABSOLUTO a proposito
        # (como /comando_led y /emergency_stop): un unico
        # WarehouseSupervisor sirve a toda la celda, no es un dato por
        # robot. self._grasp_z_before se rellena en approach_and_grasp()
        # justo antes de cerrar la pinza; ver lift_shift_place() para la
        # comprobacion real tras levantar. self.cube_pos_real[color] es
        # (x,y,z) -- usado tambien en leg() (sesion 2026-08-31) como
        # posicion de repuesto cuando la camara no ve el cubo, en vez de
        # una coordenada fija asumida que puede coincidir con OTRO cubo
        # vecino (bug real reportado: "coge otro color y cree que es el
        # rojo" cuando el rojo de verdad esta fuera del recorte de la
        # camara, p.ej. en una esquina).
        self.cube_pos_real = {}
        self._grasp_z_before = None
        self.create_subscription(Float64MultiArray, '/warehouse/cube_positions',
                                  self._on_cube_positions, 10)

        # Boton fisico de parada en la Pico W -> button_listener.py publica
        # aqui (sesion 2026-08-26), y ademas el panel manual estop_panel.py
        # publica el mismo topic para poder parar/rearmar sin hardware. Al
        # llegar True, se deja de publicar nuevas posiciones de motor (el
        # brazo se queda donde este, los motores de Webots mantienen la
        # ultima posicion pedida solos) -- ver _wait_while_stopped: NO
        # aborta la secuencia, la bloquea, y al llegar False (rearme)
        # continua exactamente desde el mismo paso en el que se quedo.
        self.emergency_stopped = False
        self.create_subscription(Bool, '/emergency_stop', self._on_emergency_stop, 10)

        # Parada automatica por pinza dislocada (ver GRIPPER_DISLOCATION_MAX
        # arriba). Publica en los MISMOS topics que teleop_gui.send_stop()
        # (mismo patron: /emergency_stop + 'parada' a las DOS Picos, no solo
        # la de este robot -- la parada es global, el aviso visual tambien
        # tiene que serlo). self._pinza_perdida_avisada evita reenviar en
        # bucle mientras el valor siga fuera de rango; se resetea con el
        # rearme (ver _on_emergency_stop).
        self.pub_emergency_stop = self.create_publisher(Bool, '/emergency_stop', 10)
        self.pub_led_alarm_sorter = self.create_publisher(String, '/comando_led', 10)
        self.pub_led_alarm_loader = self.create_publisher(String, '/comando_led_loader', 10)
        self._pinza_perdida_avisada = False
        self._auto_parada_ts = None
        self._dislocaciones_seguidas = 0

    def _on_gripper_state(self, msg):
        self.gripper_state = list(msg.data)
        if any(v < -0.005 or v > GRIPPER_DISLOCATION_MAX for v in self.gripper_state):
            self._avisar_pinza_perdida()

    def _avisar_pinza_perdida(self):
        if self._pinza_perdida_avisada:
            return
        self._pinza_perdida_avisada = True
        self.get_logger().error(
            f'PINZA DISLOCADA detectada ({self.gripper_state}, rango real 0-0.04) -- '
            'lanzando parada de emergencia automatica.')
        self.pub_emergency_stop.publish(Bool(data=True))
        self.pub_led_alarm_sorter.publish(String(data='parada'))
        self.pub_led_alarm_loader.publish(String(data='parada'))
        # Marca de que ESTA parada la hemos disparado nosotros: es la unica
        # que se auto-rearma (ver AUTO_REARME_DISLOCACION_S).
        self._auto_parada_ts = time.time()
        self._dislocaciones_seguidas += 1

    def _on_cube_positions(self, msg):
        """9 floats en orden fijo R,G,B (x,y,z cada uno) -- ver
        warehouse_supervisor_driver.py::__publish_positions."""
        d = msg.data
        if len(d) >= 9:
            self.cube_pos_real = {'R': tuple(d[0:3]), 'G': tuple(d[3:6]), 'B': tuple(d[6:9])}

    def _on_emergency_stop(self, msg):
        if msg.data and not self.emergency_stopped:
            self.emergency_stopped = True
            self.get_logger().error(
                'PARADA DE EMERGENCIA recibida en /emergency_stop -- '
                'dejo de mover el brazo donde este.')
        elif not msg.data and self.emergency_stopped:
            self.emergency_stopped = False
            self._pinza_perdida_avisada = False
            self._auto_parada_ts = None
            self.get_logger().warn(
                'REARME recibido en /emergency_stop -- continuo donde me quede.')

    def _wait_while_stopped(self, label):
        """Bloquea (sin abortar) mientras dure la parada de emergencia, y
        devuelve True para seguir en cuanto llegue el rearme. Solo devuelve
        False si rclpy se apaga durante la espera (p.ej. Ctrl+C), el unico
        caso real en el que no tiene sentido seguir esperando."""
        if not self.emergency_stopped:
            return True
        self.get_logger().warn(f'En PAUSA por parada de emergencia durante "{label}" -- esperando rearme...')
        while self.emergency_stopped:
            if not rclpy.ok():
                return False
            rclpy.spin_once(self, timeout_sec=0.1)
            if (self._auto_parada_ts is not None
                    and time.time() - self._auto_parada_ts >= AUTO_REARME_DISLOCACION_S):
                if self._dislocaciones_seguidas > MAX_DISLOCACIONES_SEGUIDAS:
                    self.get_logger().error(
                        f'{self._dislocaciones_seguidas} dislocaciones SEGUIDAS sin un solo '
                        'agarre bueno: la pinza no se esta recuperando sola y rearmar otra vez '
                        'solo la castiga mas. Dejo la parada puesta y NO me rearmo -- hace '
                        'falta mirar el robot (reiniciar Webots recoloca la articulacion).')
                    self._auto_parada_ts = None
                    continue
                self.get_logger().warn(
                    f'Parada automatica por pinza dislocada ({self._dislocaciones_seguidas} '
                    f'seguida/s de {MAX_DISLOCACIONES_SEGUIDAS}): me rearmo solo tras '
                    f'{AUTO_REARME_DISLOCACION_S:.0f}s para no quedarme parado encima de '
                    'mi propia camara (ver AUTO_REARME_DISLOCACION_S). Una parada de '
                    'boton o de panel NO se rearma sola.')
                self._auto_parada_ts = None
                # Abrir la pinza ANTES de bajar la bandera de parada (sesion
                # 2026-09-10, causa real de "aprieta mucho y se rompe"): el
                # auto-rearme de antes solo quitaba la bandera, y el SIGUIENTE
                # paso programado -- casi siempre 'levantar', ver
                # lift_shift_place() -- se ejecutaba igual con la pinza
                # TODAVIA cerrada sobre el dedo que se disloco. _verify_grasp()
                # solo mira el PROMEDIO de los dos dedos contra grasp_close, y
                # una lectura dislocada (0.084) pasa ese filtro igual que un
                # agarre real -- asi que el brazo se creia con el cubo bien
                # cogido y tiraba hacia arriba de un dedo ya trabado. Capturado
                # en vivo: 0.084 -> (auto-rearme, levanta) -> -0.166 -> ... La
                # cinematica del lift no cambia por abrir aqui: si de verdad
                # habia un cubo bien agarrado, GRIPPER_OPEN antes de moverse lo
                # deja caer en el sitio de siempre y el flujo existente
                # (verificacion por ground truth en lift_shift_place, "no subio
                # lo esperado") ya lo trata como fallo y reintenta desde cero
                # -- no hace falta logica nueva para ese caso.
                #
                # Y limpiar el parpadeo de alarma en las dos Pico (sesion
                # 2026-09-10, aviso real del usuario: "si se auto rearma...
                # tambien hay que resetear los led"). led_publisher.py /
                # led_publisher_usb.py NO escuchan /emergency_stop -- solo
                # reaccionan a 'parada'/'rearme' explicito en /comando_led y
                # /comando_led_loader (_avisar_pinza_perdida ya publica
                # 'parada' ahi arriba al disparar la parada). El panel
                # (teleop_gui.send_rearm) ya hace este mismo publish cuando
                # una persona pulsa REARME; aqui hacia falta el equivalente
                # para cuando se rearma solo.
                self.pub_led_alarm_sorter.publish(String(data='rearme'))
                self.pub_led_alarm_loader.publish(String(data='rearme'))
                self.pub_emergency_stop.publish(Bool(data=False))
        # Abrir la pinza si SIGUE dislocada en este mismo instante, sea cual
        # sea el origen del rearme (sesion 2026-09-10, segundo bug real: "se
        # ha roto la pinza y no puedo rearmar" -- el arreglo de arriba solo
        # cubria el auto-rearme por temporizador; un rearme MANUAL desde el
        # panel/boton llega por /emergency_stop directamente a
        # _on_emergency_stop, sin pasar por la rama de arriba, asi que salia
        # de aqui con la pinza TODAVIA cerrada sobre el atasco -- el
        # siguiente movimiento la volvia a disparar en 1-2s, una y otra vez
        # (visto en vivo: 4 -> 5 -> 7 dislocaciones seguidas sin que ningun
        # rearme, automatico o manual, arreglara nada). Se comprueba el
        # ESTADO REAL del sensor, no de donde vino el rearme -- si la pinza
        # ya no esta dislocada (se rearmo por otro motivo, p.ej. boton
        # fisico con un cubo bien agarrado en curso) esto no toca nada y no
        # suelta un agarre bueno.
        if self.gripper_state is not None and any(
                v < -0.005 or v > GRIPPER_DISLOCATION_MAX for v in self.gripper_state):
            self.get_logger().warn(
                f'Sigo viendo la pinza fuera de rango ({self.gripper_state}) tras el '
                'rearme -- abro antes de continuar para no seguir tirando del atasco.')
            self._gripper(GRIPPER_OPEN, 0.5)
        self.get_logger().warn(f'Rearmado -- continuo "{label}".')
        return True

    def spin_for(self, seconds):
        """Espera de verdad 'seconds' (reloj de pared), procesando
        callbacks mientras tanto. Bug real corregido en sesion 2026-08-28:
        `rclpy.spin_once(self, timeout_sec=seconds)` NO espera ese tiempo
        si ya hay mensajes en cola (p.ej. la camara publicando varias
        veces por segundo) -- spin_once vuelve en cuanto procesa UNO,
        aunque sea en milisegundos. Un bucle de "40 reintentos de 1s" asi
        se vaciaba entero en poco mas de un segundo real (confirmado con
        timestamps de log), dejando al Sorter sin apenas margen real para
        esperar al Loader en trabajo concurrente."""
        t0 = time.time()
        while rclpy.ok() and time.time() - t0 < seconds:
            rclpy.spin_once(self, timeout_sec=max(0.0, seconds - (time.time() - t0)))

    def wait_for_subscribers(self):
        t0 = time.time()
        while rclpy.ok() and time.time() - t0 < self.max_wait_seconds:
            if (self.pub_j.get_subscription_count() > 0
                    and self.pub_g.get_subscription_count() > 0):
                return True
            rclpy.spin_once(self, timeout_sec=0.1)
        return False

    def _publish(self, th):
        self.real_theta = th
        self.pub_j.publish(Float64MultiArray(data=th.tolist()))
        rclpy.spin_once(self, timeout_sec=0.02)
        time.sleep(self.sleep)

    def _gripper(self, pos, wait):
        # spin_for, NO time.sleep (sesion 2026-09-09, encontrado revisando si
        # el patron del bug de las verificaciones estaba en algun sitio mas).
        # time.sleep NO procesa callbacks de ROS: durante la espera de 2s tras
        # pedir el cierre, self.gripper_state NO se actualizaba, asi que
        # _verify_grasp() -- que se llama justo despues -- comparaba una
        # lectura ANTERIOR al cierre, con los dedos todavia abiertos (~0.033
        # frente a un grasp_close de 0.014): la condicion "avg > cierre +
        # margen" se cumplia siempre y el agarre se daba por bueno pasara lo
        # que pasara. Encaja con lo que ya estaba documentado sin explicacion
        # ("el sensor de dedos da falsos positivos reales", por lo que se
        # degrado a red de seguridad frente a la ground truth). Esperando con
        # spin_for se espera lo mismo pero con los datos vivos, y de paso la
        # parada de emergencia se atiende durante el cierre en vez de quedar
        # bloqueada hasta el siguiente movimiento.
        self.pub_g.publish(Float64(data=pos))
        self.spin_for(wait)

    def _pause_conveyor(self, paused):
        """No-op por defecto -- el Loader coge de una mesa/caja FIJA, la
        cinta no le afecta para nada. SorterDemo lo sobreescribe (sesion
        2026-09-03, analisis tras un dia entero de resbalones sin causa
        clara en el codigo ni en el carril): la cinta NUNCA se para (ver
        DEF BELT en panda_industrial_cell.wbt, 'speed 0.15' fijo desde que
        arranca Webots, sin forma de pararla desde ROS2 hasta ahora) --
        significa que CADA agarre del Sorter pasa con la superficie de la
        cinta deslizando bajo el cubo, incluso con el cubo ya empujado
        contra BELT_END_STOP. El Loader nunca tiene ese rozamiento de
        fondo porque su cubo esta sobre una mesa inmovil. Es la unica
        diferencia estructural entre los dos robots que no se habia
        probado a eliminar."""
        pass

    def _solve_step(self, x, y, z, r, check_convergence=False):
        """Resuelve un paso, y si el solver de ikpy no converge de verdad
        (bug real, sesion 2026-08-26: cerca del punto B el solver a veces se
        queda a 1-4cm del objetivo con la semilla heredada), reintenta UNA
        vez con la semilla reanclada a la postura REAL del brazo antes de
        rendirse (ver el comentario del propio reintento: sembrar desde
        HOME, como se hacia hasta 2026-09-09, provocaba saltos de rama de
        hasta 100 grados a mitad de rampa).

        check_convergence (sesion 2026-08-31, bug real reportado por el
        usuario: "ha pillado el azul porque lo ha movido de la esquina, eso
        es trampa"): por defecto False, kin.solve() solo exige limites
        articulares -- 'ok=True' NO garantiza que la pinza llegue de verdad
        a (x,y,z), solo que la postura encontrada es valida. Confirmado con
        el propio solver, offline: a x>=0.60 en el punto de recogida del
        Sorter, 'ok' daba True pero la posicion real (FK) quedaba lejos del
        objetivo -- la pinza cerraba sobre el aire o solo rozaba el cubo,
        que se desplazaba con el roce y el SIGUIENTE intento (con ground
        truth) lo agarraba donde hubiera quedado. Parecia 'resolverse solo'
        tras 2-3 intentos, pero no era un agarre limpio, era la pinza
        empujando el cubo hasta una zona alcanzable por accidente. Pasar
        check_convergence=True en el paso critico (el ultimo de la rampa de
        aproximacion final, ver ramp()) hace que ESTE fallo se detecte AQUI
        -- antes de tocar el cubo -- en vez de descubrirlo despues como un
        'agarre falso' tras ya haber movido algo."""
        th, result, ok = self.kin.solve(x, y, z, r, self.cur, check_convergence=check_convergence)
        if ok:
            self.cur = result
            return th, True
        # La semilla de reintento se ancla a la postura REAL del brazo, no
        # a HOME_POSITIONS (sesion 2026-09-09, causa raiz confirmada con
        # datos en vivo). Con HOME como semilla, el solver converge a una
        # rama del espacio nulo elegida desde una postura que no tiene nada
        # que ver con donde esta el brazo AHORA -- valida en limites
        # articulares, pero a un mundo de distancia de la postura actual.
        # Capturado en el log del Sorter: en el ultimo paso de 'bajar a
        # depositar' (el brazo ya sobre la cesta, moviendose 2cm) este
        # reintento devolvio una postura que pedia mover una articulacion
        # 100.5 GRADOS de golpe. El freno de ramp() lo caza y no lo manda al
        # motor, pero entonces la rampa aborta con el cubo en la pinza y el
        # reintento de leg() lo suelta donde este ("ha perdido el verde
        # antes de dejarlo en la cesta"). Sembrando desde self.real_theta se
        # sigue escapando de una self.cur que haya derivado (que es para lo
        # que servia el reseteo), pero la solucion sale CONTINUA con la
        # postura actual por construccion, no por suerte.
        fresh_seed = self.kin.seed_from_real(self.real_theta)
        th2, result2, ok2 = self.kin.solve(x, y, z, r, fresh_seed, check_convergence=check_convergence)
        if ok2:
            self.cur = result2
            return th2, True
        self.cur = result
        return th, False

    def ramp(self, x, y, z_from, z_to, yaw_from, yaw_to, n, label, strict_final=False):
        """strict_final (ver _solve_step): exige convergencia real SOLO en
        el ultimo paso (el que de verdad importa: donde se cierra la pinza
        o se deposita) -- los pasos de transito por el aire se quedan como
        siempre (unos pocos cm de margen ahi nunca importaron en la
        practica, y exigirlo tambien alli causaba abortos nuevos en
        movimientos que no tocan nada, ver docstring de PandaIkpyKinematics.solve).

        Freno de salto entre pasos (sesion 2026-09-08, aviso real del
        usuario: "el brazo se disloca contra la cinta" -- mismo sintoma sin
        resolver de sesion 2026-09-04, "giro brusco hacia abajo...
        estrellado contra la cinta", hipotesis nunca confirmada de un salto
        de espacio nulo del solver entre dos pasos que en Cartesiano solo
        se mueven unos mm). A partir del PASO 2 (nunca el 1, que legitima-
        mente puede pedir un giro grande si viene de aparcado -- ver
        SORTER_PARK_POSITIONS con joint1 +1.4 rad, eso NO es el bug), si
        alguna articulacion pide moverse mas de JUMP_MAX_RAD de golpe
        respecto al paso anterior DE ESTA MISMA RAMPA (nunca respecto a
        self.real_theta, que puede venir de mucho mas lejos legitimamente),
        se rechaza el paso como si el solver no hubiera convergido. No
        confirmado con datos en vivo del salto exacto -- ajustar
        JUMP_MAX_RAD si sigue disparando en pasos legitimos o si deja
        pasar el salto real."""
        prev_th = None
        for i in range(1, n + 1):
            if not self._wait_while_stopped(label):
                return False
            frac = i / n
            z = z_from + (z_to - z_from) * frac
            yaw = yaw_from + (yaw_to - yaw_from) * frac
            r = GRASP_R @ rz(yaw)
            th, ok = self._solve_step(x, y, z, r, check_convergence=(strict_final and i == n))
            if ok and prev_th is not None:
                salto = np.max(np.abs(np.asarray(th) - prev_th))
                if salto > JUMP_MAX_RAD:
                    self.get_logger().error(
                        f'[freno salto IK] {label} (paso {i}/{n}): una articulacion '
                        f'pedia moverse {np.degrees(salto):.1f} grados de golpe respecto '
                        f'al paso anterior (limite {np.degrees(JUMP_MAX_RAD):.0f}) -- '
                        'posible salto de espacio nulo, no se envia al motor real.')
                    ok = False
            if not ok:
                self.get_logger().error(f'LIMITE en {label} (paso {i}/{n}, z={z:.4f})')
                return False
            prev_th = th
            self._publish(th)
        self.get_logger().info(f'{label}: OK (x={x:.3f} y={y:.3f} z={z_to:.4f})')
        return True

    def translate(self, x_from, y_from, x_to, y_to, z, yaw, n, label):
        """Interpola X e Y a la vez (no solo Y): hace falta para poder
        corregir tambien la deriva en X al depositar (ver leg())."""
        r = GRASP_R @ rz(yaw)
        for i in range(1, n + 1):
            if not self._wait_while_stopped(label):
                return False
            frac = i / n
            x = x_from + (x_to - x_from) * frac
            y = y_from + (y_to - y_from) * frac
            th, ok = self._solve_step(x, y, z, r)
            if not ok:
                self.get_logger().error(f'LIMITE en {label} (paso {i}/{n})')
                return False
            self._publish(th)
        self.get_logger().info(f'{label}: OK (x={x_to:.3f} y={y_to:.3f})')
        return True

    def _set_led(self, letter):
        self.pub_led.publish(String(data=letter))
        # LED "producto" (sesion 2026-09-01, aviso real del usuario: "si
        # vamos a hacer arandelas todo el lote, el led de producto tiene
        # que estar siempre encendido" -- fabricar es hacer MUCHAS piezas
        # del mismo color seguidas, no una sola). Antes se apagaba aqui
        # mismo cada vez que el de agarre se apagaba (entre cubo y cubo,
        # y en cada agarre fallido) -- parpadeaba en vez de quedarse fijo
        # mostrando que producto se esta fabricando. Ahora SOLO se
        # actualiza con un color real (nunca con '0' desde aqui); quien
        # apaga el LED de producto de verdad es _set_led_producto('0') al
        # terminar el lote entero (ver loader_demo.py). Si el lote es de
        # un solo producto, self._led_producto_fijo bloquea esta
        # actualizacion automatica -- el color real de CADA cubo (aqui,
        # 'letter') ya no es el producto que se esta fabricando, ver
        # _led_producto_fijo mas arriba.
        if letter != '0' and not self._led_producto_fijo:
            self._set_led_producto(letter)
        if letter in ('R', 'G', 'B'):
            self._notificar_evento_produccion('led_encendido', letter)

    def _set_led_producto(self, letter):
        if self.pub_led_producto is not None:
            self.pub_led_producto.publish(String(data=letter))

    def _notificar_evento_produccion(self, tipo, color):
        """POST best-effort a Taller_Administracion para el panel
        'Diagnostico' (sesion 2026-08-30) -- nunca lanza, nunca bloquea el
        ciclo: si el servidor no responde, se pierde ese evento y ya
        esta, igual que _notificar_taller en sorter_demo.py."""
        url = self.taller_api_base.rstrip('/') + '/taller/evento_produccion'
        body = json.dumps({'robot': self.robot_name, 'color': color, 'tipo': tipo}).encode('utf-8')
        req = urllib.request.Request(url, data=body, method='POST', headers={'Content-Type': 'application/json'})
        try:
            urllib.request.urlopen(req, timeout=1.5).close()
        except (urllib.error.URLError, urllib.error.HTTPError, TimeoutError, ValueError) as e:
            self.get_logger().warn(f'[diagnostico] no se pudo avisar a Taller_Administracion: {e}')

    def approach_and_grasp(self, x, y, color='R'):
        # Reiniciar la semilla de IK al valor de HOME en vez de arrastrar la
        # de la pierna anterior: con 7 GDL para una pose de 6D hay libertad
        # de codo (espacio nulo), y encadenar semillas entre tramos puede ir
        # derivando hacia una rama cada vez mas cerca de un limite real (bug
        # real visto en la sesion 2026-08-26: la repeticion 1 de 10 salio
        # perfecta y la 2 fallo justo al final de la aproximacion por esto).
        # Resolver siempre desde el mismo punto de partida conocido hace
        # cada tramo independiente y reproducible.
        self.cur = self.kin.seed_from_real(HOME_POSITIONS)
        self._gripper(self.approach_open, 0.5)
        # hover_z: HOVER_HIGH + rotate_clearance_extra -- normalmente igual a
        # HOVER_HIGH (extra=0.0), salvo que se pida explicitamente mas
        # margen para el giro (ver rotate_clearance_extra).
        hover_z = HOVER_HIGH + self.rotate_clearance_extra
        if not self.ramp(x, y, SAFE_Z, hover_z, 0.0, 0.0, 10, 'bajada a hover'):
            return False, False
        # Reafirmar pinza abierta justo antes de la bajada final de verdad
        # (sesion 2026-08-27, aviso real del usuario mirando Webots: se vio
        # la pinza cerrarse/hacer cosas raras ya durante la bajada, antes de
        # llegar a HOVER_GRASP) -- un solo _gripper(GRIPPER_OPEN) al
        # principio del todo no basta si algo la deja a medias por el
        # camino; repetirlo aqui, inmediatamente antes del tramo que de
        # verdad se acerca al cubo, garantiza que baja abierta.
        self._gripper(self.approach_open, 0.0)
        # Giro a la orientacion de agarre ANTES de bajar, SOLO para el
        # Sorter (self.rotate_before_descend, ver CubeShuttleDemo.__init__
        # y SorterDemo.__init__) -- sesion 2026-09-03, aviso real del
        # usuario mirando Webots: girar la muneca A LA VEZ que se bajaba
        # desplazaba el cubo de su sitio antes de que la pinza llegara a
        # cerrarse. PROBADO para los dos robots primero, pero afecto sin
        # querer tambien al Loader (aviso real del usuario: "yo no te he
        # dicho que toques Loader", y despues "ha vuelto a perder el cubo
        # antes de dejar en la cinta") -- aislado a self.rotate_before_
        # descend, que por defecto es False (comportamiento de SIEMPRE,
        # gira y baja a la vez) y SorterDemo lo pone a True.
        if self.rotate_before_descend:
            if not self.ramp(x, y, hover_z, hover_z, 0.0, self.current_grasp_yaw, 10,
                              'giro a orientacion de agarre (parado, a altura segura)'):
                return False, False
            yaw_from_bajada = self.current_grasp_yaw
        else:
            yaw_from_bajada = 0.0
        # Bajada final mas lenta/fina (sesion 2026-08-28, a peticion del
        # usuario): 30 pasos en vez de 12 para el mismo tramo -- mismo
        # step_sleep por paso, asi que el recorrido tarda 2.5x mas pero cada
        # paso es mas pequeño, dando mas margen para que un contacto lateral
        # con un cubo vecino se note/frene antes de empujarlo con fuerza.
        # Con rotate_before_descend=True ya no gira aqui (yaw_from=yaw_to);
        # con False (Loader, comportamiento de siempre) gira y baja a la vez.
        # _pause_conveyor(True) justo aqui (sesion 2026-09-03): a partir de
        # este punto la pinza esta cerca de tocar el cubo -- si hay algo
        # deslizando bajo el (la cinta, ver _pause_conveyor), que se pare
        # ANTES del contacto, no despues.
        self._pause_conveyor(True)
        if not self.ramp(x, y, hover_z, self.grasp_z,
                          yaw_from_bajada, self.current_grasp_yaw, 30,
                          'aproximacion final + giro real', strict_final=True):
            # Sesion 2026-08-31: este fallo especifico (convergencia real
            # en el paso critico, ver strict_final en ramp()) es la zona
            # genuinamente inalcanzable -- se notifica aparte de
            # 'agarre_falso' (que es cuando SI se llega pero el agarre
            # falla) para que el panel de Diagnostico pueda distinguir
            # "no llego" de "llego pero no agarro bien".
            self._notificar_evento_produccion('limite_alcance', color)
            self._pause_conveyor(False)
            return False, False
        # Asentamiento real antes de seguir (ver GRASP_SETTLE_S arriba) --
        # el brazo puede seguir moviendose de verdad un rato despues de
        # que este ultimo paso ya se haya publicado y dado por bueno.
        self.spin_for(GRASP_SETTLE_S)
        # Punto de pausa explicito CON LA PINZA YA JUNTO AL CUBO, sin cerrar
        # todavia (sesion 2026-08-27, a peticion del usuario para poder
        # tomar el control manual justo ahi): antes no habia ningun punto
        # interrumpible entre terminar esta rampa y cerrar la pinza, asi
        # que una parada de emergencia llegada en ese hueco no tenia efecto
        # hasta la SIGUIENTE rampa (demasiado tarde, ya cerrada).
        if not self._wait_while_stopped('pinza junto al cubo, sin cerrar'):
            self._pause_conveyor(False)
            return False, False
        # Posicion Z REAL del cubo buscado, justo ANTES de cerrar la pinza
        # (sesion 2026-08-30) -- ground truth de WarehouseSupervisor, para
        # poder comprobar despues de verdad "¿subio ESTE color en
        # concreto?" en vez de solo "¿hay resistencia en la pinza?". Ver
        # lift_shift_place. None si el supervisor no esta publicando
        # (degrada con gracia a las comprobaciones de siempre).
        self._grasp_z_before = self.cube_pos_real[color][2] if color in self.cube_pos_real else None
        self._gripper(self.grasp_close, 2.0)
        grasped = self._verify_grasp()
        if grasped:
            self._set_led(color)
            self.get_logger().info(
                f'pinza cerrada (agarre verificado por sensor de dedos), LED {color} encendido')
            # Racha rota: la pinza funciona (ver MAX_DISLOCACIONES_SEGUIDAS).
            self._dislocaciones_seguidas = 0
        else:
            # Reanudar la cinta AQUI (sesion 2026-09-09, bug real): sin
            # agarre no hay levantamiento, asi que lift_shift_place() -- el
            # unico sitio que reanuda la cinta en el camino bueno, ver su
            # _pause_conveyor(False) -- no llega a ejecutarse nunca. La
            # cinta se quedaba parada durante todos los reintentos y, si
            # leg() los agotaba, TAMBIEN despues: el Sorter volvia a
            # esperar un cubo que ya no podia llegar (la cinta que el mismo
            # habia parado) y el Loader se bloqueaba en
            # _esperar_hueco_en_cinta() esperando a que arrancara. Los dos
            # robots parados esperandose, sintoma exacto del atasco
            # documentado en LANZAR_PROYECTO.md.
            self._pause_conveyor(False)
            self.get_logger().warn(
                'pinza cerrada pero SIN agarre verificado (los dedos llegaron '
                'al cierre pedido, nada los bloqueaba)')
        return True, grasped

    def _verify_grasp(self):
        """Mismo principio que la accion grasp() real de franka_gripper:
        compara la posicion REAL de los dedos (sensor, no lo pedido) contra
        el cierre pedido. Si coincide (dentro de GRASP_VERIFY_MARGIN), nada
        los freno -> agarre en el aire. Si se quedan notablemente por
        encima, chocaron con el cubo -> agarre real. Sin dato (topic no
        disponible), se asume OK para no bloquear el pipeline."""
        if self.gripper_state is None:
            return True
        avg = sum(self.gripper_state) / len(self.gripper_state)
        return avg > self.grasp_close + GRASP_VERIFY_MARGIN

    def lift_shift_place(self, x_from, y_from, x_to, y_to, place_z=None, place_yaw=None, color=None):
        # Depositar es literalmente el reverso de coger, paso a paso:
        # "bajar a depositar" deshace "levantar" (mismo HOVER_GRASP, no una
        # altura inventada aparte -- eso fue lo que fallaba antes: bajar a
        # un valor distinto del agarre podia sobre-empujar la mesa si el
        # cubo no estaba perfectamente centrado, o dejarlo en el aire si se
        # quedaba corto). Luego "retirada" deshace "aproximacion + giro
        # 45deg" (mismo tramo de altura y de giro, invertido), y el ultimo
        # tramo deshace "bajada a 20cm". Mismos pasos que la ida, al reves.
        #
        # place_z (opcional): altura TCP de contacto para depositar, si es
        # distinta de HOVER_GRASP (p.ej. apilar sobre otro cubo en vez de
        # sobre la mesa). Sesion 2026-08-26: apilar con el mismo ramp de 8
        # pasos que la mesa empujo el cubo de abajo ~7-8cm en vez de posar
        # con suavidad -- el "suelo" de abajo ahi SI puede moverse, a
        # diferencia de la mesa rigida. Se baja en pasos de 0.5cm (no
        # 1.5-2cm) en el ultimo tramo para dar mas oportunidad de que el
        # contacto se resuelva de forma suave en vez de con un salto grande.
        # place_yaw (sesion 2026-08-27, Sorter): el giro para AGARRAR en la
        # cinta puede no ser el mismo que el giro para DEPOSITAR en la caja
        # -- en la cinta los cubos llegan en fila tocandose (empujados por
        # la cinta contra BELT_END_STOP), y cerrar la pinza en la direccion
        # paralela a esa fila choca con el cubo vecino en vez de agarrar el
        # objetivo (verificado con grasp_yaw_test.py); girada 90 grados mas
        # SI agarra bien, pero esa orientacion no converge en la caja
        # (verificado con el solver). Se gira la muneca DESPUES de despejar
        # BELT_END_STOP en altura, no mientras aun se sube -- girar Y subir
        # a la vez (como se hacia antes) barria la pinza contra la propia
        # pared en los primeros pasos, cuando el TCP todavia estaba a su
        # misma altura (aviso real del usuario mirando Webots: "ha vuelto a
        # pillar la pared de atras"). Primero sube recto sin girar hasta
        # quedar claramente por encima de la pared, y solo entonces gira.
        yaw_place = self.current_grasp_yaw if place_yaw is None else place_yaw
        target_z = self.grasp_z if place_z is None else place_z
        z_lift = max(self.grasp_z, target_z) + self.lift
        ok_levantar = self.ramp(x_from, y_from, self.grasp_z, z_lift, self.current_grasp_yaw, self.current_grasp_yaw, 8,
                                 f'levantar {self.lift * 100:.0f}cm')
        # _pause_conveyor(False) aqui, pase lo que pase (ver _pause_conveyor
        # / approach_and_grasp): la ventana vulnerable es JUSTO este tramo
        # de subida -- a partir de aqui el cubo ya deberia estar claramente
        # despegado de la cinta, y si algo fallo ya se ha fallado.
        self._pause_conveyor(False)
        if not ok_levantar:
            return False
        # Recomprobacion TRAS levantar (sesion 2026-08-30, analisis pedido
        # por el usuario tras ver que el Loader "cree que tiene cubos que
        # no tiene" incluso CON vision activada, y que a veces agarra un
        # color y cree que es otro). Dos fuentes posibles, en orden de
        # PRIORIDAD -- la ground truth manda cuando esta disponible, el
        # sensor de dedos es solo la red de seguridad si no lo esta:
        #
        # 1) GROUND TRUTH (preferida): ¿subio de verdad el cubo del COLOR
        #    QUE ESTOY BUSCANDO, la cantidad esperada? Usa la posicion
        #    REAL publicada por WarehouseSupervisor (Supervisor de
        #    Webots, no un sensor de la pinza) -- resuelve a la vez los
        #    dos problemas reportados: un agarre de refilon que se
        #    resbala (el cubo se queda a su Z de siempre) y agarrar un
        #    cubo DISTINTO del que se creia (el objetivo real nunca se
        #    movio, aunque la pinza SI sujete algo). Investigado
        #    buscando articulos sobre deteccion de agarre: un sensor de
        #    POSICION de la propia pinza (lo unico que habia antes) no
        #    puede distinguir un agarre real de uno de refilon -- hace
        #    falta la posicion real del OBJETO, no solo de la pinza.
        # 2) Sensor de dedos (`_verify_grasp()`, solo si no hay ground
        #    truth disponible -- self._grasp_z_before es None, p.ej. el
        #    WarehouseSupervisor no esta corriendo, o no se paso color
        #    como en StackTowerDemo): bug real encontrado probando esto
        #    en una prueba larga -- usado como comprobacion PRINCIPAL,
        #    el sensor de dedos dio varios falsos RECHAZOS seguidos sobre
        #    un agarre que la ground truth habria confirmado como bueno
        #    (mide solo la propia pinza, mas ruidoso que la posicion real
        #    del cubo) -- degradado a red de seguridad, no la fuente
        #    principal.
        # Esperar a que la subida TERMINE de verdad antes de medirla (sesion
        # 2026-09-09). La rampa de arriba solo garantiza que se ha PUBLICADO
        # el ultimo paso, no que el brazo fisico haya llegado -- mismo
        # desfase comandado/real de GRASP_SETTLE_S, aplicado aqui a la
        # medida. Sin esta pausa la comprobacion lee el cubo a medio subir y
        # rechaza agarres buenos: capturado al subir la velocidad a 2.0
        # rad/s, dos abortos seguidos con subidas reales de 4.2cm y 4.0cm
        # contra un umbral de 4.2cm -- el cubo estaba bien agarrado y el
        # aborto lo solto en el aire ("se le ha caido tres veces"). Explica
        # tambien por que el umbral se fue bajando 0.6 -> 0.35 -> 0.28 en
        # sesiones anteriores: se estaba compensando una medida prematura en
        # vez de arreglarla. spin_for procesa callbacks, asi que
        # cube_pos_real llega actualizado a la comparacion.
        self.spin_for(LIFT_SETTLE_S)
        ground_truth_disponible = color is not None and self._grasp_z_before is not None and color in self.cube_pos_real
        if ground_truth_disponible:
            subida_real = self.cube_pos_real[color][2] - self._grasp_z_before
            subida_esperada = self.lift * GROUND_TRUTH_LIFT_FRACTION
            if subida_real < subida_esperada:
                self.get_logger().error(
                    f'[verificacion ground truth] el cubo {color} NO subio lo '
                    f'esperado de verdad (subida real {subida_real * 100:.1f}cm, '
                    f'se esperaban al menos {subida_esperada * 100:.1f}cm) -- '
                    'lo que tengo en la pinza (si tengo algo) no es el cubo '
                    'objetivo, o se resbalo. Abortando sin depositar nada en destino.')
                self._notificar_evento_produccion('agarre_falso', color)
                self._gripper(GRIPPER_OPEN, 0.5)
                self._set_led('0')
                return False
        elif not self._verify_grasp():
            self.get_logger().error(
                '[verificacion pinza] agarre PERDIDO durante el levantamiento -- '
                'el sensor de dedos ya no detecta resistencia. Abortando sin depositar nada en destino.')
            if color is not None:
                self._notificar_evento_produccion('agarre_falso', color)
            self._gripper(GRIPPER_OPEN, 0.5)
            self._set_led('0')
            return False
        if yaw_place != self.current_grasp_yaw:
            if not self.ramp(x_from, y_from, z_lift, z_lift, self.current_grasp_yaw, yaw_place, 8,
                              'girar muneca ya despejado de la pared'):
                return False
        if not self.translate(x_from, y_from, x_to, y_to, z_lift, yaw_place, 12,
                               'desplazar a destino fijo'):
            return False
        # Recomprobacion con camara (sesion 2026-08-28): el sensor de dedos
        # (_verify_grasp) da falsos positivos reales (visto varias veces
        # esta sesion, confirmado con la pestaña Position de Webots -- el
        # cubo se quedaba intacto en la mesa mientras el log decia "agarre
        # verificado"/"CUBO CLASIFICADO"). Una camara cenital NO distingue
        # "sigue en la mesa" de "esta en el aire a la misma X/Y" (mismo
        # problema que motivo el desplazamiento de prueba en
        # grasp_yaw_test.py) -- pero AHORA que el brazo ya se ha
        # desplazado lejos de x_from,y_from, si la camara SIGUE viendo un
        # cubo de este color significa que el real nunca se movio de ahi
        # (agarre falso), sin ambiguedad. Solo se comprueba si nos pasan
        # 'color' (None = no aplica, p.ej. StackTowerDemo que no usa este
        # dato).
        #
        # Bug real corregido en el mismo dia: con colores REPETIDOS (caja
        # del Loader con 2 filas del mismo color) "se ve un cubo de este
        # color" no basta -- puede ser el OTRO cubo del mismo color en la
        # otra fila, no el que se acaba de intentar coger. Solo cuenta como
        # agarre falso si el cubo detectado esta CERCA del punto de
        # recogida real (x_from,y_from), no en cualquier sitio del recorte.
        if color is not None:
            still_there = self.locator.locate_with_yaw(CUBE_VISION_Z, color=color)
            if still_there is not None:
                sx, sy, _ = still_there
                dist = ((sx - x_from) ** 2 + (sy - y_from) ** 2) ** 0.5
                if dist < STILL_THERE_RADIUS:
                    self.get_logger().error(
                        f'[verificacion camara] agarre FALSO: la camara sigue viendo un '
                        f'cubo {color} en ({sx:.3f},{sy:.3f}), cerca del punto de recogida '
                        f'({x_from:.3f},{y_from:.3f}) -- nunca se levanto de verdad. '
                        'Abortando sin depositar nada en destino.')
                    self._notificar_evento_produccion('agarre_falso', color)
                    self._gripper(GRIPPER_OPEN, 0.5)
                    self._set_led('0')
                    return False
        descent_steps = 8 if place_z is None else max(8, int(round((z_lift - target_z) / 0.005)))
        if not self.ramp(x_to, y_to, z_lift, target_z, yaw_place, yaw_place, descent_steps,
                          'bajar a depositar (reverso de levantar)'):
            # Soltar AQUI de todas formas (sesion 2026-09-09, bug real). El
            # desplazamiento a destino ya termino bien, asi que el brazo esta
            # justo ENCIMA del sitio correcto y como mucho unos cm alto: abrir
            # la pinza deja caer el cubo en su destino. Abortar con el cubo
            # todavia en la mano era mucho peor -- el reintento de leg() abre
            # la pinza nada mas empezar, soltandolo en cualquier punto del
            # recorrido. Visto en vivo: el freno de salto IK corto el descenso
            # del Loader en el paso 3/8 y el cubo acabo tirado a medio camino
            # (y=0.304), desde donde la cinta se lo llevo.
            #
            # Y la entrega CUENTA como buena (arreglo del mismo dia, horas
            # despues): al principio esto devolvia False y el resultado fue
            # peor todavia -- el cubo verde quedo fisicamente en su caja pero
            # nadie publico la entrega, asi que el WarehouseSupervisor no lo
            # reciclo nunca y el Loader se quedo 200s esperandolo ("el verde
            # se ha quedado en sorter y no vuelve a la caja... y ahora esta
            # todo parado"). Devolver False era mentir sobre el resultado: el
            # cubo ESTA en su destino, solo que soltado desde unos cm mas
            # alto. Asi que no se corta aqui -- se sigue por el camino normal
            # de "soltar, apagar LED, retirarse y aparcar", que es lo unico
            # que faltaba por hacer de todas formas.
            self.get_logger().error(
                'no se pudo completar el descenso al destino -- suelto el cubo '
                'aqui mismo, sobre el destino, en vez de llevarmelo puesto.')
        self._gripper(GRIPPER_OPEN, 1.2)
        self._set_led('0')
        self.get_logger().info('pinza abierta (soltado), LED apagado')
        # ¿Llego el cubo de verdad al destino? (sesion 2026-09-09). Detecta el
        # unico hueco que quedaba: un cubo que se suelta DURANTE el traslado.
        # Las otras dos verificaciones miran antes de moverse (ground truth
        # tras levantar) o en el sitio de origen (la camara, mas arriba), asi
        # que el brazo podia llegar con la pinza vacia, abrirla y darse por
        # entregado -- el aviso del usuario "deja el cubo gran parte en la
        # cinta y otra parte en el marco y luego se va por la cinta" era
        # exactamente eso, con el cubo caido en el arranque de la cinta.
        #
        # VA AQUI, DESPUES DE DEPOSITAR, y no antes del descenso (donde
        # estuvo un rato el mismo dia): comprobar antes obligaba a abortar
        # con el cubo aun en la pinza, y el aborto la abria a la altura de
        # traslado -- 15cm por encima del destino. Si la comprobacion se
        # equivocaba (y se equivocaba: el brazo tarda en llegar de verdad,
        # ver mas abajo), el cubo caia desde ahi en vez de posarse. Aviso
        # real: "ahora lo sueltas de mas arriba... antes tiraba el brazo
        # para delante y lo dejaba como mas suave". Depositando SIEMPRE y
        # comprobando despues, el movimiento es el suave de siempre y la
        # comprobacion sale gratis: si el cubo venia en la pinza, ya esta
        # en su sitio; si no, no habia nada que posar.
        #
        # Se ESPERA a que llegue, no se mide de golpe: el ultimo paso de la
        # rampa esta PUBLICADO pero el brazo real sigue viajando (el desfase
        # comandado/real de siempre), asi que una medida inmediata daba por
        # perdidos cubos bien agarrados -- vistos a 0.16m y 0.20m en el
        # Sorter, cuyo traslado es de 53cm. Y como un aborto no publica la
        # entrega, aquellos cubos se quedaban sin reciclar y paraban la
        # celda ("los cubos no vuelven de la rejilla a la caja de salida").
        if color is not None and color in self.cube_pos_real:
            t0 = time.time()
            dist_destino = None
            while time.time() - t0 < CARRIED_TIMEOUT_S:
                cx, cy = self.cube_pos_real[color][0], self.cube_pos_real[color][1]
                dist_destino = ((cx - x_to) ** 2 + (cy - y_to) ** 2) ** 0.5
                if dist_destino <= CARRIED_MAX_DIST:
                    break
                self.spin_for(0.1)
            if dist_destino is not None and dist_destino > CARRIED_MAX_DIST:
                cx, cy = self.cube_pos_real[color][0], self.cube_pos_real[color][1]
                self.get_logger().error(
                    f'[verificacion transporte] deposite en ({x_to:.3f},{y_to:.3f}) '
                    f'pero el cubo {color} esta en ({cx:.3f},{cy:.3f}), a '
                    f'{dist_destino:.2f}m -- se solto durante el traslado, no ha '
                    'llegado nada al destino. Reintento.')
                self._notificar_evento_produccion('agarre_falso', color)
                return False
        # Bug real (sesion 2026-08-26): este ramp arrancaba siempre desde
        # HOVER_GRASP en vez de desde target_z (donde el brazo esta de
        # verdad tras depositar). Cuando target_z != HOVER_GRASP (apilar,
        # target_z > HOVER_GRASP), el primer paso ordenaba una z POR DEBAJO
        # de la posicion real -- se vio como un bajon brusco justo al abrir
        # la pinza. Arrancar desde target_z (no HOVER_GRASP a secas)
        # arregla el salto.
        #
        # Segundo bug real (mismo dia): este mismo ramp giraba la muneca
        # (des-giro YAW->0) A LA VEZ que subia, justo encima del cubo recien
        # soltado -- se veia como un giro brusco al alejarse en vez de una
        # subida limpia. Ahora sube en linea recta manteniendo YAW constante
        # (sin girar) en los dos tramos de retirada; el des-giro se deja
        # para park_at_home(), que interpola en espacio de articulaciones
        # ya en SAFE_Z, lejos del cubo, donde girar no arriesga nada.
        #
        # Tercer bug real (mismo dia): HOVER_HIGH (0.97) es una altura FIJA
        # pensada para depositar sobre la mesa. Al apilar sobre 2 niveles
        # (target_z=0.99 > HOVER_HIGH), este ramp ordenaba BAJAR de 0.99 a
        # 0.97 antes de subir de verdad -- bajon con presion justo sobre la
        # torre recien construida. retreat_high nunca puede quedar por
        # debajo de target_z: si target_z ya supera HOVER_HIGH, este primer
        # tramo no baja nada (se queda a la misma altura) y toda la subida
        # ocurre en el segundo tramo.
        # A PARTIR DE AQUI LA ENTREGA YA ESTA HECHA (sesion 2026-09-09, bug
        # real que paro la produccion dos veces). La pinza ya se abrio y el
        # cubo esta en su destino: lo que queda es apartar el brazo. Un fallo
        # en esos tramos es un problema de MOVIMIENTO, no de entrega, y no
        # puede convertir en fracaso algo que ya salio bien -- devolver False
        # aqui hacia que leg() lo contara como "agarre falso", asi que nunca
        # se publicaba /warehouse/cube_delivered, el WarehouseSupervisor no
        # reciclaba el cubo y el Loader se quedaba esperandolo para siempre.
        # Capturado literal: "bajar a depositar: OK" -> "pinza abierta
        # (soltado)" -> "LIMITE en retirada (paso 2/12)" -> "agarre falso".
        # El cubo verde estaba perfectamente puesto en su caja.
        # Si la retirada falla, se va directo a park_at_home(), que interpola
        # en espacio de ARTICULACIONES (sin IK, ver su docstring) y por tanto
        # es la via de escape que mas probabilidades tiene de funcionar
        # justo cuando el solver acaba de fallar.
        retreat_high = max(HOVER_HIGH, target_z)
        if not self.ramp(x_to, y_to, target_z, retreat_high, yaw_place, yaw_place, 12,
                          'retirada (sube recto, sin girar)'):
            self.get_logger().error(
                'fallo al retirarme, pero el cubo YA esta entregado en su destino -- '
                'aparco en espacio de articulaciones y cuento la entrega como buena.')
            self.park_at_home()
            return True
        if not self.ramp(x_to, y_to, retreat_high, SAFE_Z, yaw_place, yaw_place, 10,
                          'subir a home (sigue recto, sin girar)'):
            self.get_logger().error(
                'fallo subiendo a home, pero el cubo YA esta entregado -- '
                'aparco en espacio de articulaciones y cuento la entrega como buena.')
            self.park_at_home()
            return True
        # Volver a HOME real (interpolacion en espacio de articulaciones,
        # segura porque ya estamos en SAFE_Z, lejos de la mesa): con la
        # camara cenital viendo la mesa entera (sesion 2026-08-26) ya no
        # existe un "fuera de su vista" en XY -- lo que hace falta es que el
        # BRAZO (no solo la muneca) deje de colgar sobre la columna vertical
        # del cubo, y HOME es la postura "recogida" de fabrica, la mas
        # probable de dejar esa columna libre de verdad.
        # El resultado del aparcado tampoco decide si la entrega fue buena
        # (ver el bloque de la retirada): park_at_home() solo devuelve False
        # si rclpy se esta apagando, y ni siquiera eso deshace un cubo que ya
        # esta en su caja.
        self.park_at_home()
        return True

    def park_at_home(self):
        if not self._wait_while_stopped('antes de aparcar en HOME'):
            return False
        n = 10
        start = self.real_theta.copy()
        for i in range(1, n + 1):
            if not self._wait_while_stopped('aparcado en HOME'):
                return False
            frac = i / n
            th = start + (HOME_POSITIONS - start) * frac
            self._publish(th)
        self.cur = self.kin.seed_from_real(HOME_POSITIONS)
        self.get_logger().info('aparcado en HOME real')
        return True

    def _dance(self, duration=5.0):
        """Aviso visual de 'cambio de lote' (sesion 2026-08-31, peticion
        del usuario: "cuando hay un cambio de lote los dos robots se van a
        su posicion inicial y bailan durante cinco segundos para que
        sepamos que hay cambio"). Se llama SIEMPRE justo despues de aparcar
        (park_at_home o el aparcado propio del Sorter), nunca a mitad de un
        movimiento real -- menea solo la ultima articulacion (muneca, sin
        efecto en el resto de la pose) con un vaiven suave, para que se
        note de un vistazo en Webots sin arriesgar ningun choque ni salir
        del sitio aparcado."""
        centro = self.real_theta.copy()
        amplitud = 0.35  # rad -- bien dentro del limite de panda_joint7, movimiento visible pero pequeño
        periodo = 0.6    # segundos por vaiven completo
        t0 = time.time()
        while rclpy.ok() and time.time() - t0 < duration:
            if not self._wait_while_stopped('bailando (cambio de lote)'):
                return False
            fase = (time.time() - t0) / periodo * (2 * np.pi)
            th = centro.copy()
            th[6] += amplitud * np.sin(fase)
            self._publish(th)
        self._publish(centro)
        return True

    def _adjust_grasp_yaw_for_obstacles(self, x, y):
        """Hook para que subclases corrijan el giro de agarre por
        obstaculos CONOCIDOS del entorno (p.ej. una pared cerca del punto
        de recogida) -- sesion 2026-08-27, tras confirmar que el giro
        detectado por vision (correcto para un cubo aislado) puede seguir
        siendo una orientacion que choca contra algo si el cubo esta junto
        a un obstaculo. Por defecto no hace nada; ver SorterDemo para la
        implementacion real (collision-aware grasp planning con la unica
        pared que existe en ese punto de recogida, sin sensor nuevo, solo
        con la geometria ya conocida del mundo)."""
        return self.current_grasp_yaw

    def _adjust_grasp_position_for_obstacles(self, x, y):
        """Mismo patron que _adjust_grasp_yaw_for_obstacles pero para DONDE
        cierra la pinza, no como (sesion 2026-08-27): con el giro ya
        corregido (perpendicular a la pared), una prueba de agarre +
        desplazamiento seguia sin mover el cubo de verdad -- el usuario,
        mirando Webots en directo, vio que la pinza tocaba la propia pared
        en vez del cubo. Por defecto no hace nada; ver SorterDemo para el
        retranqueo real hacia el inicio de la cinta."""
        return x, y

    def leg(self, source, dest, tag, color='R', place_z=None, place_yaw=None):
        """Devuelve ok. 'source' y 'dest' son puntos (x,y) FIJOS y
        absolutos (nunca "lo detectado + desplazamiento") -- bug real
        corregido en esta sesion: con el destino recalculado a partir de
        cada deteccion, el propio patron de ida/vuelta se iba desplazando
        despacio y en 46-47 repeticiones acabo saliendose de la zona
        alcanzable del brazo (parando en un limite articular real). Al
        anclar A y B a coordenadas fijas, la vision solo corrige DONDE
        agarrar (que puede variar unos mm/cm de verdad), nunca A DONDE se
        apunta a dejarlo -- así el patron completo no puede derivar."""
        x_guess, y_guess = source
        x_to, y_to = dest
        x, y = x_guess, y_guess
        grasped = False
        for attempt in range(1, MAX_GRASP_RETRIES + 2):
            x, y = x_guess, y_guess
            self.current_grasp_yaw = self.base_grasp_yaw
            if attempt > 1 and self.use_vision:
                # Despejar la camara ANTES de volver a mirar (bug real
                # 2026-09-10, reportado en vivo: "en vez de ir a por el cubo
                # ha repetido varias veces un movimiento arriba, como que voy
                # pero no voy a por el cubo"). Tras un agarre fallido el brazo
                # se queda justo ENCIMA del cubo, con la pinza abierta a su
                # alrededor: la camara cenital ve el brazo, no el cubo. Cada
                # reintento releia esa vista tapada y salia mal de las dos
                # formas posibles -- o "cubo no detectado" (y se caia al
                # fallback), o peor, una mancha FALSA del propio brazo. En el
                # log del Sorter, el intento 3 se fue a (0.521,1.099), 17cm
                # por detras del cubo de verdad, y cerro la pinza en el aire
                # otra vez; en cuanto se agotaron los reintentos y aparco,
                # la lectura salio buena (0.529,1.269) y agarro a la primera.
                # Aparcar cuesta ~1s y convierte tres intentos ciegos en un
                # reintento que de verdad ve donde esta el cubo -- ademas de
                # dejar de dar pinzazos al lado de los cubos, que es lo que
                # los descolocaba y los amontonaba.
                self.park_at_home()
            if self.use_vision:
                # locate_with_yaw en vez de locate (sesion 2026-08-27): tras
                # muchos ciclos de agarrar/soltar seguidos un cubo puede
                # acabar girado de verdad (empujones, rebotes en la cinta),
                # y el giro fijo de siempre cerraba la pinza en el aire por
                # un lado. Se corrige el giro de agarre con el angulo real
                # detectado, no solo la posicion.
                located = self.locator.locate_with_yaw(CUBE_VISION_Z, color=color)
                if located is None:
                    # Sesion 2026-08-31, bug real reportado por el usuario:
                    # si la camara no ve el cubo objetivo (p.ej. esta fuera
                    # de su recorte, en una esquina/pared), el fallback de
                    # SIEMPRE era la coordenada FIJA asumida -- si otro
                    # cubo distinto anda cerca de esa coordenada (en la
                    # cinta, en fila), el brazo lo agarraba a EL creyendo
                    # que era el color buscado ("coge otro color y cree que
                    # es el rojo"). Ahora, si hay ground truth disponible
                    # (posicion REAL publicada por WarehouseSupervisor), se
                    # usa esa en vez de la coordenada fija -- apunta al
                    # cubo objetivo DE VERDAD este donde este, en vez de a
                    # "lo que sea que haya en el sitio donde se esperaba
                    # que estuviera". Si de verdad esta fuera de alcance
                    # (p.ej. contra la pared), el intento fallara limpio
                    # por limite articular/IK (ya gestionado por la
                    # resiliencia de leg()) en vez de agarrar un vecino por
                    # error.
                    pos_real = self.cube_pos_real.get(color)
                    if pos_real is not None:
                        # ...pero SOLO si sigue siendo "el mismo sitio, un
                        # poco movido" (sesion 2026-09-09, bug real): la
                        # ground truth es global (los 3 cubos de la celda,
                        # esten donde esten), asi que sin este limite manda
                        # al brazo a por un cubo que ya no le toca a el. Visto
                        # en vivo: el Loader solto un cubo a medio camino tras
                        # un aborto de rampa, la cinta se lo llevo, y la
                        # ground truth le mando a buscarlo a (0.496,1.270) --
                        # el punto de recogida del SORTER, medio metro fuera
                        # de su zona y encima sobre una cinta en marcha. Si el
                        # cubo se ha ido tan lejos ya no es "esta un poco
                        # descolocado", es que le toca a otro (o al rescate del
                        # WarehouseSupervisor): se abandona este intento limpio
                        # en vez de perseguirlo.
                        dist_guess = ((pos_real[0] - x_guess) ** 2
                                      + (pos_real[1] - y_guess) ** 2) ** 0.5
                        if dist_guess > GROUND_TRUTH_MAX_DIST:
                            self.get_logger().error(
                                f'[vision] no se vio el cubo {color} cerca de '
                                f'({x_guess:.3f},{y_guess:.3f}) y su posicion real '
                                f'({pos_real[0]:.3f},{pos_real[1]:.3f}) esta a '
                                f'{dist_guess:.2f}m -- fuera de mi zona de trabajo '
                                f'(limite {GROUND_TRUTH_MAX_DIST:.2f}m). No voy a por el.')
                            return False
                        x, y = pos_real[0], pos_real[1]
                        self.get_logger().warn(
                            f'[vision] no se vio el cubo {color} cerca de '
                            f'({x_guess:.3f},{y_guess:.3f}); uso su posicion real '
                            f'conocida ({x:.3f},{y:.3f}) en su lugar (ground truth).')
                    else:
                        self.get_logger().warn(
                            f'[vision] no se vio el cubo cerca de ({x_guess:.3f},{y_guess:.3f}); '
                            'sigo con la posicion y el giro asumidos (sin ground truth disponible).')
                else:
                    x, y, yaw_offset = located
                    self.current_grasp_yaw = self.base_grasp_yaw + yaw_offset
                    self.get_logger().info(
                        f'[vision] cubo relocalizado en ({x:.4f},{y:.4f}) '
                        f'(guess era ({x_guess:.4f},{y_guess:.4f})), '
                        f'giro real={np.degrees(yaw_offset):.1f} grados extra')
                    # Contraste con la posicion REAL (sesion 2026-09-09): si
                    # las dos fuentes discrepan mucho, la camara se ha
                    # equivocado de mancha y manda la ground truth, que la
                    # publica el Supervisor leyendo el mundo directamente.
                    # Bug real: una sola lectura mala nada mas cargar el
                    # mundo situo un cubo en (0.205,0.271) cuando estaba en
                    # (0.50,0.16), y el brazo se fue alli y se quedo colgado
                    # ("va a por el cubo verde y se queda arriba quieto, no
                    # llega al cubo"). Medido despues sobre el fotograma real:
                    # con el recorte aplicado la deteccion es exacta (error
                    # 0.2cm en los tres colores), asi que fue un fallo
                    # puntual -- pero basto uno para bloquear la celda, y
                    # habia un dato fiable al lado sin usar.
                    pos_real = self.cube_pos_real.get(color)
                    if pos_real is not None:
                        desvio = ((pos_real[0] - x) ** 2 + (pos_real[1] - y) ** 2) ** 0.5
                        if desvio > VISION_GROUND_TRUTH_MAX_DESVIO:
                            # ...pero antes de fiarse de la ground truth hay
                            # que comprobar que apunte a MI zona, exactamente
                            # igual que en la rama de arriba (bug real
                            # 2026-09-10: aqui faltaba ese limite y era el
                            # mismo fallo que ya se habia arreglado el
                            # 2026-09-09 en el otro camino). Visto en vivo: la
                            # camara del Loader cogio una mancha en
                            # (0.205,0.271) y la ground truth dijo que el cubo
                            # R estaba en (-0.004,1.456) -- el Sorter acababa
                            # de depositarlo en su caja, al otro lado de la
                            # celda, todavia sin reciclar. Sin este limite el
                            # Loader se creyo la ground truth y se fue a por un
                            # cubo a 1.20m, muy fuera de su alcance: la IK
                            # exploto y el proceso entero se murio, dejando la
                            # celda parada con el Sorter esperando cubos.
                            dist_guess = ((pos_real[0] - x_guess) ** 2
                                          + (pos_real[1] - y_guess) ** 2) ** 0.5
                            if dist_guess > GROUND_TRUTH_MAX_DIST:
                                self.get_logger().error(
                                    f'[vision] la camara dice ({x:.4f},{y:.4f}) y la '
                                    f'posicion real del cubo {color} es '
                                    f'({pos_real[0]:.4f},{pos_real[1]:.4f}): las dos '
                                    f'discrepan ({desvio:.2f}m) y encima la real esta a '
                                    f'{dist_guess:.2f}m de mi punto de recogida '
                                    f'(limite {GROUND_TRUTH_MAX_DIST:.2f}m) -- ese cubo '
                                    'no me toca a mi. Abandono este intento.')
                                return False
                            self.get_logger().error(
                                f'[vision] la camara dice ({x:.4f},{y:.4f}) pero la '
                                f'posicion real del cubo {color} es '
                                f'({pos_real[0]:.4f},{pos_real[1]:.4f}), a {desvio:.2f}m '
                                '-- me fio de la posicion real (la camara ha cogido '
                                'otra mancha).')
                            x, y = pos_real[0], pos_real[1]
                            self.current_grasp_yaw = self.base_grasp_yaw
            # _adjust_grasp_yaw_for_obstacles SIEMPRE, fuera del if/else de
            # arriba (bug real, sesion 2026-09-08: cuando la camara NO ve
            # el cubo y se cae al fallback de ground truth -- que pasa a
            # menudo justo en el punto de recogida, con la pinza tapando
            # la vista -- self.current_grasp_yaw se quedaba en
            # base_grasp_yaw=45 grados SIN pasar por aqui. En el Sorter
            # eso es precisamente el angulo peligroso que cierra paralelo
            # a la fila/pared -- "pega al cubo de al lado, sale disparado,
            # se rompe la pinza", visto en vivo). Ahora se aplica siempre,
            # tanto si vino de vision como de cualquiera de los dos
            # fallbacks de arriba.
            self.current_grasp_yaw = self._adjust_grasp_yaw_for_obstacles(x, y)
            x, y = self._adjust_grasp_position_for_obstacles(x, y)
            # Giro mas corto SIEMPRE, tanto si vino de vision+obstaculos
            # como del fallback de base_grasp_yaw de arriba -- ver
            # shortest_grasp_yaw().
            self.current_grasp_yaw = shortest_grasp_yaw(self.current_grasp_yaw)
            # ¿El punto donde la pinza puede cerrar de verdad sigue estando
            # SOBRE el cubo? (ver GRASP_REACH_MARGIN). Se comprueba despues
            # del acotado anticolision, que es justo el que puede alejar el
            # agarre del cubo para no meter un dedo en un carril.
            pos_real = self.cube_pos_real.get(color) if color else None
            if pos_real is not None:
                hueco = (2 * self.approach_open - CUBE_SIDE) / 2.0
                desvio = ((x - pos_real[0]) ** 2 + (y - pos_real[1]) ** 2) ** 0.5
                if desvio > max(hueco - GRASP_REACH_MARGIN, 0.0):
                    self.get_logger().error(
                        f'[{tag}] puedo cerrar en ({x:.4f},{y:.4f}) pero el cubo {color} '
                        f'esta en ({pos_real[0]:.4f},{pos_real[1]:.4f}): {desvio * 100:.1f}cm '
                        f'de desvio y solo tengo {hueco * 100:.1f}cm de hueco entre los dedos '
                        'y el cubo. Bajar ahi lo empujaria en vez de cogerlo (y me disloca la '
                        'pinza). NO bajo: dejo que se recoloque o que lo rescate el almacen.')
                    self._notificar_evento_produccion('limite_alcance', color)
                    return False
            alcance = ((x - self.kin.base[0]) ** 2 + (y - self.kin.base[1]) ** 2) ** 0.5
            if alcance > REACH_MAX_XY:
                self.get_logger().error(
                    f'[{tag}] el punto de agarre ({x:.3f},{y:.3f}) esta a {alcance:.2f}m '
                    f'de mi base ({self.kin.base[0]:.2f},{self.kin.base[1]:.2f}) -- fuera '
                    f'de mi alcance fisico (limite {REACH_MAX_XY:.2f}m). No me estiro a por el.')
                return False
            self.get_logger().info(
                f'-- {tag} (intento {attempt}): coger en ({x:.3f},{y:.3f}) '
                f'-> dejar en ({x_to:.3f},{y_to:.3f})')
            ok, grasped = self.approach_and_grasp(x, y, color=color)
            if not ok:
                return False
            if grasped:
                # Reintentar tambien si la CAMARA desmiente el sensor de
                # dedos (sesion 2026-08-28): antes lift_shift_place() se
                # llamaba UNA vez fuera de este bucle, asi que un falso
                # positivo aqui abortaba el leg() entero sin mas intentos,
                # incluso con reintentos de sobra disponibles. Ahora un
                # "agarre falso" detectado por camara cuenta igual que
                # "sensor dice no agarrado": reabre la pinza y vuelve a
                # relocalizar con vision desde cero (el cubo puede haberse
                # movido un poco tras el intento fallido).
                if self.lift_shift_place(x, y, x_to, y_to, place_z=place_z, place_yaw=place_yaw,
                                          color=color if self.use_vision else None):
                    return True
                grasped = False
                if attempt <= MAX_GRASP_RETRIES:
                    self.get_logger().warn(
                        f'[{tag}] agarre falso (camara) -- reintentando desde cero...')
                continue
            self._gripper(GRIPPER_OPEN, 0.5)
            if attempt <= MAX_GRASP_RETRIES:
                self.get_logger().warn(f'[{tag}] reintentando agarre...')
        self.get_logger().error(f'[{tag}] agotados los reintentos sin agarre verificado.')
        # Sesion 2026-08-31: la señal mas accionable para alguien que NO
        # puede ver la simulacion -- "este cubo concreto se ha quedado
        # atascado de verdad, ningun reintento lo resolvio" -- para el
        # panel de Diagnostico de Taller_Administracion.
        if color is not None:
            self._notificar_evento_produccion('fallo_definitivo', color)
        return False

    def run_cube(self, color):
        """Localiza el cubo de este color UNA vez (un solo disparo, la
        camara cenital ve la mesa entera) para fijar el punto A real de
        ESTE cubo, y repite el ciclo ida/vuelta reps_per_color veces sobre
        el, con A y B anclados a esa posicion inicial (mismo principio anti-
        deriva que el resto de la sesion: la vision corrige DONDE agarrar en
        cada intento, nunca el propio punto de destino)."""
        name = COLOR_NAMES.get(color, color)
        self.get_logger().info(f'=== Cubo {name} ({color}): localizando punto de partida ===')
        located = self.locator.locate(CUBE_VISION_Z, color=color)
        if located is None:
            self.get_logger().error(f'No se ha localizado el cubo {name}; lo salto.')
            return False
        point_a = located
        point_b = (point_a[0], point_a[1] + self.shift)
        self.get_logger().info(
            f'Cubo {name}: A={tuple(round(v, 4) for v in point_a)} '
            f'B={tuple(round(v, 4) for v in point_b)}')
        for rep in range(1, self.reps_per_color + 1):
            self.get_logger().info(f'--- {name} repeticion {rep}/{self.reps_per_color} ---')
            if not self.leg(point_a, point_b, 'ida', color=color):
                self.get_logger().error(f'Abortando cubo {name} en repeticion {rep} (ida)')
                return False
            if not self.leg(point_b, point_a, 'vuelta', color=color):
                self.get_logger().error(f'Abortando cubo {name} en repeticion {rep} (vuelta)')
                return False
            self.get_logger().info(f'--- {name} repeticion {rep}/{self.reps_per_color} completada ---')
        self.get_logger().info(f'=== Cubo {name}: {self.reps_per_color} repeticiones completadas ===')
        return True

    def run(self):
        results = {}
        for color in self.colors:
            results[color] = self.run_cube(color)
        ok_colors = [c for c, ok in results.items() if ok]
        bad_colors = [c for c, ok in results.items() if not ok]
        self.get_logger().info(
            f'TODOS LOS CUBOS PROCESADOS. OK: {ok_colors}. Con fallos: {bad_colors}.')
        return len(bad_colors) == 0


def main(args=None):
    rclpy.init(args=args)
    node = CubeShuttleDemo()
    node.get_logger().info('Esperando a que el driver se suscriba...')
    if not node.wait_for_subscribers():
        node.get_logger().error(
            f'Nadie se ha suscrito tras {node.max_wait_seconds:.0f}s '
            "(revisa que 'ros2 launch panda_controller robot_launch.py' este corriendo).")
    else:
        node.run()
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
