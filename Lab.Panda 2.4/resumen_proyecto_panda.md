# Resumen del proyecto: Lab.Panda 2.4

Proyecto nuevo, creado a partir de `Lad.Claude 2.2` (brazo UR5e) para
trabajar con un robot que SI tiene pinza real: el **Franka Emika Panda**.
El proyecto UR5e se deja tal cual, sin tocar, como referencia.

## Por que este cambio

`Lad.Claude 2.2` demostro todo el flujo (Docker + ROS2 + Webots + IK
numerica + pick&place + LED) pero el UR5e no trae pinza en su PROTO
oficial de Webots, asi que hubo que simular el agarre con un truco de
Supervisor (teletransportar la bola). El Panda incluye la mano/pinza
(`panda_finger_joint1/2`) directamente en su PROTO oficial
(`projects/robots/franka_emika/panda/protos/Panda.proto`), asi que se
puede hacer un agarre real (cerrar los dedos sobre la bola) sin trucos.

## Estructura de carpetas

```
Lab.Panda 2.4/
├── .devcontainer/
│   ├── devcontainer.json
│   └── docker-compose.yml
├── webots/
│   └── Dockerfile
├── ros2_app/
│   └── Dockerfile
├── worlds/
│   └── panda_bolas.wbt
├── ros2_ws/
│   └── src/
│       └── panda_controller/
│           ├── package.xml
│           ├── setup.py
│           ├── setup.cfg
│           ├── resource/
│           │   ├── panda_controller
│           │   └── panda_webots.urdf
│           ├── launch/
│           │   └── robot_launch.py
│           ├── test/
│           │   ├── test_copyright.py
│           │   ├── test_flake8.py
│           │   └── test_pep257.py
│           └── panda_controller/
│               ├── __init__.py
│               ├── my_robot_driver.py
│               ├── move_above_ball.py
│               ├── pick_and_place.py
│               ├── visit_balls.py
│               ├── lift_ball.py
│               ├── vision_lift_cube.py
│               ├── best_color_repeat_lift.py
│               └── led_publisher.py
├── .gitignore
└── resumen_proyecto_panda.md   (este archivo)
```

## Arquitectura (heredada del UR5e, con cambios)

- Dos contenedores Docker (`webots_panda_sim24` y `ros2_panda_dev24`),
  conectados por red bridge `panda_ros_net`, igual patron que el UR5e.
- `ROS_DOMAIN_ID=31` (el UR5e usa 30) para poder tener ambos proyectos
  corriendo sin cruzarse si algun dia coinciden.
- `webots_ros2_driver` en modo extern-controller (`controller
  "<extern>"` en el `.wbt`, `WebotsController` + puerto 1234 en el
  launch), igual que el UR5e.
- Driver propio `my_robot_driver.py` (no `ros2_control` generico):
  suscrito a `/joint_positions` (`Float64MultiArray`, 7 valores, uno por
  joint del brazo) y a **`/gripper_position`** (`Float64`, nuevo topic,
  posicion en metros que se aplica en espejo a los dos dedos).
- Autosource de `install/setup.bash` ya integrado desde el principio en
  el `Dockerfile` de `ros2_app` (no hubo que aplicarlo a posteriori como
  en el UR5e).

## Diferencias clave respecto al UR5e

1. **7 articulaciones, no 6.** El Panda es redundante para una tarea de
   6 grados de libertad (posicion + orientacion del TCP). El Jacobiano
   numerico es de 6x7; el solver de minimos cuadrados amortiguados
   (damped least squares) funciona igual con matrices no cuadradas, asi
   que no hizo falta cambiar el algoritmo, solo el tamano.
2. **Convencion DH distinta.** El UR5e usa DH estandar; el Panda se ha
   modelado con **DH modificados (convencion de Craig)**, que es la
   convencion en la que se publica habitualmente la tabla de parametros
   del Panda. La formula de la transformacion homogenea es distinta:
   `Rx(alpha_{i-1}) · Tx(a_{i-1}) · Rz(theta_i) · Tz(d_i)`.
3. **Pinza real, no simulada.** No hace falta el Supervisor-trick del
   UR5e. Se manda una posicion (metros, 0 = cerrada, ~0.04 = abierta por
   dedo) al topic `/gripper_position` y los dos `LinearMotor` de los
   dedos se mueven en espejo.
4. **Enumeracion defensiva de devices.** Los nombres de joints/dedos del
   Panda (`panda_jointN`, `panda_finger_jointN`) se han tomado de la
   convencion estandar del ecosistema Franka/MoveIt/URDF, **no
   verificados contra el PROTO real de Webots** (GitHub bloqueado para
   fetch en este entorno, docs de Cyberbotics ilegibles por ser SPA). Por
   eso `my_robot_driver.py` imprime por consola, al arrancar, TODOS los
   devices que expone el robot en Webots (`robot.getNumberOfDevices()` /
   `getDeviceByIndex()`), para poder corregir nombres sin adivinar si
   algo no encaja — mismo patron que resolvio el desfase de 180 grados
   del UR5e.

## Estado actual (hitos)

- **Hito 0 (infraestructura):** Dockerfiles, docker-compose, mundo
  `.wbt` con el Panda y las 3 bolas (mismas posiciones que el UR5e:
  roja/verde/azul), paquete ROS2 `panda_controller` con `package.xml`,
  `setup.py`, `setup.cfg`, tests de linter — completo.
- **Hito 1 (equivalente al del UR5e) — confirmado en Webots.**
  `move_above_ball.py` calcula por IK numerica la postura para poner el
  TCP encima de la bola verde y la publica en `/joint_positions`;
  `my_robot_driver.py` la recibe y mueve los 7 motores del brazo. Probado
  contra Webots real (contenedores `webots_panda_sim` + `ros2_panda_dev`):
  IK converge, los 7 angulos llegan dentro de los limites de cada joint,
  sin warnings en el log de Webots. `joint_offsets_deg` no ha hecho falta
  tocarlo (sigue en `[0,0,0,0,0,0,0]`).
- **Hito 2 (equivalente al del UR5e, pero con pinza real) — confirmado
  en Webots.** `pick_and_place.py` coge la bola verde y la deja a la
  derecha de la azul (destino = bola_azul + offset en +Y, misma asuncion
  sin confirmar que en el UR5e, ajustable con `place_offset_x/y/z`). A
  diferencia del UR5e no hace falta Supervisor ni bola teletransportada:
  se publica directamente en `/gripper_position` (0.025 m cerrado / 0.04
  m abierto por dedo, valores aproximados pendientes de calibracion fina)
  y es la fisica de Webots la que sujeta la bola entre los dedos reales.
  Secuencia completa probada sin errores. Ademas, al cerrar la pinza
  sobre la bola verde publica "G" en `/comando_led`, y "0" al soltarla
  (ver Hito 4).
- **Hito 3 (equivalente al del UR5e) — confirmado en Webots.**
  `visit_balls.py` recorre las 3 bolas en orden verde -> azul -> roja
  (hover -> bajar a tocar -> esperar `hold_seconds` -> subir -> siguiente),
  sin agarrar ninguna, y vuelve a la pose de reposo al terminar. Publica
  el color de cada bola en `/comando_led` igual que el UR5e. Secuencia
  completa probada sin errores.
- **Hito 4 — LED RGB por Raspberry Pi Pico W, confirmado con hardware
  real.** Se copio `led_publisher.py` tal cual desde `Lad.Claude 2.2`
  (nodo independiente del robot, reenvia `/comando_led` por socket TCP a
  la Pico W en el puerto 5001) y se integro en `pick_and_place.py` (LED
  del color de la bola mientras la lleva agarrada) ademas de en
  `visit_balls.py` (que ya lo tenia). **Probado end-to-end: la Pico W en
  `192.168.1.101:5001` respondio de verdad a la conexion TCP** (no es
  solo un topic de ROS2 sin efecto), confirmado con `pick_and_place.py`
  encendiendo verde al agarrar y apagando al soltar.
- **Bugs encontrados y corregidos durante las pruebas en Webots:**
  1. Los nombres de los dedos NO eran `panda_finger_joint1/2` (asuncion
     sin verificar) sino **`panda_finger::right`** / **`panda_finger::left`**
     (LinearMotor). Se vio en el listado de devices que imprime
     `my_robot_driver.py` al arrancar y se corrigio ahi.
  2. `joint4` del Panda NO acepta 0 rad como reposo (su rango real es
     aprox. -3.14 a -0.4 rad, el codo viene "pre-doblado" de fabrica);
     mandarle 0 dispara el warning de Webots `too big requested position:
     0 > -0.4`. Se anadio una postura de reposo segura, `HOME_POSITIONS =
     [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]` (la misma "ready pose"
     que ya se usaba como semilla heuristica de la IK), en
     `my_robot_driver.py` (pose inicial de los motores) y en
     `pick_and_place.py`/`visit_balls.py` (pose de referencia para estimar
     duraciones y, en `visit_balls.py`, pose de vuelta al terminar el
     recorrido).
- **Hito 5 — bolas cambiadas a cubos, distancias reajustadas (pendiente de
  confirmar en Webots).** El usuario probo las 3 posiciones originales de
  las bolas (todas en X=0.5, solo separadas en Y: 0.4/0.5/0.6 m desde la
  base del Panda) y ninguna era alcanzable de forma fiable en la practica.
  Se cambio la geometria de `Sphere` a `Box` (0.06 m de lado, mismo tamano
  que el diametro de la bola que sustituye, para no romper la calibracion
  de `gripper_closed_position`/`gripper_open_position` ya validada en el
  Hito 2) en `worlds/panda_bolas.wbt` (nombre de fichero sin cambiar: esta
  hardcodeado en `.devcontainer/docker-compose.yml`, que estas
  herramientas no pueden editar) y se movieron los 3 objetos mas cerca de
  la base y repartidos tambien en X (no solo en Y), para evitar la
  configuracion "todo en fila, solo lateral" original:
    - `cubo_rojo`:  x=0.3, y=-0.05, z=0.77
    - `cubo_verde`: x=0.4, y=0.05,  z=0.77
    - `cubo_azul`:  x=0.5, y=0.15,  z=0.77
  Los valores por defecto de `cube_*_x/y/z` en `move_above_ball.py`,
  `pick_and_place.py`, `visit_balls.py` y `lift_ball.py` (parametros
  renombrados de `ball_*` a `cube_*`) se actualizaron a juego. Verificado
  SOLO con el mismo solver de IK del proyecto ejecutado offline (fuera de
  ROS/Webots, sin acceso a Webots en este entorno) contra los limites
  reales de las 7 articulaciones del Panda (no solo convergencia
  matematica del TCP, que ya convergia tambien para las posiciones
  originales y por tanto no explicaba el fallo): las 3 posiciones nuevas
  convergen con margen de varios grados respecto a cada limite. La IK
  numerica no modela colisiones de los eslabones del brazo contra la mesa
  o la propia base, que es la sospecha mas probable de por que fallaban
  las posiciones originales aunque "en el papel" fueran alcanzables.
  **Confirmado en Webots real** en la misma sesion: se reinicio el
  contenedor `webots_panda_sim` para recargar el `.wbt`, se recompilo el
  paquete (los `.py` se instalan como copia, no symlink, hace falta
  `colcon build` tras cada cambio) y se ejecuto `move_above_ball` y
  `visit_balls` contra el Panda real -- IK convergida, brazo posicionado
  de verdad encima de cada cubo, **cero warnings** de Webots
  (`too big requested position` o similar) en todo el recorrido.
- **Hito 6 -- localizacion por vision + agarre de "cualquier cubo",
  confirmado en Webots real.** Nodo nuevo `vision_lift_cube.py`: cuando
  los cubos ya no estan en las posiciones de `worlds/panda_bolas.wbt`
  (por ejemplo, empujados por un intento de agarre fallido previo), este
  nodo los localiza con la `wrist_camera` en vez de asumir su posicion,
  y repite la secuencia de `lift_ball.py` (agarrar, subir un poco, volver
  a dejar) sobre el que encuentre.
  - Con la orientacion normal de agarre (pinza hacia abajo, `GRASP_R`) la
    camara de muneca mira hacia el horizonte, no hacia la mesa (montaje
    fijo `rotation 0 1 0 -1.5708` de la camara respecto a la brida). Se
    definio una orientacion alternativa, `CAMERA_DOWN_R`, que si apunta
    la camara hacia abajo (comprobado con capturas reales).
  - La proyeccion pixel->mundo NO se resolvio analiticamente: se
    intento primero por calculo (rotacion de la camara derivada a mano
    de la geometria del montaje) pero las medidas reales en Webots no
    encajaban con la prediccion, y una calibracion con 2 perturbaciones
    lejos del punto de interes dio un Jacobiano mal condicionado. Lo que
    si funciono: servoing visual con **reestimacion del Jacobiano
    pixel/mundo en cada paso** (dos fotos extra por iteracion, moviendo
    un poco en X y un poco en Y) y un paso de Newton amortiguado hacia
    el centro de la imagen, con la IK de cada paso "sembrada" con la
    solucion del paso anterior (sin esto, la IK saltaba entre
    configuraciones del brazo muy distintas para objetivos casi iguales,
    lo que arruinaba la estimacion del Jacobiano). Convergencia real
    observada: <5 px de error (~1 cm) en 3-5 iteraciones.
  - Deteccion de color por umbral HSV con **saturacion minima alta
    (200)**: con un umbral mas bajo, el suelo ajedrezado (tonos
    rojizos/marrones, hue parecido al cubo rojo pero menos saturado) se
    detectaba como si fuera el cubo rojo -- bug encontrado viendo las
    capturas reales, no solo los logs.
  - **Corregido en la misma sesion:** durante la fase de busqueda (con
    `CAMERA_DOWN_R`) Webots registraba warnings puntuales de
    `too big/too low requested position` en varias articulaciones -- la
    IK para esa orientacion, en algunos pasos de calibracion del
    Jacobiano (las fotos "perturbadas" en X/Y), encontraba soluciones
    matematicamente validas pero fuera del rango fisico real del motor.
    Se anadio `JOINT_LIMITS` (limites reales, leidos del log de
    `my_robot_driver.py` al arrancar, no de la doc generica de Franka:
    para `panda_joint4`/`panda_joint6` no coinciden) y
    `solve_within_limits()`: prueba primero la semilla "calida" de
    siempre (para no perder la continuidad de configuracion entre pasos
    del servoing) y solo si esa solucion queda fuera de rango prueba
    semillas aleatorias, quedandose con la de menor error que si cumpla
    los limites. Confirmado en Webots real: misma secuencia completa
    (localizar -> agarrar -> subir -> soltar) repetida varias veces tras
    el cambio, **cero warnings nuevos** (verificado con las marcas de
    tiempo reales del log del contenedor, no solo mirando la consola de
    Webots, que acumula texto de ejecuciones anteriores).
  - Sigue pendiente de pulir (no bloquea el uso): cuando dos cubos caen
    con un tamano en imagen parecido, el servoing a veces "salta" de
    perseguir uno a perseguir otro entre pasos consecutivos en vez de
    converger sobre el mismo todo el rato -- ralentiza la convergencia
    (mas pasos, mas error final, ~5-20 cm en vez de ~1 cm) pero no ha
    impedido que el agarre final funcione en ninguna prueba.
  - Probado end-to-end en Webots real, varias veces: cubos que se habian
    desplazado de su sitio original localizados, agarrados (LED del
    color correspondiente), levantados, devueltos a su sitio, sin
    errores.
- **Hito 7 -- eleccion automatica del color mas distinguible + 20
  repeticiones, en pruebas en Webots real.** Nodo nuevo
  `best_color_repeat_lift.py`: antes de moverse, sube el TCP a un punto
  cenital (misma orientacion `CAMERA_DOWN_R` de `vision_lift_cube.py`) sobre
  el centro de los 3 cubos y puntua cada color (`COLOR_RANGES`, mismos
  umbrales HSV) con `score = area_del_mejor_contorno * (saturacion_media /
  255)` -- combina que el blob sea grande (cubo bien visible, no un borde) y
  puro en color (mas margen frente a falsos positivos tipo el suelo
  ajedrezado con el rojo, ver Hito 6). Se queda con el color de score mas
  alto (primera prueba real: elige rojo, area~1000 vs verde~950 vs azul~0-300
  segun el punto de vista). Con el color ya elegido, repite `num_repeats`
  veces (por defecto 20) la secuencia hover -> agarrar (LED del color) ->
  subir un poco -> bajar -> soltar (LED apagado) -> retreat -> vuelta
  explicita a `HOME_POSITIONS`.
  - **Bug encontrado y corregido en la primera prueba real (posicion fija +
    timing):** la primera version usaba la posicion fija ya calibrada del
    color elegido (como `lift_ball.py`) y solo verificaba color por vision
    una vez. Fallo 1 (timing): tras el movimiento cenital de eleccion de
    color no se actualizaba la posicion real del brazo antes de calcular la
    duracion del primer tramo del plan, asi que esa duracion salia
    demasiado corta y la pinza se cerraba antes de llegar encima del cubo
    (visto en Webots: no agarraba nada, sin warnings ni errores en el log).
    Corregido registrando la pose real tras el movimiento cenital.
  - **Segundo problema encontrado tras corregir el timing (posicion fija a
    lo largo de 20 repeticiones):** con el timing ya corregido, la pinza SI
    llegaba a la posicion original del cubo, pero al repetir 20 veces sobre
    una coordenada fija (sin volver a mirar donde esta el cubo de verdad)
    el agarre se fue descentrando cada vez mas -- cada cierre/apertura de
    pinza puede desplazar el cubo un poco por contacto, y ese error se
    acumula repeticion tras repeticion hasta que el agarre queda claramente
    fuera del cubo (confirmado visualmente por el usuario en Webots real
    sobre la repeticion ~11). La verificacion por cinematica directa (FK del
    angulo articular exacto que se mando a `/joint_positions` en la fase de
    agarre) confirmo que el CALCULO era correcto -- el TCP calculado caia a
    <0.5 mm de la posicion original del cubo rojo -- por lo que el problema
    no era un bug de coordenadas sino la falta de relocalizacion.
  - **Solucion:** se reescribio el nodo para relocalizar el cubo elegido por
    vision (mismo servoing con reestimacion del Jacobiano pixel/mundo de
    `vision_lift_cube.py`, restringido siempre a `target_letter`) ANTES DE
    CADA repeticion, no solo una vez. Esto obliga a que toda la secuencia
    (localizar + moverse + agarrar) se ejecute de forma bloqueante dentro
    del `__init__` (mismo patron que la fase de localizacion de
    `vision_lift_cube.py`, extendido a los 20 ciclos completos) en vez del
    patron de timer/tick de la primera version, para no anidar
    `spin_once` dentro de un callback de timer (mismo problema ya
    documentado en `vision_lift_cube.py`). Pendiente de confirmar en Webots
    que las 20 repeticiones con relocalizacion quedan centradas de verdad.
  - **Segundo bug real encontrado en la misma sesion (offset de la
    wrist_camera):** con el timing y la relocalizacion ya corregidos, el
    usuario seguia viendo la pinza cerrarse "por delante" del cubo rojo,
    desplazada hacia donde esta el verde. Causa: `Camera { translation 0 0
    0.12 ... }` en `worlds/panda_bolas.wbt` esta montada 0.12 m por delante
    del FLANGE (hermana de `PandaHand` dentro de `endEffectorSlot`, no
    hija), y la localizacion por vision (heredada de `vision_lift_cube.py`)
    asumia que la posicion del TCP cuando la imagen esta centrada ES la
    posicion del cubo, sin restar ese offset. Con la orientacion de busqueda
    `CAMERA_DOWN_R`, el offset cae sobre el eje X del mundo (no sobre Z),
    asi que el cubo localizado quedaba sistematicamente ~9 cm desplazado en
    +X -- justo hacia el verde. **Corregido en `best_color_repeat_lift.py`**
    anadiendo `flange_forward_kinematics()` (FK sin el offset de
    mano/dedos) y `camera_world_position()` (posicion real de la camara =
    flange + `flange_R @ (0,0,0.12)`), usada en vez del "xy" nominal del
    servoing para reportar la posicion final del cubo. Verificado
    numericamente offline (coincide con la posicion reconstruida a mano) y
    en Webots real (la localizacion paso de [0.363, 0.015] a [0.272, 0.015],
    la correccion en X predicha analiticamente). **Este mismo bug esta
    probablemente tambien en `vision_lift_cube.py`** (usa el mismo patron de
    localizacion sin corregir el offset de camara) -- no se ha tocado ese
    fichero todavia, pendiente de backportear el fix si se vuelve a usar.
  - **Tercer problema (confirmado, NO arreglado todavia): la pinza cierra
    entre dos caras ADYACENTES del cubo, no entre dos opuestas.** Probado a
    mano con `teleop_gui.py` (ver Hito 8): con la posicion XY ya correcta
    (offset de camara corregido), al cerrar la pinza sobre el cubo rojo esta
    intenta agarrar dos caras contiguas (como si pellizcara una esquina) en
    vez de dos caras paralelas opuestas -- la pinza esta girada respecto al
    cubo (los cubos son `Box` sin rotacion, alineados con los ejes del
    mundo). Coincide exactamente con la sospecha ya apuntada mas abajo en
    "Parametros nuevos / a vigilar": el giro de ~45 grados entre la brida de
    joint7 y la mano (`joint_offsets_deg[6]`) nunca se ha aplicado. **Esto
    es lo primero a probar en la proxima sesion**: ir sumando/restando 45
    (o 90, si el primer signo/valor no cuadra) en `joint_offsets_deg[6]` y
    usar `teleop_gui.py` (boton "Orientar pinza abajo" + botones de mover)
    para comprobar a ojo, sin tener que relanzar todo el pick-and-place,
    cuando la pinza queda alineada con los lados del cubo.

- **Hito 8 -- herramientas de teleoperacion manual, confirmadas en Webots
  real.** Dos nodos nuevos para mover el brazo a mano y verificar visualmente
  la geometria en vez de depurar a ciegas con los nodos automaticos:
  - `teleop_manual.py`: teclado en una terminal interactiva (raw termios,
    sin pulsar Intro). Controles: wasd/qe mueven el TCP en X/Y/Z del mundo
    (orientacion de agarre GRASP_R fija), `[`/`]` cambian el paso, `c`/`o`
    cierran/abren la pinza, `r`/`g`/`b`/`0` controlan el LED, `h` va a HOME,
    `v` orienta la pinza hacia abajo sin moverse en XYZ, `p` imprime la
    posicion TCP actual.
  - `teleop_gui.py`: lo mismo pero con botones Tkinter en vez de teclado (el
    usuario pidio botones porque el teclado "es muy complicado"). Funciona
    porque `DISPLAY` + `/tmp/.X11-unix` + `.Xauthority` ya estan montados en
    `ros2_panda_dev` en `.devcontainer/docker-compose.yml` (mismo mecanismo
    con el que se ve la ventana de Webots), asi que se puede lanzar con
    `docker exec -d ...` y la ventana aparece sola en la pantalla del
    usuario. Se autoorienta la pinza hacia abajo al abrir la ventana y cada
    vez que se pulsa HOME (si no, los botones de mover XYZ se rechazan todos
    de entrada por el mismo motivo que en `teleop_manual.py`). El boton HOME
    pide confirmacion (esta pegado a otros botones de uso frecuente y un
    misclick manda el brazo entero lejos de golpe). Cada click queda
    registrado en el log del nodo (`_audit`) para poder diagnosticar sin
    tener que fiarse solo de lo que recuerda el usuario que pulso.
  - **Bugs reales encontrados y corregidos con estas herramientas:**
    1. *Salto al primer movimiento tras 'h'*: `HOME_POSITIONS` NO tiene la
       pinza orientada hacia GRASP_R (verificado por FK: diferencia de
       orientacion grande, ~38 grados de giro articular maximo para
       corregirla). Si se dejaba `self.thetas` en HOME literal, el primer
       movimiento (o la propia reorientacion) necesitaba ese giro grande de
       golpe. Solucion: separar "ir a HOME" (postura literal, sin orientar)
       de "orientar pinza hacia abajo aqui mismo" (accion deliberada aparte,
       sin el filtro de salto que si aplica a los movimientos normales).
    2. *"El brazo se ha encogido" / salto brusco cerca del limite de
       `panda_joint4`*: bug de wrap-around. `_publish_joint` envolvia el
       angulo comandado con `(x+pi) % (2*pi) - pi`; el limite real de
       `panda_joint4` (-3.1416) esta pegado casi exacto a ese borde de
       envoltura, asi que un cambio pequeno en el angulo BRUTO (que si se
       mantiene continuo, sembrado siempre del valor anterior) podia cruzar
       el borde y producir un salto de ~2pi en el angulo ENVIADO al motor
       (confirmado en el log de Webots: warnings `too big/too low requested
       position` con valores ~+-3.13 justo antes del salto visual).
       Solucion: en vez de envolver por modulo, recortar (`np.clip`) a los
       limites reales de cada articulacion (`JOINT_LIMITS`, los mismos que
       imprime `my_robot_driver.py` al arrancar) y rechazar el movimiento si
       el recorte necesario supera 1 grado. Aplicado en `teleop_manual.py` y
       `teleop_gui.py`; **el resto de nodos del paquete siguen usando el
       wrap por modulo** (`move_above_ball.py`, `pick_and_place.py`,
       `visit_balls.py`, `lift_ball.py`, `vision_lift_cube.py`,
       `best_color_repeat_lift.py`) -- de momento no ha dado problemas ahi
       porque cada uno hace pocos movimientos con semillas nuevas (no
       jogging continuo acumulando muchos pasos pequenos como en teleop),
       pero es un riesgo latente a vigilar si se ve un salto raro en alguno.

- **Pendiente:** calibracion fina de `gripper_open_position` /
  `gripper_closed_position` si el cubo se escapa o el agarre queda
  demasiado apretado al verlo en Webots; ajuste visual de `hover_height`
  / `grasp_height` / `touch_height` si el TCP no queda centrado sobre los
  cubos; evitar que el servoing de `vision_lift_cube.py` cambie de cubo
  objetivo a mitad de convergencia cuando dos quedan con tamano parecido
  en la imagen (ver Hito 6).

## Parametros nuevos / a vigilar

- `joint_offsets_deg` (7 valores, grados): igual mecanismo que en el
  UR5e — correccion aditiva que se aplica DESPUES de resolver la
  cinematica, nunca en la geometria/solver.
- `FLANGE_TO_TCP_Z` en `move_above_ball.py` (constante, no parametro):
  offset fijo brida+mano (~0.107 + 0.1034 m) desde el joint7 hasta el
  TCP. Aproximado; si el punto de agarre queda desplazado en Z al
  probar, se ajusta aqui.
- Se ha ignorado deliberadamente el giro de 45 grados que algunas
  fuentes describen entre la brida del joint7 y la mano: si la pinza
  aparece rotada al probar, se corrige sumando el offset correspondiente
  en `joint_offsets_deg[6]` (mismo patron de correccion empirica que el
  UR5e, no hace falta tocar el modelo).

## Restricciones que se mantienen igual que en el UR5e

- **`.devcontainer/` no se puede escribir por las herramientas remotas**
  de este entorno (error confirmado). `devcontainer.json` y
  `docker-compose.yml` se entregan como bloques de texto para pegar a
  mano.
- **Sin copia en la nube.** Backup solo local con `git init` / `git add`
  / `git commit` en este proyecto tambien, sin configurar ningun
  remoto, salvo que se pida explicitamente lo contrario.
- No se toca `proyectos_ros2` ni `Lab.Robotica 2.1`.
- `Lad.Claude 2.2` (UR5e) se deja tal cual, sin tocar.

## Comandos habituales

```bash
cd "Lab.Panda 2.4/.devcontainer"
docker compose up -d --build

# Terminal del contenedor ros2_app (build inicial del paquete)
docker exec -it ros2_panda_dev24 bash
cd /workspace
colcon build --packages-select panda_controller
source install/setup.bash   # no hace falta en terminales nuevas, ya autosourcea

# Terminal 1: arrancar el driver (conecta con Webots)
ros2 launch panda_controller robot_launch.py

# Terminal 2: probar Hito 1 (ir encima de la bola verde)
ros2 run panda_controller move_above_ball

# Terminal 2 (alternativa): probar Hito 2 (pick & place con pinza real)
ros2 run panda_controller pick_and_place

# Terminal 2 (alternativa): probar Hito 3 (recorrido de los 3 cubos)
ros2 run panda_controller visit_balls

# Terminal 2 (alternativa): coger el cubo verde, subirlo un poco y devolverlo
ros2 run panda_controller lift_ball

# Terminal 2 (alternativa): localizar CUALQUIER cubo con la camara (por si
# se ha movido de su sitio original) y repetir la secuencia de lift_ball
ros2 run panda_controller vision_lift_cube

# Terminal 2 (alternativa): elegir por vision el cubo de color mas
# distinguible y repetir 20 veces agarrar/levantar/soltar/volver a HOME
ros2 run panda_controller best_color_repeat_lift
# (para cambiar el numero de repeticiones: --ros-args -p num_repeats:=5)

# Terminal 3 (opcional, para que el LED de la Pico W se encienda de verdad):
ros2 run panda_controller led_publisher
```

## Migracion a Lab.Panda 2.4 (2026-08-25)

Proyecto renombrado de `Lab.Panda 2.3` a `Lab.Panda 2.4` para partir de un
directorio limpio con lo que esta confirmado que funciona (infra Docker,
ROS2, camara, LED/Pico, teleop) tras una sesion larga de depuracion. El
`2.3` se borra despues de esta migracion -- no volver a buscarlo.

**Nombres de contenedor cambiados** para no chocar con el `2.3` mientras
convivieran: `webots_panda_sim24` / `ros2_panda_dev24` (antes
`webots_panda_sim` / `ros2_panda_dev`). Docker Compose usa el nombre de la
carpeta `.devcontainer` como "project name" por defecto, que es igual en
ambos proyectos -- si algun dia hay que levantar dos versiones a la vez,
esto ya esta resuelto con estos nombres explicitos; si no, no importa.

`ros2_ws/build`, `install` y `log` NO se copiaron (había que recompilar
limpio): `colcon build --packages-select panda_controller` funciona igual
que antes.

### BUG REAL CONFIRMADO: la cinematica hecha a mano no coincide con Webots

Tras conseguir agarrar y levantar el cubo verde una vez (con
`joint_offsets_deg[6] = -90`, ver seccion de calibracion mas abajo), se
intento repetir el agarre en un mundo de un solo cubo
(`worlds/panda_un_cubo.wbt`) y fallo de forma reproducible: la pinza
terminaba a mas de 30cm de donde el propio codigo creia estar.

Se construyo un **Supervisor de solo lectura** (`controllers/orientation_probe/`,
`DEF PANDA_HAND` dentro del `endEffectorSlot` del Panda, `DEF CUBO_ROJO` en
el cubo) que imprime la posicion/orientacion REAL de la pinza y del cubo
cada 0.5s -- ground truth independiente de cualquier formula propia. Con
el se confirmo, sin ambiguedad, que **incluso en la postura HOME_POSITIONS
sin mover nada**, mi tabla de cinematica (duplicada a mano en
`teleop_gui.py`, `teleop_manual.py`, `pick_and_place.py`,
`visit_balls.py`, `lift_ball.py`, `vision_lift_cube.py`,
`best_color_repeat_lift.py`, `move_above_ball.py`) predice una posicion
~30cm distinta de la real, y la orientacion de la pinza en `align_gripper_down()`
NO queda mirando hacia abajo aunque el codigo cree que si. **No es un
problema de offset de calibracion como el de joint7 -- es un error de
fondo en el modelo DH**, que por suerte no se nota en la zona de trabajo
donde se probo el agarre del cubo verde (por eso funciono una vez).

De paso se encontro y se arreglo un bug real (aunque no es la causa de lo
anterior): en `_publish_joint` de `teleop_gui.py`/`teleop_manual.py`,
`self.thetas` se quedaba con el valor SIN recortar de la IK en vez del
valor realmente publicado (recortado a limites reales) -- puede acumular
deriva silenciosa tras muchos pasos de jogging seguidos. Ya corregido.

### Pista buena a medio resolver: `ikpy` + URDF real exportado de Webots

Webots puede exportar el URDF real de un robot con
`Supervisor.getUrdf()` (visto en el ejemplo oficial
`projects/robots/abb/irb/controllers/inverse_kinematics/inverse_kinematics.py`,
que usa esto con la libreria `ikpy` -- **confirmado funcionando de fabrica
en esta misma instalacion de Webots**, dibuja un circulo y sigue un
objetivo sin fallos). Es la fuente de verdad correcta, mucho mas fiable
que teclear una tabla DH a mano.

Pasos para reexportar el URDF del Panda (requiere que el Panda tenga
`supervisor TRUE` y controller local, no `<extern>`, temporalmente):

```
# en el .wbt, cambiar temporalmente:
#   controller "<extern>"  ->  supervisor TRUE / controller "urdf_export_test"
# controller ya listo en controllers/urdf_export_test/urdf_export_test.py
# escribe /workspace/controllers/panda_exported.urdf (ya esta en el repo)
```

Ya en el repo: `controllers/panda_exported.urdf` (tal cual lo exporta
Webots) y `controllers/panda_ikpy.urdf` (misma cosa, pero quitando la
rama de la `wrist_camera` y los dedos -- ambos son ramas alternativas de
`panda_link7`/`panda hand` que `ikpy` no sabe elegir solo -- y anadiendo
un link `panda_tcp` virtual a 0.1034m de la mano).

**Hallazgo clave (no error mio ni de ikpy):** el URDF se exporta con los
7 joints en la postura POR DEFECTO del PROTO (`position` de
`HingeJointParameters` en `Panda.proto`:
`[0, 0, 0, -1.7708, -1.6, 1.6, 0.79]`), no en cero. Verificado
comparando la rotacion de reposo de joint4 reconstruida desde el rpy del
URDF contra la rotacion axis-angle original del `.proto` -- coinciden
exactamente, `ikpy` tambien reconstruye la misma matriz que a mano, asi
que ninguno de los dos tiene un bug de interpretacion. El fallo estaba en
pasarle a `ikpy` el angulo absoluto de Franka en vez de
`angulo_real - snapshot_default` para cada joint.

Con esa correccion, para `HOME_POSITIONS`, la posicion mundial predicha
por `ikpy` fue `(0.7849, -0.0317, 1.524)` contra el ground truth real
medido `(0.7857, -0.1931, 1.5182)` -- **X y Z casi exactos (~1mm), pero Y
se queda a ~16cm**. Sin resolver todavia por que; probablemente algun
detalle de precision en el snapshot de joint6/7 o en el eje
`0.000001 0 1` (deberia ser exactamente `0 0 1`) que exporta Webots para
joint4. Antes de seguir insistiendo a mano, probar con
`chain.inverse_kinematics()` de `ikpy` resolviendo hacia atras el angulo
que SI reproduce el ground truth medido, para varias posturas, y ver si
el patron de error es consistente (apuntaria a un joint concreto).

**Recomendacion para la proxima sesion:** terminar de cerrar este ~16cm
en Y usando `orientation_probe` como arbitro, y una vez `ikpy` prediga
bien, sustituir la cinematica hecha a mano en los 8 ficheros por una
unica funcion compartida basada en `ikpy` + `controllers/panda_ikpy.urdf`
(un solo sitio que mantener, en vez de 8 copias que se pueden desincronizar).

### Comandos actualizados para 2.4

```bash
cd "Lab.Panda 2.4/.devcontainer"
docker compose up -d --build

docker exec -it ros2_panda_dev24 bash
cd /workspace
colcon build --packages-select panda_controller

# mundo de un solo cubo con el supervisor de verificacion (activo ahora mismo)
# worlds/panda_un_cubo.wbt -- cambiar en docker-compose.yml si se quiere
# volver al mundo completo de 3 cubos: worlds/panda_bolas.wbt
```

## Sesion 2026-08-25 (segunda parte): ikpy funciona para posicion/orientacion

Se termino de cerrar el hallazgo de la seccion anterior. Resultado: **con
`ikpy` + el URDF real exportado + una correccion empirica fija, el
posicionamiento y la orientacion de la pinza son exactos**, verificado
repetidas veces con `orientation_probe` (nunca mas de 5mm de error, con la
orientacion perfectamente vertical: `x_axis=(1,0,0) y_axis=(0,-1,0)
z_axis=(0,0,-1)`).

**La receta que SI funciona** (ver `controllers/ikpy_scripts/final_grasp.py`
y `controllers/panda_ikpy.urdf`):

1. Cargar la cadena con `ikpy.chain.Chain.from_urdf_file('panda_ikpy.urdf')`
   (URDF real de Webots, sin la rama de la `wrist_camera` ni los dedos --
   `ikpy` no sabe elegir solo entre ramas hermanas de un mismo link).
2. Resolver con `chain.inverse_kinematics(target_local, target_orientation=GRASP_R,
   orientation_mode='all', initial_position=seed)`, donde `GRASP_R =
   [[1,0,0],[0,-1,0],[0,0,-1]]` (pinza mirando hacia abajo) y `target_local
   = target_world - BASE` (`BASE = (0.5,-0.3,0.74)`, la traslacion del
   Panda en el `.wbt`).
3. El resultado de `ikpy` esta en "delta respecto al snapshot de
   exportacion" (ver seccion anterior), asi que el angulo REAL a mandar al
   robot es `delta_solved + SNAPSHOT`, con
   `SNAPSHOT = [0, 0, 0, -1.7708, -1.6, 1.6, 0.79]`.
4. **Correccion empirica fija: `CORRECTION_Z = -0.16`** -- hay que sumar
   esto a la Z del objetivo antes de resolver la IK (p.ej. para que la
   pinza llegue de verdad a Z=0.90 real, pedirle a `ikpy` Z=0.90-0.16=0.74).
   Confirmado en dos configuraciones muy distintas (HOME y un hover sobre
   el cubo) que el error real es SIEMPRE ~0.16m a lo largo del eje propio
   de la pinza (aparece en Y o en Z segun hacia donde apunte esta en cada
   caso) -- probablemente el offset flange-mano-TCP que exporta Webots
   (`0.107 + 0.1034`) no es exactamente el real. Pendiente encontrar la
   causa exacta (no crítico, la correccion empirica ya funciona).
5. **Mover SIEMPRE en pasos pequenos de Z (1.5-2cm), sembrando cada
   `inverse_kinematics` con el resultado del paso anterior, y PUBLICAR
   cada paso intermedio de verdad** (no solo encadenar los calculos y
   mandar el ultimo) -- el driver mueve cada articulacion de forma
   independiente a velocidad fija (`my_robot_driver.py`,
   `COMMANDED_VELOCITY = 1.0` rad/s), asi que un salto grande en el
   espacio de articulaciones puede hacer que el brazo barra por en medio
   de la mesa/el cubo antes de llegar a su sitio. Verlo con el supervisor
   entre pasos si hay dudas.
6. Comprobar `LO/HI` (limites reales de cada joint, mismos valores que
   `JOINT_LIMITS` en `teleop_gui.py`) para cada solucion antes de
   mandarla -- `ikpy` NO respeta los limites del URDF al resolver, puede
   devolver angulos fuera de rango (visto varias veces con `panda_joint4`
   cerca de posturas con brazo muy extendido en ciertas zonas XY).

**Lo que queda sin resolver: el agarre final.** Con la pinza perfectamente
centrada y vertical sobre el cubo (confirmado con el supervisor Y
visualmente por el usuario), **cerrar los dedos empuja el cubo en vez de
levantarlo**, de forma repetida. Hipotesis mas probable: la pinza
ABIERTA mide ~8cm (2x4cm por dedo) y el cubo mide 6cm -- solo ~1cm de
margen por lado, así que los dedos (o el cuerpo de la mano) rozan el
cubo durante el descenso final incluso estando bien centrados, y lo
desplazan antes de llegar a cerrar. **Antes de seguir a ciegas la proxima
vez, probar:**
- Cerrar la pinza a un ancho intermedio (no totalmente abierta) ANTES de
  descender los ultimos centimetros, para reducir el barrido lateral.
- Afinar la altura de cierre en pasos de 0.5cm (no 1.5-2cm) justo en los
  ultimos 5cm antes del agarre, comprobando con el supervisor la posicion
  del cubo tras cada paso (si `cubo_rojo` se mueve, parar ahi y ajustar).
- Revisar si el punto TCP virtual (`panda_hand_tcp_joint`, 0.1034m desde
  "panda hand") esta realmente centrado entre los dedos, o si hay un
  pequeno desfase XY ademas del error de -0.16 en Z ya corregido.

**Nota:** los parches temporales que se probaron en `teleop_gui.py`
durante esta sesion (pisar `HOME_POSITIONS` con angulos reales del
momento, desactivar el auto-align al abrir la ventana) se **revirtieron**
al cerrar la sesion -- `teleop_gui.py` esta tal cual estaba en `2.3`. Si
se retoma el control manual con el modelo DH roto, recordar que
saltara al abrir la ventana (auto-align) y que sus lecturas de posicion
no son de fiar (ver bug de fondo, seccion anterior).

## Sesion 2026-08-26 (Cowork, no VS Code): boton de parada en la Pico W

Anadido desde otra sesion (Cowork, no la de VS Code) -- no se ha tocado
ningun fichero de cinematica/vision/teleop, solo lo siguiente:

- `Rasberry_Pi_Pico/main.py` (fuera de este repo, en `MEGA/Robotica/Rasberry_Pi_Pico/`,
  que es el firmware real que usa la Pico segun sus propios timestamps --
  el comentario de `led_publisher.py` que dice que el firmware vive en
  `proyectos_ros2/micropython/main.py` esta desactualizado): pulsador en
  GPIO16 con pull-up interna (boton entre GPIO16 y GND, sin resistencia),
  con antirrebote por tiempo (40ms). Al pulsarlo: corta el LED de forma
  INMEDIATA y local (no depende de la red) y, ademas, intenta avisar al
  PC abriendo una conexion TCP a `PC_IP:PC_PUERTO` (nuevas constantes en
  `wifi_config.py`, puerto 5002 -- `PC_IP` esta vacia, hay que rellenarla
  con la IP del ordenador en la misma red que la Pico).
- Nodo ROS2 nuevo `button_listener.py` en `panda_controller`: servidor
  TCP en el puerto 5002 (hilo aparte, no bloquea `rclpy.spin`), publica
  `std_msgs/Bool` en `/emergency_stop` al recibir "STOP" de la Pico.
  Anadido a `entry_points` en `setup.py` (hace falta `colcon build
  --packages-select panda_controller` para que aparezca con `ros2 run`).

**Deliberadamente NO hecho:** `/emergency_stop` no esta conectado a
ningun nodo que mueva el brazo (`pick_and_place.py`, `visit_balls.py`,
`vision_lift_cube.py`, `best_color_repeat_lift.py`,
`cube_shuttle_demo.py`, `teleop_*.py`). Dado el estado tan activo y fino
de esos ficheros (ver bug de fondo del modelo DH, correccion de ikpy,
etc. en las secciones anteriores), suscribirlos a `/emergency_stop` sin
haberlos revisado a fondo primero parecia mas riesgo que ayuda. Pendiente
para la proxima sesion (la de VS Code tiene mas contexto de esos
ficheros): que cada nodo de movimiento se suscriba a `/emergency_stop` y
corte su bucle/`rclpy.spin` al recibir `True`.

## Sesion 2026-08-26: recalibracion completa + camara cenital + 3 cubos

Punto de partida: la receta de agarre por `ikpy` (giro 45deg + `CORRECTION_Z`)
ya funcionaba para un cubo suelto, pero nada de esto vivia en el paquete
real -- todo eran scripts sueltos en `/tmp`, y `pick_and_place.py` /
`teleop_gui.py` seguian con la cinematica DH manual sin calibrar.

**Nuevos ficheros en `panda_controller/panda_controller/`:**
- `panda_ikpy_kinematics.py` -- cinematica compartida (`ikpy` +
  `resource/panda_ikpy.urdf` + `CORRECTION_Z`), para no seguir duplicando
  la tabla DH manual en cada nodo. `solve(x,y,z,r,seed,check_convergence=False)`:
  por defecto solo exige limites articulares (igual que toda la sesion,
  fiable); `check_convergence=True` (opt-in) exige ademas que la
  cinematica directa de la solucion coincida con el objetivo pedido
  (`CONVERGENCE_TOL_M=0.02`) -- se probo activandolo siempre pero causaba
  abortos nuevos en tramos de transito donde el margen del solver nunca
  importo en la practica; se dejo desactivado por defecto.
- `overhead_vision.py` -- localizacion de cubos por una camara cenital
  FIJA (no la wrist_camera), de un solo disparo (sin mover el brazo). Ve
  la mesa entera. Recorta la busqueda al rectangulo de pixeles de la mesa
  para no confundir el suelo ajedrezado (baldosas granates, mismo rango
  HSV que el rojo) con un cubo. `COLOR_RANGES` para R/G/B.
- `cube_vision.py` -- intento anterior con la wrist_camera + servoing
  Gauss-Newton; se abandono por fragil (un solo fallo de deteccion
  contaminaba en cascada las relocalizaciones siguientes) en favor de la
  camara cenital. Se deja el fichero de referencia, ya no se usa.
- `cube_shuttle_demo.py` -- nodo de pick-and-place repetido, en produccion
  de verdad (no un script suelto). Cicla los 3 cubos (parametro `colors`,
  default `['R','G','B']`), localiza cada uno con la camara cenital al
  empezar (eso fija su "punto A"), y hace `reps_per_color` idas-y-vueltas
  (`shift` en Y, default 0.15m) por cubo. Verificacion de agarre por
  posicion real de los dedos (ver mas abajo) con reintento automatico
  (`MAX_GRASP_RETRIES=2`). LED del color correspondiente encendido al
  cerrar la pinza, apagado al soltar.

**Cambios en ficheros existentes:**
- `my_robot_driver.py` -- ahora lee `panda_finger::right_sensor` /
  `_left_sensor` (existian en Webots, nunca se leian) y publica su valor
  real en `/gripper_state`. Sirve para verificar el agarre sin fuerza real:
  medido empiricamente, un cierre pedido a `GRIPPER_CLOSED=0.017` da
  ~0.017 real si no hay nada en medio (agarre en el aire) y ~0.026 si
  choca con el cubo -- 9mm de separacion, umbral con margen de sobra
  (`GRASP_VERIFY_MARGIN=0.004`). Mismo principio que la accion `grasp()`
  real de `franka_gripper` (`epsilon_inner/outer` comparando anchura real
  vs pedida) -- confirmado por busqueda web durante la sesion.
- `worlds/panda_un_cubo.wbt` -- añadidos `DEF CUBO_VERDE` y `DEF CUBO_AZUL`
  (mismo tamaño 6cm que el rojo). Añadida `DEF OVERHEAD_CAM` (Robot con
  Camera, `controller "<extern>"`, sin fisica) para la camara cenital;
  requirio un `resource/overhead_camera.urdf` nuevo y una segunda entrada
  `WebotsController` en `launch/robot_launch.py` (Webots permite varios
  controladores extern en el mismo puerto 1234, distinguidos por nombre de
  robot).

**Bugs reales encontrados y corregidos, en orden:**
1. *Deriva del patron ida/vuelta*: el destino de cada traspaso se
   recalculaba como "posicion detectada + desplazamiento" -- con 46-47
   repeticiones el patron completo se fue desplazando hasta salir de la
   zona alcanzable (abort por limite articular real). Arreglo: A y B son
   coordenadas FIJAS y absolutas; la vision solo corrige DONDE agarrar en
   cada intento, nunca A DONDE se apunta a depositar.
2. *Camara de muneca fragil*: el servoing con la wrist_camera dependia de
   la propia precision cinematica del brazo para saber donde miraba, y un
   solo fallo de deteccion durante el sondeo del jacobiano contaminaba en
   cascada las relocalizaciones siguientes (hasta 9cm de error "creido
   convergido"). Sustituido por la camara cenital fija (geometria
   constante, localizacion de un disparo).
3. *Aparcado que no despejaba la camara*: con la camara viendo solo un
   circulo de 30cm, aparcar la muneca en un punto XY fuera de ese circulo
   parecia bastar, pero el CODO/antebrazo seguian colgando sobre la
   columna del cubo -- casi todas las relocalizaciones (93/100 en una
   tanda) fallaban por esto, enmascarado porque el sistema caia en el
   punto fijo de respaldo sin abortar. Arreglo real, dos partes: (a) subir
   la camara para ver la mesa ENTERA (2.27m, ya no un circulo), (b)
   aparcar interpolando en espacio de articulaciones hasta HOME_POSITIONS
   real (no un punto XY arbitrario) -- HOME es la postura "recogida" de
   fabrica, la que de verdad libera la columna del cubo.
4. *Suelo confundido con el cubo*: al subir la camara para ver la mesa
   entera, el suelo ajedrezado (baldosas granates) entro en el encuadre y
   varias baldosas caian en el mismo rango HSV que el rojo, a veces con
   mas area que el propio cubo. Arreglo: recortar la busqueda al
   rectangulo de pixeles de la MESA (geometria conocida, no un ajuste de
   color).
5. *IK que no convergia de verdad*: se descubrio pidiendo aparcar en
   `(0.5,-0.15,1.30)` y viendo que el brazo real se quedaba en
   `(0.5,-0.098,1.102)` -- 6.4cm de error real con una solucion que
   respetaba los limites articulares. El chequeo de "ok" nunca habia
   verificado que la cinematica directa de la solucion coincidiera con el
   objetivo. Se probo exigirlo SIEMPRE pero causo abortos nuevos en tramos
   de transito (el solver de ikpy deja un margen normal de ~1-2cm ahi que
   nunca importo en la practica); se dejo como opcion (`check_convergence`)
   y el bug concreto del aparcado se resolvio de raiz de otra forma (ver
   bug 3b).
6. *Resbalon intermitente de la pinza*: incluso con agarre verificado por
   sensor, en repeticiones largas (10-50) a veces el cubo se soltaba a
   media bajada del deposito. Se aprieto la pinza en 3 rondas
   (0.025 -> 0.021 -> 0.017) reduciendo la frecuencia, pero no
   desaparecio del todo. **Decision del usuario: aceptar una tasa de fallo
   residual** -- se interpreta como un limite del modelo de contacto de
   Webots, no un bug corregible ajustando numeros.

**Validado en esta sesion:**
- 10 y 50 repeticiones seguidas del cubo rojo solo, sin fallos (tras los
  arreglos de arriba).
- Ciclo completo de los 3 colores (R/G/B), 2-3 repeticiones cada uno,
  18/18 y 12/12 agarres verificados sin fallos.
- Movimientos ad-hoc reutilizando `leg()`/`approach_and_grasp()` /
  `lift_shift_place()` como piezas sueltas: mover un cubo a una coordenada
  arbitraria, alinear los 3 cubos en linea, **apilar el verde encima del
  rojo** (altura de deposito ajustada a mano: `CUBE_TABLE_Z + tamano_cubo`
  en vez de `CUBE_TABLE_Z`, mismo offset TCP-cubo de 0.10m ya validado).

**Estado al cerrar la sesion:** el apilado verde-sobre-rojo **fallo**: al
bajar a `STACK_HOVER_GRASP` (0.93, calculado sumando el tamano de un cubo a
la altura de mesa) la pinza metio demasiada presion contra el cubo rojo de
abajo y lo hizo salir disparado ~7-8cm en vez de posar el verde con
suavidad encima. El verde quedo sobre el rojo (visualmente parecia
apilado, ver captura de la sesion), pero el rojo no se quedo quieto debajo
-- probablemente `STACK_HOVER_GRASP` calculado como
`(CUBE_TABLE_Z + tamano_cubo) + 0.10` no deja el margen de contacto
suave que si tiene el caso normal (mesa rigida, mucho margen debajo);
aqui el "suelo" de abajo es el propio rojo, que sí se puede desplazar. Azul
suelto en su propia posicion 2. Webots y los contenedores se pararon
limpios al final de la sesion.

**Pendiente para la proxima sesion:**
- Migrar `pick_and_place.py`, `teleop_gui.py`, `teleop_manual.py`,
  `visit_balls.py`, `lift_ball.py`, `vision_lift_cube.py`,
  `best_color_repeat_lift.py`, `move_above_ball.py` a
  `panda_ikpy_kinematics.py` (siguen con la tabla DH manual sin calibrar).
- Investigar la causa raiz del resbalon intermitente de la pinza (aceptado
  como limite por ahora, no diagnosticado a fondo).
- **Arreglar el apilado**: `STACK_HOVER_GRASP` metio demasiada presion y
  desplazo el cubo de abajo en vez de posar con suavidad -- probar una
  altura de contacto mas alta (menos agresiva) para el caso "posar sobre
  otro cubo" que para "posar sobre la mesa rigida", ya que el objeto de
  abajo si puede moverse. Si se quiere apilar de forma repetible (no solo
  ad-hoc), añadir esa altura de destino como parametro de
  `leg()`/`lift_shift_place()` en vez de un script aparte.

## Sesion 2026-08-26 (continuacion): apilado mejorado, aun no perfecto

Se aplico la recomendacion pendiente de arriba: `lift_shift_place()` y `leg()`
en `cube_shuttle_demo.py` aceptan ahora un parametro opcional `place_z`
(por defecto `None` = comportamiento identico a antes, deposita a
`HOVER_GRASP` como siempre). Cuando `place_z` es distinto de `HOVER_GRASP`
(caso apilar), el tramo final de bajada ya no usa el ramp fijo de 8 pasos:
calcula el numero de pasos para que cada uno baje ~0.5cm (en vez de los
~1.9cm/paso de antes), con un minimo de 8.

**Probado en vivo** (contenedores `webots_panda_sim24`/`ros2_panda_dev24`
reales, no offline) con un script suelto de una sola pierna: localizar rojo
y verde por la camara cenital, `leg(source=verde, dest=rojo, place_z=0.93)`
(`0.93 = CUBE_TABLE_Z + tamano_cubo + 0.10`, mismo offset TCP-centro que
`HOVER_GRASP`). Resultado con `orientation_probe` como arbitro:
- **Mejora real:** el cubo rojo se desplazo solo ~1.3cm (de (0.500,0.150) a
  (0.506,0.138), Z sin cambios), frente a los ~7-8cm de empuje de la vez
  anterior con el ramp de 8 pasos.
- **No queda perfecto:** una captura de la camara cenital tras el ciclo
  muestra el verde encima del rojo pero con un canto del rojo asomando por
  un lado (ver que el offset de ~1.3cm del rojo se traduce directamente en
  desalineacion visible, al no re-localizar el rojo justo antes de la
  bajada final -- el destino se fijo UNA vez al principio del ciclo, no se
  actualizo tras el propio desplazamiento).
- **Pendiente para dejarlo realmente centrado:** re-localizar el cubo de
  abajo (rojo) con la camara cenital justo antes de la bajada final (no solo
  al principio de `leg()`), para corregir sobre la posicion real en vez de
  la asumida; o aceptar el desplazamiento residual como limite del modelo de
  contacto de Webots, igual que el resbalon intermitente ya documentado mas
  arriba.

**Bug real encontrado y corregido justo despues (mismo dia):** el usuario
vio en vivo (Webots real, no solo logs) que justo al abrir la pinza para
soltar el verde, el brazo daba un "tik" y bajaba de golpe un poco de forma
agresiva antes de empezar a subir. Causa: el ramp de "retirada + des-giro"
en `lift_shift_place()` arrancaba siempre desde `HOVER_GRASP` en vez de
desde `target_z` (la altura real donde estaba el brazo tras depositar).
Cuando `target_z != HOVER_GRASP` (caso apilar: `target_z=0.93 >
HOVER_GRASP=0.87`), el primer paso del ramp ordenaba una z POR DEBAJO de la
posicion real -- de ahi el bajon brusco. **Corregido** cambiando el `z_from`
de ese ramp a `target_z`. Vuelto a probar en vivo tras el arreglo: el cubo
de abajo se desplazo solo **~0.7cm** (mejora frente al ~1.3cm de la prueba
anterior a este bug, y muy lejos ya de los 7-8cm del primer intento), y la
camara cenital muestra el verde casi tapando el rojo del todo (solo un hilo
de rojo visible en un borde). El bug del salto brusco no solo se veia mal --
tambien empujaba el cubo un poco mas de lo necesario.

**Torre de 3 cubos (mismo dia, a continuacion):** con el verde ya encima del
rojo, se repitio la misma receta para poner el azul encima del verde
(script suelto que localiza AMBOS cubos por vision cenital antes de mover --
el de abajo, verde, no en su posicion de diseno sino donde de verdad quedo
tras el apilado anterior -- y llama a `leg(source=azul, dest=verde,
place_z=0.99)`, con `0.99 = CUBE_TABLE_Z + 2*tamano_cubo + 0.10`). Resultado:
**la base (cubo rojo) practicamente no se movio** (0.4960,0.1556 antes y
despues, solo jitter transitorio de Z durante el contacto), y la camara
cenital confirma los 3 colores apilados con la base estable. No se ha
medido con precision milimetrica el desplazamiento del verde (intermedio)
al recibir el azul -- solo verificado visualmente.

**Hallazgo de infraestructura (no relacionado con la cinematica):**
`ikpy` no estaba instalado en el contenedor `ros2_panda_dev24` al empezar
esta sesion (`ModuleNotFoundError: No module named 'ikpy'), pese a que
sesiones anteriores lo daban por validado -- se instalo entonces con
`pip3 install ikpy` dentro del contenedor en marcha (sesion de before,
sin persistir), y como `ros2_app/Dockerfile` no lo lista, se pierde cada
vez que el contenedor se recrea (`docker compose up -d --build`). Se
reinstalo aqui de la misma forma (no persistente). **Arreglado:** añadido
`RUN pip3 install ikpy` a `ros2_app/Dockerfile`, asi que a partir del
proximo `docker compose up -d --build` ya no hay que reinstalarlo a mano.
(El contenedor en marcha durante esta sesion sigue con la instalacion
manual hasta que se reconstruya.)

## Sesion 2026-08-26 (continuacion): `stack_tower_demo`, nodo real para apilar

Los apilados de mas arriba (verde/rojo, azul/verde) se probaron con scripts
sueltos fuera del paquete, uno por par de cubos. Se paso a un nodo real,
`stack_tower_demo.py` (subclase de `CubeShuttleDemo`, reusa `locator`/`leg`),
registrado en `setup.py` (`ros2 run panda_controller stack_tower_demo`,
parametro `order` con la lista de colores de abajo a arriba, por defecto
`['R','G','B']`). Localiza la base UNA vez al principio (no hace falta re-
localizarla en cada nivel: probado que apenas se mueve, ~0.25cm en dos
niveles) y cada cubo a mover justo antes de cogerlo. Calcula `place_z` para
cada nivel como `CUBE_TABLE_Z + nivel*tamano_cubo + 0.10`.

**Probado en vivo, mundo reiniciado desde cero:** torre R-G-B completa en
una sola ejecucion del nodo, sin intervencion manual entre niveles. La base
(rojo) se desplazo solo **~0.25cm** en total tras los dos niveles (mejor
aun que el ~0.7cm de la prueba anterior con scripts sueltos -- variabilidad
normal de la fisica de contacto, no una mejora sistematica atribuible al
cambio de script en si). Confirmado visualmente con la camara cenital.

**Tercer bug real encontrado y corregido (mismo dia, viendolo en vivo):**
justo al soltar el cubo, el ramp de retirada giraba la muneca (des-giro
YAW->0) A LA VEZ que subia -- se veia como un giro brusco justo encima del
cubo recien depositado en vez de una subida limpia. Corregido: los dos
tramos de retirada (`target_z -> HOVER_HIGH -> SAFE_Z`) ahora mantienen YAW
constante (sin girar en absoluto); el des-giro se elimino de
`lift_shift_place()` y se deja para `park_at_home()`, que interpola en
espacio de articulaciones ya en `SAFE_Z` -- lejos del cubo, girar ahi no
arriesga nada. Probado en vivo tras el arreglo (torre R-G-B completa de
nuevo): la base se desplazo aun menos que antes, practicamente 0
(0.5000,0.1500 sin cambio salvo jitter de Z transitorio).

**Cuarto bug real (mismo dia, tambien viendolo en vivo, esta vez al soltar
el AZUL en el nivel 2):** `HOVER_HIGH` (0.97) es una altura de transito FIJA
pensada para el caso "depositar sobre la mesa". Al apilar 2 niveles,
`target_z` del azul es 0.99 -- ya SUPERA `HOVER_HIGH`. El ramp de retirada
(`target_z -> HOVER_HIGH`) ordenaba entonces bajar de 0.99 a 0.97 antes de
subir de verdad hacia `SAFE_Z` -- se vio como un bajon con presion sobre la
torre recien construida justo al abrir la pinza. Corregido con
`retreat_high = max(HOVER_HIGH, target_z)`: si `target_z` ya supera
`HOVER_HIGH`, ese primer tramo no baja nada (se queda a la misma altura) y
toda la subida real ocurre despues, hacia `SAFE_Z`. Confirmado en el log
tras el arreglo: el paso "retirada" del nivel 2 termina en z=0.9900 (igual
que `target_z`, sin bajar), y la base de la torre quedo sin desplazarse en
absoluto.

## Sesion 2026-08-26 (continuacion): boton de parada conectado de verdad

El boton fisico de la Pico W (GPIO16, sesion anterior "Cowork") ya cortaba
el LED local y publicaba `/emergency_stop` via `button_listener.py`, pero
NADA lo escuchaba en el lado del brazo (a proposito, ver nota en ese
fichero). El usuario probo el cableado con un script aparte
(`test_boton_led.py` + `Documentacion/boton_led_flujo.html`, LED simple en
GPIO13 en vez del RGB, sin red) para confirmar el boton en si funciona
antes de tocar nada mas -- confirmado, mismo GPIO16 con pull-up que ya usa
`main.py`, no hacia falta cambiar el cableado ni ese fichero.

**Cambios para que sea un boton de parada del PROYECTO, no solo del LED:**
- `Rasberry_Pi_Pico/wifi_config.py`: `PC_IP` estaba vacia, rellenada con
  `192.168.1.XXX` (IP de este PC en la red Wi-Fi de la Pico en el momento de
  la sesion -- si la IP no es estatica puede cambiar tras un reinicio del
  router, revisar con `hostname -I` si el aviso deja de llegar).
- `cube_shuttle_demo.py` (y por herencia `stack_tower_demo.py`): suscritos
  ahora a `/emergency_stop`. Al llegar `True`, dejan de publicar nuevas
  posiciones de motor en `ramp()`/`translate()`/`park_at_home()` -- el brazo
  se queda literalmente donde este (los motores de Webots mantienen sola la
  ultima posicion pedida) y se aborta el resto de la secuencia. No se ha
  tocado ningun otro nodo de movimiento (`pick_and_place.py`,
  `visit_balls.py`, `vision_lift_cube.py`, `best_color_repeat_lift.py`,
  `teleop_*.py`) -- si se usan, el boton solo corta el LED, no el brazo.

**Probado en vivo, extremo a extremo (sin la Pico real):** `button_listener`
corriendo + `stack_tower_demo` a mitad de la torre (justo tras coger el
verde) + un "STOP" simulado por TCP al puerto 5002 desde el propio
contenedor. Resultado en el log: `PARADA DE EMERGENCIA recibida en
/emergency_stop -- dejo de mover el brazo donde este`, y el paso en curso
(`bajar a depositar`) se aborto de inmediato. Pendiente probar con la Pico
fisica real (esta sesion no tenia el hardware delante).

**Indicador visual de parada:** antes, pulsar el boton solo apagaba el LED
(`apagar_rgb()`) -- visualmente identico a "idle normal", sin forma de ver
a simple vista si el sistema esta parado o solo inactivo. Se anadio
`parada_activa` (bandera global) + `actualizar_parpadeo_parada()` en
`main.py`: al pulsar el boton, el rojo se queda parpadeando (300ms on/off)
en vez de apagado fijo, y se ignoran los comandos RGB que lleguen por red
mientras dura (para que no se pisen con el parpadeo). Se queda parpadeando
hasta reiniciar la Pico, a proposito -- una parada de emergencia no deberia
des-activarse sola. Se eligio reusar el rojo existente (parpadeo = alerta,
distinto de fijo = "llevando el cubo rojo") en vez de montar un LED nuevo
dedicado, para no necesitar hardware adicional.

**Bug real encontrado con el boton fisico (el usuario probo con la Pico de
verdad, no un STOP simulado, y el brazo NO paro):** `docker-compose.yml`
no publicaba el puerto 5002 al host -- `button_listener` escuchaba dentro
del contenedor `ros2_panda_dev24`, pero la Pico esta FUERA de Docker, en la
red local, y sin `ports: ["5002:5002"]` esa conexion nunca llegaba al
contenedor (por eso la prueba anterior con un STOP simulado *desde dentro
del mismo contenedor* funciono -- no pasaba por la misma ruta de red que
la Pico real usa de verdad). **Corregido** anadiendo el mapeo de puerto en
`docker-compose.yml` y reconstruyendo los contenedores
(`docker compose up -d --build`). Confirmado con una conexion de prueba
desde la IP real de la red del host (`192.168.1.XXX:5002`, no `127.0.0.1`)
-- llego correctamente y `button_listener` publico `/emergency_stop`.
Pendiente confirmar con la Pico fisica de verdad (esta sesion solo pudo
simular el nivel de red, no probar el boton fisico en si).
