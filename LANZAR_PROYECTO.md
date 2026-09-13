_Última modificación: 2026-09-13 09:00_

# Arrancar el proyecto

Todos los `cd` de aquí abajo son **relativos a la carpeta del proyecto**
(donde tengas clonado o copiado `virtual-robotic`, o `Robotica` si es tu
propia copia). Sitúate ahí primero:

```bash
cd virtual-robotic   # o la carpeta donde lo tengas tu
```

**Atajo:** `./arrancar_todo.sh` hace de un tirón los pasos de la Parte A
y B de aquí abajo (sin las Pico físicas) y termina abriendo el panel de
control manual. Necesita sesión gráfica local (no vale por SSH puro) y
`xhost`.

**La primera vez, mejor NO uses el atajo — arranca a mano, paso a paso
(Parte A y B de aquí abajo).** No es que el script esté roto: es que la
primera construcción de la imagen de Webots descarga un paquete grande
(el propio Webots) y **puede parecer colgada mucho rato alrededor del
70-80%** — sin ver los pasos por separado es fácil pensar que algo ha
fallado y cortarlo a media construcción (que sí deja el build a
medias de verdad). Yendo a mano ves exactamente en qué paso se para y
cuánto tarda cada uno; una vez que sabes que en tu máquina/VM funciona,
ya usa `./arrancar_todo.sh` tranquilo para las siguientes veces, que
reaprovecha todo lo ya construido y va rápido.

Si aun así el script te falla o se ve raro, o simplemente quieres los
comandos sueltos, sigue leyendo — es la misma secuencia que hace el
script, explicada paso a paso.

Hay **dos proyectos independientes** que se hablan por red (HTTP), no
comparten código ni contenedores:

1. **`Lab.Panda 2.4`** — la simulación (Webots + ROS 2): dos brazos Panda
   (Loader y Sorter) que mueven cubos por una cinta y los clasifican por
   color, más dos Raspberry Pi Pico con un LED RGB real cada una.
2. **`Taller_Administracion`** — servidor web (FastAPI + SQLite) de
   pedidos y almacén. Se entera de lo que fabrica el robot por HTTP; si
   está apagado, la simulación sigue funcionando igual, solo que nadie
   apunta la producción en ningún pedido.

Puedes arrancar solo el que necesites. Si vas a hacer producción de
verdad (que los pedidos se completen solos), hacen falta los dos.

## Los 3 contenedores Docker de todo el proyecto

| Contenedor | De qué proyecto | Qué es |
|---|---|---|
| `webots_panda_sim24` | Lab.Panda 2.4 | La simulación 3D (Webots) |
| `ros2_panda_dev24` | Lab.Panda 2.4 | ROS 2 y todos los nodos Python (brazos, cámaras, LEDs, panel de control) |
| `taller_admin_api` | Taller_Administracion | Servidor web de pedidos/almacén |

Compruébalo con `docker ps`. **Si ves otros nombres** (`ros2_dev`,
`webots_sim`, o los mismos sin el `24` al final) son restos de un
proyecto antiguo ya borrado (`Lab.Panda 2.3`) — párralos y bórralos antes
de seguir: `docker stop <nombre> && docker rm <nombre>`.

---

## Parte A — Taller_Administracion (lo más simple, sin Webots)

```bash
cd Taller_Administracion
docker compose up -d --build
```

Abre **http://localhost:8000**: sale la web de presentación "Virtual
Robotic" (misma que `Virtual_Robotic/index.html`, pero servida de verdad
por esta app). Pulsa "Entrar" con usuario de arranque `admin` / `admin` y
pasas directo al sistema real en `/panel` -- login único, no hay que
volver a entrar ahí.

Cualquier usuario `normal` que se cree también entra con la contraseña
maestra `1111` (pensada para dejar que alguien "juegue" sin darle una
cuenta); para `admin_sistema`/`admin_cliente` la maestra solo cuela con
`TALLER_DEV_MODE=true` (ya activo en el `docker-compose.yml` de este
proyecto).

**Para verlo desde el móvil** (misma Wi-Fi que este ordenador): usa la IP
local de este PC en vez de `localhost` —
```bash
hostname -I   # coge la que empiece por 192.168.x.x
```
y entra en `http://<esa_ip>:8000`.

Apagar: `docker compose down` en esta misma carpeta.

---

## Parte B — Lab.Panda 2.4 (la simulación)

### 0. Encender las dos Raspberry Pi Pico (antes de nada)

Hay una Pico por robot, cada una con su propio LED:

- **Sorter → Pico W de siempre** (`Rasberry_Pi_Pico/`, por Wi-Fi). Dale
  corriente; `main.py` arranca solo y se conecta al Wi-Fi, quedando a la
  escucha en `192.168.1.101:5001`. Es también la que lleva el **botón
  físico de parada de emergencia** (ver más abajo).
- **Loader → Pico nueva sin Wi-Fi** (`Rasberry_Pi_Pico_USB_Loader/`).
  Conéctala por USB a este ordenador (el `docker-compose.yml` ya sabe
  encontrarla sola por su ruta estable de `/dev/serial/by-id/`, no hace
  falta tocar nada salvo que la sustituyas por otra Pico distinta).

### 1. Autorizar pantalla y levantar los 2 contenedores de la simulación

```bash
xhost +local:docker
```
Necesario para que Webots (y cualquier ventana gráfica lanzada dentro del
contenedor) pueda dibujar en tu pantalla.

```bash
cd "Lab.Panda 2.4/.devcontainer"
docker compose up -d --build
```
**La primera vez tarda de verdad, y puede parecer colgado alrededor del
70-80% del progreso** — ahí es donde se descarga el paquete de Webots en
sí (pesa bastante) desde el repo de Cyberbotics; según la red puede
tardar varios minutos sin que sea un fallo. El `-d` solo afecta a cuando
los contenedores ya están construidos y arrancan — mientras se
**construye** la imagen, la terminal se queda mostrando el progreso y
hay que dejarla abierta hasta que termine. Si la cierras a media
construcción (con la X, o Ctrl+C), el build se corta y hay que
repetirlo: no rompe nada permanente, pero antes de reintentar comprueba
que no quedó nada a medias con `docker ps -a` y, si hay algo del
proyecto, `docker compose down` antes de relanzar.

Si el contenedor `webots` falla al arrancar quejándose de `/dev/dri`
(no hay GPU/aceleración 3D en esa máquina, típico en una VM sin
aceleración habilitada), comenta la línea `- /dev/dri:/dev/dri` del
`devices:` de `webots` en este `docker-compose.yml` — Webots cae a
renderizado por software, más lento pero funciona. Si además falla
`ros2_app` quejándose de `/dev/serial/by-id/...`, es el bloque `devices:`
de la Pico USB del Loader: coméntalo igual si no tienes esa Pico
conectada (por defecto va comentado; solo se activa a mano si tienes el
hardware real, ver el comentario justo encima en el propio fichero).

Esto crea/arranca `webots_panda_sim24` (carga directamente el mundo
`worlds/panda_industrial_cell.wbt`, la celda de dos robots) y
`ros2_panda_dev24`. Compruébalo con `docker ps`.

### 2. Compilar el paquete

**Obligatorio la primera vez** (clon nuevo, o si has borrado
`ros2_ws/install/`): ese directorio son artefactos regenerables y está
en `.gitignore` a propósito, así que un `git clone` no lo trae — sin
este paso, el `ros2 launch` del paso 3 falla porque el paquete no existe
todavía. Las veces siguientes, solo hace falta si has tocado código
Python.

```bash
docker exec -it ros2_panda_dev24 bash
cd /workspace
colcon build --packages-select panda_controller --symlink-install
```

### 3. Lanzar la celda completa (deja esta terminal abierta)

```bash
docker exec -it ros2_panda_dev24 bash
cd /workspace
ros2 launch panda_controller robot_launch_industrial_cell.py
```
Esto conecta de golpe los 6 "controladores" de la celda: el brazo
Loader, el brazo Sorter, las dos cámaras cenitales (una sobre la caja del
Loader, otra sobre el punto de recogida del Sorter), el
`WarehouseSupervisor` (un robot invisible que vigila la posición real de
los 3 cubos: rescata los que se caen y recicla los ya clasificados de
vuelta a la caja) y el `SorterShuttleSupervisor` (sesión 2026-09-10,
sin cuerpo físico igual que el anterior: la "bandeja" que centra en X y
gira el cubo que llega al punto de recogida del Sorter, para que no
aterrice descentrado — ver `Documentacion/carriles_completo.html`).
**Espera a ver 6 veces** `Controller successfully connected` en el log
antes de seguir.

**Esta terminal se queda abierta mientras trabajes.** Si la cierras (o
el propio comando se para por lo que sea), este paso deja de estar
"hecho" aunque los contenedores del paso 1 sigan arriba — y **nada** de
lo de abajo (LEDs, panel, producción) va a funcionar hasta que lo
relances. Para comprobar si ya lo tienes corriendo en OTRA terminal antes
de lanzarlo otra vez:
```bash
docker exec ros2_panda_dev24 bash -c "ps -ef | grep robot_launch_industrial_cell | grep -v grep"
```
Si no sale nada, no está corriendo — hazlo ahora antes de seguir con el
paso 4 o el 5.

### 4. Puentes de LED (uno por robot, cada uno en su terminal)

```bash
docker exec -it ros2_panda_dev24 bash
ros2 run panda_controller led_publisher       # Sorter, habla con la Pico por Wi-Fi
```
```bash
docker exec -it ros2_panda_dev24 bash
ros2 run panda_controller led_publisher_usb   # Loader, habla con la Pico por USB
```
Cada uno reenvía lo que le llega por su topic ROS (`/comando_led` para el
Sorter, `/comando_led_loader` para el Loader) al hardware real. Sin esto
el robot se sigue moviendo igual, solo que el LED físico no reacciona.

### 5. Panel de control manual (el sitio desde el que se maneja todo)

**Necesita el paso 3 corriendo de verdad** (ver el recuadro de arriba) —
si no, se queda 45s intentándolo y falla con `ERROR [...] Nadie se ha
suscrito tras 45s` y la ventana no llega a abrirse.

```bash
docker exec -it ros2_panda_dev24 bash
ros2 run panda_controller teleop_gui
```
Trae, en una sola ventana:
- Selector **LOADER / SORTER** para elegir qué brazo mueves a mano, sin
  reiniciar nada.
- **STOP / REARME** de toda la celda (pausa el movimiento en curso, NO lo
  aborta; sigue exactamente donde se quedó al rearmar) — funciona tanto
  si mueves el brazo a mano como si hay una producción automática en
  marcha.
- Pestaña **Movimiento**: jog manual del brazo elegido.
- Pestaña **Producción**: botón **"Lanzar lote"** (fabrica una cantidad
  de un solo producto elegido a mano) y botón **"Lanzar todos los
  pedidos pendientes"** (mira los pedidos reales de Taller_Administracion
  y fabrica lo que falte de los tres colores a la vez — necesita la Parte
  A arrancada).

Con esto solo, ya puedes producir sin tocar ninguna terminal más.

### 6. Producción automática a mano (alternativa a los botones del panel)

Solo si prefieres lanzarlo tú mismo por terminal en vez de usar los
botones del paso 5 (por ejemplo para dejarlo con parámetros concretos):

```bash
docker exec -it ros2_panda_dev24 bash
# Sorter: recoge lo que llegue a la cinta y lo clasifica, da igual el color
ros2 run panda_controller sorter_demo --ros-args -r __ns:=/sorter \
  -p robot_base_x:=0.0 -p robot_base_y:=1.00 -p robot_base_z:=0.74
```
```bash
docker exec -it ros2_panda_dev24 bash
# Loader: reparte los tres colores a la vez, 6 vueltas
ros2 run panda_controller loader_demo --ros-args -r __ns:=/loader \
  -p cycles:=6 -p led_topic:=/comando_led_loader -p led_topic_producto:=/comando_led_producto
# o un solo color (ej. 30 tuercas = verde):
ros2 run panda_controller loader_demo --ros-args -r __ns:=/loader \
  -p cycles:=30 -p only_color:=G -p led_topic:=/comando_led_loader -p led_topic_producto:=/comando_led_producto
```
**No lo lances si ya lo tienes corriendo desde el panel** (paso 5) — se
pelearían por el mismo brazo.

---

## Si algo se atasca (cubos amontonados en la cinta, los dos robots parados)

Puede pasar si un brazo se rinde con un cubo difícil tras varios
intentos: hasta el 2026-08-31 podía quedarse colgado tapando su propia
cámara para siempre (ya arreglado — se aparca solo). Si aun así algo se
lía, la forma limpia de resetear sin perder nada importante:

```bash
docker restart webots_panda_sim24   # repone los 3 cubos a su sitio de siempre
```
Espera unos 8 segundos y vuelve a hacer los pasos **3, 4, 5** (y 6 si
usabas producción por terminal). Los pedidos y el stock de
Taller_Administracion NO se tocan con esto — solo se resetea la
simulación.

---

## Apagar todo al terminar

**Atajo:** `./cerrar_todo.sh` (para los dos proyectos y avisa si algo se
resiste). Equivale a:

```bash
cd "Lab.Panda 2.4/.devcontainer" && docker compose down
cd ../../Taller_Administracion && docker compose down
```

---

## Parada de emergencia — referencia rápida

- **Botones físicos (uno por robot, sesión 2026-09-01)**: los dos paran
  la celda ENTERA (`/emergency_stop` es global, no por robot).
  - **Sorter**: GPIO16 a GND en la Pico W. Corta el LED al instante (no
    depende de la red) y avisa a ROS por el puerto 5002 (wifi). **El
    nodo puente (`button_listener`) va incluido en el paso 3** (desde el
    2026-08-31, arreglo real: el botón "no funcionaba" porque este nodo
    era un paso manual aparte, fácil de olvidar).
  - **Loader**: GPIO16 a GND en la Pico USB. Corta el LED al instante
    igual que el del Sorter, pero al no tener wifi avisa por el mismo
    cable USB (imprime `BOTON_PARADA`, que lee `led_publisher_usb` —
    paso 4). Sin ese paso corriendo, este botón tampoco avisa a ROS.

  Ambos publican en `/emergency_stop`, el mismo topic que ya escuchan
  `teleop_gui`, `loader_demo` y `sorter_demo`. Si un botón físico no
  corta el movimiento, lo primero que hay que comprobar es que el paso 3
  (`button_listener`, Sorter) o el paso 4 (`led_publisher_usb`, Loader)
  estén de verdad corriendo — sin ellos, tampoco hay puente.
- **Sensor de proximidad HC-SR04 (Loader, sesión 2026-09-11)**: tercera
  forma de que salte la parada, sin tocar ningún botón — si algo se acerca
  a menos de **10 cm** del sensor, la Pico del Loader dispara exactamente
  el mismo camino que su botón físico (`BOTON_PARADA` por USB). Cableado y
  umbral en `Documentacion/cableado_hcsr04.html`; el umbral es
  `DISTANCIA_MIN_CM` en `Rasberry_Pi_Pico_USB_Loader/main.py`. Si se
  autodispara durante la producción normal, es que el brazo pasa por
  delante del sensor: bajar el umbral o reubicarlo, no quitar el aviso.
- **Desde el panel**: el botón STOP/REARME de `teleop_gui` (paso 5) hace
  lo mismo sin tocar hardware — es la forma recomendada del día a día.

**Rearmar bien (trampa real, 2026-09-11):** mandar `REARME` al tópico del
LED (`/comando_led_loader` o `/comando_led`) solo apaga el parpadeo de esa
Pico; **no** limpia `/emergency_stop`, así que los dos robots siguen
parados en silencio y parece que el rearme "no funciona". Lo que de verdad
rearma es `Bool(false)` en `/emergency_stop` — que es justo lo que hace el
botón REARME del panel. A mano hacen falta las tres publicaciones:

```bash
ros2 topic pub --once /emergency_stop std_msgs/msg/Bool 'data: false'
ros2 topic pub --once /comando_led_loader std_msgs/msg/String "data: 'rearme'"
ros2 topic pub --once /comando_led std_msgs/msg/String "data: 'rearme'"
```

Durante la parada el **jog manual del panel sigue funcionando a propósito**:
el operario mueve el brazo (lo aparta del sensor, desatasca un cubo) y al
rearmar el robot retoma su trabajo donde lo dejó.
- **`estop_panel.py`**: una ventana aparte solo con STOP/REARME, de antes
  de que `teleop_gui` los integrara. Sigue funcionando pero ya es
  redundante si usas el panel completo; solo útil si quieres un botón de
  pánico en una ventana pequeña separada.

Protocolo de LED por cable/wifi: un carácter por Pico —
`R`/`G`/`B`/`0` (apagado), más `PARADA`/`REARME` para el parpadeo de
emergencia. Los dos Pico entienden el mismo protocolo, cada una en su
canal (Wi-Fi puerto 5001 / USB serie).

---

## Apéndice — demos y mundos antiguos (proyecto de un solo robot, ya NO se usan)

Todo esto es de antes de la celda industrial de dos robots. Sigue
existiendo en el código (por si hace falta comparar o recuperar algo),
pero **no forma parte del flujo actual** — no lo lances pensando que es
parte del arranque normal:

- Mundos: `panda_un_cubo.wbt`, `panda_bolas.wbt`
- Launch: `robot_launch.py` (un solo robot, sin namespaces)
- Demos: `stack_tower_demo`, `move_above_ball`, `pick_and_place`,
  `visit_balls`, `lift_ball`, `vision_lift_cube`,
  `best_color_repeat_lift`, `teleop_manual`
- Herramientas de desarrollo de la celda actual (tampoco parte del
  arranque normal, se usaron para construirla): `panda_two_arms_smoke_test.wbt`,
  `panda_sorter_grasp_test.wbt`, `grasp_yaw_test`, `sorter_hover_test`,
  `robot_launch_two_arms.py`, `robot_launch_sorter_grasp_test.py`.
