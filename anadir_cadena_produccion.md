# Cómo añadir una nueva línea de producción

_Última modificación: 2026-09-13 12:28_

Guía pensada para poder seguirla sin ser programador, paso a paso, para
montar una línea de producción más (la 2ª, la 3ª... hasta la 5ª, se
explica el límite al final). Si algún término técnico no se entiende,
se explica la primera vez que aparece.

## Qué es "una línea de producción" aquí

Todo el taller (los dos robots, la cinta, la simulación en Webots) vive
dentro de "contenedores" — son como cajas cerradas que llevan dentro un
ordenador completo ya preparado, para no tener que instalar nada a
mano cada vez. "Una línea" es el conjunto completo de esas cajas: una
simulación de Webots + el programa que mueve los robots. Añadir una
línea nueva es, literalmente, encender una segunda copia de esas
cajas — con el mismo diseño de taller, los mismos robots, el mismo
código — trabajando en paralelo con la primera, sin que se pisen entre
sí.

Lo importante: **todas las líneas comparten el mismo panel de
pedidos** (la web de Taller_Administracion). No hace falta duplicar
nada de eso ni configurar nada especial ahí — cada línea simplemente
avisa cuando termina una pieza, y el sistema de pedidos ya sabe
repartir el trabajo entre las líneas que haya encendidas en cada
momento (se explica cómo en el paso 4).

## 1. Copiar la plantilla ya hecha

Dentro de la carpeta `Lab.Panda 2.4` hay una carpeta llamada
`.devcontainer2` que es justo esto: la "receta" de cómo montar una
segunda línea, ya escrita y ya probada. Para crear la línea número N,
se copia entera con otro nombre:

```bash
cp -r "Lab.Panda 2.4/.devcontainer2" "Lab.Panda 2.4/.devcontainerN"
```

(sustituir la N por el número real, por ejemplo `.devcontainer3`).

## 2. Abrir esa copia y cambiar solo tres cosas

Dentro de la carpeta nueva hay un fichero llamado `docker-compose.yml`
— es la "receta" en sí, dice qué cajas hay que encender y cómo. Hay que
abrirlo con un editor de texto normal y cambiar únicamente estas tres
cosas (aparecen dos veces cada una, una por cada caja):

1. **El nombre de las cajas** (`container_name` y `hostname` en el
   fichero): tienen que llevar el número de línea, por ejemplo
   `webots_panda_sim24_linea3` y `ros2_panda_dev24_linea3`. Dos cajas
   nunca pueden tener el mismo nombre en el mismo ordenador, así que
   esto es obligatorio.
2. **El nombre de la red** (`panda_ros_net_lineaN` en el fichero): cada
   línea necesita su propia red interna para que los robots de una
   línea no "escuchen por error" a los de otra.
3. **El número de "dominio"** (`ROS_DOMAIN_ID` en el fichero): es el
   número que de verdad mantiene cada línea aislada de las demás —
   como si cada línea hablara en un canal de radio distinto y no
   pudiera oír a las otras aunque estén en la misma sala. Cada línea
   necesita un número que ninguna otra esté usando ya (la línea 1 usa
   uno, la línea 2 usa el `32`; la siguiente podría ser el `33`, etc.).

Todo lo demás del fichero se deja tal cual está copiado — apunta a
propósito a las mismas carpetas de código que la línea 1 (para no
tener que mantener copias sueltas del mismo taller: si se mejora algo
en el código, todas las líneas lo tienen a la vez).

**Un aviso sobre los "puertos"** (la forma en que una caja habla con el
mundo exterior, como el botón físico de parada de una Raspberry Pi
Pico): si esta línea nueva tiene su propio botón físico conectado, hay
que darle un puerto que ninguna otra línea esté usando. Si es solo
simulación (sin hardware físico propio, como la línea 2 de pruebas), se
deja sin esa parte — usar el mismo puerto que otra línea hace que el
arranque falle directamente, con un aviso claro de qué puerto choca.

## 3. Encender la línea nueva

Desde un terminal, dentro de la carpeta que se acaba de crear:

```bash
cd "Lab.Panda 2.4/.devcontainerN"
docker compose up -d --build
```

Esto tarda un rato la primera vez (está preparando las cajas). Cuando
termina, la línea ya está funcionando por dentro, aunque todavía no se
ve ninguna ventana en pantalla — eso es el siguiente paso.

## 4. Abrir el panel de control de esa línea y decirle qué número es

```bash
docker exec -it ros2_panda_dev24_lineaN bash
source /opt/ros/humble/setup.bash && source /workspace/install/setup.bash
ros2 run panda_controller teleop_gui
```

Esto abre la ventana del panel de control, igual que la de la línea 1
pero para esta línea nueva. Dentro de esa ventana, junto a los botones
de STOP y REARME, hay dos cosas que rellenar (las dos piden la clave
maestra `1111` para guardarse, así nadie las cambia sin querer):

- **"Nombre de la cadena"**: un texto libre, por ejemplo "Línea 3" —
  sirve solo para distinguir de un vistazo qué ventana es cuál cuando
  hay varias abiertas a la vez (se ve también en el título de la
  ventana).
- **"Nº Máquina"**: un número del desplegable que ninguna otra línea
  esté usando ya. Esto es lo que hace que el reparto de pedidos
  funcione bien: cada línea solo coge pedidos que estén "libres" (sin
  número asignado todavía) o que ya sean suyos — en cuanto una línea
  coge un pedido, las demás dejan de verlo como disponible, así que
  nunca se duplica el mismo trabajo por accidente. Esto ya funciona
  solo, no hay que tocar nada más.

## 5. Apagar una línea cuando no se necesite

```bash
cd "Lab.Panda 2.4/.devcontainerN"
docker compose down
```

Esto apaga y borra las cajas de esa línea, pero no borra nada de
código ni de la base de datos de pedidos — se puede volver a encender
en cualquier momento repitiendo el paso 3. Si esa línea se quedó con
algún pedido "suyo" a medias, ese pedido se queda reservado para ella
hasta que alguien lo libere a mano (hay un botón "Liberar" en la web de
pedidos, para quien tenga permiso de administrador del sistema).

## Cuántas líneas caben

El panel de control solo deja elegir un número de línea del 1 al 5 —
es un límite puesto a propósito para este taller, no una limitación
técnica de fondo. Si algún día hiciera falta una sexta línea, habría
que tocar una sola constante en el código (`MAX_MAQUINAS` en
`teleop_gui.py`) para ampliar la lista; el resto de este proceso (los
pasos 1 a 5) sería exactamente igual.
