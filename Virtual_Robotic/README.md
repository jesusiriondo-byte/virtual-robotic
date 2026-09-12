# Virtual Robotic

Página de presentación del proyecto Robotica, en tono de aficionado (no de
empresa de verdad): qué hace la celda, cómo está hecha por dentro y cómo
levantarla tú mismo. Un solo fichero HTML autocontenido, sin build ni
servidor: `index.html` (doble clic o `xdg-open index.html`).

Secciones: Origen, Cómo funciona (Loader/Sorter, cinta, panel de pedidos),
Cómo está hecho (Webots+ROS2, visión, almacén, y las Raspberry Pi Pico como
añadido opcional no crítico) y Juega con él (cómo levantarlo en Docker).

El botón "Producción" del menú es un cerrojo de cortesía, no autenticación
real: usuario `admin`, contraseña `admin` (hardcodeado en el propio HTML,
cualquiera que vea el código fuente la ve). Una vez dentro, enlaza a
`http://localhost:8000`, el panel de `Taller_Administracion` (tiene que
estar levantado con `docker compose up -d` en ese proyecto para que el
enlace funcione). El propio modal ya avisa de esto.

Nota: las credenciales del panel real (`Taller_Administracion`) son
independientes de este cerrojo de cortesía — ver
`../Taller_Administracion/README.md`.
