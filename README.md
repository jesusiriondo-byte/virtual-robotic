# Virtual Robotic

Proyecto personal de robótica, no una empresa de verdad: dos brazos
Franka Emika Panda simulados (Webots + ROS 2) que cogen, clasifican y
entregan cubos de colores por una cinta, más un panel web (FastAPI +
SQLite) que lleva pedidos y almacén como si fuera un taller real. Dos
Raspberry Pi Pico añaden hardware físico opcional (LED de producto y un
botón de parada de emergencia) — si no las tienes, el resto funciona
exactamente igual.

Construido junto con **[Claude Code](https://claude.com/claude-code)**
(Anthropic): el diseño, la arquitectura y buena parte del código de este
repositorio salieron de sesiones de trabajo con Claude.

## Probarlo

Con Docker instalado, dentro de `Taller_Administracion/`:

```bash
docker compose up -d --build
```

Abre **http://localhost:8000** — ahí está la web de presentación, y
entrando (usuario `admin` / contraseña `admin`) pasas al sistema real de
pedidos y almacén. No hace falta el resto del hardware para que funcione.

Para levantar también la simulación (Webots + ROS 2, dos contenedores
más) y el resto de detalles, sigue **[LANZAR_PROYECTO.md](LANZAR_PROYECTO.md)**.

## Estructura

- `Lab.Panda 2.4/` — la simulación: Webots + ROS 2, cinemática de los
  brazos, visión, control del agarre.
- `Taller_Administracion/` — el panel web (pedidos, almacén, roles de
  usuario) y la web de presentación (`app/static/landing.html`).
- `Virtual_Robotic/` — la misma web de presentación, en una copia suelta
  que no necesita Docker (ábrela con doble clic).
- `Rasberry_Pi_Pico/` y `Rasberry_Pi_Pico_USB_Loader/` — el código de las
  dos Pico (LED de producto y botón de parada). `wifi_config.py` y
  `webrepl_cfg.py` llevan una plantilla — pon tus propias claves si vas a
  usar hardware real.

Sin licencia explícita todavía: es un proyecto personal en marcha, no
pensado (de momento) para reutilización de terceros.
