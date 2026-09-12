# Taller_Administracion

Servidor web + base de datos de pedidos y almacén de la celda industrial. Es
un proyecto **separado** de la celda: solo se hablan por HTTP. Si está
apagado, la celda sigue funcionando, simplemente nadie apunta la producción.

Stack: FastAPI 0.115.0 + SQLAlchemy 2.0.35 + pydantic 2.9.2 + SQLite, servido
por uvicorn con `--reload`. API interactiva en `/docs`.

## Rutas servidas

- `/` — `app/static/landing.html`: la web de presentación "Virtual Robotic"
  (misma marca/tipografía que `../Virtual_Robotic/index.html`, pero aquí el
  login es de verdad porque esta misma app ya está corriendo).
- `/panel` — `app/static/panel.html`: el sistema real (pedidos, almacén,
  usuarios). Comparte sesión con `/` vía `sessionStorage` (`taller_token`,
  `taller_yo`): entra una vez desde la landing y ya no vuelve a pedir login.
- `/manual/{lanzar,taller,panda}` — sirve en crudo `LANZAR_PROYECTO.md`, el
  README de este proyecto y el resumen de `Lab.Panda 2.4`, enlazados desde la
  landing. Requiere el volumen `..:/workspace/repo:ro` del `docker-compose.yml`
  (repo padre montado solo lectura) — fuera de Docker cae solo a la ruta real
  en disco (`TALLER_REPO_DIR`, por defecto dos niveles por encima de `app/`).

## Arranque

```bash
docker compose up -d --build
```

Publica el puerto **8000** del host. `data/taller.db` se crea sola al
arrancar (dentro del volumen `./data`, ignorada por git). La primera vez que
arranca sin base de datos, el servidor siembra:

- Productos: `Tornillos` (R), `Tuercas` (G), `Arandelas` (B).
- Usuario `admin` (`admin_sistema`, contraseña `admin` por defecto —
  configurable con `TALLER_ADMIN_PASSWORD`).
- Configuración de almacén con `reparto_automatico = True`.

## Modelo

Cada **color** es la identidad del producto (`Producto.color` es único): la
celda solo distingue colores. R/G/B tienen cubo físico; Y/M/C/W son solo
colores de LED de producto (modo comodín), fabricados con los tres cubos
reales.

Tablas: `clientes`, `usuarios`, `audit_log`, `productos`, `cliente_productos`,
`pedidos`, `stock`, `movimientos_stock` (libro de solo añadir),
`eventos_produccion`, `configuracion_almacen` (fila única).

## Roles

- `admin_sistema`: todo. Es el único que gestiona productos, clientes,
  almacén, diagnóstico y auditoría.
- `admin_cliente`: gestiona su empresa (usuarios propios, catálogo asignado,
  pedidos de su empresa).
- `normal`: hace pedidos y ve los suyos.

La contraseña maestra (`TALLER_MASTER_PASSWORD`, por defecto `1111`) vale
siempre para entrar como usuario `normal` — pensado para que quien solo
quiera "jugar" con el panel no necesite que le demos de alta una cuenta.
Para `admin_sistema`/`admin_cliente` la maestra **solo** cuela si
`TALLER_DEV_MODE=true` (activo en nuestro `docker-compose.yml` local); sin
ese flag hace falta la contraseña real de la cuenta. El usuario `admin`
sembrado siempre tiene contraseña real (`admin` por defecto), así que
funciona con o sin `TALLER_DEV_MODE`.

## `POST /taller/cubo_clasificado`

Evento que manda la celda cada vez que un cubo llega a su caja. **Siempre**
responde `200` y suma 1 al stock del color. Si el reparto automático está
activo (el único interruptor de la regla), además aplica esa unidad a un
pedido pendiente: el indicado en `pedido_id` si sigue activo, si no el más
urgente y antiguo de ese producto. (En versiones anteriores de este
documento se decía que devolvía 404 si no había pedido pendiente: es falso,
siempre devuelve 200 con `pedido: null` en ese caso.)

**Nunca reparte stock a pedidos por iniciativa propia** fuera de esta regla:
`/almacen/repartir` es una decisión del operario.

## Endpoints

Ver `/docs` (Swagger generado por FastAPI) para el listado completo con
esquemas de entrada y salida.

## Tests automáticos (2026-09-11)

76 tests con pytest + `TestClient` de FastAPI, cubriendo login/roles,
productos, clientes/usuarios y sus límites de permiso, pedidos (alta,
alcance por rol, cancelar, **reprocesar**), almacén (`reparto_automatico`
como único gate, `cubo_clasificado`, reparto FIFO de `stock_disponible`,
ajustar/quitar stock), tiempos de proceso, diagnóstico y auditoría.

**Nunca tocan `data/taller.db`**: `tests/conftest.py` fija
`TALLER_DB_PATH` a un fichero temporal *antes* de importar nada de `app`
(la variable se lee en el momento del import en `database.py`), y cada
test arranca con una base de datos limpia y recién sembrada.

```bash
docker exec taller_admin_api pip install -r requirements-dev.txt   # una vez
docker exec -w /workspace taller_admin_api python -m pytest -v
```

Dos de los tests documentan regresiones reales de la sesión 2026-09-11
para que no se repitan sin que alguien se entere: el mensaje de error al
cancelar un pedido no pendiente (se rompió a `"c/p ya fabricadas"` en una
reescritura) y el comportamiento silencioso de `POST /usuarios` cuando un
`admin_cliente` intenta colar un usuario en otra empresa (no da 403:
ignora el `cliente_id` recibido y fuerza el suyo propio).
