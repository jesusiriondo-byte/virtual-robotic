import datetime
import os
from pathlib import Path

from fastapi import Depends, FastAPI, Header, HTTPException
from fastapi.responses import FileResponse
from fastapi.staticfiles import StaticFiles
from sqlalchemy import func
from sqlalchemy.orm import Session

from . import auth, models, schemas
from .database import Base, SessionLocal, engine, get_db

BASE_DIR = Path(__file__).resolve().parent

PALETA_COLORES = {
    "R": {"nombre": "Rojo", "rgb": "#e6423a", "fisico": True},
    "G": {"nombre": "Verde", "rgb": "#2da05a", "fisico": True},
    "B": {"nombre": "Azul", "rgb": "#3778c8", "fisico": True},
    "Y": {"nombre": "Amarillo", "rgb": "#e1c428", "fisico": False},
    "M": {"nombre": "Magenta", "rgb": "#be3cb4", "fisico": False},
    "C": {"nombre": "Cian", "rgb": "#2daba8", "fisico": False},
    "W": {"nombre": "Blanco", "rgb": "#e8e8e2", "fisico": False},
}
COLORES_VALIDOS = set(PALETA_COLORES)
SEED_PRODUCTOS = [("Tornillos", "R"), ("Tuercas", "G"), ("Arandelas", "B")]
ESTADOS_PEDIDO_INACTIVOS = ("completado", "cancelado")  # todo estado "ya no va a mas" entra aqui
TIPOS_EVENTO_PRODUCCION = {
    "led_encendido",
    "agarre_falso",
    "limite_alcance",
    "fallo_definitivo",
}


def _migrar_columnas_faltantes() -> None:
    """No hay Alembic y create_all no anade columnas a tablas ya existentes.

    Toda columna nueva que se anada a un modelo debe registrarse aqui.
    """
    with engine.connect() as conn:
        columnas_pedidos = {
            fila[1] for fila in conn.exec_driver_sql("PRAGMA table_info(pedidos)")
        }
        if "urgente" not in columnas_pedidos:
            conn.exec_driver_sql(
                "ALTER TABLE pedidos ADD COLUMN urgente BOOLEAN NOT NULL DEFAULT 0"
            )
        if "numero_maquina" not in columnas_pedidos:
            conn.exec_driver_sql(
                "ALTER TABLE pedidos ADD COLUMN numero_maquina INTEGER NOT NULL DEFAULT 0"
            )
        columnas_productos = {
            fila[1] for fila in conn.exec_driver_sql("PRAGMA table_info(productos)")
        }
        if "activo" not in columnas_productos:
            conn.exec_driver_sql(
                "ALTER TABLE productos ADD COLUMN activo BOOLEAN NOT NULL DEFAULT 1"
            )
        conn.exec_driver_sql(
            "CREATE UNIQUE INDEX IF NOT EXISTS ix_productos_color ON productos(color)"
        )
        conn.commit()


Base.metadata.create_all(bind=engine)
_migrar_columnas_faltantes()

app = FastAPI(title="Taller - Administracion")
app.mount("/static", StaticFiles(directory=str(BASE_DIR / "static")), name="static")


def _audit(
    db: Session,
    tabla: str,
    registro_id: int,
    accion: str,
    usuario_id: int | None,
    detalle: str | None = None,
) -> None:
    db.add(
        models.AuditLog(
            tabla=tabla,
            registro_id=registro_id,
            accion=accion,
            usuario_id=usuario_id,
            detalle=detalle,
        )
    )
    db.commit()


def _servir_desde_stock(db: Session, pedido: models.Pedido) -> None:
    stock_row = (
        db.query(models.Stock).filter(models.Stock.producto_id == pedido.producto_id).first()
    )
    if stock_row is None or stock_row.cantidad_actual <= 0:
        return
    restante = pedido.cantidad_pedida - pedido.cantidad_completada
    if restante <= 0:
        return
    servir = min(stock_row.cantidad_actual, restante)
    if servir <= 0:
        return
    stock_row.cantidad_actual -= servir
    stock_row.actualizado_en = datetime.datetime.utcnow()
    pedido.cantidad_completada += servir
    pedido.estado = (
        "completado" if pedido.cantidad_completada >= pedido.cantidad_pedida else "en_proceso"
    )
    db.add(
        models.MovimientoStock(
            producto_id=pedido.producto_id,
            tipo="salida",
            cantidad=servir,
            motivo="envio_pedido",
            pedido_id=pedido.id,
            usuario_id=pedido.usuario_id,
        )
    )
    db.commit()


def _con_stock_disponible(
    db: Session, pedidos: list[models.Pedido]
) -> list[models.Pedido]:
    """Rellena stock_disponible simulando el reparto FIFO real, sin tocar la base."""
    productos_ids = {p.producto_id for p in pedidos}
    asignado_por_pedido: dict[int, int] = {}
    for producto_id in productos_ids:
        stock_row = (
            db.query(models.Stock).filter(models.Stock.producto_id == producto_id).first()
        )
        disponible = stock_row.cantidad_actual if stock_row else 0
        activos = (
            db.query(models.Pedido)
            .filter(models.Pedido.producto_id == producto_id)
            .filter(models.Pedido.estado.notin_(ESTADOS_PEDIDO_INACTIVOS))
            .all()
        )
        activos.sort(key=lambda p: (not p.urgente, p.creado_en))
        for pedido in activos:
            restante = pedido.cantidad_pedida - pedido.cantidad_completada
            asignado = max(0, min(disponible, restante))
            asignado_por_pedido[pedido.id] = asignado
            disponible -= asignado
    for pedido in pedidos:
        pedido.stock_disponible = asignado_por_pedido.get(pedido.id, 0)
    _con_tiempos_de_proceso(db, pedidos)
    return pedidos


def _con_tiempos_de_proceso(db: Session, pedidos: list[models.Pedido]) -> None:
    """Rellena cuanto ha tardado cada lote, sin columnas nuevas en Pedido.

    Cada unidad que entra a un pedido deja su MovimientoStock de salida con
    fecha (ver cubo_clasificado / _servir_desde_stock), asi que el tiempo real
    sale de la primera y la ultima: 'proceso' es lo que tardo en completarse
    una vez empezo a recibir piezas, y 'total' incluye ademas la espera en cola
    desde que se hizo el pedido. Un lote servido de golpe desde almacen da
    proceso=0 a proposito: no se fabrico nada, salio de stock.

    Una sola consulta agregada para toda la lista (no una por pedido).
    """
    ids = [p.id for p in pedidos]
    for pedido in pedidos:
        pedido.segundos_proceso = None
        pedido.segundos_total = None
    if not ids:
        return
    filas = (
        db.query(
            models.MovimientoStock.pedido_id,
            func.min(models.MovimientoStock.fecha),
            func.max(models.MovimientoStock.fecha),
            func.count(models.MovimientoStock.id),
        )
        .filter(models.MovimientoStock.pedido_id.in_(ids))
        .filter(models.MovimientoStock.motivo == "envio_pedido")
        .group_by(models.MovimientoStock.pedido_id)
        .all()
    )
    por_pedido = {fila[0]: (fila[1], fila[2], fila[3]) for fila in filas}
    for pedido in pedidos:
        datos = por_pedido.get(pedido.id)
        if datos is None:
            continue
        primera, ultima, _n = datos
        if primera is None or ultima is None:
            continue
        pedido.segundos_proceso = max(0.0, (ultima - primera).total_seconds())
        if pedido.creado_en is not None:
            pedido.segundos_total = max(0.0, (ultima - pedido.creado_en).total_seconds())


@app.on_event("startup")
def sembrar_datos() -> None:
    db = SessionLocal()
    try:
        if db.query(models.Producto).count() == 0:
            for nombre, color in SEED_PRODUCTOS:
                db.add(models.Producto(nombre=nombre, color=color, activo=True))
            db.commit()
        if db.query(models.Usuario).count() == 0:
            admin_password = os.environ.get("TALLER_ADMIN_PASSWORD", "admin")
            db.add(
                models.Usuario(
                    username="admin",
                    nombre_completo="Administrador",
                    rol=auth.ROL_ADMIN_SISTEMA,
                    cliente_id=None,
                    password_hash=auth.hash_password(admin_password),
                    activo=True,
                )
            )
            db.commit()
        else:
            # Migracion de bases de datos ya sembradas antes de este cambio
            # (usuario "aladin" sin password_hash propia, solo entraba con
            # la maestra): lo pasamos a "admin" con contrasena real, una
            # vez, para que el endurecimiento de auth.check_password no
            # deje fuera al admin de siempre.
            legado = (
                db.query(models.Usuario)
                .filter(models.Usuario.username == "aladin")
                .first()
            )
            if legado is not None and legado.username != "admin":
                legado.username = "admin"
                if not legado.password_hash:
                    admin_password = os.environ.get("TALLER_ADMIN_PASSWORD", "admin")
                    legado.password_hash = auth.hash_password(admin_password)
                db.commit()
        if db.query(models.ConfiguracionAlmacen).filter_by(id=1).first() is None:
            db.add(models.ConfiguracionAlmacen(id=1, reparto_automatico=True))
            db.commit()
    finally:
        db.close()


# ---------------------------------------------------------------- generales


NO_CACHE = {"Cache-Control": "no-cache, no-store, must-revalidate"}
# Sin cache a proposito en todo lo servido aqui: el panel se toca a menudo
# y el navegador se quedaba con la version vieja -- el sintoma es siempre
# el mismo y despista mucho (un boton nuevo que "no aparece", o que no
# responde al clic) hasta que alguien se acuerda de recargar con Ctrl+Shift+R.

# Manuales del repo (proyecto padre, montado solo lectura -- ver
# docker-compose.yml) enlazados desde landing.html "Como esta hecho".
REPO_DIR = Path(os.environ.get("TALLER_REPO_DIR", BASE_DIR.parent.parent))
MANUALES = {
    "lanzar": REPO_DIR / "LANZAR_PROYECTO.md",
    "taller": REPO_DIR / "Taller_Administracion" / "README.md",
    "panda": REPO_DIR / "Lab.Panda 2.4" / "resumen_proyecto_panda.md",
}


@app.get("/")
def landing() -> FileResponse:
    return FileResponse(str(BASE_DIR / "static" / "landing.html"), headers=NO_CACHE)


@app.get("/panel")
def panel() -> FileResponse:
    return FileResponse(str(BASE_DIR / "static" / "panel.html"), headers=NO_CACHE)


@app.get("/manual/{nombre}")
def manual(nombre: str) -> FileResponse:
    ruta = MANUALES.get(nombre)
    if ruta is None or not ruta.exists():
        raise HTTPException(404, "Manual no encontrado.")
    return FileResponse(str(ruta), media_type="text/markdown; charset=utf-8")


@app.post("/login", response_model=schemas.LoginResponse)
def login(payload: schemas.LoginRequest, db: Session = Depends(get_db)):
    username_norm = payload.username.strip().lower()
    usuario = (
        db.query(models.Usuario).filter(models.Usuario.username == username_norm).first()
    )
    if usuario is None or not usuario.activo or not auth.check_password(payload.password, usuario):
        raise HTTPException(status_code=401, detail="Usuario o contrasena incorrectos.")
    token = auth.create_session(usuario.id)
    return schemas.LoginResponse(token=token, usuario=usuario)


@app.post("/logout")
def logout(x_session_token: str | None = Header(default=None)):
    if x_session_token:
        auth.destroy_session(x_session_token)
    return {"ok": True}


@app.get("/me", response_model=schemas.UsuarioOut)
def me(usuario: models.Usuario = Depends(auth.get_current_usuario)):
    return usuario


@app.get("/paleta_colores")
def paleta_colores():
    return PALETA_COLORES


# ---------------------------------------------------------------- productos


@app.get("/productos", response_model=list[schemas.ProductoOut])
def listar_productos(db: Session = Depends(get_db)):
    return db.query(models.Producto).order_by(models.Producto.id).all()


@app.post("/productos", response_model=schemas.ProductoOut)
def crear_producto(
    payload: schemas.ProductoCreate,
    usuario: models.Usuario = Depends(auth.require_roles(auth.ROL_ADMIN_SISTEMA)),
    db: Session = Depends(get_db),
):
    nombre = payload.nombre.strip()
    if not nombre:
        raise HTTPException(400, "El nombre no puede estar vacio.")
    color = payload.color.upper()
    if color not in COLORES_VALIDOS:
        raise HTTPException(400, f"Color '{color}' no valido.")
    if db.query(models.Producto).filter(models.Producto.nombre == nombre).first():
        raise HTTPException(400, f"Ya existe un producto llamado '{nombre}'.")
    if db.query(models.Producto).filter(models.Producto.color == color).first():
        raise HTTPException(400, f"El color '{color}' ya lo usa otro producto.")
    producto = models.Producto(nombre=nombre, color=color, activo=True)
    db.add(producto)
    db.commit()
    db.refresh(producto)
    _audit(db, "productos", producto.id, "alta", usuario.id, f"{nombre} ({color})")
    return producto


@app.patch("/productos/{producto_id}", response_model=schemas.ProductoOut)
def actualizar_producto(
    producto_id: int,
    payload: schemas.ProductoUpdate,
    usuario: models.Usuario = Depends(auth.require_roles(auth.ROL_ADMIN_SISTEMA)),
    db: Session = Depends(get_db),
):
    producto = db.query(models.Producto).filter(models.Producto.id == producto_id).first()
    if producto is None:
        raise HTTPException(404, "Producto no encontrado.")
    cambios = payload.model_dump(exclude_unset=True)
    if not cambios:
        raise HTTPException(400, "No se ha indicado ningun cambio.")
    if "nombre" in cambios:
        nombre = (cambios["nombre"] or "").strip()
        if not nombre:
            raise HTTPException(400, "El nombre no puede estar vacio.")
        existente = (
            db.query(models.Producto)
            .filter(models.Producto.nombre == nombre, models.Producto.id != producto_id)
            .first()
        )
        if existente:
            raise HTTPException(400, f"Ya existe un producto llamado '{nombre}'.")
        producto.nombre = nombre
    if "color" in cambios:
        color = (cambios["color"] or "").upper()
        if color not in COLORES_VALIDOS:
            raise HTTPException(400, f"Color '{color}' no valido.")
        existente = (
            db.query(models.Producto)
            .filter(models.Producto.color == color, models.Producto.id != producto_id)
            .first()
        )
        if existente:
            raise HTTPException(400, f"El color '{color}' ya lo usa otro producto.")
        producto.color = color
    if "activo" in cambios:
        producto.activo = cambios["activo"]
    db.commit()
    db.refresh(producto)
    _audit(db, "productos", producto.id, "modificacion", usuario.id, str(cambios))
    return producto


# ----------------------------------------------------------------- usuarios


@app.get("/usuarios", response_model=list[schemas.UsuarioOut])
def listar_usuarios(
    usuario: models.Usuario = Depends(
        auth.require_roles(auth.ROL_ADMIN_SISTEMA, auth.ROL_ADMIN_CLIENTE)
    ),
    db: Session = Depends(get_db),
):
    query = db.query(models.Usuario)
    if usuario.rol == auth.ROL_ADMIN_CLIENTE:
        query = query.filter(models.Usuario.cliente_id == usuario.cliente_id)
    return query.order_by(models.Usuario.id).all()


@app.post("/usuarios", response_model=schemas.UsuarioOut)
def crear_usuario(
    payload: schemas.UsuarioCreate,
    usuario: models.Usuario = Depends(
        auth.require_roles(auth.ROL_ADMIN_SISTEMA, auth.ROL_ADMIN_CLIENTE)
    ),
    db: Session = Depends(get_db),
):
    if payload.rol not in auth.ROLES_VALIDOS:
        raise HTTPException(400, f"Rol '{payload.rol}' no valido.")
    cliente_id = payload.cliente_id
    if usuario.rol == auth.ROL_ADMIN_CLIENTE:
        if payload.rol == auth.ROL_ADMIN_SISTEMA:
            raise HTTPException(403, "No puedes crear un administrador del sistema.")
        cliente_id = usuario.cliente_id
    if payload.rol != auth.ROL_ADMIN_SISTEMA and cliente_id is None:
        raise HTTPException(400, "Este rol necesita un cliente.")
    # El login busca por username.strip().lower(): se guarda ya normalizado
    # para que un usuario creado con mayusculas pueda entrar.
    username = payload.username.strip().lower()
    if db.query(models.Usuario).filter(models.Usuario.username == username).first():
        raise HTTPException(409, f"El usuario '{username}' ya existe.")
    nuevo = models.Usuario(
        username=username,
        nombre_completo=payload.nombre_completo,
        rol=payload.rol,
        cliente_id=cliente_id,
        sucursal=payload.sucursal,
        activo=True,
        password_hash=auth.hash_password(payload.password) if payload.password else None,
        creado_por_id=usuario.id,
    )
    db.add(nuevo)
    db.commit()
    db.refresh(nuevo)
    _audit(db, "usuarios", nuevo.id, "alta", usuario.id, username)
    return nuevo


@app.patch("/usuarios/{usuario_id}", response_model=schemas.UsuarioOut)
def actualizar_usuario(
    usuario_id: int,
    payload: schemas.UsuarioUpdate,
    usuario: models.Usuario = Depends(
        auth.require_roles(auth.ROL_ADMIN_SISTEMA, auth.ROL_ADMIN_CLIENTE)
    ),
    db: Session = Depends(get_db),
):
    objetivo = db.query(models.Usuario).filter(models.Usuario.id == usuario_id).first()
    if objetivo is None:
        raise HTTPException(404, "Usuario no encontrado.")
    if not auth.puede_gestionar_cliente(usuario, objetivo.cliente_id):
        raise HTTPException(403, "No tienes permiso para gestionar este usuario.")
    cambios = payload.model_dump(exclude_unset=True)
    if "rol" in cambios:
        if cambios["rol"] not in auth.ROLES_VALIDOS:
            raise HTTPException(400, f"Rol '{cambios['rol']}' no valido.")
        if usuario.rol == auth.ROL_ADMIN_CLIENTE and cambios["rol"] == auth.ROL_ADMIN_SISTEMA:
            raise HTTPException(403, "No puedes ascender a administrador del sistema.")
        objetivo.rol = cambios["rol"]
    if "nombre_completo" in cambios:
        objetivo.nombre_completo = cambios["nombre_completo"]
    if "sucursal" in cambios:
        objetivo.sucursal = cambios["sucursal"]
    if cambios.get("password"):
        objetivo.password_hash = auth.hash_password(cambios["password"])
    dado_de_baja = False
    if "activo" in cambios:
        objetivo.activo = cambios["activo"]
        dado_de_baja = cambios["activo"] is False
    objetivo.modificado_en = datetime.datetime.utcnow()
    objetivo.modificado_por_id = usuario.id
    db.commit()
    db.refresh(objetivo)
    _audit(
        db,
        "usuarios",
        objetivo.id,
        "baja" if dado_de_baja else "modificacion",
        usuario.id,
        str(cambios),
    )
    return objetivo


# ----------------------------------------------------------------- clientes


@app.get("/clientes", response_model=list[schemas.ClienteOut])
def listar_clientes(
    usuario: models.Usuario = Depends(auth.get_current_usuario),
    db: Session = Depends(get_db),
):
    query = db.query(models.Cliente)
    if usuario.rol != auth.ROL_ADMIN_SISTEMA:
        query = query.filter(models.Cliente.id == usuario.cliente_id)
    return query.order_by(models.Cliente.id).all()


@app.post("/clientes", response_model=schemas.ClienteOut)
def crear_cliente(
    payload: schemas.ClienteCreate,
    usuario: models.Usuario = Depends(auth.require_roles(auth.ROL_ADMIN_SISTEMA)),
    db: Session = Depends(get_db),
):
    # El login busca por username.strip().lower(): se guarda ya normalizado.
    primer_username = payload.primer_usuario.username.strip().lower()
    if db.query(models.Usuario).filter(models.Usuario.username == primer_username).first():
        raise HTTPException(409, f"El usuario '{primer_username}' ya existe.")
    cliente = models.Cliente(
        razon_social=payload.razon_social,
        cif=payload.cif,
        activo=True,
        creado_por_id=usuario.id,
    )
    db.add(cliente)
    db.commit()
    db.refresh(cliente)
    _audit(db, "clientes", cliente.id, "alta", usuario.id, payload.razon_social)

    primer_usuario = models.Usuario(
        username=primer_username,
        nombre_completo=payload.primer_usuario.nombre_completo,
        rol=auth.ROL_ADMIN_CLIENTE,
        cliente_id=cliente.id,
        activo=True,
        password_hash=(
            auth.hash_password(payload.primer_usuario.password)
            if payload.primer_usuario.password
            else None
        ),
        creado_por_id=usuario.id,
    )
    db.add(primer_usuario)
    db.commit()
    db.refresh(primer_usuario)
    _audit(db, "usuarios", primer_usuario.id, "alta", usuario.id, primer_username)
    return cliente


@app.patch("/clientes/{cliente_id}", response_model=schemas.ClienteOut)
def actualizar_cliente(
    cliente_id: int,
    payload: schemas.ClienteUpdate,
    usuario: models.Usuario = Depends(auth.get_current_usuario),
    db: Session = Depends(get_db),
):
    cliente = db.query(models.Cliente).filter(models.Cliente.id == cliente_id).first()
    if cliente is None:
        raise HTTPException(404, "Cliente no encontrado.")
    if not auth.puede_gestionar_cliente(usuario, cliente_id):
        raise HTTPException(403, "No tienes permiso para gestionar este cliente.")
    cambios = payload.model_dump(exclude_unset=True)
    if "activo" in cambios and usuario.rol != auth.ROL_ADMIN_SISTEMA:
        raise HTTPException(403, "Solo un administrador del sistema puede cambiar el alta/baja.")
    if "razon_social" in cambios:
        cliente.razon_social = cambios["razon_social"]
    if "cif" in cambios:
        cliente.cif = cambios["cif"]
    if "activo" in cambios:
        cliente.activo = cambios["activo"]
    cliente.modificado_en = datetime.datetime.utcnow()
    cliente.modificado_por_id = usuario.id
    db.commit()
    db.refresh(cliente)
    _audit(db, "clientes", cliente.id, "modificacion", usuario.id, str(cambios))
    return cliente


@app.get("/clientes/{cliente_id}/productos", response_model=list[schemas.ProductoOut])
def productos_de_cliente(
    cliente_id: int,
    usuario: models.Usuario = Depends(auth.get_current_usuario),
    db: Session = Depends(get_db),
):
    if not (
        auth.puede_gestionar_cliente(usuario, cliente_id) or usuario.cliente_id == cliente_id
    ):
        raise HTTPException(403, "No tienes permiso para ver estos productos.")
    asignaciones = (
        db.query(models.ClienteProducto)
        .filter(models.ClienteProducto.cliente_id == cliente_id)
        .all()
    )
    return [a.producto for a in asignaciones]


@app.post("/clientes/{cliente_id}/productos", response_model=list[schemas.ProductoOut])
def asignar_producto(
    cliente_id: int,
    payload: schemas.ClienteProductoAsignar,
    usuario: models.Usuario = Depends(
        auth.require_roles(auth.ROL_ADMIN_SISTEMA, auth.ROL_ADMIN_CLIENTE)
    ),
    db: Session = Depends(get_db),
):
    if not auth.puede_gestionar_cliente(usuario, cliente_id):
        raise HTTPException(403, "No tienes permiso para gestionar este cliente.")
    producto = db.query(models.Producto).filter(models.Producto.id == payload.producto_id).first()
    if producto is None or not producto.activo:
        raise HTTPException(400, "El producto no existe o esta de baja.")
    existente = (
        db.query(models.ClienteProducto)
        .filter_by(cliente_id=cliente_id, producto_id=payload.producto_id)
        .first()
    )
    if existente is None:
        db.add(
            models.ClienteProducto(
                cliente_id=cliente_id,
                producto_id=payload.producto_id,
                asignado_por_id=usuario.id,
            )
        )
        db.commit()
    asignaciones = (
        db.query(models.ClienteProducto)
        .filter(models.ClienteProducto.cliente_id == cliente_id)
        .all()
    )
    return [a.producto for a in asignaciones]


@app.delete("/clientes/{cliente_id}/productos/{producto_id}", response_model=list[schemas.ProductoOut])
def quitar_producto(
    cliente_id: int,
    producto_id: int,
    usuario: models.Usuario = Depends(
        auth.require_roles(auth.ROL_ADMIN_SISTEMA, auth.ROL_ADMIN_CLIENTE)
    ),
    db: Session = Depends(get_db),
):
    if not auth.puede_gestionar_cliente(usuario, cliente_id):
        raise HTTPException(403, "No tienes permiso para gestionar este cliente.")
    db.query(models.ClienteProducto).filter_by(
        cliente_id=cliente_id, producto_id=producto_id
    ).delete()
    db.commit()
    asignaciones = (
        db.query(models.ClienteProducto)
        .filter(models.ClienteProducto.cliente_id == cliente_id)
        .all()
    )
    return [a.producto for a in asignaciones]


# ------------------------------------------------------------------ pedidos


@app.post("/pedidos", response_model=schemas.PedidoOut)
def crear_pedido(
    payload: schemas.PedidoCreate,
    usuario: models.Usuario = Depends(
        auth.require_roles(auth.ROL_ADMIN_CLIENTE, auth.ROL_NORMAL)
    ),
    db: Session = Depends(get_db),
):
    producto = db.query(models.Producto).filter(models.Producto.id == payload.producto_id).first()
    if producto is None or not producto.activo:
        raise HTTPException(400, "El producto no existe o esta de baja.")
    if payload.cantidad_pedida <= 0:
        raise HTTPException(400, "La cantidad debe ser mayor que 0.")
    asignado = (
        db.query(models.ClienteProducto)
        .filter_by(cliente_id=usuario.cliente_id, producto_id=producto.id)
        .first()
    )
    if asignado is None:
        raise HTTPException(403, "Este producto no esta asignado a tu empresa.")
    pedido = models.Pedido(
        producto_id=producto.id,
        cliente_id=usuario.cliente_id,
        usuario_id=usuario.id,
        cantidad_pedida=payload.cantidad_pedida,
        cantidad_completada=0,
        estado="pendiente",
        urgente=False,
    )
    db.add(pedido)
    db.commit()
    db.refresh(pedido)
    config = db.query(models.ConfiguracionAlmacen).filter_by(id=1).first()
    if config and config.reparto_automatico:
        _servir_desde_stock(db, pedido)
        db.refresh(pedido)
    _audit(db, "pedidos", pedido.id, "alta", usuario.id, f"{producto.nombre} x{payload.cantidad_pedida}")
    return _con_stock_disponible(db, [pedido])[0]


@app.get("/pedidos", response_model=list[schemas.PedidoOut])
def listar_pedidos(
    usuario: models.Usuario = Depends(auth.get_current_usuario),
    db: Session = Depends(get_db),
):
    query = db.query(models.Pedido)
    if usuario.rol == auth.ROL_NORMAL:
        query = query.filter(models.Pedido.usuario_id == usuario.id)
    elif usuario.rol == auth.ROL_ADMIN_CLIENTE:
        query = query.filter(models.Pedido.cliente_id == usuario.cliente_id)
    pedidos = query.order_by(models.Pedido.creado_en.asc()).all()
    return _con_stock_disponible(db, pedidos)


@app.get("/pedidos/{pedido_id}", response_model=schemas.PedidoOut)
def obtener_pedido(
    pedido_id: int,
    usuario: models.Usuario = Depends(auth.get_current_usuario),
    db: Session = Depends(get_db),
):
    pedido = db.query(models.Pedido).filter(models.Pedido.id == pedido_id).first()
    if pedido is None:
        raise HTTPException(404, "Pedido no encontrado.")
    if usuario.rol == auth.ROL_NORMAL and pedido.usuario_id != usuario.id:
        raise HTTPException(403, "No tienes permiso para ver este pedido.")
    if usuario.rol == auth.ROL_ADMIN_CLIENTE and pedido.cliente_id != usuario.cliente_id:
        raise HTTPException(403, "No tienes permiso para ver este pedido.")
    return _con_stock_disponible(db, [pedido])[0]


@app.patch("/pedidos/{pedido_id}", response_model=schemas.PedidoOut)
def actualizar_pedido(
    pedido_id: int,
    payload: schemas.PedidoUpdate,
    usuario: models.Usuario = Depends(
        auth.require_roles(auth.ROL_ADMIN_SISTEMA, auth.ROL_ADMIN_CLIENTE)
    ),
    db: Session = Depends(get_db),
):
    pedido = db.query(models.Pedido).filter(models.Pedido.id == pedido_id).first()
    if pedido is None:
        raise HTTPException(404, "Pedido no encontrado.")
    if not auth.puede_gestionar_cliente(usuario, pedido.cliente_id):
        raise HTTPException(403, "No tienes permiso para gestionar este pedido.")
    pedido.urgente = payload.urgente
    db.commit()
    db.refresh(pedido)
    return _con_stock_disponible(db, [pedido])[0]


@app.post("/pedidos/{pedido_id}/cancelar", response_model=schemas.PedidoOut)
def cancelar_pedido(
    pedido_id: int,
    usuario: models.Usuario = Depends(
        auth.require_roles(auth.ROL_ADMIN_SISTEMA, auth.ROL_ADMIN_CLIENTE)
    ),
    db: Session = Depends(get_db),
):
    pedido = db.query(models.Pedido).filter(models.Pedido.id == pedido_id).first()
    if pedido is None:
        raise HTTPException(404, "Pedido no encontrado.")
    if not auth.puede_gestionar_cliente(usuario, pedido.cliente_id):
        raise HTTPException(403, "No tienes permiso para gestionar este pedido.")
    if pedido.estado != "pendiente":
        raise HTTPException(
            400,
            f"Solo se puede cancelar un pedido 'pendiente' -- este esta '{pedido.estado}' "
            f"({pedido.cantidad_completada}/{pedido.cantidad_pedida} ya fabricadas).",
        )
    pedido.estado = "cancelado"
    db.commit()
    db.refresh(pedido)
    _audit(db, "pedidos", pedido.id, "baja", usuario.id, "cancelado")
    return _con_stock_disponible(db, [pedido])[0]


@app.post("/pedidos/{pedido_id}/reclamar", response_model=schemas.PedidoOut)
def reclamar_pedido(
    pedido_id: int,
    payload: schemas.PedidoReclamar,
    usuario: models.Usuario = Depends(auth.require_roles(auth.ROL_ADMIN_SISTEMA)),
    db: Session = Depends(get_db),
):
    """Marca un pedido como asignado a una maquina/celda concreta -- ver
    Pedido.numero_maquina y Documentacion/analisis_ampliacion_taller.md.
    Con una sola celda (estado actual del proyecto) siempre tiene exito;
    la comprobacion de abajo es la que hace que, con dos o mas, no puedan
    quedarse el mismo pedido las dos a la vez."""
    pedido = db.query(models.Pedido).filter(models.Pedido.id == pedido_id).first()
    if pedido is None:
        raise HTTPException(404, "Pedido no encontrado.")
    if payload.numero_maquina <= 0:
        raise HTTPException(400, "numero_maquina tiene que ser mayor que 0 (0 significa libre).")
    if payload.forzar:
        pedido.numero_maquina = payload.numero_maquina
        db.commit()
    else:
        # UPDATE condicionado (no "leo, comparo en Python, escribo" en dos
        # pasos): solo escribe si sigue libre (0) o ya era mio. Es lo que
        # evita que dos maquinas preguntando casi a la vez se queden las
        # dos con el mismo pedido.
        filas = (
            db.query(models.Pedido)
            .filter(
                models.Pedido.id == pedido_id,
                models.Pedido.numero_maquina.in_([0, payload.numero_maquina]),
            )
            .update({"numero_maquina": payload.numero_maquina})
        )
        db.commit()
        if filas == 0:
            db.refresh(pedido)
            raise HTTPException(
                409, f"Este pedido ya esta asignado a la maquina nº {pedido.numero_maquina}."
            )
    db.refresh(pedido)
    return _con_stock_disponible(db, [pedido])[0]


@app.post("/pedidos/{pedido_id}/liberar", response_model=schemas.PedidoOut)
def liberar_pedido(
    pedido_id: int,
    usuario: models.Usuario = Depends(auth.require_roles(auth.ROL_ADMIN_SISTEMA)),
    db: Session = Depends(get_db),
):
    """Vuelve a poner numero_maquina a 0 (libre) -- para el caso de una
    maquina que se cayo con un pedido reclamado a su nombre y necesita
    liberarse a mano. Sin condicion: administracion manda."""
    pedido = db.query(models.Pedido).filter(models.Pedido.id == pedido_id).first()
    if pedido is None:
        raise HTTPException(404, "Pedido no encontrado.")
    pedido.numero_maquina = 0
    db.commit()
    db.refresh(pedido)
    return _con_stock_disponible(db, [pedido])[0]


@app.post("/pedidos/{pedido_id}/reprocesar", response_model=schemas.PedidoOut)
def reprocesar_pedido(
    pedido_id: int,
    usuario: models.Usuario = Depends(
        auth.require_roles(auth.ROL_ADMIN_SISTEMA, auth.ROL_ADMIN_CLIENTE)
    ),
    db: Session = Depends(get_db),
):
    """Vuelve a mandar al taller un lote ya completado.

    Crea un pedido NUEVO identico (mismo producto, cliente, solicitante y
    cantidad) en vez de reabrir el viejo: asi el historico de que aquel lote
    SE FABRICO de verdad no se pierde, y el reproceso se puede cancelar como
    cualquier otro pedido si era un error. Pasa por las mismas reglas que un
    pedido normal (producto activo, asignado al cliente, y reparto_automatico
    como unico gate para servirlo desde stock) -- si hay stock suficiente en
    almacen y el reparto automatico esta activo, se sirve de ahi en vez de
    fabricarlo, igual que cualquier pedido nuevo.
    """
    original = db.query(models.Pedido).filter(models.Pedido.id == pedido_id).first()
    if original is None:
        raise HTTPException(404, "Pedido no encontrado.")
    if not auth.puede_gestionar_cliente(usuario, original.cliente_id):
        raise HTTPException(403, "No tienes permiso para gestionar este pedido.")
    if original.estado != "completado":
        raise HTTPException(
            400,
            f"Solo se puede volver a procesar un pedido ya completado -- "
            f"este esta '{original.estado}'.",
        )
    producto = (
        db.query(models.Producto).filter(models.Producto.id == original.producto_id).first()
    )
    if producto is None or not producto.activo:
        raise HTTPException(
            400, "El producto esta de baja: dalo de alta antes de volver a fabricarlo."
        )
    asignado = (
        db.query(models.ClienteProducto)
        .filter_by(cliente_id=original.cliente_id, producto_id=producto.id)
        .first()
    )
    if asignado is None:
        raise HTTPException(
            403, f"'{producto.nombre}' ya no esta asignado al catalogo de esa empresa."
        )

    nuevo = models.Pedido(
        producto_id=original.producto_id,
        cliente_id=original.cliente_id,
        usuario_id=original.usuario_id,
        cantidad_pedida=original.cantidad_pedida,
        cantidad_completada=0,
        estado="pendiente",
        urgente=False,
    )
    db.add(nuevo)
    db.commit()
    db.refresh(nuevo)
    config = db.query(models.ConfiguracionAlmacen).filter_by(id=1).first()
    if config and config.reparto_automatico:
        _servir_desde_stock(db, nuevo)
        db.refresh(nuevo)
    _audit(
        db,
        "pedidos",
        nuevo.id,
        "alta",
        usuario.id,
        f"reproceso del pedido {original.id} -- {producto.nombre} x{nuevo.cantidad_pedida}",
    )
    return _con_stock_disponible(db, [nuevo])[0]


# ------------------------------------------------------------- celda -> taller


@app.post("/taller/cubo_clasificado", response_model=schemas.CuboClasificadoResultado)
def cubo_clasificado(payload: schemas.CuboClasificado, db: Session = Depends(get_db)):
    color = payload.color.upper()
    if color not in COLORES_VALIDOS:
        raise HTTPException(400, f"Color '{color}' no valido.")
    producto = db.query(models.Producto).filter(models.Producto.color == color).first()
    if producto is None:
        raise HTTPException(404, f"No hay producto para el color '{color}'.")

    stock_row = db.query(models.Stock).filter_by(producto_id=producto.id).first()
    if stock_row is None:
        stock_row = models.Stock(producto_id=producto.id, cantidad_actual=0)
        db.add(stock_row)
        db.flush()
    stock_row.cantidad_actual += 1
    stock_row.actualizado_en = datetime.datetime.utcnow()
    db.add(
        models.MovimientoStock(
            producto_id=producto.id, tipo="entrada", cantidad=1, motivo="produccion"
        )
    )
    db.commit()

    pedido_resultado = None
    config = db.query(models.ConfiguracionAlmacen).filter_by(id=1).first()
    if config and config.reparto_automatico:
        candidato = None
        if payload.pedido_id is not None:
            candidato = (
                db.query(models.Pedido)
                .filter(models.Pedido.id == payload.pedido_id)
                .filter(models.Pedido.producto_id == producto.id)
                .filter(models.Pedido.estado.notin_(ESTADOS_PEDIDO_INACTIVOS))
                .first()
            )
        if candidato is None:
            candidato = (
                db.query(models.Pedido)
                .filter(models.Pedido.producto_id == producto.id)
                .filter(models.Pedido.estado.notin_(ESTADOS_PEDIDO_INACTIVOS))
                .order_by(models.Pedido.urgente.desc(), models.Pedido.creado_en.asc())
                .first()
            )
        if candidato is not None:
            stock_row.cantidad_actual -= 1
            stock_row.actualizado_en = datetime.datetime.utcnow()
            candidato.cantidad_completada += 1
            candidato.estado = (
                "completado"
                if candidato.cantidad_completada >= candidato.cantidad_pedida
                else "en_proceso"
            )
            db.add(
                models.MovimientoStock(
                    producto_id=producto.id,
                    tipo="salida",
                    cantidad=1,
                    motivo="envio_pedido",
                    pedido_id=candidato.id,
                )
            )
            db.commit()
            db.refresh(candidato)
            pedido_resultado = _con_stock_disponible(db, [candidato])[0]

    db.refresh(stock_row)
    return schemas.CuboClasificadoResultado(
        color=color, stock_actual=stock_row.cantidad_actual, pedido=pedido_resultado
    )


@app.post("/taller/evento_produccion", response_model=schemas.EventoProduccionOut)
def evento_produccion(payload: schemas.EventoProduccionCreate, db: Session = Depends(get_db)):
    robot = (payload.robot or "").strip().lower() or "desconocido"
    if payload.tipo not in TIPOS_EVENTO_PRODUCCION:
        raise HTTPException(400, f"Tipo de evento '{payload.tipo}' no valido.")
    evento = models.EventoProduccion(robot=robot, color=payload.color, tipo=payload.tipo)
    db.add(evento)
    db.commit()
    db.refresh(evento)
    return evento


@app.get("/taller/diagnostico", response_model=schemas.DiagnosticoOut)
def diagnostico(
    ventana_minutos: int = 60,
    usuario: models.Usuario = Depends(auth.require_roles(auth.ROL_ADMIN_SISTEMA)),
    db: Session = Depends(get_db),
):
    desde = datetime.datetime.utcnow() - datetime.timedelta(minutes=ventana_minutos)
    productos = db.query(models.Producto).order_by(models.Producto.id).all()
    por_color = []
    for producto in productos:
        eventos_ventana = (
            db.query(models.EventoProduccion)
            .filter(models.EventoProduccion.color == producto.color)
            .filter(models.EventoProduccion.fecha >= desde)
            .all()
        )
        led_loader = sum(
            1 for e in eventos_ventana if e.tipo == "led_encendido" and e.robot == "loader"
        )
        led_sorter = sum(
            1 for e in eventos_ventana if e.tipo == "led_encendido" and e.robot == "sorter"
        )
        agarre_falso = sum(1 for e in eventos_ventana if e.tipo == "agarre_falso")
        limite_alcance = sum(1 for e in eventos_ventana if e.tipo == "limite_alcance")
        fallo_definitivo = sum(1 for e in eventos_ventana if e.tipo == "fallo_definitivo")
        piezas_reales = (
            db.query(models.MovimientoStock)
            .filter(models.MovimientoStock.producto_id == producto.id)
            .filter(models.MovimientoStock.motivo == "produccion")
            .filter(models.MovimientoStock.fecha >= desde)
            .count()
        )
        piezas_reales_total = (
            db.query(models.MovimientoStock)
            .filter(models.MovimientoStock.producto_id == producto.id)
            .filter(models.MovimientoStock.motivo == "produccion")
            .count()
        )
        if fallo_definitivo > 0:
            estado = "critico"
        elif limite_alcance > 0 or agarre_falso > piezas_reales:
            estado = "atencion"
        else:
            estado = "ok"
        por_color.append(
            schemas.DiagnosticoColor(
                color=producto.color,
                producto_nombre=producto.nombre,
                led_loader=led_loader,
                led_sorter=led_sorter,
                agarre_falso=agarre_falso,
                limite_alcance=limite_alcance,
                fallo_definitivo=fallo_definitivo,
                piezas_reales=piezas_reales,
                piezas_reales_total=piezas_reales_total,
                estado=estado,
            )
        )
    eventos_recientes = (
        db.query(models.EventoProduccion)
        .order_by(models.EventoProduccion.fecha.desc())
        .limit(50)
        .all()
    )
    return schemas.DiagnosticoOut(
        ventana_minutos=ventana_minutos, por_color=por_color, eventos_recientes=eventos_recientes
    )


# ------------------------------------------------------------------- almacen


@app.get("/stock", response_model=list[schemas.StockOut])
def listar_stock(
    usuario: models.Usuario = Depends(auth.get_current_usuario),
    db: Session = Depends(get_db),
):
    return db.query(models.Stock).order_by(models.Stock.producto_id).all()


@app.get("/movimientos_stock", response_model=list[schemas.MovimientoStockOut])
def listar_movimientos(
    usuario: models.Usuario = Depends(auth.require_roles(auth.ROL_ADMIN_SISTEMA)),
    db: Session = Depends(get_db),
):
    return db.query(models.MovimientoStock).order_by(models.MovimientoStock.fecha.desc()).all()


@app.post("/almacen/repartir")
def repartir_stock(
    usuario: models.Usuario = Depends(auth.require_roles(auth.ROL_ADMIN_SISTEMA)),
    db: Session = Depends(get_db),
):
    pedidos_activos = (
        db.query(models.Pedido)
        .filter(models.Pedido.estado.notin_(ESTADOS_PEDIDO_INACTIVOS))
        .order_by(models.Pedido.urgente.desc(), models.Pedido.creado_en.asc())
        .all()
    )
    repartidos = []
    for pedido in pedidos_activos:
        completada_antes = pedido.cantidad_completada
        _servir_desde_stock(db, pedido)
        db.refresh(pedido)
        if pedido.cantidad_completada > completada_antes:
            repartidos.append(pedido)
    if repartidos:
        _audit(
            db,
            "pedidos",
            0,
            "modificacion",
            usuario.id,
            f"reparto de stock: {len(repartidos)} pedido(s)",
        )
    return {"repartidos": _con_stock_disponible(db, repartidos)}


@app.post("/almacen/ajustar", response_model=schemas.StockOut)
def ajustar_stock(
    payload: schemas.AjusteStock,
    usuario: models.Usuario = Depends(auth.require_roles(auth.ROL_ADMIN_SISTEMA)),
    db: Session = Depends(get_db),
):
    if payload.cantidad <= 0:
        raise HTTPException(400, "La cantidad debe ser mayor que 0.")
    producto = db.query(models.Producto).filter(models.Producto.id == payload.producto_id).first()
    if producto is None:
        raise HTTPException(404, "Producto no encontrado.")
    stock_row = db.query(models.Stock).filter_by(producto_id=payload.producto_id).first()
    if stock_row is None:
        stock_row = models.Stock(producto_id=payload.producto_id, cantidad_actual=0)
        db.add(stock_row)
        db.flush()
    stock_row.cantidad_actual += payload.cantidad
    stock_row.actualizado_en = datetime.datetime.utcnow()
    db.add(
        models.MovimientoStock(
            producto_id=payload.producto_id,
            tipo="entrada",
            cantidad=payload.cantidad,
            motivo="ajuste_manual",
            usuario_id=usuario.id,
        )
    )
    db.commit()
    db.refresh(stock_row)
    _audit(db, "stock", payload.producto_id, "modificacion", usuario.id, f"+{payload.cantidad}")
    return stock_row


@app.post("/almacen/quitar", response_model=schemas.StockOut)
def quitar_stock(
    payload: schemas.AjusteStock,
    usuario: models.Usuario = Depends(auth.require_roles(auth.ROL_ADMIN_SISTEMA)),
    db: Session = Depends(get_db),
):
    if payload.cantidad <= 0:
        raise HTTPException(400, "La cantidad debe ser mayor que 0.")
    stock_row = db.query(models.Stock).filter_by(producto_id=payload.producto_id).first()
    disponible = stock_row.cantidad_actual if stock_row else 0
    if payload.cantidad > disponible:
        unidad = "unidad" if disponible == 1 else "unidades"
        raise HTTPException(400, f"Solo hay {disponible} {unidad} disponibles.")
    stock_row.cantidad_actual -= payload.cantidad
    stock_row.actualizado_en = datetime.datetime.utcnow()
    db.add(
        models.MovimientoStock(
            producto_id=payload.producto_id,
            tipo="salida",
            cantidad=payload.cantidad,
            motivo="ajuste_manual",
            usuario_id=usuario.id,
        )
    )
    db.commit()
    db.refresh(stock_row)
    _audit(db, "stock", payload.producto_id, "modificacion", usuario.id, f"-{payload.cantidad}")
    return stock_row


@app.get("/almacen/configuracion", response_model=schemas.ConfiguracionAlmacenOut)
def obtener_configuracion(
    usuario: models.Usuario = Depends(auth.get_current_usuario),
    db: Session = Depends(get_db),
):
    return db.query(models.ConfiguracionAlmacen).filter_by(id=1).first()


@app.patch("/almacen/configuracion", response_model=schemas.ConfiguracionAlmacenOut)
def actualizar_configuracion(
    payload: schemas.ConfiguracionAlmacenUpdate,
    usuario: models.Usuario = Depends(auth.require_roles(auth.ROL_ADMIN_SISTEMA)),
    db: Session = Depends(get_db),
):
    config = db.query(models.ConfiguracionAlmacen).filter_by(id=1).first()
    config.reparto_automatico = payload.reparto_automatico
    db.commit()
    db.refresh(config)
    _audit(db, "configuracion_almacen", 1, "modificacion", usuario.id, str(payload.reparto_automatico))
    return config


@app.get("/audit", response_model=list[schemas.AuditLogOut])
def listar_audit(
    usuario: models.Usuario = Depends(auth.require_roles(auth.ROL_ADMIN_SISTEMA)),
    db: Session = Depends(get_db),
):
    return db.query(models.AuditLog).order_by(models.AuditLog.fecha.desc()).all()
