"""segundos_proceso / segundos_total: calculados en el momento de leer
(no son columnas), a partir de los MovimientoStock del pedido. Feature de
la sesion 2026-09-11.

Para controlar el tiempo exacto sin dormir el test de verdad, los
MovimientoStock se insertan directo por la sesion de SQLAlchemy con
fechas concretas, en vez de pasar por /taller/cubo_clasificado (que
siempre usa datetime.utcnow()).
"""
import datetime

from app.database import SessionLocal
from app import models

from .conftest import (
    asignar_producto,
    auth,
    crear_cliente_con_usuario,
    crear_usuario_normal,
    login,
    producto_por_color,
)


def _crear_pedido_pendiente(client, admin_headers, cantidad=10):
    cliente, tok_admin_cliente = crear_cliente_con_usuario(
        client, admin_headers, "Empresa Tiempos", "emp_tiempos"
    )
    producto = producto_por_color(client, admin_headers, "R")
    asignar_producto(client, admin_headers, cliente["id"], producto["id"])
    crear_usuario_normal(client, admin_headers, cliente["id"], "op_tiempos")
    tok_normal = login(client, "op_tiempos")
    pedido = client.post(
        "/pedidos",
        headers=auth(tok_normal),
        json={"producto_id": producto["id"], "cantidad_pedida": cantidad},
    ).json()
    return pedido, tok_admin_cliente


def _insertar_movimiento(pedido_id, producto_id, fecha):
    db = SessionLocal()
    try:
        db.add(
            models.MovimientoStock(
                producto_id=producto_id,
                tipo="salida",
                cantidad=1,
                motivo="envio_pedido",
                pedido_id=pedido_id,
                fecha=fecha,
            )
        )
        db.commit()
    finally:
        db.close()


def test_pedido_sin_movimientos_da_tiempos_none(client, admin_headers):
    pedido, tok = _crear_pedido_pendiente(client, admin_headers)
    r = client.get(f"/pedidos/{pedido['id']}", headers=auth(tok))
    assert r.json()["segundos_proceso"] is None
    assert r.json()["segundos_total"] is None


def test_segundos_proceso_es_la_diferencia_entre_primera_y_ultima_pieza(
    client, admin_headers
):
    pedido, tok = _crear_pedido_pendiente(client, admin_headers, cantidad=3)
    producto_id = pedido["producto_id"]

    t0 = datetime.datetime(2026, 1, 1, 12, 0, 0)
    _insertar_movimiento(pedido["id"], producto_id, t0)
    _insertar_movimiento(pedido["id"], producto_id, t0 + datetime.timedelta(seconds=45))
    _insertar_movimiento(pedido["id"], producto_id, t0 + datetime.timedelta(seconds=90))

    r = client.get(f"/pedidos/{pedido['id']}", headers=auth(tok))
    body = r.json()
    assert body["segundos_proceso"] == 90.0


def test_segundos_total_incluye_la_espera_desde_que_se_creo_el_pedido(
    client, admin_headers
):
    pedido, tok = _crear_pedido_pendiente(client, admin_headers, cantidad=1)
    creado_en = datetime.datetime.fromisoformat(pedido["creado_en"])

    # la primera (y unica) pieza llega 5 minutos despues de crear el pedido
    fecha_pieza = creado_en + datetime.timedelta(minutes=5)
    _insertar_movimiento(pedido["id"], pedido["producto_id"], fecha_pieza)

    r = client.get(f"/pedidos/{pedido['id']}", headers=auth(tok))
    body = r.json()
    assert body["segundos_proceso"] == 0.0  # una sola pieza: primera == ultima
    assert abs(body["segundos_total"] - 300.0) < 2.0  # margen por redondeo de milisegundos


def test_pedido_servido_de_golpe_desde_almacen_da_proceso_cero(client, admin_headers):
    """Un lote que se completa al instante desde stock (sin pasar por
    produccion real) no debe aparentar que tardo nada -- 0s es lo
    correcto, no un numero inventado."""
    for _ in range(2):
        client.post("/taller/cubo_clasificado", json={"color": "R"})
    pedido, tok = _crear_pedido_pendiente(client, admin_headers, cantidad=2)

    r = client.get(f"/pedidos/{pedido['id']}", headers=auth(tok))
    body = r.json()
    assert body["estado"] == "completado"
    assert body["segundos_proceso"] == 0.0


def test_listar_varios_pedidos_calcula_tiempos_independientes(client, admin_headers):
    """_con_tiempos_de_proceso hace UNA consulta agregada para toda la
    lista -- si se confunden los pedido_id entre si, este test lo detecta."""
    pedido_a, tok = _crear_pedido_pendiente(client, admin_headers, cantidad=1)
    cliente_b, _tok_b = crear_cliente_con_usuario(client, admin_headers, "Empresa B2", "empb2")
    producto = producto_por_color(client, admin_headers, "R")
    asignar_producto(client, admin_headers, cliente_b["id"], producto["id"])
    crear_usuario_normal(client, admin_headers, cliente_b["id"], "op_b2")
    tok_b = login(client, "op_b2")
    pedido_b = client.post(
        "/pedidos", headers=auth(tok_b), json={"producto_id": producto["id"], "cantidad_pedida": 1}
    ).json()

    t0 = datetime.datetime(2026, 2, 1, 8, 0, 0)
    _insertar_movimiento(pedido_a["id"], producto["id"], t0)
    _insertar_movimiento(pedido_a["id"], producto["id"], t0 + datetime.timedelta(seconds=10))
    _insertar_movimiento(pedido_b["id"], producto["id"], t0)
    _insertar_movimiento(pedido_b["id"], producto["id"], t0 + datetime.timedelta(seconds=999))

    pedidos = client.get("/pedidos", headers=admin_headers).json()
    por_id = {p["id"]: p for p in pedidos}
    assert por_id[pedido_a["id"]]["segundos_proceso"] == 10.0
    assert por_id[pedido_b["id"]]["segundos_proceso"] == 999.0
