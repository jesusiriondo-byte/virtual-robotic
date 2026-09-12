"""Diagnostico de agarre (eventos de la celda) y el rastro de auditoria."""
import datetime

from app import models
from app.database import SessionLocal

from .conftest import auth, crear_cliente_con_usuario, producto_por_color


def test_evento_produccion_tipo_valido(client):
    r = client.post(
        "/taller/evento_produccion",
        json={"robot": "sorter", "color": "R", "tipo": "led_encendido"},
    )
    assert r.status_code == 200
    assert r.json()["robot"] == "sorter"


def test_evento_produccion_tipo_invalido(client):
    r = client.post(
        "/taller/evento_produccion",
        json={"robot": "sorter", "color": "R", "tipo": "algo_raro"},
    )
    assert r.status_code == 400


def test_diagnostico_solo_admin_sistema(client, admin_headers):
    _c, tok = crear_cliente_con_usuario(client, admin_headers, "Empresa Diag", "empdiag")
    assert client.get("/taller/diagnostico", headers=admin_headers).status_code == 200
    assert client.get("/taller/diagnostico", headers=auth(tok)).status_code == 403


def test_estado_ok_sin_eventos(client, admin_headers):
    body = client.get("/taller/diagnostico", headers=admin_headers).json()
    rojo = next(c for c in body["por_color"] if c["color"] == "R")
    assert rojo["estado"] == "ok"


def test_fallo_definitivo_marca_critico(client, admin_headers):
    client.post(
        "/taller/evento_produccion",
        json={"robot": "sorter", "color": "R", "tipo": "fallo_definitivo"},
    )
    body = client.get("/taller/diagnostico", headers=admin_headers).json()
    rojo = next(c for c in body["por_color"] if c["color"] == "R")
    assert rojo["estado"] == "critico"
    assert rojo["fallo_definitivo"] == 1


def test_limite_alcance_marca_atencion(client, admin_headers):
    client.post(
        "/taller/evento_produccion",
        json={"robot": "loader", "color": "G", "tipo": "limite_alcance"},
    )
    body = client.get("/taller/diagnostico", headers=admin_headers).json()
    verde = next(c for c in body["por_color"] if c["color"] == "G")
    assert verde["estado"] == "atencion"


def test_mas_agarres_falsos_que_piezas_reales_marca_atencion(client, admin_headers):
    for _ in range(3):
        client.post(
            "/taller/evento_produccion",
            json={"robot": "sorter", "color": "B", "tipo": "agarre_falso"},
        )
    # sin ninguna pieza real todavia (piezas_reales=0 < agarre_falso=3)
    body = client.get("/taller/diagnostico", headers=admin_headers).json()
    azul = next(c for c in body["por_color"] if c["color"] == "B")
    assert azul["estado"] == "atencion"
    assert azul["agarre_falso"] == 3


def test_ventana_minutos_excluye_eventos_viejos(client, admin_headers):
    producto = producto_por_color(client, admin_headers, "R")
    db = SessionLocal()
    try:
        viejo = models.EventoProduccion(
            robot="sorter",
            color="R",
            tipo="fallo_definitivo",
            fecha=datetime.datetime.utcnow() - datetime.timedelta(hours=5),
        )
        db.add(viejo)
        db.commit()
    finally:
        db.close()

    body = client.get(
        "/taller/diagnostico", headers=admin_headers, params={"ventana_minutos": 60}
    ).json()
    rojo = next(c for c in body["por_color"] if c["color"] == "R")
    assert rojo["fallo_definitivo"] == 0
    assert rojo["estado"] == "ok"
    assert producto["color"] == "R"


class TestAuditoria:
    def test_alta_de_producto_deja_rastro(self, client, admin_headers):
        client.post("/productos", headers=admin_headers, json={"nombre": "Clavos", "color": "Y"})
        r = client.get("/audit", headers=admin_headers)
        assert any(a["tabla"] == "productos" and a["accion"] == "alta" for a in r.json())

    def test_cancelar_pedido_deja_rastro_de_baja(self, client, admin_headers):
        from .conftest import (
            asignar_producto,
            crear_usuario_normal,
            login,
            producto_por_color,
        )

        cliente, tok_admin_cliente = crear_cliente_con_usuario(
            client, admin_headers, "Empresa Audit", "empaudit"
        )
        producto = producto_por_color(client, admin_headers, "R")
        asignar_producto(client, admin_headers, cliente["id"], producto["id"])
        crear_usuario_normal(client, admin_headers, cliente["id"], "op_audit")
        tok_normal = login(client, "op_audit")
        pedido = client.post(
            "/pedidos",
            headers=auth(tok_normal),
            json={"producto_id": producto["id"], "cantidad_pedida": 3},
        ).json()
        client.post(f"/pedidos/{pedido['id']}/cancelar", headers=auth(tok_admin_cliente))

        entradas = client.get("/audit", headers=admin_headers).json()
        del_pedido = [a for a in entradas if a["tabla"] == "pedidos" and a["registro_id"] == pedido["id"]]
        acciones = {a["accion"] for a in del_pedido}
        assert "alta" in acciones
        assert "baja" in acciones

    def test_solo_admin_sistema_ve_la_auditoria(self, client, admin_headers):
        _c, tok = crear_cliente_con_usuario(client, admin_headers, "Empresa Audit2", "empaudit2")
        assert client.get("/audit", headers=auth(tok)).status_code == 403
