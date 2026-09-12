"""Pedidos: alta, alcance por rol, cancelacion, reproceso.

Dos de estos tests documentan regresiones REALES de la sesion 2026-09-11:
- test_cancelar_pedido_no_pendiente_da_mensaje_completo: el mensaje de error
  se rompio a "c/p ya fabricadas" en una reescritura; si alguien lo vuelve a
  romper, este test lo caza.
- La clase TestReprocesar entera: endpoint nuevo de esa misma sesion.
"""
import pytest

from .conftest import (
    asignar_producto,
    auth,
    crear_cliente_con_usuario,
    crear_usuario_normal,
    login,
    producto_por_color,
)


@pytest.fixture()
def cliente_con_tornillos(client, admin_headers):
    """Empresa con el producto 'Tornillos' (rojo) ya asignado, y un usuario
    normal para pedir. Devuelve dict con todo lo que suelen necesitar los
    tests de pedidos."""
    cliente, tok_admin_cliente = crear_cliente_con_usuario(
        client, admin_headers, "Ferreteria Norte", "fnorte"
    )
    tornillos = producto_por_color(client, admin_headers, "R")
    asignar_producto(client, admin_headers, cliente["id"], tornillos["id"])
    normal = crear_usuario_normal(client, admin_headers, cliente["id"], "operario_norte")
    tok_normal = login(client, "operario_norte")
    return {
        "cliente": cliente,
        "tok_admin_cliente": tok_admin_cliente,
        "producto": tornillos,
        "tok_normal": tok_normal,
    }


def test_crear_pedido_ok(client, cliente_con_tornillos):
    ctx = cliente_con_tornillos
    r = client.post(
        "/pedidos",
        headers=auth(ctx["tok_normal"]),
        json={"producto_id": ctx["producto"]["id"], "cantidad_pedida": 20},
    )
    assert r.status_code == 200
    body = r.json()
    assert body["estado"] == "pendiente"
    assert body["cantidad_pedida"] == 20
    assert body["cantidad_completada"] == 0
    assert body["urgente"] is False


def test_no_se_puede_pedir_producto_no_asignado(client, admin_headers):
    cliente, _tok_admin = crear_cliente_con_usuario(
        client, admin_headers, "Sin Catalogo SL", "sincat"
    )
    arandelas = producto_por_color(client, admin_headers, "B")
    normal = crear_usuario_normal(client, admin_headers, cliente["id"], "op_sincat")
    tok = login(client, "op_sincat")

    r = client.post(
        "/pedidos", headers=auth(tok), json={"producto_id": arandelas["id"], "cantidad_pedida": 5}
    )
    assert r.status_code == 403


def test_cantidad_cero_o_negativa_rechazada(client, cliente_con_tornillos):
    ctx = cliente_con_tornillos
    r = client.post(
        "/pedidos",
        headers=auth(ctx["tok_normal"]),
        json={"producto_id": ctx["producto"]["id"], "cantidad_pedida": 0},
    )
    assert r.status_code == 400


def test_producto_de_baja_no_admite_pedidos_nuevos(client, admin_headers, cliente_con_tornillos):
    ctx = cliente_con_tornillos
    client.patch(f"/productos/{ctx['producto']['id']}", headers=admin_headers, json={"activo": False})
    r = client.post(
        "/pedidos",
        headers=auth(ctx["tok_normal"]),
        json={"producto_id": ctx["producto"]["id"], "cantidad_pedida": 5},
    )
    assert r.status_code == 400


def test_normal_solo_ve_sus_propios_pedidos(client, admin_headers, cliente_con_tornillos):
    ctx = cliente_con_tornillos
    client.post(
        "/pedidos",
        headers=auth(ctx["tok_normal"]),
        json={"producto_id": ctx["producto"]["id"], "cantidad_pedida": 5},
    )
    otro_normal = crear_usuario_normal(client, admin_headers, ctx["cliente"]["id"], "otro_operario")
    tok_otro = login(client, "otro_operario")
    client.post(
        "/pedidos",
        headers=auth(tok_otro),
        json={"producto_id": ctx["producto"]["id"], "cantidad_pedida": 7},
    )

    r = client.get("/pedidos", headers=auth(ctx["tok_normal"]))
    assert len(r.json()) == 1
    assert r.json()[0]["cantidad_pedida"] == 5


def test_admin_cliente_ve_todos_los_de_su_empresa(client, cliente_con_tornillos):
    ctx = cliente_con_tornillos
    client.post(
        "/pedidos",
        headers=auth(ctx["tok_normal"]),
        json={"producto_id": ctx["producto"]["id"], "cantidad_pedida": 5},
    )
    r = client.get("/pedidos", headers=auth(ctx["tok_admin_cliente"]))
    assert len(r.json()) == 1


def test_admin_cliente_no_ve_pedidos_de_otra_empresa(client, admin_headers, cliente_con_tornillos):
    ctx = cliente_con_tornillos
    client.post(
        "/pedidos",
        headers=auth(ctx["tok_normal"]),
        json={"producto_id": ctx["producto"]["id"], "cantidad_pedida": 5},
    )
    _c2, tok_ajeno = crear_cliente_con_usuario(client, admin_headers, "Empresa Ajena", "ajena")
    r = client.get("/pedidos", headers=auth(tok_ajeno))
    assert r.json() == []


def test_marcar_urgente(client, admin_headers, cliente_con_tornillos):
    ctx = cliente_con_tornillos
    pedido = client.post(
        "/pedidos",
        headers=auth(ctx["tok_normal"]),
        json={"producto_id": ctx["producto"]["id"], "cantidad_pedida": 5},
    ).json()
    r = client.patch(f"/pedidos/{pedido['id']}", headers=admin_headers, json={"urgente": True})
    assert r.status_code == 200
    assert r.json()["urgente"] is True


class TestCancelar:
    def test_cancelar_pedido_pendiente_ok(self, client, cliente_con_tornillos):
        ctx = cliente_con_tornillos
        pedido = client.post(
            "/pedidos",
            headers=auth(ctx["tok_normal"]),
            json={"producto_id": ctx["producto"]["id"], "cantidad_pedida": 5},
        ).json()
        r = client.post(f"/pedidos/{pedido['id']}/cancelar", headers=auth(ctx["tok_admin_cliente"]))
        assert r.status_code == 200
        assert r.json()["estado"] == "cancelado"

    def test_cancelar_pedido_no_pendiente_da_mensaje_completo(
        self, client, admin_headers, cliente_con_tornillos
    ):
        """Regresion real (2026-09-11): este mensaje se rompio a "c/p ya
        fabricadas" en una reescritura del backend. Tiene que decir el
        estado real y cuanto se ha fabricado ya, no una abreviatura críptica."""
        ctx = cliente_con_tornillos
        pedido = client.post(
            "/pedidos",
            headers=auth(ctx["tok_normal"]),
            json={"producto_id": ctx["producto"]["id"], "cantidad_pedida": 5},
        ).json()
        client.post(
            "/taller/cubo_clasificado", json={"color": "R", "pedido_id": pedido["id"]}
        )
        r = client.post(f"/pedidos/{pedido['id']}/cancelar", headers=admin_headers)
        assert r.status_code == 400
        detalle = r.json()["detail"]
        assert "pendiente" in detalle
        assert "en_proceso" in detalle
        assert "1" in detalle and "5" in detalle  # 1/5 ya fabricadas
        assert detalle != "c/p ya fabricadas"

    def test_no_se_puede_cancelar_dos_veces(self, client, cliente_con_tornillos):
        ctx = cliente_con_tornillos
        pedido = client.post(
            "/pedidos",
            headers=auth(ctx["tok_normal"]),
            json={"producto_id": ctx["producto"]["id"], "cantidad_pedida": 5},
        ).json()
        client.post(f"/pedidos/{pedido['id']}/cancelar", headers=auth(ctx["tok_admin_cliente"]))
        r = client.post(f"/pedidos/{pedido['id']}/cancelar", headers=auth(ctx["tok_admin_cliente"]))
        assert r.status_code == 400


class TestReprocesar:
    def _completar_pedido(self, client, ctx, cantidad=3):
        pedido = client.post(
            "/pedidos",
            headers=auth(ctx["tok_normal"]),
            json={"producto_id": ctx["producto"]["id"], "cantidad_pedida": cantidad},
        ).json()
        for _ in range(cantidad):
            client.post("/taller/cubo_clasificado", json={"color": "R", "pedido_id": pedido["id"]})
        return client.get(f"/pedidos/{pedido['id']}", headers=auth(ctx["tok_admin_cliente"])).json()

    def test_reprocesar_pedido_completado_crea_uno_nuevo(
        self, client, admin_headers, cliente_con_tornillos
    ):
        ctx = cliente_con_tornillos
        original = self._completar_pedido(client, ctx)
        assert original["estado"] == "completado"

        r = client.post(f"/pedidos/{original['id']}/reprocesar", headers=admin_headers)
        assert r.status_code == 200
        nuevo = r.json()
        assert nuevo["id"] != original["id"]
        assert nuevo["estado"] == "pendiente"
        assert nuevo["cantidad_completada"] == 0
        assert nuevo["producto_id"] == original["producto_id"]
        assert nuevo["cliente_id"] == original["cliente_id"]
        assert nuevo["cantidad_pedida"] == original["cantidad_pedida"]

        # el original NO se toca -- sigue completado, para no perder el
        # historico de que aquel lote se fabrico de verdad
        original_tras = client.get(f"/pedidos/{original['id']}", headers=admin_headers).json()
        assert original_tras["estado"] == "completado"

    def test_no_se_puede_reprocesar_un_pendiente(self, client, admin_headers, cliente_con_tornillos):
        ctx = cliente_con_tornillos
        pedido = client.post(
            "/pedidos",
            headers=auth(ctx["tok_normal"]),
            json={"producto_id": ctx["producto"]["id"], "cantidad_pedida": 5},
        ).json()
        r = client.post(f"/pedidos/{pedido['id']}/reprocesar", headers=admin_headers)
        assert r.status_code == 400

    def test_no_se_puede_reprocesar_uno_cancelado(self, client, cliente_con_tornillos):
        ctx = cliente_con_tornillos
        pedido = client.post(
            "/pedidos",
            headers=auth(ctx["tok_normal"]),
            json={"producto_id": ctx["producto"]["id"], "cantidad_pedida": 5},
        ).json()
        client.post(f"/pedidos/{pedido['id']}/cancelar", headers=auth(ctx["tok_admin_cliente"]))
        r = client.post(
            f"/pedidos/{pedido['id']}/reprocesar", headers=auth(ctx["tok_admin_cliente"])
        )
        assert r.status_code == 400

    def test_reprocesar_id_inexistente_da_404(self, client, admin_headers):
        r = client.post("/pedidos/999999/reprocesar", headers=admin_headers)
        assert r.status_code == 404

    def test_reprocesar_requiere_sesion(self, client, admin_headers, cliente_con_tornillos):
        ctx = cliente_con_tornillos
        original = self._completar_pedido(client, ctx)
        r = client.post(f"/pedidos/{original['id']}/reprocesar")
        assert r.status_code == 401

    def test_normal_no_puede_reprocesar(self, client, cliente_con_tornillos):
        ctx = cliente_con_tornillos
        original = self._completar_pedido(client, ctx)
        r = client.post(
            f"/pedidos/{original['id']}/reprocesar", headers=auth(ctx["tok_normal"])
        )
        assert r.status_code == 403

    def test_admin_cliente_no_puede_reprocesar_pedido_de_otra_empresa(
        self, client, admin_headers, cliente_con_tornillos
    ):
        ctx = cliente_con_tornillos
        original = self._completar_pedido(client, ctx)
        _c2, tok_ajeno = crear_cliente_con_usuario(client, admin_headers, "Empresa Ajena 2", "ajena2")
        r = client.post(f"/pedidos/{original['id']}/reprocesar", headers=auth(tok_ajeno))
        assert r.status_code == 403

    def test_reprocesar_producto_de_baja_da_400(self, client, admin_headers, cliente_con_tornillos):
        ctx = cliente_con_tornillos
        original = self._completar_pedido(client, ctx)
        client.patch(f"/productos/{ctx['producto']['id']}", headers=admin_headers, json={"activo": False})
        r = client.post(f"/pedidos/{original['id']}/reprocesar", headers=admin_headers)
        assert r.status_code == 400
