"""Almacen: cubo_clasificado (evento de la celda), reparto_automatico como
UNICO gate, ajustar/quitar stock a mano, y el reparto FIFO de
stock_disponible entre varios pedidos del mismo color."""
from .conftest import (
    asignar_producto,
    auth,
    crear_cliente_con_usuario,
    crear_usuario_normal,
    login,
    producto_por_color,
)


def test_cubo_clasificado_sin_pedidos_solo_suma_stock(client, admin_headers):
    r = client.post("/taller/cubo_clasificado", json={"color": "R"})
    assert r.status_code == 200
    body = r.json()
    assert body["stock_actual"] == 1
    assert body["pedido"] is None

    stock = client.get("/stock", headers=admin_headers).json()
    tornillos_stock = next(s for s in stock if s["producto"]["color"] == "R")
    assert tornillos_stock["cantidad_actual"] == 1


def test_cubo_clasificado_color_invalido(client):
    r = client.post("/taller/cubo_clasificado", json={"color": "Q"})
    assert r.status_code == 400


_contador_pedidos_test = 0


def _preparar_pedido(client, admin_headers, color="R", cantidad=5):
    """OJO: cada llamada crea una empresa/usuario NUEVOS (sufijo incremental)
    -- reusar el mismo color dos veces en un test no colisiona con un 409
    de username duplicado, a diferencia de si el nombre saliera solo del
    color."""
    global _contador_pedidos_test
    _contador_pedidos_test += 1
    sufijo = f"{color.lower()}{_contador_pedidos_test}"
    cliente, tok_admin_cliente = crear_cliente_con_usuario(
        client, admin_headers, f"Empresa {sufijo}", f"emp_{sufijo}"
    )
    producto = producto_por_color(client, admin_headers, color)
    asignar_producto(client, admin_headers, cliente["id"], producto["id"])
    crear_usuario_normal(client, admin_headers, cliente["id"], f"op_{sufijo}")
    tok_normal = login(client, f"op_{sufijo}")
    pedido = client.post(
        "/pedidos",
        headers=auth(tok_normal),
        json={"producto_id": producto["id"], "cantidad_pedida": cantidad},
    ).json()
    return {
        "pedido": pedido,
        "producto": producto,
        "tok_admin_cliente": tok_admin_cliente,
        "tok_normal": tok_normal,
    }


def test_reparto_automatico_activo_aplica_la_pieza_al_pedido_mas_antiguo(
    client, admin_headers
):
    ctx = _preparar_pedido(client, admin_headers, cantidad=2)
    r = client.post("/taller/cubo_clasificado", json={"color": "R"})
    assert r.status_code == 200
    assert r.json()["pedido"]["id"] == ctx["pedido"]["id"]
    assert r.json()["pedido"]["cantidad_completada"] == 1
    assert r.json()["pedido"]["estado"] == "en_proceso"
    # con reparto automatico, la pieza no se queda en almacen
    assert r.json()["stock_actual"] == 0


def test_pedido_se_completa_al_llegar_a_la_cantidad_pedida(client, admin_headers):
    ctx = _preparar_pedido(client, admin_headers, cantidad=2)
    client.post("/taller/cubo_clasificado", json={"color": "R"})
    r2 = client.post("/taller/cubo_clasificado", json={"color": "R"})
    assert r2.json()["pedido"]["estado"] == "completado"
    assert r2.json()["pedido"]["cantidad_completada"] == 2


def test_reparto_automatico_desactivado_la_pieza_se_queda_en_stock(client, admin_headers):
    """El bug real que motivo que reparto_automatico sea el UNICO gate
    (ver el comentario en main.py): con el interruptor apagado, NINGUNA
    pieza debe aplicarse a un pedido, venga con pedido_id explicito o sin
    el. Si esto falla, un pedido se completaria solo con el interruptor
    desactivado -- exactamente el bug que el usuario reporto en su dia."""
    ctx = _preparar_pedido(client, admin_headers, cantidad=2)
    client.patch(
        "/almacen/configuracion", headers=admin_headers, json={"reparto_automatico": False}
    )
    r = client.post(
        "/taller/cubo_clasificado", json={"color": "R", "pedido_id": ctx["pedido"]["id"]}
    )
    assert r.status_code == 200
    assert r.json()["pedido"] is None
    assert r.json()["stock_actual"] == 1

    pedido_tras = client.get(
        f"/pedidos/{ctx['pedido']['id']}", headers=auth(ctx["tok_admin_cliente"])
    ).json()
    assert pedido_tras["estado"] == "pendiente"
    assert pedido_tras["cantidad_completada"] == 0


def test_pedido_nuevo_se_sirve_al_instante_si_hay_stock_y_reparto_activo(
    client, admin_headers
):
    # stock de sobra ANTES de que exista el pedido (sobreproduccion)
    for _ in range(3):
        client.post("/taller/cubo_clasificado", json={"color": "R"})
    ctx = _preparar_pedido(client, admin_headers, cantidad=2)
    # _servir_desde_stock se dispara solo, sin ningun cubo_clasificado extra
    pedido_tras = client.get(
        f"/pedidos/{ctx['pedido']['id']}", headers=auth(ctx["tok_admin_cliente"])
    ).json()
    assert pedido_tras["estado"] == "completado"
    assert pedido_tras["cantidad_completada"] == 2


def test_pedido_nuevo_no_se_sirve_solo_si_reparto_desactivado(client, admin_headers):
    client.patch(
        "/almacen/configuracion", headers=admin_headers, json={"reparto_automatico": False}
    )
    for _ in range(3):
        client.post("/taller/cubo_clasificado", json={"color": "R"})
    ctx = _preparar_pedido(client, admin_headers, cantidad=2)
    pedido_tras = client.get(
        f"/pedidos/{ctx['pedido']['id']}", headers=auth(ctx["tok_admin_cliente"])
    ).json()
    assert pedido_tras["estado"] == "pendiente"
    assert pedido_tras["cantidad_completada"] == 0


def test_urgente_se_sirve_antes_que_el_mas_antiguo(client, admin_headers):
    viejo = _preparar_pedido(client, admin_headers, color="R", cantidad=1)
    nuevo = _preparar_pedido(client, admin_headers, color="R", cantidad=1)
    client.patch(
        f"/pedidos/{nuevo['pedido']['id']}", headers=admin_headers, json={"urgente": True}
    )
    r = client.post("/taller/cubo_clasificado", json={"color": "R"})
    assert r.json()["pedido"]["id"] == nuevo["pedido"]["id"]

    viejo_tras = client.get(
        f"/pedidos/{viejo['pedido']['id']}", headers=admin_headers
    ).json()
    assert viejo_tras["estado"] == "pendiente"


class TestAjustarQuitarStock:
    def test_ajustar_suma_stock_y_deja_movimiento(self, client, admin_headers):
        r_val = producto_por_color(client, admin_headers, "G")
        r = client.post(
            "/almacen/ajustar",
            headers=admin_headers,
            json={"producto_id": r_val["id"], "cantidad": 10},
        )
        assert r.status_code == 200
        assert r.json()["cantidad_actual"] == 10
        movimientos = client.get("/movimientos_stock", headers=admin_headers).json()
        assert any(m["motivo"] == "ajuste_manual" and m["tipo"] == "entrada" for m in movimientos)

    def test_quitar_no_deja_bajar_de_cero(self, client, admin_headers):
        producto = producto_por_color(client, admin_headers, "G")
        client.post(
            "/almacen/ajustar", headers=admin_headers, json={"producto_id": producto["id"], "cantidad": 3}
        )
        r = client.post(
            "/almacen/quitar", headers=admin_headers, json={"producto_id": producto["id"], "cantidad": 10}
        )
        assert r.status_code == 400

    def test_quitar_cantidad_valida_ok(self, client, admin_headers):
        producto = producto_por_color(client, admin_headers, "G")
        client.post(
            "/almacen/ajustar", headers=admin_headers, json={"producto_id": producto["id"], "cantidad": 5}
        )
        r = client.post(
            "/almacen/quitar", headers=admin_headers, json={"producto_id": producto["id"], "cantidad": 3}
        )
        assert r.status_code == 200
        assert r.json()["cantidad_actual"] == 2

    def test_solo_admin_sistema_ajusta_stock(self, client, admin_headers):
        _c, tok = crear_cliente_con_usuario(client, admin_headers, "Empresa G", "empg")
        producto = producto_por_color(client, admin_headers, "G")
        r = client.post(
            "/almacen/ajustar",
            headers=auth(tok),
            json={"producto_id": producto["id"], "cantidad": 1},
        )
        assert r.status_code == 403


class TestStockDisponibleFIFO:
    """_con_stock_disponible simula el reparto FIFO SIN aplicarlo de verdad
    -- lo que muestra el panel como "cubre todo/parte" tiene que coincidir
    con lo que /almacen/repartir haria de verdad si se pulsara ahora."""

    def test_dos_pedidos_del_mismo_color_no_se_solapan_el_stock(self, client, admin_headers):
        p1 = _preparar_pedido(client, admin_headers, color="R", cantidad=5)
        client.patch(
            "/almacen/configuracion", headers=admin_headers, json={"reparto_automatico": False}
        )
        p2_cliente, tok_p2 = crear_cliente_con_usuario(client, admin_headers, "Empresa R2", "empr2")
        producto = producto_por_color(client, admin_headers, "R")
        asignar_producto(client, admin_headers, p2_cliente["id"], producto["id"])
        crear_usuario_normal(client, admin_headers, p2_cliente["id"], "op_r2")
        tok_normal2 = login(client, "op_r2")
        pedido2 = client.post(
            "/pedidos",
            headers=auth(tok_normal2),
            json={"producto_id": producto["id"], "cantidad_pedida": 5},
        ).json()

        client.post("/almacen/ajustar", headers=admin_headers, json={"producto_id": producto["id"], "cantidad": 7})

        pedidos = client.get("/pedidos", headers=admin_headers).json()
        d = {p["id"]: p["stock_disponible"] for p in pedidos if p["id"] in (p1["pedido"]["id"], pedido2["id"])}
        # 7 unidades repartidas FIFO entre dos pedidos de 5: el primero (mas
        # antiguo) se lleva sus 5, al segundo solo le quedan 2 -- NUNCA los
        # 7 completos en los dos a la vez (ese fue el hueco real que motivo
        # esta funcion, ver su docstring en main.py).
        assert d[p1["pedido"]["id"]] == 5
        assert d[pedido2["id"]] == 2

    def test_repartir_stock_manual_aplica_lo_que_stock_disponible_prometia(
        self, client, admin_headers
    ):
        ctx = _preparar_pedido(client, admin_headers, color="B", cantidad=4)
        client.patch(
            "/almacen/configuracion", headers=admin_headers, json={"reparto_automatico": False}
        )
        producto = ctx["producto"]
        client.post("/almacen/ajustar", headers=admin_headers, json={"producto_id": producto["id"], "cantidad": 4})

        pedido_antes = client.get(f"/pedidos/{ctx['pedido']['id']}", headers=admin_headers).json()
        assert pedido_antes["stock_disponible"] == 4
        assert pedido_antes["estado"] == "pendiente"

        r = client.post("/almacen/repartir", headers=admin_headers)
        assert r.status_code == 200
        assert len(r.json()["repartidos"]) == 1

        pedido_despues = client.get(f"/pedidos/{ctx['pedido']['id']}", headers=admin_headers).json()
        assert pedido_despues["estado"] == "completado"
        assert pedido_despues["cantidad_completada"] == 4
