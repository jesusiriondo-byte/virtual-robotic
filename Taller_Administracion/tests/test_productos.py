"""Catalogo de productos: alta/baja/modificacion, colores unicos por producto."""


def test_seed_tiene_los_tres_productos_reales(client, admin_headers):
    r = client.get("/productos", headers=admin_headers)
    assert r.status_code == 200
    colores = {p["color"] for p in r.json()}
    assert colores == {"R", "G", "B"}


def test_paleta_colores_trae_los_7(client, admin_headers):
    r = client.get("/paleta_colores", headers=admin_headers)
    assert r.status_code == 200
    assert set(r.json()) == {"R", "G", "B", "Y", "M", "C", "W"}


def test_crear_producto_ok(client, admin_headers):
    r = client.post(
        "/productos", headers=admin_headers, json={"nombre": "Clavos", "color": "Y"}
    )
    assert r.status_code == 200
    body = r.json()
    assert body["nombre"] == "Clavos"
    assert body["color"] == "Y"
    assert body["activo"] is True


def test_no_se_puede_repetir_color(client, admin_headers):
    client.post("/productos", headers=admin_headers, json={"nombre": "Clavos", "color": "Y"})
    r = client.post(
        "/productos", headers=admin_headers, json={"nombre": "Otra cosa", "color": "Y"}
    )
    assert r.status_code == 400


def test_no_se_puede_repetir_nombre(client, admin_headers):
    r = client.post(
        "/productos", headers=admin_headers, json={"nombre": "Tornillos", "color": "Y"}
    )
    assert r.status_code == 400


def test_color_invalido_rechazado(client, admin_headers):
    r = client.post(
        "/productos", headers=admin_headers, json={"nombre": "Cosa Rara", "color": "Z"}
    )
    assert r.status_code == 400


def test_nombre_vacio_rechazado(client, admin_headers):
    r = client.post("/productos", headers=admin_headers, json={"nombre": "   ", "color": "Y"})
    assert r.status_code == 400


def test_baja_logica_no_borra_el_producto(client, admin_headers):
    productos = client.get("/productos", headers=admin_headers).json()
    tornillos = next(p for p in productos if p["color"] == "R")

    r = client.patch(
        f"/productos/{tornillos['id']}", headers=admin_headers, json={"activo": False}
    )
    assert r.status_code == 200
    assert r.json()["activo"] is False

    # sigue apareciendo en el listado (baja logica, no DELETE fisico)
    productos_tras_baja = client.get("/productos", headers=admin_headers).json()
    assert any(p["id"] == tornillos["id"] for p in productos_tras_baja)


def test_patch_sin_cambios_da_400(client, admin_headers):
    productos = client.get("/productos", headers=admin_headers).json()
    pid = productos[0]["id"]
    r = client.patch(f"/productos/{pid}", headers=admin_headers, json={})
    assert r.status_code == 400


def test_solo_admin_sistema_puede_crear_productos(client, admin_headers):
    from .conftest import crear_cliente_con_usuario

    _cliente, token_cliente = crear_cliente_con_usuario(
        client, admin_headers, "Ferreteria Sur", "ferreteria_sur"
    )
    r = client.post(
        "/productos",
        headers={"X-Session-Token": token_cliente},
        json={"nombre": "Intento", "color": "Y"},
    )
    assert r.status_code == 403
