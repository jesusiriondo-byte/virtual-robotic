"""Clientes, su primer usuario, y los limites de permisos entre roles."""
from .conftest import auth, crear_cliente_con_usuario, login


def test_crear_cliente_crea_tambien_su_admin_cliente(client, admin_headers):
    cliente, token = crear_cliente_con_usuario(client, admin_headers, "Ferreteria Sur", "fsur")
    assert cliente["razon_social"] == "Ferreteria Sur"
    assert cliente["activo"] is True
    assert token  # el login del admin_cliente recien creado funciono


def test_admin_cliente_no_ve_otros_clientes(client, admin_headers):
    _c1, tok1 = crear_cliente_con_usuario(client, admin_headers, "Empresa Uno", "uno")
    crear_cliente_con_usuario(client, admin_headers, "Empresa Dos", "dos")

    r = client.get("/clientes", headers=auth(tok1))
    assert r.status_code == 200
    nombres = {c["razon_social"] for c in r.json()}
    assert nombres == {"Empresa Uno"}


def test_admin_sistema_ve_todos_los_clientes(client, admin_headers):
    crear_cliente_con_usuario(client, admin_headers, "Empresa Uno", "uno2")
    crear_cliente_con_usuario(client, admin_headers, "Empresa Dos", "dos2")
    r = client.get("/clientes", headers=admin_headers)
    nombres = {c["razon_social"] for c in r.json()}
    assert {"Empresa Uno", "Empresa Dos"} <= nombres


def test_admin_cliente_no_puede_crear_admin_sistema(client, admin_headers):
    _c, tok = crear_cliente_con_usuario(client, admin_headers, "Empresa Tres", "tres")
    r = client.post(
        "/usuarios",
        headers=auth(tok),
        json={
            "username": "intento_admin",
            "nombre_completo": "Intento",
            "rol": "admin_sistema",
            "cliente_id": None,
            "password": None,
        },
    )
    assert r.status_code == 403


def test_admin_cliente_no_puede_colar_usuario_en_otra_empresa(client, admin_headers):
    """No es un 403: el backend ni se molesta en avisar -- ignora en
    silencio el cliente_id que venga en el payload y fuerza SIEMPRE la
    propia empresa del actor (ver crear_usuario en main.py). Un
    admin_cliente no tiene forma de colar un usuario en otra empresa por
    mucho que lo intente."""
    c1, tok1 = crear_cliente_con_usuario(client, admin_headers, "Empresa A", "empa")
    c2, _tok2 = crear_cliente_con_usuario(client, admin_headers, "Empresa B", "empb")

    r = client.post(
        "/usuarios",
        headers=auth(tok1),
        json={
            "username": "colado",
            "nombre_completo": "Colado",
            "rol": "normal",
            "cliente_id": c2["id"],  # intenta crear en la empresa AJENA
            "password": None,
        },
    )
    assert r.status_code == 200
    assert r.json()["cliente_id"] == c1["id"]  # forzado a la SUYA, no a la B


def test_admin_cliente_no_puede_ascender_a_admin_sistema(client, admin_headers):
    c, tok = crear_cliente_con_usuario(client, admin_headers, "Empresa C", "empc")
    normal = client.post(
        "/usuarios",
        headers=auth(tok),
        json={
            "username": "operario1",
            "nombre_completo": "Operario Uno",
            "rol": "normal",
            "cliente_id": c["id"],
            "password": None,
        },
    ).json()
    r = client.patch(
        f"/usuarios/{normal['id']}", headers=auth(tok), json={"rol": "admin_sistema"}
    )
    assert r.status_code == 403


def test_solo_admin_sistema_da_de_alta_o_baja_un_cliente_entero(client, admin_headers):
    c, tok_dueno = crear_cliente_con_usuario(client, admin_headers, "Empresa D", "empd")
    # el propio admin_cliente puede tocar su razon_social...
    r_ok = client.patch(
        f"/clientes/{c['id']}", headers=auth(tok_dueno), json={"razon_social": "Empresa D Renombrada"}
    )
    assert r_ok.status_code == 200
    # ...pero no puede darse de baja a si mismo
    r_baja = client.patch(f"/clientes/{c['id']}", headers=auth(tok_dueno), json={"activo": False})
    assert r_baja.status_code == 403
    # el admin_sistema si puede
    r_baja_admin = client.patch(f"/clientes/{c['id']}", headers=admin_headers, json={"activo": False})
    assert r_baja_admin.status_code == 200
    assert r_baja_admin.json()["activo"] is False


def test_username_duplicado_rechazado(client, admin_headers):
    crear_cliente_con_usuario(client, admin_headers, "Empresa E", "repetido")
    r = client.post(
        "/clientes",
        headers=admin_headers,
        json={
            "razon_social": "Empresa E Segunda",
            "cif": None,
            "primer_usuario": {
                "username": "REPETIDO",  # mismo username, distintas mayusculas
                "nombre_completo": "Otro",
                "password": None,
            },
        },
    )
    assert r.status_code == 409


def test_usuario_normal_no_puede_listar_usuarios(client, admin_headers):
    c, tok_dueno = crear_cliente_con_usuario(client, admin_headers, "Empresa F", "empf")
    operario = client.post(
        "/usuarios",
        headers=auth(tok_dueno),
        json={
            "username": "operario_f",
            "nombre_completo": "Operario F",
            "rol": "normal",
            "cliente_id": c["id"],
            "password": None,
        },
    ).json()
    tok_operario = login(client, "operario_f")
    r = client.get("/usuarios", headers=auth(tok_operario))
    assert r.status_code == 403
    assert operario["rol"] == "normal"
