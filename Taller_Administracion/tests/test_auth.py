"""Login, contrasena maestra, sesiones y control de acceso basico."""
from app import auth as auth_module

from .conftest import auth as auth_headers, login


def test_login_admin_sembrado_con_master_password(client):
    # TALLER_DEV_MODE esta activo en los tests (ver conftest), igual que en
    # nuestro Docker local: la clave maestra vale tambien para el admin
    # sembrado. Si se comparte/despliega sin ese flag, deja de colar (ver
    # test_master_password_no_sirve_para_admin_fuera_de_dev_mode).
    r = client.post("/login", json={"username": "admin", "password": "1111"})
    assert r.status_code == 200
    body = r.json()
    assert body["usuario"]["rol"] == "admin_sistema"
    assert body["usuario"]["username"] == "admin"
    assert "token" in body and len(body["token"]) > 10


def test_login_admin_con_password_real(client):
    r = client.post("/login", json={"username": "admin", "password": "admin"})
    assert r.status_code == 200


def test_master_password_no_sirve_para_admin_fuera_de_dev_mode(client, monkeypatch):
    monkeypatch.setattr(auth_module, "DEV_MODE", False)
    r = client.post("/login", json={"username": "admin", "password": "1111"})
    assert r.status_code == 401
    # la contrasena real del admin sigue funcionando aunque se apague el modo desarrollo
    r2 = client.post("/login", json={"username": "admin", "password": "admin"})
    assert r2.status_code == 200


def test_master_password_siempre_sirve_para_rol_normal(client, admin_headers, monkeypatch):
    from .conftest import crear_cliente_con_usuario, crear_usuario_normal

    cliente, _token = crear_cliente_con_usuario(client, admin_headers, "Cliente X", "admin-x")
    crear_usuario_normal(client, admin_headers, cliente["id"], "operario1")

    monkeypatch.setattr(auth_module, "DEV_MODE", False)
    r = client.post("/login", json={"username": "operario1", "password": "1111"})
    assert r.status_code == 200


def test_login_username_case_insensitive(client):
    r = client.post("/login", json={"username": "ADMIN", "password": "1111"})
    assert r.status_code == 200


def test_login_password_incorrecta(client):
    r = client.post("/login", json={"username": "admin", "password": "otra-cosa"})
    assert r.status_code == 401


def test_login_usuario_inexistente(client):
    r = client.post("/login", json={"username": "no-existe", "password": "1111"})
    assert r.status_code == 401


def test_sin_token_da_401(client):
    r = client.get("/pedidos")
    assert r.status_code == 401


def test_token_basura_da_401(client):
    r = client.get("/pedidos", headers=auth_headers("token-que-no-existe"))
    assert r.status_code == 401


def test_logout_invalida_el_token(client):
    token = login(client)
    assert client.get("/pedidos", headers=auth_headers(token)).status_code == 200
    r = client.post("/logout", headers=auth_headers(token))
    assert r.status_code == 200
    assert client.get("/pedidos", headers=auth_headers(token)).status_code == 401


def test_usuario_dado_de_baja_no_puede_entrar(client, admin_headers):
    r = client.post(
        "/usuarios",
        headers=admin_headers,
        json={
            "username": "temporal",
            "nombre_completo": "Usuario Temporal",
            "rol": "admin_sistema",
            "cliente_id": None,
            "password": "clave123",
        },
    )
    assert r.status_code == 200
    assert login(client, "temporal", "clave123")

    client.patch(
        f"/usuarios/{r.json()['id']}", headers=admin_headers, json={"activo": False}
    )
    r2 = client.post("/login", json={"username": "temporal", "password": "clave123"})
    assert r2.status_code == 401
