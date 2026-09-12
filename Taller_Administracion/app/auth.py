import hashlib
import os
import secrets

from fastapi import Depends, Header, HTTPException
from sqlalchemy.orm import Session

from . import models
from .database import get_db

ROL_ADMIN_SISTEMA = "admin_sistema"
ROL_ADMIN_CLIENTE = "admin_cliente"
ROL_NORMAL = "normal"
ROLES_VALIDOS = {ROL_ADMIN_SISTEMA, ROL_ADMIN_CLIENTE, ROL_NORMAL}

MASTER_PASSWORD = os.environ.get("TALLER_MASTER_PASSWORD", "1111")

# En desarrollo (nuestro Docker local) la clave maestra vale tambien para
# entrar como administrador, sin necesidad de contrasena real. Si esto se
# comparte o se despliega de verdad, TALLER_DEV_MODE se deja sin poner (o en
# "false") y entonces la maestra solo sirve para usuarios "normal" -- los
# admin_sistema/admin_cliente necesitan su contrasena real.
DEV_MODE = os.environ.get("TALLER_DEV_MODE", "false").strip().lower() in ("1", "true", "yes", "si")

_SESSIONS: dict[str, int] = {}  # en memoria: se pierden al reiniciar (y --reload reinicia)


def hash_password(password: str) -> str:
    sal = secrets.token_hex(16)
    return f"{sal}${hashlib.sha256((sal + password).encode()).hexdigest()}"


def verify_password(password: str, password_hash: str) -> bool:
    try:
        sal, digest = password_hash.split("$", 1)
    except ValueError:
        return False
    return hashlib.sha256((sal + password).encode()).hexdigest() == digest


def check_password(password: str, usuario: models.Usuario) -> bool:
    if password == MASTER_PASSWORD:
        if usuario.rol == ROL_NORMAL:
            return True
        if DEV_MODE:
            return True
    if usuario.password_hash:
        return verify_password(password, usuario.password_hash)
    return False


def create_session(usuario_id: int) -> str:
    token = secrets.token_hex(24)
    _SESSIONS[token] = usuario_id
    return token


def destroy_session(token: str) -> None:
    _SESSIONS.pop(token, None)


def get_current_usuario(
    x_session_token: str | None = Header(default=None),
    db: Session = Depends(get_db),
) -> models.Usuario:
    if not x_session_token or x_session_token not in _SESSIONS:
        raise HTTPException(
            status_code=401, detail="Sesion no valida -- inicia sesion de nuevo."
        )
    usuario_id = _SESSIONS[x_session_token]
    usuario = db.query(models.Usuario).filter(models.Usuario.id == usuario_id).first()
    if usuario is None or not usuario.activo:
        raise HTTPException(
            status_code=401, detail="Sesion no valida -- inicia sesion de nuevo."
        )
    return usuario


def require_roles(*roles: str):
    def dependency(
        usuario: models.Usuario = Depends(get_current_usuario),
    ) -> models.Usuario:
        if usuario.rol not in roles:
            raise HTTPException(status_code=403, detail="No tienes permiso para esto.")
        return usuario

    return dependency


def puede_gestionar_cliente(usuario: models.Usuario, cliente_id: int | None) -> bool:
    if usuario.rol == ROL_ADMIN_SISTEMA:
        return True
    if usuario.rol == ROL_ADMIN_CLIENTE:
        return usuario.cliente_id == cliente_id
    return False
