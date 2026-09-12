import datetime
from typing import Optional

from pydantic import BaseModel, ConfigDict


class ProductoOut(BaseModel):
    model_config = ConfigDict(from_attributes=True)
    id: int
    nombre: str
    color: str
    activo: bool


class ProductoCreate(BaseModel):
    nombre: str
    color: str


class ProductoUpdate(BaseModel):
    nombre: Optional[str] = None
    color: Optional[str] = None
    activo: Optional[bool] = None


class UsuarioOut(BaseModel):
    model_config = ConfigDict(from_attributes=True)
    id: int
    username: str
    nombre_completo: str
    rol: str
    cliente_id: Optional[int] = None
    sucursal: Optional[str] = None
    activo: bool
    creado_en: datetime.datetime


class UsuarioCreate(BaseModel):
    username: str
    nombre_completo: str
    rol: str = "normal"
    cliente_id: Optional[int] = None
    sucursal: Optional[str] = None
    password: Optional[str] = None


class UsuarioUpdate(BaseModel):
    nombre_completo: Optional[str] = None
    rol: Optional[str] = None
    sucursal: Optional[str] = None
    activo: Optional[bool] = None
    password: Optional[str] = None


class LoginRequest(BaseModel):
    username: str
    password: str


class LoginResponse(BaseModel):
    token: str
    usuario: UsuarioOut


class ClientePrimerUsuario(BaseModel):
    username: str
    nombre_completo: str
    password: Optional[str] = None


class ClienteCreate(BaseModel):
    razon_social: str
    cif: Optional[str] = None
    primer_usuario: ClientePrimerUsuario


class ClienteUpdate(BaseModel):
    razon_social: Optional[str] = None
    cif: Optional[str] = None
    activo: Optional[bool] = None


class ClienteOut(BaseModel):
    model_config = ConfigDict(from_attributes=True)
    id: int
    razon_social: str
    cif: Optional[str] = None
    activo: bool
    creado_en: datetime.datetime


class ClienteProductoAsignar(BaseModel):
    producto_id: int


class PedidoCreate(BaseModel):
    producto_id: int
    cantidad_pedida: int


class UsuarioResumen(BaseModel):
    model_config = ConfigDict(from_attributes=True)
    id: int
    username: str
    nombre_completo: str
    sucursal: Optional[str] = None


class PedidoOut(BaseModel):
    model_config = ConfigDict(from_attributes=True)
    id: int
    producto_id: int
    cliente_id: int
    usuario_id: int
    usuario: UsuarioResumen
    cantidad_pedida: int
    cantidad_completada: int
    estado: str
    urgente: bool
    creado_en: datetime.datetime
    producto: ProductoOut
    stock_disponible: int = 0
    # Calculados desde los MovimientoStock del pedido, no son columnas (ver
    # _con_tiempos_de_proceso en main.py). None = aun no ha recibido piezas.
    segundos_proceso: Optional[float] = None
    segundos_total: Optional[float] = None


class PedidoUpdate(BaseModel):
    urgente: bool


class CuboClasificado(BaseModel):
    """Evento de la celda: un cubo ha llegado a su caja de color.

    Siempre suma 1 al stock. Si el reparto automatico esta activo (el unico
    interruptor de la regla), se aplica ademas a un pedido pendiente de ese
    color: el indicado en pedido_id si sigue activo, si no el mas urgente y
    antiguo. forzar_reparto se admite por compatibilidad pero no cambia nada.
    """

    color: str
    pedido_id: Optional[int] = None
    forzar_reparto: bool = False


class AjusteStock(BaseModel):
    producto_id: int
    cantidad: int


class StockOut(BaseModel):
    model_config = ConfigDict(from_attributes=True)
    producto_id: int
    cantidad_actual: int
    actualizado_en: Optional[datetime.datetime] = None
    producto: ProductoOut


class MovimientoStockOut(BaseModel):
    model_config = ConfigDict(from_attributes=True)
    id: int
    producto_id: int
    tipo: str
    cantidad: int
    motivo: str
    pedido_id: Optional[int] = None
    usuario_id: Optional[int] = None
    fecha: datetime.datetime


class ConfiguracionAlmacenOut(BaseModel):
    model_config = ConfigDict(from_attributes=True)
    reparto_automatico: bool


class ConfiguracionAlmacenUpdate(BaseModel):
    reparto_automatico: bool


class CuboClasificadoResultado(BaseModel):
    color: str
    stock_actual: int
    pedido: Optional[PedidoOut] = None


class EventoProduccionCreate(BaseModel):
    robot: str
    color: str
    tipo: str


class EventoProduccionOut(BaseModel):
    model_config = ConfigDict(from_attributes=True)
    id: int
    robot: str
    color: str
    tipo: str
    fecha: datetime.datetime


class DiagnosticoColor(BaseModel):
    color: str
    producto_nombre: str
    led_loader: int
    led_sorter: int
    agarre_falso: int
    limite_alcance: int
    fallo_definitivo: int
    piezas_reales: int
    piezas_reales_total: int
    estado: str


class DiagnosticoOut(BaseModel):
    ventana_minutos: int
    por_color: list[DiagnosticoColor]
    eventos_recientes: list[EventoProduccionOut]


class AuditLogOut(BaseModel):
    model_config = ConfigDict(from_attributes=True)
    id: int
    tabla: str
    registro_id: int
    accion: str
    usuario_id: Optional[int] = None
    detalle: Optional[str] = None
    fecha: datetime.datetime
