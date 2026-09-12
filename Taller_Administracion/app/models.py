import datetime

from sqlalchemy import Boolean, Column, DateTime, ForeignKey, Integer, String
from sqlalchemy.orm import relationship

from .database import Base


class Cliente(Base):
    __tablename__ = "clientes"

    id = Column(Integer, primary_key=True)
    razon_social = Column(String, nullable=False)
    cif = Column(String, nullable=True)
    activo = Column(Boolean, nullable=False, default=True)
    creado_en = Column(DateTime, default=datetime.datetime.utcnow)
    creado_por_id = Column(Integer, ForeignKey("usuarios.id"), nullable=True)
    modificado_en = Column(DateTime, nullable=True)
    modificado_por_id = Column(Integer, ForeignKey("usuarios.id"), nullable=True)

    usuarios = relationship(
        "Usuario", foreign_keys="Usuario.cliente_id", back_populates="cliente"
    )
    pedidos = relationship("Pedido", back_populates="cliente")


class Usuario(Base):
    __tablename__ = "usuarios"

    id = Column(Integer, primary_key=True)
    username = Column(String, unique=True, nullable=False)
    password_hash = Column(String, nullable=True)
    nombre_completo = Column(String, nullable=False)
    rol = Column(String, nullable=False, default="normal")
    cliente_id = Column(Integer, ForeignKey("clientes.id"), nullable=True)
    sucursal = Column(String, nullable=True)
    activo = Column(Boolean, nullable=False, default=True)
    creado_en = Column(DateTime, default=datetime.datetime.utcnow)
    creado_por_id = Column(Integer, ForeignKey("usuarios.id"), nullable=True)
    modificado_en = Column(DateTime, nullable=True)
    modificado_por_id = Column(Integer, ForeignKey("usuarios.id"), nullable=True)

    cliente = relationship(
        "Cliente", foreign_keys=[cliente_id], back_populates="usuarios"
    )
    pedidos = relationship("Pedido", back_populates="usuario")


class AuditLog(Base):
    __tablename__ = "audit_log"

    id = Column(Integer, primary_key=True)
    tabla = Column(String, nullable=False)
    registro_id = Column(Integer, nullable=False)
    accion = Column(String, nullable=False)  # alta / baja / modificacion
    usuario_id = Column(Integer, ForeignKey("usuarios.id"), nullable=True)
    detalle = Column(String, nullable=True)
    fecha = Column(DateTime, default=datetime.datetime.utcnow)


class Producto(Base):
    __tablename__ = "productos"

    id = Column(Integer, primary_key=True)
    nombre = Column(String, unique=True, nullable=False)
    color = Column(String, unique=True, nullable=False)
    activo = Column(Boolean, nullable=False, default=True)  # baja logica: nunca se borra

    pedidos = relationship("Pedido", back_populates="producto")
    stock = relationship("Stock", back_populates="producto", uselist=False)


class ClienteProducto(Base):
    __tablename__ = "cliente_productos"

    cliente_id = Column(Integer, ForeignKey("clientes.id"), primary_key=True)
    producto_id = Column(Integer, ForeignKey("productos.id"), primary_key=True)
    asignado_en = Column(DateTime, default=datetime.datetime.utcnow)
    asignado_por_id = Column(Integer, ForeignKey("usuarios.id"), nullable=True)

    cliente = relationship("Cliente", backref="productos_asignados")
    producto = relationship("Producto", backref="clientes_asignados")


class Pedido(Base):
    __tablename__ = "pedidos"

    id = Column(Integer, primary_key=True)
    producto_id = Column(Integer, ForeignKey("productos.id"), nullable=False)
    cliente_id = Column(Integer, ForeignKey("clientes.id"), nullable=False)
    usuario_id = Column(Integer, ForeignKey("usuarios.id"), nullable=False)
    cantidad_pedida = Column(Integer, nullable=False)
    cantidad_completada = Column(Integer, nullable=False, default=0)
    estado = Column(String, nullable=False, default="pendiente")
    urgente = Column(Boolean, nullable=False, default=False)
    creado_en = Column(DateTime, default=datetime.datetime.utcnow)

    producto = relationship("Producto", back_populates="pedidos")
    cliente = relationship("Cliente", back_populates="pedidos")
    usuario = relationship("Usuario", back_populates="pedidos")


class Stock(Base):
    __tablename__ = "stock"

    producto_id = Column(Integer, ForeignKey("productos.id"), primary_key=True)
    cantidad_actual = Column(Integer, nullable=False, default=0)
    actualizado_en = Column(
        DateTime, default=datetime.datetime.utcnow, onupdate=datetime.datetime.utcnow
    )

    producto = relationship("Producto", back_populates="stock")


class MovimientoStock(Base):
    """Libro de solo anadir: nunca se edita ni se borra una fila existente."""

    __tablename__ = "movimientos_stock"

    id = Column(Integer, primary_key=True)
    producto_id = Column(Integer, ForeignKey("productos.id"), nullable=False)
    tipo = Column(String, nullable=False)  # entrada / salida
    cantidad = Column(Integer, nullable=False)
    motivo = Column(String, nullable=False)  # produccion, envio_pedido, ajuste_manual
    pedido_id = Column(Integer, ForeignKey("pedidos.id"), nullable=True)
    usuario_id = Column(Integer, ForeignKey("usuarios.id"), nullable=True)
    fecha = Column(DateTime, default=datetime.datetime.utcnow)


class EventoProduccion(Base):
    __tablename__ = "eventos_produccion"

    id = Column(Integer, primary_key=True)
    robot = Column(String, nullable=False)  # loader / sorter / desconocido
    color = Column(String, nullable=False)
    tipo = Column(String, nullable=False)
    fecha = Column(DateTime, default=datetime.datetime.utcnow)


class ConfiguracionAlmacen(Base):
    __tablename__ = "configuracion_almacen"

    id = Column(Integer, primary_key=True)  # fila unica, id = 1
    reparto_automatico = Column(Boolean, nullable=False, default=True)
