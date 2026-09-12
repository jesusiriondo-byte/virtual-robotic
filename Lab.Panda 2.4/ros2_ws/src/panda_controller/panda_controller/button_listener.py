#!/usr/bin/env python3
"""
Raspberry Pi Pico W -> ROS2: escucha el aviso de "parada de emergencia"
que manda la Pico W cuando se pulsa el boton fisico cableado en su
GPIO16, y lo publica en el topic `/emergency_stop` (std_msgs/Bool).

Es el sentido CONTRARIO a `led_publisher.py` (que manda PC -> Pico). Este
nodo abre un servidor TCP (puerto 5002 por defecto) y espera a que la
Pico se conecte y mande el texto "STOP".

En el lado de la Pico corre `Rasberry_Pi_Pico/main.py`: al detectar la
pulsacion (con antirrebote), corta el LED de forma INMEDIATA y local (no
depende de la red ni de este nodo) y, ademas, intenta avisar al PC
abriendo una conexion a `PC_IP:PC_PUERTO` (constantes en
`Rasberry_Pi_Pico/wifi_config.py` -- hay que rellenar `PC_IP` con la IP
del ordenador en la misma red que la Pico).

IMPORTANTE -- alcance de este nodo: solo publica `/emergency_stop`. NO
para por si solo ningun movimiento del brazo: los nodos que mueven el
Panda (`pick_and_place.py`, `visit_balls.py`, `vision_lift_cube.py`,
`best_color_repeat_lift.py`, `cube_shuttle_demo.py`, `teleop_*.py`, etc.)
tendrian que suscribirse a este topic y cortar su propio bucle -- eso no
se ha tocado aqui a proposito, para no arriesgar nada en esos nodos sin
haberlos revisado primero.

Como lanzarlo (no depende de Webots, solo necesita ver la Pico W por la
red y tener el puerto 5002 libre):

    ros2 run panda_controller button_listener

Para probarlo sin la Pico real (desde otra terminal del contenedor):

    echo -n "STOP" | ncat <IP_DEL_PC> 5002
"""

import socket
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

REINTENTO_BIND_S = 5.0


class ButtonListenerNode(Node):
    def __init__(self):
        super().__init__('button_listener')

        self.declare_parameter('listen_host', '0.0.0.0')
        self.declare_parameter('listen_port', 5002)

        self.listen_host = self.get_parameter('listen_host').value
        self.listen_port = int(self.get_parameter('listen_port').value)

        self.publisher = self.create_publisher(Bool, '/emergency_stop', 10)

        self._stop_thread = threading.Event()
        self._server_thread = threading.Thread(target=self._servir, daemon=True)
        self._server_thread.start()

        self.get_logger().info(
            f'button_listener escuchando en {self.listen_host}:{self.listen_port}, '
            'esperando el aviso de parada de la Pico W...'
        )

    def _abrir_servidor(self):
        """Abre el socket de escucha, reintentando cada REINTENTO_BIND_S si el
        puerto esta ocupado (sesion 2026-09-11). Antes, un fallo de bind() --
        tipicamente otro button_listener suelto de una prueba anterior -- hacia
        que este hilo terminara en silencio: el nodo seguia vivo pero sordo, y
        el boton fisico dejaba de parar nada si se cerraba el otro proceso."""
        intentos = 0
        while not self._stop_thread.is_set():
            servidor = None
            try:
                servidor = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                servidor.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                servidor.bind((self.listen_host, self.listen_port))
                servidor.listen(1)
                if intentos:
                    self.get_logger().info(
                        f'Servidor de parada abierto por fin en {self.listen_host}:{self.listen_port}.')
                return servidor
            except Exception as exc:
                if servidor is not None:
                    servidor.close()
                intentos += 1
                self.get_logger().error(
                    f'No se pudo abrir el servidor de parada ({exc}) -- '
                    f'reintento en {REINTENTO_BIND_S:.0f}s (intento {intentos}).')
                self._stop_thread.wait(REINTENTO_BIND_S)
        return None

    def _servir(self):
        servidor = self._abrir_servidor()
        if servidor is None:
            return

        while not self._stop_thread.is_set():
            try:
                servidor.settimeout(1.0)
                conexion, direccion = servidor.accept()
            except socket.timeout:
                continue
            except Exception as exc:
                self.get_logger().error(f'Error aceptando conexion: {exc}')
                continue

            try:
                datos = conexion.recv(1024)
                mensaje = datos.decode('utf-8').strip().upper() if datos else ''
                if mensaje == 'STOP':
                    self.get_logger().warning(
                        'PARADA DE EMERGENCIA recibida de la Pico W -> publicando /emergency_stop'
                    )
                    self.publisher.publish(Bool(data=True))
                else:
                    self.get_logger().warning(f'Mensaje inesperado en el puerto de parada: "{mensaje}"')
            except Exception as exc:
                self.get_logger().error(f'Error leyendo la conexion de parada: {exc}')
            finally:
                conexion.close()

        servidor.close()

    def destroy_node(self):
        self._stop_thread.set()
        self._server_thread.join(timeout=2.0)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ButtonListenerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
