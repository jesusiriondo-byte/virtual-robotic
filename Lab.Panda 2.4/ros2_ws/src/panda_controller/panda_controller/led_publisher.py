#!/usr/bin/env python3
"""
Puente ROS2 -> Raspberry Pi Pico W: escucha comandos de color en el topic
`/comando_led` (std_msgs/String: "R"/"G"/"B"/"rojo"/"verde"/"azul"/"0"/
"apagar") y se los reenvia a la Pico W por Wi-Fi (socket TCP), que los
interpreta y enciende el LED RGB correspondiente.

Copiado tal cual (sin cambios de logica, solo de nombre de paquete en los
comentarios) desde `led_publisher.py` del proyecto hermano del UR5e
(`Lad.Claude 2.2`), que a su vez lo adapto de
`MEGA/Robotica/proyectos_ros2/led_publisher.py` +
`proyectos_ros2/micropython/main.py` -- una carpeta hermana totalmente
distinta a este proyecto (no usa Webots ni el Panda, era para un TurtleBot en
un laberinto). No se ha tocado ni se va a tocar esa carpeta; solo se ha
copiado y adaptado el patron de comunicacion con la Pico.

En el lado de la Pico W corre `proyectos_ros2/micropython/main.py` (ese
firmware NO forma parte de este repo, sigue viviendo en la otra carpeta): un
servidor TCP no bloqueante en el puerto 5001 que espera un caracter
("R"/"G"/"B"/"0") y enciende/apaga los 3 LEDs cableados en los pines:
  - rojo  -> GPIO 13
  - verde -> GPIO 14
  - azul  -> GPIO 15
Si la Pico tiene otra IP, otro puerto, o los pines cambiaron, se ajusta con
los parametros ROS2 `pico_ip` / `pico_port` (no hace falta tocar codigo):

    ros2 run panda_controller led_publisher --ros-args -p pico_ip:=192.168.1.101

Como lanzarlo (en cualquier momento, no depende de Webots ni del driver del
Panda -- solo necesita ver la Pico W por la red):

    ros2 run panda_controller led_publisher
"""

import socket

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class LedPublisherNode(Node):
    def __init__(self):
        super().__init__('led_publisher')

        self.declare_parameter('pico_ip', '192.168.1.101')
        self.declare_parameter('pico_port', 5001)
        self.declare_parameter('socket_timeout', 2.0)

        self.pico_ip = self.get_parameter('pico_ip').value
        self.pico_port = int(self.get_parameter('pico_port').value)
        self.socket_timeout = float(self.get_parameter('socket_timeout').value)

        self.create_subscription(String, '/comando_led', self._on_comando, 10)

        self.get_logger().info(
            f'led_publisher listo, reenviando /comando_led a la Pico W en '
            f'{self.pico_ip}:{self.pico_port}'
        )
        # Limpiar la alarma al arrancar (sesion 2026-09-09, aviso real del
        # usuario: "los led llevan tiempo como parada de emergencia y no
        # estaban funcionando"). La Pico guarda su estado ella sola: si se
        # quedo parpadeando por una parada (boton fisico, o la parada
        # automatica por pinza dislocada) sigue asi para siempre, porque el
        # rearme solo se envia desde el panel -- y si entre medias se matan
        # y relanzan los procesos ROS, ese rearme no llega NUNCA y el LED
        # se queda en alarma aunque la celda este recien arrancada y sana.
        # Arrancar limpio evita quedarse con un aviso visual mintiendo.
        if self._enviar_a_pico('REARME'):
            self.get_logger().info('Alarma previa limpiada en la Pico W (rearme de arranque).')

    def _enviar_a_pico(self, estado: str) -> bool:
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(self.socket_timeout)
            sock.connect((self.pico_ip, self.pico_port))
            sock.sendall(estado.encode('utf-8'))
            sock.close()
            return True
        except Exception as exc:
            self.get_logger().error(f'No se pudo conectar con la Pico W ({self.pico_ip}:{self.pico_port}): {exc}')
            return False

    def _on_comando(self, msg: String):
        comando = msg.data.strip().lower()
        if comando in ('r', 'rojo', 'red'):
            if self._enviar_a_pico('R'):
                self.get_logger().info('LED -> ROJO')
        elif comando in ('g', 'verde', 'green'):
            if self._enviar_a_pico('G'):
                self.get_logger().info('LED -> VERDE')
        elif comando in ('b', 'azul', 'blue'):
            if self._enviar_a_pico('B'):
                self.get_logger().info('LED -> AZUL')
        elif comando in ('0', 'apagar', 'off'):
            if self._enviar_a_pico('0'):
                self.get_logger().info('LED -> APAGADO')
        elif comando in ('rearme', 'rearm'):
            # Unico comando que la Pico acepta incluso en parada (ver
            # main.py): apaga el parpadeo rojo sin tener que reiniciarla.
            if self._enviar_a_pico('REARME'):
                self.get_logger().info('Rearme enviado a la Pico W')
        elif comando in ('parada', 'stop'):
            # Parada manual desde el panel (no el boton fisico): mismo
            # efecto en la Pico que pulsar el boton -- parpadeo rojo local.
            if self._enviar_a_pico('PARADA'):
                self.get_logger().info('Parada manual enviada a la Pico W')
        else:
            self.get_logger().warning(f'Comando LED no reconocido: "{msg.data}" (usa R, G, B o 0).')


def main(args=None):
    rclpy.init(args=args)
    node = LedPublisherNode()
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
