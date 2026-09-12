#!/usr/bin/env python3
"""Puente ROS2 -> Raspberry Pi Pico (SIN wifi, solo USB): escucha comandos
de color en un topic (por defecto `/comando_led_loader`, std_msgs/String:
"R"/"G"/"B"/"rojo"/"verde"/"azul"/"0"/"apagar") y se los reenvia por el
puerto serie USB a la Pico, que los interpreta y enciende el LED RGB
correspondiente.

Hermano de `led_publisher.py` (ese usa Wi-Fi/TCP para la Pico W del
Sorter, que ademas lleva el boton fisico de parada). Esta Pico nueva
(sesion 2026-08-30) es una Pico normal sin chip de radio -- mismo cableado
de LED (GPIO 13/14/15) pero sin forma de hablar por red, asi que se
controla por el cable USB con el que esta conectada al PC, reenviado al
contenedor via 'devices:' en docker-compose.yml.

En el lado de la Pico corre `Rasberry_Pi_Pico_USB_Loader/main.py`: lee
comandos de texto terminados en '\\n' de su propio USB serie (sys.stdin,
con select.poll para no bloquear) y enciende/apaga los 3 LEDs. Sigue
funcionando SIN esta Pico conectada -- si el puerto serie no existe o se
desenchufa, este nodo avisa por log una vez y sigue vivo, reintentando la
apertura en el siguiente comando (igual que led_publisher.py con la Pico
W apagada).

Ademas (sesion 2026-09-01), esta Pico tiene tambien un boton fisico de
parada de emergencia en GPIO16: al no tener wifi, avisa al PC imprimiendo
la linea 'BOTON_PARADA' por el mismo USB en vez de abrir un socket como
la Pico W (ver button_listener.py) -- un hilo de fondo de este nodo la
lee y publica /emergency_stop, misma parada GLOBAL de toda la celda.

Si el puerto serie cambia de nombre (por ejemplo si algun dia se conecta
mas de una Pico por USB a la vez y el orden ttyACM0/ttyACM1 se invierte),
ajustar con el parametro ROS2 'serial_port' -- mejor usar la ruta estable
de /dev/serial/by-id/... (ver README) que ttyACM0 a secas:

    ros2 run panda_controller led_publisher_usb --ros-args -p serial_port:=/dev/ttyACM0

Como lanzarlo:

    ros2 run panda_controller led_publisher_usb
"""

import threading
import time

import serial

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String

# Tabla color -> RGB para el LED "producto" (sesion 2026-08-31, LED nuevo
# de 4 patas por PWM en la Pico del Loader, ver Rasberry_Pi_Pico_USB_Loader
# /main.py). Vive aqui, no en la Pico, porque es la unica tabla en todo el
# proyecto y asi anadir un producto nuevo es una linea en un solo sitio.
# Ampliar aqui cuando haya un color de cubo nuevo (ver PALETA_COLORES en
# Taller_Administracion/app/main.py -- debe crecer A LA VEZ, mismas
# letras, mismo orden de ideas: R/G/B tienen cubo fisico real en Webots;
# Y/M/C/W (sesion 2026-09-02, catalogo ampliable desde el panel web) son
# combinaciones del mismo LED de 3 canales -- se pueden dar de alta y
# mostrar en este LED desde ya, pero el Sorter todavia no fabrica ni
# clasifica solo esos colores (no existe el cubo fisico correspondiente
# en Webots) -- de momento solo sirven para produccion marcada a mano
# ("+1 pieza" del panel de teleop), no para la celda automatica.
COLOR_A_RGB = {
    'R': (255, 0, 0),
    'G': (0, 255, 0),
    'B': (0, 0, 255),
    # Amarillo calibrado a ojo en vivo (sesion 2026-09-02): a partes
    # iguales (255,255,0) o incluso (255,200,0) se ve MAS VERDE que
    # amarillo -- el chip verde del LED satura mucho mas que el rojo al
    # mismo nivel. (255,60,0) confirmado por el usuario como el que mas
    # se acerca a amarillo real. Sigue viendose el verde y el rojo por
    # SEPARADO en vez de mezclados sin un difusor fisico sobre el LED
    # (cinta esmerilada, gota de silicona, lijar la cupula...) -- eso es
    # un tema optico del LED, no de estos valores.
    'Y': (255, 60, 0),
    'M': (200, 0, 200),
    'C': (0, 200, 200),
    'W': (255, 255, 255),
}


class LedPublisherUsbNode(Node):
    def __init__(self):
        super().__init__('led_publisher_usb')

        self.declare_parameter('serial_port', '/dev/ttyACM_LOADER')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('led_topic', '/comando_led_loader')
        self.declare_parameter('led_topic_producto', '/comando_led_producto')

        self.serial_port = self.get_parameter('serial_port').value
        self.baudrate = int(self.get_parameter('baudrate').value)
        self.ser = None
        self._puerto_lock = threading.Lock()
        self._abrir_puerto(avisar_error=True)

        self.create_subscription(String, self.get_parameter('led_topic').value, self._on_comando, 10)
        self.create_subscription(
            String, self.get_parameter('led_topic_producto').value, self._on_comando_producto, 10)

        # Parada de emergencia (sesion 2026-09-01): esta Pico tiene ahora
        # tambien un boton fisico (GPIO16), pero al no tener wifi no puede
        # abrir un socket como hace la Pico W (ver button_listener.py) --
        # en vez de eso, imprime la linea centinela 'BOTON_PARADA' por el
        # mismo USB, y este hilo la detecta y publica /emergency_stop.
        self.pub_stop = self.create_publisher(Bool, '/emergency_stop', 10)
        self._stop_reader = threading.Event()
        self._reader_thread = threading.Thread(target=self._leer_de_pico, daemon=True)
        self._reader_thread.start()

        self.get_logger().info(
            f'led_publisher_usb listo, reenviando "{self.get_parameter("led_topic").value}" '
            f'y "{self.get_parameter("led_topic_producto").value}" '
            f'a la Pico por {self.serial_port} (baudrate {self.baudrate}), '
            'escuchando su boton de parada'
        )
        # Arrancar con los dos LEDs limpios (sesion 2026-09-09, aviso real
        # del usuario: "los led llevan tiempo como parada de emergencia y no
        # estaban funcionando" / "y el de producto esta en rojo"). La Pico
        # mantiene su estado por su cuenta, asi que sobrevive a que se maten
        # y relancen los procesos ROS: un parpadeo de alarma de una parada
        # anterior, o el color de producto de un lote que se corto a mitad,
        # se quedan puestos para siempre porque quien los apaga (el rearme
        # del panel, o el fin de lote del Sorter) ya no va a llegar nunca.
        # Al arrancar no hay ni alarma ni lote en curso: dejar los dos LEDs
        # diciendo la verdad.
        if self._enviar_a_pico('REARME'):
            self.get_logger().info('Alarma previa limpiada en la Pico del Loader (rearme de arranque).')
        if self._enviar_a_pico('PROD:0,0,0'):
            self.get_logger().info('LED de producto apagado al arrancar (sin lote en curso).')

    def _abrir_puerto(self, avisar_error=False):
        with self._puerto_lock:
            try:
                self.ser = serial.Serial(self.serial_port, self.baudrate, timeout=1.0)
                return True
            except Exception as exc:
                self.ser = None
                if avisar_error:
                    self.get_logger().error(
                        f'No se pudo abrir el puerto serie de la Pico ({self.serial_port}): {exc}'
                    )
                return False

    def _enviar_a_pico(self, estado: str) -> bool:
        if self.ser is None and not self._abrir_puerto():
            return False
        try:
            self.ser.write((estado + '\n').encode('utf-8'))
            return True
        except Exception as exc:
            self.get_logger().error(f'Error escribiendo en {self.serial_port}: {exc}')
            with self._puerto_lock:
                try:
                    self.ser.close()
                except Exception:
                    pass
                self.ser = None
            return False

    def _leer_de_pico(self):
        """Hilo de fondo: lee lineas que manda la Pico por su cuenta (el
        print('BOTON_PARADA') del boton fisico) mientras el nodo vive.
        Reintenta abrir el puerto solo si _enviar_a_pico no lo ha hecho
        ya -- comparte self.ser con el hilo principal via _puerto_lock."""
        while not self._stop_reader.is_set():
            if self.ser is None:
                if not self._abrir_puerto():
                    time.sleep(1.0)
                    continue
            try:
                linea = self.ser.readline().decode('utf-8', errors='ignore').strip()
            except Exception:
                # Mismo puerto que puede estar cerrando _enviar_a_pico a la
                # vez -- no hace falta loggear cada fallo, el hilo de
                # escritura ya avisa si el puerto se ha caido de verdad.
                time.sleep(0.2)
                continue
            if linea == 'BOTON_PARADA':
                self.get_logger().warning(
                    'PARADA DE EMERGENCIA recibida de la Pico del Loader -> publicando /emergency_stop'
                )
                self.pub_stop.publish(Bool(data=True))

    def _on_comando(self, msg: String):
        comando = msg.data.strip().lower()
        if comando in ('r', 'rojo', 'red'):
            if self._enviar_a_pico('R'):
                self.get_logger().info('LED (USB) -> ROJO')
        elif comando in ('g', 'verde', 'green'):
            if self._enviar_a_pico('G'):
                self.get_logger().info('LED (USB) -> VERDE')
        elif comando in ('b', 'azul', 'blue'):
            if self._enviar_a_pico('B'):
                self.get_logger().info('LED (USB) -> AZUL')
        elif comando in ('0', 'apagar', 'off'):
            if self._enviar_a_pico('0'):
                self.get_logger().info('LED (USB) -> APAGADO')
        elif comando in ('rearme', 'rearm'):
            # Unico comando que la Pico acepta incluso en parada (ver
            # main.py): apaga el parpadeo rojo sin tener que reiniciarla.
            if self._enviar_a_pico('REARME'):
                self.get_logger().info('Rearme enviado a la Pico del Loader')
        elif comando in ('parada', 'stop'):
            # Parada manual desde el panel (no el boton fisico): mismo
            # efecto en la Pico que pulsar el boton -- parpadeo rojo local.
            if self._enviar_a_pico('PARADA'):
                self.get_logger().info('Parada manual enviada a la Pico del Loader')
        else:
            self.get_logger().warning(f'Comando LED no reconocido: "{msg.data}" (usa R, G, B o 0).')

    def _on_comando_producto(self, msg: String):
        comando = msg.data.strip().upper()
        if comando in ('0', 'APAGAR', 'OFF'):
            if self._enviar_a_pico('PROD:0,0,0'):
                self.get_logger().info('LED producto (USB) -> APAGADO')
            return
        rgb = COLOR_A_RGB.get(comando)
        if rgb is None:
            self.get_logger().warning(
                f'Color de producto no reconocido: "{msg.data}" (colores validos: '
                f'{", ".join(COLOR_A_RGB)} o 0).')
            return
        if self._enviar_a_pico('PROD:{},{},{}'.format(*rgb)):
            self.get_logger().info(f'LED producto (USB) -> {comando} (RGB {rgb})')

    def destroy_node(self):
        self._stop_reader.set()
        self._reader_thread.join(timeout=2.0)
        if self.ser is not None:
            try:
                self.ser.close()
            except Exception:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = LedPublisherUsbNode()
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
