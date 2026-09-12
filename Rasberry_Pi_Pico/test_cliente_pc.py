# Cliente de prueba para el servidor TCP de main.py.
# EJECUTAR EN EL PC (no en la Pico), con la Pico ya encendida y conectada al Wi-Fi.
#
# Uso:
#   python3 test_cliente_pc.py <ip_de_la_pico>
#   python3 test_cliente_pc.py            (usa la IP por defecto de abajo)

import socket
import sys
import time

PICO_IP_POR_DEFECTO = "192.168.1.101"
PICO_PORT = 5001


def enviar(ip, comando):
    with socket.create_connection((ip, PICO_PORT), timeout=3) as s:
        s.sendall(comando.encode("utf-8"))
    print("Enviado:", comando)


def main():
    ip = sys.argv[1] if len(sys.argv) > 1 else PICO_IP_POR_DEFECTO
    print(f"Probando la Pico en {ip}:{PICO_PORT} ...")

    secuencia = [
        ("R", "debería encenderse el LED rojo"),
        ("G", "debería encenderse el LED verde"),
        ("B", "debería encenderse el LED azul"),
        ("0", "deberían apagarse los tres"),
    ]

    for comando, esperado in secuencia:
        try:
            enviar(ip, comando)
            print("  ->", esperado)
        except OSError as e:
            print(f"  -> ERROR conectando con la Pico: {e}")
            print("     Comprueba: Pico encendida, misma red Wi-Fi, IP correcta.")
            return
        time.sleep(2)

    print("Prueba terminada.")


if __name__ == "__main__":
    main()
