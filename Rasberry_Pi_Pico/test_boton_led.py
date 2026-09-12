from machine import Pin
import time

# Prueba de hardware: botón (GPIO16, pull-up interna) enciende el LED
# rojo (GPIO13) mientras se mantiene pulsado. No usa Wi-Fi ni ROS2 --
# solo sirve para comprobar que el cableado del botón funciona antes de
# tocar main.py.

led_rojo = Pin(13, Pin.OUT)
boton = Pin(16, Pin.IN, Pin.PULL_UP)

led_rojo.value(0)  # apagado al arrancar

print("Listo. Pulsa el boton -> LED rojo encendido. Sueltalo -> apagado.")

while True:
    if boton.value() == 0:      # pull-up: reposo=1, pulsado=0
        led_rojo.value(1)
    else:
        led_rojo.value(0)
    time.sleep(0.05)
