import machine
import time

# 1. Configurar el LED interno de la Pico W (Se llama "LED" en MicroPython)
led_interno = machine.Pin("LED", machine.Pin.OUT)

# 2. Configurar tu LED externo físico en el Pin 14
led_externo = machine.Pin(14, machine.Pin.OUT)

print("Iniciando prueba de hardware (Parpadeo a 1 segundo)...")

while True:
    # Encender ambos
    led_interno.value(1)
    led_externo.value(1)
    print("🟢 ENCENDIDOS")
    time.sleep(1)
    
    # Apagar ambos
    led_interno.value(0)
    led_externo.value(0)
    print("⚫ APAGADOS")
    time.sleep(1)