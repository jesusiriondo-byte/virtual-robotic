import machine
import time

# 1. Configuración de los pines de cada color
# Debe coincidir con el mapeo usado en main.py: rojo=13, verde=14, azul=15
PIN_ROJO = 13
PIN_VERDE = 14
PIN_AZUL = 15

print("--- Iniciando Prueba de Hardware del LED RGB ---")

# 2. Inicializamos los pines como salida
led_rojo = machine.Pin(PIN_ROJO, machine.Pin.OUT)
led_verde = machine.Pin(PIN_VERDE, machine.Pin.OUT)
led_azul = machine.Pin(PIN_AZUL, machine.Pin.OUT)

# Función para apagar todo antes de cambiar de color
def apagar_todos():
    led_rojo.value(0)
    led_verde.value(0)
    led_azul.value(0)

# 3. Bucle de prueba: va a encender un color cada segundo
try:
    for i in range(5):  # Va a hacer el ciclo 5 veces
        print(f"Ciclo {i+1}...")
        
        # Encender Rojo
        print(" -> Rojo 🔴")
        apagar_todos()
        led_rojo.value(1)
        time.sleep(1.0)
        
        # Encender Verde
        print(" -> Verde 🟢")
        apagar_todos()
        led_verde.value(1)
        time.sleep(1.0)
        
        # Encender Azul
        print(" -> Azul 🔵")
        apagar_todos()
        led_azul.value(1)
        time.sleep(1.0)

    # Al terminar la prueba, dejamos todo apagado
    apagar_todos()
    print("--- ¡Prueba terminada con éxito! Todos los colores testeados. ---")

except KeyboardInterrupt:
    # Si lo detienes con Ctrl+C, apaga el LED para que no se quede encendido
    apagar_todos()
    print("\nPrueba cancelada por el usuario.")