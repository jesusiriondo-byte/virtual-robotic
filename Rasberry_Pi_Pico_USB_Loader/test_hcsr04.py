# Version: 2026-09-11 20:05 -- test suelto del HC-SR04 (GP20=TRIG, GP21=ECHO)
# No toca main.py. Se ejecuta a mano con mpremote/Thonny, solo para
# comprobar el sensor recien montado antes de meterlo en el codigo real.
import machine
import time

trig = machine.Pin(20, machine.Pin.OUT)
echo = machine.Pin(21, machine.Pin.IN)
trig.value(0)
time.sleep_ms(50)


def medir_distancia_cm():
    trig.value(0)
    time.sleep_us(5)
    trig.value(1)
    time.sleep_us(10)
    trig.value(0)
    try:
        duracion_us = machine.time_pulse_us(echo, 1, 30000)  # timeout 30ms ~ 5m
    except OSError:
        return None
    if duracion_us < 0:
        return None
    return duracion_us / 58.0


print('Test HC-SR04 -- GP20=TRIG, GP21=ECHO, 20 lecturas cada 300ms')
for i in range(20):
    d = medir_distancia_cm()
    if d is None:
        print(i, '-- sin eco (fuera de rango o mal cableado)')
    else:
        print(i, '-- distancia: {:.1f} cm'.format(d))
    time.sleep_ms(300)
print('Test terminado.')
