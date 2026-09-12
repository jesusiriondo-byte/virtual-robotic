# Version: 2026-09-11 20:20 -- Pico del LOADER (USB, sin wifi) + HC-SR04
import machine
import select
import sys
import time

# =====================================================================
# Pico del robot LOADER (sesion 2026-08-30): misma configuracion de LED
# que la Pico W del Sorter (Rasberry_Pi_Pico/main.py), pero esta placa NO
# tiene chip de wifi -- solo el cable USB con el que esta conectada al PC.
# Por eso, en vez de un servidor TCP, lee los comandos directamente del
# propio puerto serie USB (sys.stdin), que es el mismo cable por el que
# habla Thonny -- y para avisar AL PC (boton de parada, ver mas abajo) usa
# ese mismo cable en sentido contrario, con un print().
# =====================================================================
led_rojo = machine.Pin(13, machine.Pin.OUT)
led_verde = machine.Pin(14, machine.Pin.OUT)
led_azul = machine.Pin(15, machine.Pin.OUT)


def apagar_rgb():
    led_rojo.value(0)
    led_verde.value(0)
    led_azul.value(0)


apagar_rgb()

# =====================================================================
# LED "PRODUCTO" (sesion 2026-08-31, LED nuevo de 4 patas): segundo LED
# RGB fisico, independiente del de arriba (que se queda EXACTAMENTE
# igual -- indica "agarre en curso" con solo R/G/B/0). Este otro muestra
# el color del producto que se esta fabricando ahora, y usa PWM en vez
# de on/off para poder mostrar cualquier color (no solo los 3 basicos),
# de cara a productos futuros que no sean R/G/B puros. Pines GP17/18/19
# (sesion 2026-09-01: movidos desde 16/17/18 porque esta placa tiene
# ademas un boton fisico cableado en GP16, ver bloque de abajo).
PWM_FREQ = 1000
pwm_prod_r = machine.PWM(machine.Pin(17))
pwm_prod_g = machine.PWM(machine.Pin(18))
pwm_prod_b = machine.PWM(machine.Pin(19))
for _pwm in (pwm_prod_r, pwm_prod_g, pwm_prod_b):
    _pwm.freq(PWM_FREQ)


def set_led_producto(r, g, b):
    """r/g/b en 0-255, igual que cualquier color RGB de 8 bits por canal."""
    pwm_prod_r.duty_u16(int(max(0, min(255, r)) / 255 * 65535))
    pwm_prod_g.duty_u16(int(max(0, min(255, g)) / 255 * 65535))
    pwm_prod_b.duty_u16(int(max(0, min(255, b)) / 255 * 65535))


set_led_producto(0, 0, 0)

# =====================================================================
# BOTON PULSADOR DE PARADA DE EMERGENCIA (sesion 2026-09-01, segundo
# boton fisico del proyecto -- el otro es el de la Pico W del Sorter, ver
# robotica_panda_boton_parada en la memoria). Mismo cableado y mismo
# comportamiento que ese: un terminal a GPIO16, el otro a GND, pull-up
# interna (reposo=1, pulsado=0). Es una parada GLOBAL: para avisar al PC
# no hay wifi aqui, asi que en vez de abrir un socket se imprime una
# linea centinela por el mismo USB que ya usa el protocolo de comandos --
# led_publisher_usb.py la detecta y publica /emergency_stop, igual que
# button_listener.py hace con el aviso por TCP de la Pico W.
# =====================================================================
boton = machine.Pin(16, machine.Pin.IN, machine.Pin.PULL_UP)

boton_estado_anterior = 1
boton_ultimo_cambio_ms = time.ticks_ms()
DEBOUNCE_MS = 40  # tiempo minimo estable antes de aceptar la pulsacion como real

# Igual que en la Pico W: una vez pulsado, el rojo parpadea hasta REARME
# (por USB, comando REARME) o hasta reiniciar la Pico -- una parada de
# emergencia no deberia des-activarse sola. Mientras dura, se ignoran los
# comandos R/G/B/0 que lleguen por USB para no pisar el parpadeo (el LED
# de producto y el PROD: si se siguen aceptando, son informativos).
parada_activa = False
PARPADEO_MS = 300  # medio periodo: 300ms encendido, 300ms apagado
parpadeo_ultimo_cambio_ms = time.ticks_ms()
parpadeo_estado = False


def actualizar_parpadeo_parada():
    global parpadeo_ultimo_cambio_ms, parpadeo_estado
    ahora = time.ticks_ms()
    if time.ticks_diff(ahora, parpadeo_ultimo_cambio_ms) >= PARPADEO_MS:
        parpadeo_ultimo_cambio_ms = ahora
        parpadeo_estado = not parpadeo_estado
        led_verde.value(0)
        led_azul.value(0)
        led_rojo.value(1 if parpadeo_estado else 0)


def comprobar_boton():
    """Debounce simple por tiempo. Se llama una vez por vuelta del bucle
    principal (cada ~50ms, ver poller.poll mas abajo)."""
    global boton_estado_anterior, boton_ultimo_cambio_ms, parada_activa
    estado = boton.value()
    ahora = time.ticks_ms()
    if estado != boton_estado_anterior:
        if time.ticks_diff(ahora, boton_ultimo_cambio_ms) >= DEBOUNCE_MS:
            boton_estado_anterior = estado
            boton_ultimo_cambio_ms = ahora
            # "if not parada_activa" (sesion 2026-09-01, bug real visto en
            # vivo): este boton en concreto rebota/hace falso contacto
            # MUCHO mientras se mantiene pulsado -- sin este guard,
            # comprobar_boton() volvia a imprimir 'BOTON_PARADA' en cada
            # rebote, inundando el USB con mensajes solapados que
            # llegaban corruptos al puente (nunca una linea limpia). Con
            # el guard, solo se dispara UNA vez hasta el proximo REARME,
            # pase lo que pase con el ruido electrico mientras tanto.
            if estado == 0 and not parada_activa:  # flanco de bajada = pulsacion real (a GND)
                parada_activa = True
                apagar_rgb()  # corte local inmediato, no depende del PC
                print('BOTON_PARADA')  # linea centinela para led_publisher_usb.py
    else:
        boton_ultimo_cambio_ms = ahora


# =====================================================================
# HC-SR04 -- PARADA DE EMERGENCIA POR PROXIMIDAD (sesion 2026-09-11)
# =====================================================================
# Cableado: TRIG a GP20 (pata fisica 26, directo, 3.3V basta para
# disparar), ECHO a GP21 (pata fisica 27) a traves de un divisor
# resistivo 1k/2k2 que baja los 5V reales del ECHO a ~3.44V -- sin eso
# se frie el GPIO (ver Documentacion/cableado_hcsr04.html). Probado
# suelto con test_hcsr04.py antes de integrarlo aqui.
#
# Reutiliza EXACTAMENTE el mismo camino que el boton fisico (parada_activa
# + la linea centinela 'BOTON_PARADA'): led_publisher_usb.py ya la
# escucha y publica /emergency_stop, asi que no hace falta tocar nada
# fuera de esta Pico.
#
# Deliberadamente SIN debounce de varias lecturas seguidas (a diferencia
# del boton): una lectura ruidosa que dispare la parada de mas es
# molesta pero segura; esperar a confirmar el patron antes de parar no lo
# es. Si en la practica da falsos positivos, subir DISTANCIA_MIN_CM antes
# que anadir un filtro que retrase la parada real.
HCSR04_TRIG = machine.Pin(20, machine.Pin.OUT)
HCSR04_ECHO = machine.Pin(21, machine.Pin.IN)
HCSR04_TRIG.value(0)
DISTANCIA_MIN_CM = 10
HCSR04_TIMEOUT_US = 30000  # ~5m de alcance maximo


def medir_distancia_cm():
    HCSR04_TRIG.value(0)
    time.sleep_us(5)
    HCSR04_TRIG.value(1)
    time.sleep_us(10)
    HCSR04_TRIG.value(0)
    try:
        duracion_us = machine.time_pulse_us(HCSR04_ECHO, 1, HCSR04_TIMEOUT_US)
    except OSError:
        return None
    if duracion_us < 0:
        return None
    return duracion_us / 58.0


def comprobar_proximidad():
    """Se llama una vez por vuelta del bucle principal, igual que
    comprobar_boton(). None (sin eco, fuera de rango) no dispara nada."""
    global parada_activa
    if parada_activa:
        return
    distancia = medir_distancia_cm()
    if distancia is not None and distancia < DISTANCIA_MIN_CM:
        parada_activa = True
        apagar_rgb()  # corte local inmediato, no depende del PC
        print('BOTON_PARADA')  # misma linea centinela que el boton fisico


# =====================================================================
# LECTURA NO BLOQUEANTE DEL USB (sys.stdin) CON select.poll
# =====================================================================
# El mismo cable USB que usa Thonny para la consola/REPL sirve aqui como
# canal de datos: mientras este script corre, cualquier byte que llegue
# por USB pasa por sys.stdin. select.poll() con timeout permite comprobar
# "hay algo esperando" sin bloquear el bucle (igual que el socket no
# bloqueante de la Pico W). Para volver a programar esta Pico desde
# Thonny, basta pulsar "Stop/Restart" (Ctrl-C) como siempre -- eso
# interrumpe este bucle igual que interrumpiria cualquier otro script.
poller = select.poll()
poller.register(sys.stdin, select.POLLIN)

buffer_entrada = ''

print('Pico Loader lista, esperando comandos RGB por USB (R/G/B/0)...')

while True:
    comprobar_boton()
    comprobar_proximidad()
    if parada_activa:
        actualizar_parpadeo_parada()

    eventos = poller.poll(50)  # espera hasta 50ms, no bloquea si no llega nada
    if eventos:
        caracter = sys.stdin.read(1)
        if caracter in ('\n', '\r'):
            comando = buffer_entrada.strip().upper()
            buffer_entrada = ''
            if not comando:
                continue
            print('Recibido comando RGB:', comando)
            if comando == 'PARADA':
                # Parada pedida desde el PC (panel STOP, no el boton
                # fisico) -- mismo efecto que pulsarlo: parpadeo rojo
                # local. No hace falta el print centinela: el PC ya sabe
                # que ha mandado esto.
                if not parada_activa:
                    parada_activa = True
                    print('PARADA manual desde el PC')
            elif comando == 'REARME':
                # Unica forma de salir del parpadeo sin reiniciar la Pico,
                # igual que en la Pico W -- se acepta a proposito incluso
                # en parada.
                parada_activa = False
                apagar_rgb()
                print('REARMADO desde el PC')
            elif comando.startswith('PROD:'):
                try:
                    r, g, b = (int(v) for v in comando[len('PROD:'):].split(','))
                    set_led_producto(r, g, b)
                except (ValueError, TypeError):
                    print('Color de producto invalido (se esperaba PROD:r,g,b):', comando)
            elif parada_activa:
                # En parada, el rojo parpadeante manda -- ignoramos el
                # comando para no pisarlo con el color del cubo.
                print('(ignorado: en parada de emergencia)')
            elif comando == 'R':
                apagar_rgb()
                led_rojo.value(1)
            elif comando == 'G':
                apagar_rgb()
                led_verde.value(1)
            elif comando == 'B':
                apagar_rgb()
                led_azul.value(1)
            elif comando in ('0', 'APAGAR'):
                apagar_rgb()
            else:
                print('Comando no reconocido:', comando)
        else:
            buffer_entrada += caracter
