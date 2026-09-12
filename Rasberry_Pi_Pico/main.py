# Version: 2026-09-11 10:55 -- Pico W del SORTER (wifi + boton de parada)
import machine
import network
import time
import socket
import webrepl

from wifi_config import SSID, PASSWORD, PC_IP, PC_PUERTO

# =====================================================================
# 1. CONFIGURACIÓN DEL HARDWARE (LED RGB - Tus Pines Originales)
# =====================================================================
led_rojo = machine.Pin(13, machine.Pin.OUT)
led_verde = machine.Pin(14, machine.Pin.OUT)
led_azul = machine.Pin(15, machine.Pin.OUT)

# Apagamos todos al arrancar
def apagar_rgb():
    led_rojo.value(0)
    led_verde.value(0)
    led_azul.value(0)

apagar_rgb()

# =====================================================================
# 1b. BOTÓN PULSADOR DE PARADA DE EMERGENCIA
# =====================================================================
# Cableado: un terminal del pulsador a GPIO16, el otro a GND. NO hace
# falta resistencia externa -- se usa la pull-up interna de la Pico, así
# que en reposo el pin lee 1 y al pulsar (cierra a GND) lee 0.
boton = machine.Pin(16, machine.Pin.IN, machine.Pin.PULL_UP)

boton_estado_anterior = 1
boton_ultimo_cambio_ms = time.ticks_ms()
DEBOUNCE_MS = 40  # tiempo minimo estable antes de aceptar la pulsacion como real

# Estado de parada: una vez pulsado el boton, el rojo se queda parpadeando
# hasta reiniciar la Pico (de proposito -- una parada de emergencia no
# deberia des-activarse sola). Mientras dura, se ignoran los comandos RGB
# que lleguen por red para que el parpadeo no se pise con el color del cubo.
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


def avisar_parada_a_pc():
    """Intenta avisar al PC (ROS2) de la parada. Si el Wi-Fi/PC no
    responde no pasa nada grave: el corte del LED de abajo ya es
    inmediato y no depende de la red."""
    if not PC_IP:
        print("AVISO: PC_IP no configurada en wifi_config.py, no se avisa a ROS2.")
        return
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.settimeout(1.0)
        s.connect((PC_IP, PC_PUERTO))
        s.sendall(b"STOP")
        s.close()
        print("Aviso de parada enviado al PC (", PC_IP, ":", PC_PUERTO, ")")
    except Exception as e:
        print("No se pudo avisar al PC de la parada:", e)


def comprobar_boton():
    """Debounce simple por tiempo. Se llama una vez por vuelta del bucle
    principal (cada ~0.05s)."""
    global boton_estado_anterior, boton_ultimo_cambio_ms, parada_activa
    estado = boton.value()
    ahora = time.ticks_ms()
    if estado != boton_estado_anterior:
        if time.ticks_diff(ahora, boton_ultimo_cambio_ms) >= DEBOUNCE_MS:
            boton_estado_anterior = estado
            boton_ultimo_cambio_ms = ahora
            if estado == 0:  # flanco de bajada = pulsacion real (a GND)
                print("BOTON DE PARADA PULSADO")
                parada_activa = True
                apagar_rgb()          # corte local inmediato, no depende de la red
                avisar_parada_a_pc()  # aviso a ROS2 (mejor esfuerzo)
    else:
        boton_ultimo_cambio_ms = ahora


# =====================================================================
# 2. CONEXIÓN WI-FI (credenciales en wifi_config.py)
# =====================================================================
PUERTO = 5001

wlan = network.WLAN(network.STA_IF)
wlan.active(True)

def conectar_wifi():
    if wlan.isconnected():
        return
    print("Conectando al Wi-Fi...")
    wlan.connect(SSID, PASSWORD)
    # Atender el boton MIENTRAS se conecta (sesion 2026-09-11): antes era un
    # time.sleep(1) a ciegas, y mientras la Wi-Fi no conectaba (al arrancar o
    # en una reconexion) el boton de parada no se atendia ni para cortar el
    # LED en local.
    while not wlan.isconnected():
        comprobar_boton()
        if parada_activa:
            actualizar_parpadeo_parada()
        time.sleep(0.05)
    print("¡Conectado! IP de la Pico W:", wlan.ifconfig()[0])

conectar_wifi()

# Arrancamos WebREPL para que Thonny pueda hablar por el aire
try:
    webrepl.start()
except Exception as e:
    print("WebREPL ya estaba activo o nota:", e)

# =====================================================================
# 3. SERVIDOR TCP (se recrea si la IP cambia tras una reconexión Wi-Fi)
# =====================================================================
servidor = None
mi_ip = None

def crear_servidor():
    global servidor, mi_ip
    if servidor:
        servidor.close()
    mi_ip = wlan.ifconfig()[0]
    servidor = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    servidor.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    servidor.bind((mi_ip, PUERTO))
    servidor.listen(1)
    servidor.setblocking(False)  # Modo no bloqueante para no congelar la CPU
    print("Esperando órdenes RGB de ROS 2 en", mi_ip, "puerto", PUERTO)

crear_servidor()

conexion = None
CICLOS_CHEQUEO_WIFI = 100  # ~5s (100 * 0.05s) entre comprobaciones de Wi-Fi
contador = 0

# =====================================================================
# 4. BUCLE DE CONTROL INTEGRADO CON ROS 2
# =====================================================================
while True:
    try:
        comprobar_boton()

        if parada_activa:
            actualizar_parpadeo_parada()

        # Vigilamos la Wi-Fi para no quedarnos "sordos" si se cae en pleno recorrido
        contador += 1
        if contador >= CICLOS_CHEQUEO_WIFI:
            contador = 0
            if not wlan.isconnected():
                print("Wi-Fi caída, reconectando...")
                if conexion:
                    conexion.close()
                    conexion = None
                conectar_wifi()
                if wlan.ifconfig()[0] != mi_ip:
                    crear_servidor()

        if conexion is None:
            try:
                conexion, direccion = servidor.accept()
                conexion.setblocking(False)
            except OSError:
                pass
        else:
            try:
                datos = conexion.recv(1024)
                if datos:
                    # Decodificamos y pasamos a mayúsculas ('r' -> 'R')
                    comando = datos.decode('utf-8').strip().upper()
                    print("Recibido comando RGB:", comando)

                    if comando == "PARADA":
                        # Parada pedida desde el PC (boton del panel, no el
                        # fisico) -- mismo efecto que pulsar el boton: entra
                        # en parpadeo rojo local de inmediato. Sirve para
                        # probar la parada sin tener que tocar el boton.
                        if not parada_activa:
                            parada_activa = True
                            print("PARADA manual desde el PC")
                    elif comando == "REARME":
                        # Rearme explicito desde el PC (boton REARME del
                        # panel, ver estop_panel.py): es la UNICA forma de
                        # salir del parpadeo sin reiniciar la Pico -- se
                        # acepta a proposito incluso estando en parada,
                        # porque es una accion deliberada del operario, no
                        # un comando de color que pudiera pisar el aviso
                        # por error.
                        parada_activa = False
                        apagar_rgb()
                        print("REARMADO desde el PC")
                    elif parada_activa:
                        # En parada, el rojo parpadeante manda -- ignoramos
                        # el comando para no pisarlo con el color del cubo.
                        print("(ignorado: en parada de emergencia)")
                    # Control del color sincronizado con tu navegación autónoma
                    elif comando == "R":    # 🔴 GIRO_EVASION
                        apagar_rgb()
                        led_rojo.value(1)
                    elif comando == "G":  # 🟢 AVANZAR
                        apagar_rgb()
                        led_verde.value(1)
                    elif comando == "B":  # 🔵 SALIR_PASILLO
                        apagar_rgb()
                        led_azul.value(1)
                    elif comando == "0" or comando == "APAGAR":
                        apagar_rgb()
                else:
                    # Si no hay datos, cerramos el canal para recibir la siguiente orden de ROS 2
                    conexion.close()
                    conexion = None
            except OSError as e:
                # Error 11 es el equivalente a "EAGAIN" (no hay datos listos todavía)
                if e.args[0] != 11:
                    conexion.close()
                    conexion = None
    except Exception as e:
        if conexion:
            conexion.close()
        conexion = None

    # Pausa de control para que el procesador respire y WebREPL no pierda la conexión
    time.sleep(0.05)
    #jesus
