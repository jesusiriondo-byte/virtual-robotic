import network
import time
import webrepl

from wifi_config import SSID, PASSWORD

# 1. Conectar al Wi-Fi
wlan = network.WLAN(network.STA_IF)
wlan.active(True)
wlan.connect(SSID, PASSWORD)

print("Conectando...")
while not wlan.isconnected():
    time.sleep(1)

mi_ip = wlan.ifconfig()[0]
print("¡Conectado! IP actual:", mi_ip)

# 2. Arrancar solo WebREPL
webrepl.start()