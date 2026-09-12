FROM osrf/ros:humble-desktop-full

ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=Etc/UTC

RUN apt-get update && apt-get install -y \
    wget gnupg2 ca-certificates lsb-release software-properties-common \
    && rm -rf /var/lib/apt/lists/*

# Version FIJA (sesion 2026-08-30): el repo apt de Cyberbotics es rodante
# -- solo sirve la ULTIMA version (comprobado con "apt-cache madison
# webots"), asi que "apt-get install webots" a secas instala lo que sea
# que este de moda ese dia. Bug real encontrado esta sesion: se probo a
# fijar R2023b (la version con la que se escribio originalmente el
# proyecto) via un .deb de las releases de GitHub, y arreglo el aspecto
# visual (mallas/texturas) -- pero rompio funcionalidad real: el paquete
# ROS 'ros-humble-webots-ros2' (instalado mas abajo) TAMBIEN es un repo
# rodante y solo tiene disponible la 2025.0.0 (pensada para R2025a) --
# con Webots en R2023b, esa version del driver hace que
# WarehouseSupervisor se caiga (Segmentation fault, usa funciones de
# Supervisor) y que las camaras cenitales dejen de entregar frames
# (_esperar_color/_esperar_reciclado se quedan colgados para siempre).
# Con R2025a (version pinneada aqui explicitamente, "=2025a" en vez de
# "webots" a secas -- asi un rebuild futuro FALLA con un error claro si
# esa version deja de estar disponible, en vez de instalar otra en
# silencio) TODO funciona de verdad (agarre, camaras, reciclaje); el
# unico coste es cosmetico: algunas mallas/texturas que R2023b traia
# incluidas de fabrica, R2025a las descarga bajo demanda de
# raw.githubusercontent.com, y esa descarga falla en este entorno de
# red restringida -- el robot y el escenario se ven mal, pero se mueven
# y funcionan bien. Ver memoria del proyecto (robotica_webots_version_pin)
# antes de volver a tocar esto.
RUN mkdir -p /etc/apt/keyrings \
    && wget -qO /etc/apt/keyrings/Cyberbotics.asc https://cyberbotics.com/Cyberbotics.asc \
    && echo "deb [signed-by=/etc/apt/keyrings/Cyberbotics.asc] https://cyberbotics.com/debian/ binary-amd64/" > /etc/apt/sources.list.d/Cyberbotics.list \
    && apt-get update \
    && apt-get install -y webots=2025a \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y \
    ros-humble-webots-ros2 \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /workspace
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

ENTRYPOINT ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash && exec \"$@\"", "--"]
