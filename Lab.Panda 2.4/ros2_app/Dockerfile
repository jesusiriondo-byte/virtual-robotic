FROM osrf/ros:humble-desktop-full

ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=Etc/UTC

RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    python3-pip \
    ros-humble-webots-ros2 \
    && rm -rf /var/lib/apt/lists/*

# ikpy: cinematica inversa real usada por panda_ikpy_kinematics.py (sesion
# 2026-08-25/26). No es un paquete ROS, se perdia cada vez que se recreaba
# el contenedor porque solo se instalaba a mano con pip dentro del
# contenedor ya en marcha -- se fija aqui para que persista.
RUN pip3 install ikpy==4.0.0

# pyserial: usado por led_publisher_usb.py (sesion 2026-08-30) para hablar
# con la segunda Pico (sin wifi, solo USB) del robot Loader.
RUN pip3 install pyserial==3.5

WORKDIR /workspace

# Autosource: primero el ROS base, y si ya existe el overlay del workspace
# (creado por 'colcon build', persiste en el host via el volumen montado),
# se sourcea tambien automaticamente. Asi no hace falta escribirlo a mano
# en cada terminal nueva (leccion aprendida en el proyecto del UR5e).
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && \
    echo "if [ -f /workspace/install/setup.bash ]; then source /workspace/install/setup.bash; fi" >> ~/.bashrc

ENTRYPOINT ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash && if [ -f /workspace/install/setup.bash ]; then source /workspace/install/setup.bash; fi && exec \"$@\"", "--"]
