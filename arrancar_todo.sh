#!/usr/bin/env bash
# 2026-09-12: arranca todo el proyecto de un tiron -- Taller_Administracion,
# la simulacion (Webots+ROS2), compila si hace falta, lanza la celda
# completa en segundo plano y al final abre el panel de control manual
# (teleop_gui) como unica ventana interactiva. Pensado para "quiero
# probarlo todo sin tener que abrir 5 terminales" (sin hardware real: no
# toca las Raspberry Pi Pico -- si las tienes, sigue lanzando a mano los
# puentes de LED como dice LANZAR_PROYECTO.md paso 4).
set -e

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "== 1/5 -- Taller_Administracion =="
(cd "$DIR/Taller_Administracion" && docker compose up -d --build)

echo "== 2/5 -- Simulacion (Webots + ROS2) =="
xhost +local:docker >/dev/null 2>&1 || echo "(xhost no disponible -- sigo igualmente, puede que Webots no dibuje si no hay sesion grafica)"
(cd "$DIR/Lab.Panda 2.4/.devcontainer" && docker compose up -d --build)

echo "== 3/5 -- Compilando el paquete ROS2 (necesario la primera vez) =="
docker exec ros2_panda_dev24 bash -c "cd /workspace && colcon build --packages-select panda_controller --symlink-install"

echo "== 4/5 -- Lanzando la celda completa en segundo plano =="

lanzar_celda() {
  docker exec ros2_panda_dev24 bash -c "pkill -f robot_launch_industrial_cell 2>/dev/null; true"
  sleep 1
  docker exec ros2_panda_dev24 bash -c "rm -f /tmp/robot_launch.log"
  # docker exec -d NO es una shell interactiva -- no lee ~/.bashrc, asi que
  # hay que sourcear ROS a mano (base + overlay del workspace) o 'ros2' no
  # se encuentra (bug real, visto probando este script).
  docker exec -d ros2_panda_dev24 bash -c "source /opt/ros/humble/setup.bash && source /workspace/install/setup.bash && cd /workspace && ros2 launch panda_controller robot_launch_industrial_cell.py > /tmp/robot_launch.log 2>&1"
}

# Devuelve 0 si conectan los 6 controladores dentro del tiempo dado, 1 si no.
esperar_controladores() {
  local segundos="$1" i n
  for i in $(seq 1 "$segundos"); do
    # 'grep -c' devuelve exit code 1 cuando cuenta 0 -- sin el '; true' eso
    # se confundia con un fallo real y duplicaba la salida (bug real visto
    # probando este script).
    n=$(docker exec ros2_panda_dev24 bash -c "grep -c 'Controller successfully connected' /tmp/robot_launch.log 2>/dev/null; true")
    n=${n:-0}
    if [ "$n" -ge 6 ] 2>/dev/null; then
      echo "Listo: 6 controladores conectados (tras ${i}s)."
      return 0
    fi
    sleep 1
  done
  return 1
}

lanzar_celda
echo "Esperando a que conecten los 6 controladores (hasta 60s)..."
if ! esperar_controladores 60; then
  # Arreglo conocido (visto varias veces, en esta VM y en la maquina real):
  # Webots a veces se queda colgado cargando el mundo la primera vez que
  # arranca en frio -- un 'docker restart' lo desatasca. Se intenta solo,
  # y si tras eso sigue sin conectar, ahi si hace falta mirarlo a mano.
  echo "No han conectado en 60s -- probando el arreglo conocido: reiniciar Webots en frio."
  docker restart webots_panda_sim24 >/dev/null
  echo "Esperando a que Webots vuelva a arrancar (20s)..."
  sleep 20
  lanzar_celda
  echo "Reintentando la espera de los 6 controladores (hasta 60s mas)..."
  if ! esperar_controladores 60; then
    echo "AVISO: sigue sin conectar tras el reinicio en frio."
    echo "Revisa el log de ROS2 con: docker exec ros2_panda_dev24 cat /tmp/robot_launch.log"
    echo "Y el de Webots con:        docker logs webots_panda_sim24 --tail 30"
    echo "Sigo igualmente al panel de control por si acaso, pero probablemente falle tambien."
  fi
fi

echo "== 5/5 -- Abriendo el panel de control manual =="
docker exec -it ros2_panda_dev24 bash -c "source /opt/ros/humble/setup.bash && source /workspace/install/setup.bash && ros2 run panda_controller teleop_gui"
