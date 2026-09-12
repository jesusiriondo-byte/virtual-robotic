#!/usr/bin/env bash
# 2026-09-12: para y elimina los contenedores de los dos proyectos
# (Lab.Panda 2.4 y Taller_Administracion). Complementario a
# arrancar_todo.sh. Seguro de ejecutar aunque no haya nada corriendo --
# no aborta si un "docker compose down" no encuentra nada que parar.

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "== Parando Lab.Panda 2.4 (simulacion) =="
(cd "$DIR/Lab.Panda 2.4/.devcontainer" && docker compose down)

echo "== Parando Taller_Administracion =="
(cd "$DIR/Taller_Administracion" && docker compose down)

echo
echo "Comprobando que no quede nada del proyecto corriendo:"
restos=$(docker ps -a --format '{{.Names}}' | grep -E "^(webots_panda_sim24|ros2_panda_dev24|taller_admin_api)$")
if [ -n "$restos" ]; then
  echo "AVISO: todavia queda esto por limpiar a mano:"
  echo "$restos"
else
  echo "Todo limpio."
fi
