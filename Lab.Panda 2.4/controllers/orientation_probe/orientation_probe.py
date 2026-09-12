#!/usr/bin/env python3
"""Supervisor de solo lectura: imprime periodicamente la posicion y
orientacion REALES (segun el motor de fisica de Webots) del nodo
DEF PANDA_HAND, para comparar contra lo que predice nuestro propio modelo
de cinematica (duplicado a mano en teleop_gui.py y demas nodos). No mueve
nada, solo observa -- el brazo lo sigue controlando el nodo ROS externo
como siempre."""

import sys

from controller import Supervisor

robot = Supervisor()
timestep = int(robot.getBasicTimeStep())

hand = robot.getFromDef('PANDA_HAND')
if hand is None:
    print('[orientation_probe] ERROR: no encuentro DEF PANDA_HAND', flush=True)
    sys.exit(1)

panda_node = robot.getFromDef('PANDA_ROBOT')
if panda_node is not None:
    try:
        urdf_path = '/tmp/panda_exported.urdf'
        panda_node.exportUrdf(urdf_path)
        print(f'[orientation_probe] URDF del Panda exportado a {urdf_path}', flush=True)
    except Exception as exc:
        print(f'[orientation_probe] AVISO: exportUrdf fallo: {exc}', flush=True)
else:
    print('[orientation_probe] AVISO: no encuentro DEF PANDA_ROBOT', flush=True)

cube = robot.getFromDef('CUBO_ROJO')
if cube is None:
    print('[orientation_probe] AVISO: no encuentro DEF CUBO_ROJO (sin seguimiento de altura)', flush=True)
else:
    # Reset de posicion al arrancar: tras varias pruebas de agarre el cubo
    # puede quedar empujado fuera de una zona comoda de alcance. Vuelve
    # siempre a su sitio original del .wbt al reiniciar el supervisor.
    cube.getField('translation').setSFVec3f([0.5, 0.15, 0.77])
    cube.getField('rotation').setSFRotation([0, 0, 1, 0])
    cube.resetPhysics()
    print('[orientation_probe] cubo_rojo reseteado a (0.5,0.15,0.77)', flush=True)

print('[orientation_probe] listo, observando PANDA_HAND' + (' y CUBO_ROJO' if cube else ''), flush=True)

last_print = -1e9
while robot.step(timestep) != -1:
    t = robot.getTime()
    if t - last_print < 0.5:
        continue
    last_print = t
    pos = hand.getPosition()
    rot = hand.getOrientation()  # 9 floats, row-major 3x3
    r00, r01, r02, r10, r11, r12, r20, r21, r22 = rot
    cube_txt = ''
    if cube is not None:
        cp = cube.getPosition()
        cube_txt = f' cubo_rojo=({cp[0]:.4f},{cp[1]:.4f},{cp[2]:.4f})'
    print(
        f'[orientation_probe] t={t:6.1f} pos=({pos[0]:.4f},{pos[1]:.4f},{pos[2]:.4f}) '
        f'x_axis=({r00:.3f},{r10:.3f},{r20:.3f}) '
        f'y_axis=({r01:.3f},{r11:.3f},{r21:.3f}) '
        f'z_axis=({r02:.3f},{r12:.3f},{r22:.3f})' + cube_txt,
        flush=True,
    )
