#!/usr/bin/env python3
"""Prueba unica: exporta el URDF real del propio robot (debe ser el
controller local, Supervisor TRUE) a un fichero compartido, para poder
usarlo con ikpy en vez de la cinematica DH hecha a mano."""
from controller import Supervisor

robot = Supervisor()
timestep = int(robot.getBasicTimeStep())

urdf_text = robot.getUrdf()
with open('/workspace/controllers/panda_exported.urdf', 'w') as f:
    f.write(urdf_text)

print('[urdf_export_test] URDF exportado, longitud=', len(urdf_text), flush=True)
print(urdf_text[:2000], flush=True)

robot.step(timestep)
