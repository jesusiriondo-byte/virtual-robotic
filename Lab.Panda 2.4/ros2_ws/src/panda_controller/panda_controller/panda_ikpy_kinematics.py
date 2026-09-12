#!/usr/bin/env python3
"""Cinematica compartida del Panda basada en ikpy + el URDF real exportado
de Webots (resource/panda_ikpy.urdf), con la correccion empirica
CORRECTION_Z a lo largo del eje de la pinza.

Validada en la sesion del 2026-08-26 con el Supervisor de solo lectura
orientation_probe: posicion y orientacion con error <5mm frente al estado
real de Webots. Sustituye a la tabla DH manual duplicada en los demas nodos
(teleop_manual.py, teleop_gui.py, pick_and_place.py, visit_balls.py,
lift_ball.py, vision_lift_cube.py, best_color_repeat_lift.py,
move_above_ball.py), que tiene un error conocido de posicion (~30cm en
algunas posturas) por no estar calibrada contra la fisica real de Webots
-- ver resumen_proyecto_panda.md."""

import os

import numpy as np
from ikpy.chain import Chain
from ament_index_python.packages import get_package_share_directory

DEFAULT_BASE = np.array([0.5, -0.3, 0.74])

# Postura "cero" que usa el URDF exportado (no coincide con el cero real de
# Franka): todo lo que sale de ikpy hay que sumarle esto para convertirlo en
# angulo real de articulacion.
SNAPSHOT = np.array([0.0, 0.0, 0.0, -1.7708, -1.6, 1.6, 0.79])

# Postura de reposo real del driver (my_robot_driver.py, HOME_POSITIONS).
HOME_POSITIONS = np.array([0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785])

# Orientacion con la pinza mirando hacia abajo. Su eje Z (columna 2) es el
# eje de aproximacion; girarla con rz(yaw) cambia solo el giro de muneca.
GRASP_R = np.array([
    [1.0, 0.0, 0.0],
    [0.0, -1.0, 0.0],
    [0.0, 0.0, -1.0],
])

# A lo largo del eje propio de la pinza: compensa que el offset flange->TCP
# exportado por Webots no es exacto (ver resumen_proyecto_panda.md).
CORRECTION_Z = -0.16

JOINT_LO = np.array([-2.9671, -1.8326, -2.9671, -3.1416, -2.9671, -0.0873, -2.9671])
JOINT_HI = np.array([2.9671, 1.8326, 2.9671, -0.4, 2.9671, 3.8223, 2.9671])

# Tolerancia de convergencia real de la IK (metros). Bug real (sesion
# 2026-08-26): el chequeo de "ok" solo miraba limites articulares, nunca si
# el solver de ikpy habia convergido de verdad -- se descubrio pidiendo
# aparcar en (0.5,-0.15,1.30) y viendo que el brazo real se quedaba en
# (0.5,-0.098,1.102): la solucion devuelta respetaba los limites pero su
# propia cinematica directa no coincidia ni de lejos con el objetivo
# pedido (6.4cm de error, el solver se habia quedado atascado en un minimo
# local). Sin este chequeo, "ok=True" no garantizaba llegar a donde se
# pedia. 5mm resulto DEMASIADO estricto: en movimientos continuos de varios
# pasos el solver deja un ruido residual normal de ~1cm que no es un fallo
# real (visto en un paso de "desplazar" con 12.5mm de error, benigno). Este
# umbral sigue cazando el fallo gordo del aparcado (6.4cm) sin rechazar
# ruido normal del solver.
CONVERGENCE_TOL_M = 0.02

# Margen (rad) al meter la semilla dentro de los limites de la cadena, ver
# _clip_seed(). Bug real 2026-09-10, el Loader MURIO entero por esto:
# scipy.optimize.least_squares exige que el punto de partida x0 este dentro
# de 'bounds' y aborta con ValueError("`x0` is infeasible.") si no lo esta.
# La semilla de reintento sale de seed_from_real(self.real_theta), o sea de
# la postura REAL del brazo -- que puede quedarse pegada a un tope
# articular (o un pelo fuera por el redondeo de SNAPSHOT) despues de un
# movimiento saturado. Entonces ikpy no devolvia "no converge", que
# ramp() ya sabe gestionar: lanzaba una excepcion que subia sin que nadie
# la cogiera hasta main() y tumbaba el proceso del demo -- la celda se
# quedaba a medias, con el Sorter esperando cubos que ya no iba a poner
# nadie. Una semilla es solo un punto de partida del optimizador, asi que
# recortarla al rango valido es inocuo (no cambia el objetivo pedido).
SEED_BOUND_MARGIN = 1e-6


# Cubo de 6cm: cerrar hasta 0.0 (tope real) empuja el cubo en vez de
# agarrarlo -- 0.025 deja el margen justo para presionar sin exigir que la
# pinza atraviese el cubo. Validado agarrando y levantando 15cm sin perder
# el cubo (sesion 2026-08-26). Valor POR DEFECTO para los dos robots (usado
# por CubeShuttleDemo.__init__ como self.grasp_close) -- NO tocar aqui para
# un ajuste de un solo robot, ver self.grasp_close en sorter_demo.py /
# loader_demo.py (sesion 2026-09-03: cambiar esta constante global afecto
# sin querer al Loader cuando el ajuste era solo para el Sorter).
GRIPPER_OPEN = 0.04
GRIPPER_CLOSED = 0.017


def rz(theta):
    """Rotacion pura alrededor del eje Z (de la propia pinza, si se
    postmultiplica sobre GRASP_R): gira el agarre sin tocar la orientacion
    "mirando hacia abajo". Para un cubo sin rotacion en el mundo, alinear
    los dedos con dos caras opuestas en vez de con las aristas requiere
    theta=pi/4 (o cualquier multiplo de pi/2 desde ahi)."""
    c, s = np.cos(theta), np.sin(theta)
    return np.array([
        [c, -s, 0.0],
        [s, c, 0.0],
        [0.0, 0.0, 1.0],
    ])


def _urdf_path():
    share = get_package_share_directory('panda_controller')
    return os.path.join(share, 'resource', 'panda_ikpy.urdf')


class PandaIkpyKinematics:
    def __init__(self, base=None, base_yaw=0.0):
        # 'base' parametriza donde esta plantado ESTE robot en el mundo --
        # antes era la constante de modulo BASE, compartida por narices por
        # cualquier instancia. Necesario para tener dos Panda simultaneos en
        # sitios distintos del mundo (sesion 2026-08-27, celda industrial de
        # dos robots): cada CubeShuttleDemo pasa la base de SU robot.
        self.base = DEFAULT_BASE if base is None else np.asarray(base, dtype=float)
        # 'base_yaw' (sesion 2026-09-08, experimento angulo de base del
        # Sorter): giro en radianes de la base alrededor de Z, POR DEFECTO
        # 0.0 -- hasta ahora la cinematica solo admitia traslacion (ver nota
        # historica en sorter_demo.py), a proposito para no complicarla sin
        # necesidad real. Con 0.0 el comportamiento es IDENTICO al de
        # siempre (Rz_inv se queda en la identidad en solve()), asi que esto
        # no cambia nada para el Loader ni para el Sorter en produccion.
        self.base_yaw = float(base_yaw)
        self.chain = Chain.from_urdf_file(_urdf_path())
        # Ultimo fallo del optimizador (ver solve()), para diagnostico.
        self._last_solver_error = None

    def seed_from_real(self, real_theta):
        """Convierte una postura articular REAL (angulos que de verdad
        recibiria motor.setPosition() en Webots) en la semilla que espera
        ikpy.inverse_kinematics (delta respecto a SNAPSHOT, con los 2
        eslabones fijos de la base y los 2 de la muneca)."""
        return [0.0, 0.0] + list(np.asarray(real_theta) - SNAPSHOT) + [0.0, 0.0]

    def _clip_seed(self, seed):
        """Mete la semilla dentro de los limites de la propia cadena ikpy
        (ver SEED_BOUND_MARGIN): si x0 se sale aunque sea por redondeo,
        scipy aborta la optimizacion con ValueError en vez de devolver una
        solucion mala, y esa excepcion mataba el proceso entero."""
        seed = np.asarray(seed, dtype=float).copy()
        for i, link in enumerate(self.chain.links):
            bounds = getattr(link, 'bounds', None)
            if not bounds:
                continue
            lo, hi = bounds
            if lo is None or hi is None or not np.isfinite(lo) or not np.isfinite(hi):
                continue
            if hi - lo <= 2 * SEED_BOUND_MARGIN:
                # Eslabon fijo (lo == hi, los 2 de la base y los 2 de la
                # muneca del URDF): el unico valor valido es el propio tope.
                seed[i] = 0.5 * (lo + hi)
            else:
                seed[i] = float(np.clip(seed[i], lo + SEED_BOUND_MARGIN,
                                        hi - SEED_BOUND_MARGIN))
        return seed

    def solve(self, x, y, z, target_r, seed, check_convergence=False):
        """Resuelve la IK para que el TCP llegue a (x,y,z) en el mundo con
        la orientacion target_r. z ya incluye la correccion (se le resta
        CORRECTION_Z aqui dentro, no hace falta aplicarla fuera). Devuelve
        (real_theta, result_completo_para_siguiente_semilla, ok).

        Por defecto 'ok' solo exige limites articulares (asi ha funcionado,
        de forma fiable, en todas las validaciones de la sesion 2026-08-25).
        check_convergence=True exige ADEMAS que la cinematica directa de la
        solucion coincida de verdad con el objetivo pedido (CONVERGENCE_TOL_M)
        -- el solver de ikpy puede quedarse atascado en un minimo local que
        respeta los limites pero esta lejos del objetivo real. Se probo
        activandolo SIEMPRE (bug real: aparcar en (0.5,-0.15,1.30) acababa a
        6.4cm de ahi, tapando la camara), pero causaba abortos nuevos en
        pasos de transito (mover el brazo por el aire) donde el solver deja
        un margen de unos pocos cm que nunca importo en la practica -- ese
        bug concreto ya se arreglo aparte (park_at_home() interpola en
        espacio de articulaciones, sin pasar por aqui). Dejar este chequeo
        como opt-in para los pasos donde SI importa de verdad (el contacto
        final de agarrar/depositar)."""
        target_local = np.array([x, y, z + CORRECTION_Z]) - self.base
        target_r_local = target_r
        if self.base_yaw:
            # La base esta girada base_yaw (radianes, eje Z) respecto al
            # mundo -- x,y,z y target_r llegan siempre en coordenadas de
            # MUNDO (igual que antes de existir base_yaw), asi que hay que
            # pasarlos al marco LOCAL de la base (Rz(-base_yaw)) antes de
            # dárselos a ikpy, que resuelve la cadena en su propio marco.
            target_local = rz(-self.base_yaw) @ target_local
            target_r_local = rz(-self.base_yaw) @ target_r
        try:
            result = self.chain.inverse_kinematics(
                target_local, target_orientation=target_r_local,
                orientation_mode='all', initial_position=self._clip_seed(seed))
        except (ValueError, np.linalg.LinAlgError) as exc:
            # Un fallo del optimizador es "no hay solucion", no el fin del
            # mundo: se devuelve ok=False y ramp() aborta el movimiento
            # limpiamente (LIMITE en ..., reintento o fallo de pieza), en
            # vez de propagar la excepcion hasta main() y tumbar el demo
            # entero. Ver SEED_BOUND_MARGIN para el caso concreto que lo
            # destapo. Se devuelve la semilla como 'result' para que la
            # siguiente llamada parta de algo valido en vez de de basura.
            self._last_solver_error = str(exc)
            return np.asarray(seed[2:9]) + SNAPSHOT, list(seed), False
        delta_solved = np.array(result[2:9])
        real_theta = delta_solved + SNAPSHOT
        ok = bool(np.all(real_theta >= JOINT_LO) and np.all(real_theta <= JOINT_HI))
        if ok and check_convergence:
            fk = self.chain.forward_kinematics(result)
            pos_error = float(np.linalg.norm(fk[:3, 3] - target_local))
            ok = pos_error <= CONVERGENCE_TOL_M
        return real_theta, result, ok
