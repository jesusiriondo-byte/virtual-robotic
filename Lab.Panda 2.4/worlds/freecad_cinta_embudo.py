"""Macro de FreeCAD: reconstruye la cinta + el embudo de
panda_industrial_cell.wbt con las medidas REALES (mismos numeros que el
mundo de Webots, sesion 2026-09-03) para poder disenar la pared nueva del
embudo con herramientas de CAD de verdad, en vez de a mano alzada.

Como usarlo:
    1. Abre FreeCAD.
    2. Menu Macro -> Macros... -> boton "Ejecutar" (o "Execute") ->
       navega hasta este fichero y ejecutalo. Tambien puedes pegar el
       contenido en la consola Python de FreeCAD (Vista -> Paneles ->
       consola Python).
    3. Se crea un documento nuevo "cinta_embudo" con todas las piezas de
       abajo, cada una nombrada igual que su DEF en el .wbt para que sea
       facil ir y volver entre los dos ficheros.

Unidades: Webots trabaja en metros: aqui se convierte todo a mm (unidad
nativa de FreeCAD) multiplicando por 1000, sin cambiar ninguna proporcion.

Todas las piezas se crean CENTRADAS en su propio origen local y despues
se colocan con Placement (Position + Rotation) exactamente como Webots
coloca un Solid: rotacion alrededor del centro local, despues traslacion
al punto del mundo -- mismo orden, mismo resultado.
"""

import math

import FreeCAD
import Part


def add_box(doc, name, size_m, translation_m, rot_axis=(0, 0, 1), rot_deg=0.0,
            color=(0.2, 0.2, 0.2), anchor="center"):
    """size_m/translation_m en METROS (como en el .wbt). rot_deg en GRADOS
    (el .wbt usa radianes -- convertir al llamar, ver mas abajo).

    anchor="center" (por defecto): vale para un Solid+Box normal de
    Webots (rieles, tope, cubos) -- la Box esta CENTRADA en el origen
    local, igual que aqui.
    anchor="bottom": para PROTOs como ConveyorBelt o Table, cuyo origen
    local esta en la BASE (se apoyan en el suelo/mesa y crecen hacia
    arriba) -- 'size.z' es la altura total desde translation.z hacia
    arriba, NO esta centrada. Bug real (sesion 2026-09-03): tratar BELT
    como centrada la dejaba mitad bajo tierra y desplazada del resto de
    piezas, pareciendo "un rectangulo" solo, tapando/alejando todo lo
    demas al hacer Ajustar todo."""
    length_mm, width_mm, height_mm = [s * 1000.0 for s in size_m]
    # Bug real (sesion 2026-09-03, encontrado comparando BoundBox a mano):
    # Part.makeBox(...).translate(...) NO mutaba la forma "en el sitio" en
    # esta version de FreeCAD -- el desplazamiento se perdia en silencio, y
    # Placement rotaba la caja alrededor de su ESQUINA (0,0,0) en vez de su
    # centro, dejando todo desplazado (comprobado con belt.Shape.BoundBox:
    # el centro real no coincidia con la traslacion pedida). Arreglo: pasar
    # el punto de la esquina YA CORRECTO directamente a Part.makeBox (su
    # parametro 'pnt'), sin translate() por separado.
    z0 = -height_mm / 2 if anchor == "center" else 0.0
    corner = FreeCAD.Vector(-length_mm / 2, -width_mm / 2, z0)
    box_shape = Part.makeBox(length_mm, width_mm, height_mm, corner)

    obj = doc.addObject("Part::Feature", name)
    obj.Shape = box_shape

    x_mm, y_mm, z_mm = [t * 1000.0 for t in translation_m]
    rotation = FreeCAD.Rotation(FreeCAD.Vector(*rot_axis), rot_deg)
    obj.Placement = FreeCAD.Placement(FreeCAD.Vector(x_mm, y_mm, z_mm), rotation)

    if hasattr(obj, "ViewObject") and obj.ViewObject is not None:
        obj.ViewObject.ShapeColor = color
    return obj


def rad2deg(r):
    return math.degrees(r)


doc = FreeCAD.newDocument("cinta_embudo")

# --- Cinta transportadora (DEF BELT ConveyorBelt) ---------------------
# translation 0.5 0.85 0 | rotation Z 1.5708 | size 1.1 0.3 0.74
# anchor="bottom": el ConveyorBelt (igual que Table) se apoya en
# translation.z=0 y crece hacia arriba hasta la altura de la mesa
# (0.74m) -- NO esta centrada en translation.z como una Box normal.
add_box(doc, "BELT",
        size_m=(1.1, 0.3, 0.74),
        translation_m=(0.5, 0.85, 0.0),
        rot_deg=rad2deg(1.5708),
        color=(0.5, 0.5, 0.55),
        anchor="bottom")

# --- Tope final (DEF BELT_END_STOP) ------------------------------------
# translation 0.5 1.30 0.755 | sin rotacion | size 0.32 0.02 0.03
add_box(doc, "BELT_END_STOP",
        size_m=(0.32, 0.02, 0.03),
        translation_m=(0.5, 1.30, 0.755),
        color=(0.2, 0.2, 0.2))

# --- Carriles del EMBUDO (rampa que va estrechando la cinta) -----------
# translation 0.415/0.585 0.775 0.755 | rotation Z +-0.1543 | size 0.015 0.4554 0.03
add_box(doc, "BELT_RAIL_L_EMBUDO",
        size_m=(0.015, 0.4554, 0.03),
        translation_m=(0.415, 0.775, 0.755),
        rot_deg=rad2deg(-0.1543),
        color=(0.2, 0.2, 0.2))
add_box(doc, "BELT_RAIL_R_EMBUDO",
        size_m=(0.015, 0.4554, 0.03),
        translation_m=(0.585, 0.775, 0.755),
        rot_deg=rad2deg(0.1543),
        color=(0.2, 0.2, 0.2))

# --- Carriles del tramo RECTO (donde agarra la pinza -- NO TOCAR) ------
# translation 0.45/0.55 1.14 0.755 | sin rotacion | size 0.015 0.28 0.03
add_box(doc, "BELT_RAIL_L_RECTO",
        size_m=(0.015, 0.28, 0.03),
        translation_m=(0.45, 1.14, 0.755),
        color=(0.15, 0.45, 0.2))
add_box(doc, "BELT_RAIL_R_RECTO",
        size_m=(0.015, 0.28, 0.03),
        translation_m=(0.55, 1.14, 0.755),
        color=(0.15, 0.45, 0.2))

# --- Cubo de referencia (6cm de lado) en el punto de recogida del
# Sorter (PICKUP_X, PICKUP_Y = 0.5, 1.25 en sorter_demo.py), solo para
# ver a escala real cuanto sobresale el cubo por encima del carril --
# NO es parte del mundo de Webots, es solo una referencia visual aqui.
add_box(doc, "CUBO_referencia_6cm",
        size_m=(0.06, 0.06, 0.06),
        translation_m=(0.5, 1.25, 0.77),
        color=(0.9, 0.1, 0.1))

doc.recompute()

FreeCAD.Console.PrintMessage(
    "Cinta + embudo reconstruidos a escala real (mm). "
    "El cubo rojo de referencia mide 6cm de lado -- comparalo con la "
    "altura de los carriles (3cm) para ver cuanto queda expuesto por "
    "encima.\n"
)
