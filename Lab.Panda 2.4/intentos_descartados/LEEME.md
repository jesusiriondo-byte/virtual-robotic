# Intentos descartados del agarre del Sorter (2026-09-10)

Dos cambios **probados en produccion y revertidos** porque empeoraron el
resultado, pese a estar bien razonados. Se guardan porque el analisis sigue
siendo valido y puede que la solucion este cerca -- pero no se vuelven a
aplicar sin leer esto antes.

Referencia buena: commit `f1e4a94` (canal 11 cm, `approach_open` 0.040,
cubo de 6 cm) = **~2,0 cubos/min**, unos 120/hora.

## `intento_pieza_50mm.patch`

Pieza de 5 cm en vez de 6. Sobre el papel: hueco de 10 -> 15 mm y giro
tolerado de 25 -> 45 grados. **Medido: 0 clasificados y 39 abortos.**

- Error propio: se dejo `approach_open` en 0.040 con el canal ya estrechado
  a 95 mm, asi que la ventana de cierre quedo en 7 mm.
- De fondo: las piezas de 5 cm **acaban pegadas a los carriles** mucho mas
  que las de 6. El embudo estrecha a 85 mm y el canal era de 95: la pieza
  sale encarrilada y se vuelve a descentrar justo donde agarra la pinza.

Si se retoma: **el canal tiene que ser igual o mas estrecho que la salida
del embudo (85 mm)**, no mas ancho.

## `intento_deteccion_y_doble_check.patch`

Dos cosas juntas: bajar el suelo de deteccion de dislocacion (-0.005 ->
-0.020, con 3 lecturas seguidas) y repetir la comprobacion de hueco con la
cinta ya parada.

El diagnostico era correcto y esta medido: el umbral de -0.005 caza la
**compresion normal** de cerrar sobre el cubo (-0.007, -0.012) frente a
dislocaciones reales (+0.177, +0.257), y esa falsa alarma paraba el brazo a
media rampa; al rearmar saltaba y ENTONCES se rompia de verdad. Escalada
capturada: `[-0.007] -> rearme -> [0.257] -> rearme -> [0.177]`.

**Medido: 1,63 cubos/min frente a 1,97**, con 13 dislocaciones reales en 19
clasificados. Hipotesis que queda: el umbral sensible funciona como
**proteccion temprana** -- para antes de que el daño progrese, aunque a
veces sea falsa alarma.

## Como medir cualquier intento futuro

- `clasificados / min` sobre **15 minutos como minimo**. La variabilidad
  entre lotes es enorme: con el MISMO codigo se midieron 27 clasificados /
  3 dislocaciones y 5 / 9.
- **No** comparar contadores de dislocaciones entre configuraciones si se
  ha tocado el umbral: cambia el significado del contador y los lotes dejan
  de ser comparables.
