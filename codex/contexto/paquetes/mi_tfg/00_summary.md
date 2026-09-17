# mi_tfg - referencia historica de vision

## Alcance

`mi_tfg` es codigo historico y no forma parte del runtime vigente de Fase 6.
Se consulta como referencia conceptual para la orientacion respecto a una
fachada; no se deben copiar sus topics, estado de control ni convenciones de
trayectoria al pipeline actual.

## Estimacion de fachada reutilizable

Archivo `mi_tfg/src/vision/vision.py` -> `im2prof()` y `normales_punto()`
(`rg -n "def im2prof|def normales_punto"`).

La cadena historica:

```text
estereo + WLS
-> mascara por textura y gradiente depth/intensidad
-> limpieza morfologica de regiones
-> nube y voxel downsample
-> normales locales orientadas hacia la camara
-> rechazo de normales con poca proyeccion XZ
-> histograma angular de 10 grados
-> direccion dominante o fusion de bins proximos
```

La idea importante es estimar muchas normales locales y votar una direccion,
no ajustar un unico plano a todos los endpoints depth. Las normales se orientan
hacia la camara; una pared frontal produce aproximadamente 180 grados.

## Uso historico en control

Archivo `mi_tfg/src/vision/control_dron.cpp` ->
`colocarse_frente_pared()` (`rg -n "colocarse_frente_pared"`).

El controlador convierte `correccion = 180 - angulo_normal`, por lo que una
normal frontal de 180 grados genera correccion yaw cero. Esto coincide con el
contrato actual: la normal de un plano es axial y `n`/`-n` son equivalentes;
una normal relativa de 0 o 180 grados no debe mover el dron.

## Limites de la referencia

- usa Python/Open3D y procesamiento diagnostico pesado;
- pierde el signo con `abs()` en parte del control;
- el histograma 0/360 no es circular de forma robusta;
- mezcla estado y ordenes mediante un byte de control;
- contiene convenciones antiguas de action/yaw incompatibles con Fase 6.

La migracion recomendada es implementar el principio en
`DepthObservationProcessor` C++: mascara fiable, normales locales, histograma
axial circular, soporte/confianza y gate temporal. `task_manager` conserva el
yaw estable y decide la correccion; `mi_tfg` no se incorpora como dependencia.
