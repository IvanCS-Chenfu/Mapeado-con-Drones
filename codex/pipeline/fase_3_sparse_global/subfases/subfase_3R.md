# Subfase 3R - Scoring raw y fused incremental

## Estado vigente

```text
CONSEGUIDA: RECALIBRACION TECNICA Y CIERRE RVIZ2 CONFIRMADOS
```

## Objetivo

Convertir `LandmarkScoreManager` en la unica autoridad numerica para que el
score evolucione con la calidad ORB, la geometria global y las fusiones, sin
usar ground truth, bloquear publicacion ni reconstruir snapshots completos.

Para cada raw MapPoint:

```text
score_raw = clamp(
  base_score_orb * factor_distancia * factor_aislamiento
  + 0.04 * inliers_confirmados,
  0, 1)
```

- `base_score_orb` se recalcula cuando ORB-SLAM3 cambia sus datos de calidad;
- un raw no anclado conserva la base ORB y los refuerzos confirmados;
- al anclarse se aplican factores geometricos configurables y acotados;
- una distancia fisicamente sospechosa o excesiva respecto al keyframe
  observador reduce score de forma progresiva;
- el aislamiento global persistente reduce score cuando ya existe soporte
  suficiente para juzgarlo;
- ambos factores son recuperables si ORB actualiza la posicion o aparecen
  vecinos coherentes.

Para cada fused track:

```text
score_fused = clamp(media(score_raw de todos los miembros) + 0.04 * N, 0, 1)
```

`N` es el numero de raw MapPoints miembros. Por tanto, la primera fusion de dos
miembros suma `0.08` y cada miembro posterior suma `0.04`. Se conserva ademas
el refuerzo raw `+0.04` por inlier confirmado: el doble refuerzo es
intencional.

## Integracion

- Los cambios raw ORB se limitan a IDs del `RawChangeSet`.
- El indice espacial actualiza solo celdas vecinas a altas, movimientos o
  retiradas; no recorre toda la nube por llegada.
- Cambios raw o geometricos recalculan solo fused tracks que contienen esos
  miembros.
- Altas, extensiones, merges y bajas de tracks recalculan el fused score en el
  mismo commit logico de fusion.
- `ScoreChangeSet` entrega IDs exactos a `GlobalMapBuilder`; el builder publica
  todos los puntos y solo copia `score`/`rgb`.
- RViz2 conserva el gradiente rojo-amarillo-verde para scores `0-0.5-1`.

## Oclusiones

La visibilidad sparse de 3P se conserva como diagnostico, pero no modifica
numericamente el score en 3R. Decidir si un punto esta realmente ocluido exige
la nube densa prevista para Fase 8; alli se podran corregir posiciones y scores
sin premiar accidentalmente ruido foreground.

## Limites

- sin GT, nueva cola o worker de score;
- sin cambiar geometria, fusion u optimizacion para mejorar una puntuacion;
- sin ocultar puntos por score dentro de `GlobalMapBuilder`;
- sin score visible procedente de una fusion rechazada o stale;
- sin snapshots completos en la ruta incremental.

## Prueba acordada

Ejecutar `prueba_tipica_rodeo_edificio_dos_fiduciales.yaml`, reducir logs y
comprobar grafo web, commits `F3R-*`, score raw/fused y RViz2. La inspeccion
visual se centra especialmente en observar que el ruido pierde score sin que
desaparezca de la nube publicada.

La prueba 193 confirma el refuerzo por revisitas, pero no valida los umbrales
actuales: estructura habitual queda demasiado penalizada y puntos a menos de
1 m conservan score excesivo. La recalibracion acordada fija el limite cercano
fisico en 1 m y hace el limite lejano proporcional al baseline: con los `0.06 m`
actuales, la banda neutra es 1-5 m. Ambos extremos usan caida cuadratica
acotada. La penalizacion permanece en cada raw y puede diluirse mediante la
media fused, sin cap permanente.

## Evolucion futura acordada

No se modifica en esta iteracion la formula ni los paquetes de 3R. Antes de
recalibrar el uso de MapPoints por el mapa voxel de Fase 6, se incorporara en
una futura revision de score una penalizacion adicional, incremental y
reversible para dos casos: un MapPoint al que se aproximan en exceso KeyFrames
de otro dron, y un MapPoint que permanece pobremente agrupado con otros puntos
coherentes. La regla concreta de distancia, madurez, intensidad y recuperacion
se definira y probara entonces; no se puede inferir de GT ni convertir la mera
proximidad en una eliminacion definitiva. Hasta esa revision se conserva la
banda geometrica actual de 1--5 m y la formula vigente. Fase 6 solo consume el
score publicado, filtrando ocupacion con `score >= 0.2` y soporte local minimo
por voxel; no modifica scores desde el servidor de tareas.

La misma revision futura se coordinara con 6D para separar definitivamente la
confianza del MapPoint de la ocupacion voxelizada. 3R seguira siendo la unica
autoridad de cada `score_raw`/`score_fused`, incluidos los ajustes futuros por
aislamiento y observaciones cercanas de otros drones. 6D consumira esos scores
como contribuciones identificadas y calculara de forma reversible un
`voxel_occupancy_score` agregado por voxel; solo su propio umbral de ocupacion
decidira `OCCUPIED`. No se reutilizara el umbral individual del MapPoint como
umbral final del voxel ni se modificara 3R desde `task_server`.

Tambien queda pendiente distinguir la observacion del cuerpo de otro dron de
la geometria estatica cercana. Cuando una futura asociacion canonica confirme
que un MapPoint corresponde al otro dron observado, su score debe pasar a `0`
de manera reversible; al desaparecer o invalidarse esa asociacion, el score se
recalcula desde sus demas evidencias. No basta la cercania espacial entre un KF
y un MP para aplicar este cero, porque podria ser una pared u objeto real junto
al dron. La identidad, umbrales de confirmacion y recuperacion se acordaran y
probaran junto con la recalibracion 3R/6D.

La sustitucion del filtro transitorio de 6D (`score >= 0.2` y cuatro
identidades) debe acordarse, implementarse y validarse como un unico cambio
entre 3R y 6D. Hasta entonces se conserva exactamente el contrato actual y no
se inicia una recalibracion parcial de scores ni de ocupacion.

## Subdocumentos

- `subfase_3R_especificacion.md`: ownership, formula e invariantes.
- `subfase_3R_implementacion.md`: APIs e integracion incremental.
- `subfase_3R_testing.md`: regresiones y simulacion.
- `subfase_3R_criterios.md`: criterios de cierre.
