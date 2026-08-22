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

## Subdocumentos

- `subfase_3R_especificacion.md`: ownership, formula e invariantes.
- `subfase_3R_implementacion.md`: APIs e integracion incremental.
- `subfase_3R_testing.md`: regresiones y simulacion.
- `subfase_3R_criterios.md`: criterios de cierre.
