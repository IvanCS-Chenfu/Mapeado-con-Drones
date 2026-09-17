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
  base_score_orb * factor_distancia * factor_aislamiento * factor_cuerpo_dron
  + 0.04 * inliers_confirmados,
  0, 1)
```

- `base_score_orb` se recalcula cuando ORB-SLAM3 cambia sus datos de calidad;
- un raw no anclado conserva la base ORB y los refuerzos confirmados;
- al anclarse se aplican factores geometricos configurables y acotados;
- una distancia fisicamente sospechosa o excesiva respecto al keyframe
  observador reduce score de forma progresiva;
- tras tres observaciones, un aislamiento global persistente sin dos vecinos
  validos a distancia euclidea `<=0.30 m` fija score `0`; ese cero prevalece
  sobre nuevos inliers y se recupera al aparecer un vecino;
- un MapPoint dentro del volumen fisico de un dron anclado tambien fija score
  `0`. La mascara usa una esfera centrada en el `base_link` de cada KeyFrame
  global valido y radio igual a la semidiagonal de `dimensions_m`, sin margen;
  es independiente de quien creo u observo el MapPoint y se retira al cambiar
  la pose, la dimension o la validez del KeyFrame;
- ambos factores son recuperables si ORB actualiza la posicion o aparecen
  vecinos coherentes.

Para cada fused track:

```text
score_fused = clamp(media(score_raw de miembros soportados) + 0.04 * N_soportados, 0, 1)
```

`N_soportados` es el numero de raw MapPoints no aislados ni enmascarados por el
cuerpo de un dron. Un track formado solo por miembros sin soporte publica `0`;
al recuperar al menos un miembro, el track se calcula
solo con ese soporte. Se conserva ademas el refuerzo raw `+0.04` por inlier
confirmado: el doble refuerzo es intencional para miembros soportados.

## Integracion

- Los cambios raw ORB se limitan a IDs del `RawChangeSet`.
- El indice espacial actualiza solo celdas vecinas a altas, movimientos o
  retiradas; no recorre toda la nube por llegada.
- Las esferas de cuerpo se indexan con la misma granularidad. Un cambio de KF
  solo reevalua MapPoints de la union de su volumen anterior y nuevo; una alta
  de MapPoint consulta las esferas que solapan su propia celda.
- `global_map_server` consume el snapshot transitorio `/mission/registry` para
  cachear `dimensions_m` por dron. Esta entrada solo alimenta Fase 3; no llama
  a `task_server`, `VoxelMapWorker`, D* ni modifica la politica de Fase 6.
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

La penalizacion de MapPoints pobremente agrupados esta implementada en 3R con
indice incremental, radio `0.30 m`, dos vecinos validos y cero duro tras tres
observaciones. Tambien se enmascara reversiblemente la evidencia que cae dentro
del cuerpo fisico de cualquier dron anclado: no requiere asociacion canonica,
ownership ni que el KF y el MapPoint pertenezcan al mismo dron. Fase 6 solo
consume el score publicado y no modifica 3R.

La misma revision futura se coordinara con 6D para separar definitivamente la
confianza del MapPoint de la ocupacion voxelizada. 3R seguira siendo la unica
autoridad de cada `score_raw`/`score_fused`, incluidos los ajustes futuros por
aislamiento y observaciones cercanas de otros drones. 6D consumira esos scores
como contribuciones identificadas y calculara de forma reversible un
`voxel_occupancy_score` agregado por voxel; solo su propio umbral de ocupacion
decidira `OCCUPIED`. No se reutilizara el umbral individual del MapPoint como
umbral final del voxel ni se modificara 3R desde `task_server`.

La esfera fisica es una primera defensa deliberadamente geometrica. Si en una
fase posterior se necesita distinguir el cuerpo real de una pared muy cercana,
se podra anadir una asociacion visual canonica sin trasladar esa responsabilidad
a Fase 6 ni cambiar la semantica reversible del score.

La sustitucion del filtro transitorio de 6D (`score >= 0.2` y cuatro
identidades) debe acordarse, implementarse y validarse como un unico cambio
entre 3R y 6D. Hasta entonces se conserva exactamente el contrato actual y no
se inicia una recalibracion parcial de scores ni de ocupacion.

## Subdocumentos

- `subfase_3R_especificacion.md`: ownership, formula e invariantes.
- `subfase_3R_implementacion.md`: APIs e integracion incremental.
- `subfase_3R_testing.md`: regresiones y simulacion.
- `subfase_3R_criterios.md`: criterios de cierre.
