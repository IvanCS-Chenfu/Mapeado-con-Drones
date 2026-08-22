# Subfase 3R - Implementacion acordada

## Flujo raw

```text
RawMapDatabase commit
-> ApplyRawChanges(ids ORB afectados)
-> construir entradas globales para MPs anclados afectados
-> ApplyGeometryChanges(upserts/removals y vecindad espacial)
-> recalcular fused tracks de raw scores modificados
-> MarkScoreChanges(ScoreChangeSet exacto)
```

`LandmarkScoreRecord` conserva `base_score_orb`, `distance_factor`,
`isolation_factor`, ajustes confirmados y score final. Los cambios de input que
no alteran la salida se almacenan sin ensuciar publicacion.

## Refinamiento geometrico

El backend transforma cada raw MP mediante la pose global activa de un
keyframe observador. Entrega posicion world, distancia camara-punto y baseline
estereo al manager. Si no existe pose activa, retira la entrada geometrica y el
raw vuelve a factores neutros.

El manager mantiene un indice voxel persistente. Una alta, baja o movimiento
solo reevalua el propio punto y las celdas vecinas antiguas/nuevas. Anchors y
commits de optimizacion entregan los keyframes movidos para actualizar sus MPs.

## Fusion

`FusedLandmarkManager` prepara inliers raw `+0.04` y fused upserts usando la
media de todos los miembros mas `0.04 * N`. `BuildScoreUpdatesForMembers`
propaga cambios ORB/geometricos posteriores a los tracks afectados. Merge y
retirada emiten los upserts/removals exactos en el patch de fusion.

La visibilidad sparse conserva presupuesto y contadores, pero no añade
`ExpectedVisibleMiss` ni `ForegroundContradiction` a `raw_score_evidence`.

## Configuracion

Todos los defaults geometricos son conservadores y parametros ROS:

- radio/minimo de vecinos y madurez para aislamiento;
- factor minimo de aislamiento;
- umbral cercano fijo `1.0 m`, factor minimo `0.05` y caida cuadratica;
- multiplicador lejano `83.333333`, fallback `5.0 m` y factor minimo `0.25`;
- reward raw por inlier y bonus fused por miembro, ambos `0.04`.

El limite lejano se calcula como
`max(near_limit, far_baseline_multiplier * baseline)`. La banda intermedia usa
factor `1`; una actualizacion de geometria puede reducir o recuperar el factor.

## Telemetria y publicacion

Los eventos usan prefijo `F3R-*` y exponen revisiones, altas/cambios, factores,
tracks fused y estadisticas min/mean/max. El grafo web muestra patches ligeros;
no recibe arrays completos de score.

`GlobalMapBuilder` copia score incrementalmente y mantiene igual el conjunto de
puntos publicados. `orbslam3_server` deriva `rgb` con rojo en `0`, amarillo en
`0.5` y verde en `1`.

## Exclusiones

No se añaden mensajes, worker, prioridad, GT ni filtrado. Scoring no modifica
posiciones, anchors, asociaciones, decisiones de loop o resultado de fusion.
