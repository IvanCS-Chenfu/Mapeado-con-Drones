# Subfase 3F - Especificacion acordada

## Estado

```text
CONSEGUIDA
Preparacion: CERRADA
Acuerdo cerrado: si
Autorizacion funcional de implementacion: EJECUTADA
Dudas abiertas: ninguna
Pendiente: ninguno dentro de 3F
```

## Objetivo

Crear la primera vista sparse global visible y coherente del backend rehecho.
La subfase incorpora:

- `LandmarkScoreManager` como autoridad de scores iniciales de MapPoints raw;
- `GlobalMapBuilder` como vista incremental stateful de KFs y nube sparse;
- publicacion de `/global_sparse_cloud` y `/global_keyframes` al final del
  `PrimaryWorker` creado en 3C;
- ampliacion del grafo web con score, builder y RViz2.

Antes del primer anchor de un submapa no se publica ninguno de sus KFs o
MapPoints. El primer anchor hace visible el backfill acumulado y los deltas
posteriores actualizan solo las dependencias afectadas.

## Resultado funcional esperado

```text
delta material
-> RawMapDatabase
-> LandmarkScoreManager
-> GlobalPoseStore / anchor
-> GlobalMapBuilder incremental
-> GlobalMapServer serializa
-> PointCloud2 + MarkerArray
-> fin de PrimaryTask
```

`GlobalMapBuilder` no es una nueva base autoritativa. Sus caches son una vista
derivada y reconstruible de las autoridades:

- `RawMapDatabase`: geometria y asociaciones ORB-SLAM3 crudas;
- `GlobalPoseStore`: anchors y poses world vigentes de KFs;
- `LandmarkScoreManager`: score vigente por `RawMapPointId`;
- en fases posteriores, `FusedLandmarkManager`: tracks fusionados vigentes.

## Ownership

### `LandmarkScoreManager`

Posee la base de datos de score y la politica que transforma informacion ORB o
eventos semanticos futuros en un valor. No se crea un `LandmarkScoreStore`
separado en 3F.

### `GlobalMapBuilder`

Posee caches publicables, slots estables, indices inversos y dirty sets. Puede
consultar autoridades, pero no modifica raw, poses, anchors, scores ni fusion.

### `GlobalMapServer`

Orquesta el orden del flujo y adapta el resultado de dominio a ROS. No calcula
el score ni reconstruye geometria. En 3F tambien aplica temporalmente el color
de presentacion de cada punto a partir del score para RViz2.

### `PrimaryWorker`

Continua siendo el unico ejecutor del flujo principal. Una entrada no termina
hasta publicar nube y KFs cuando existe cambio publicable. No aparece un
publication worker adicional.

## Alcance de 3F

- Scores iniciales derivados solo de campos ya exportados por ORB-SLAM3.
- Cache world de keyframes.
- Cache world de MapPoints, cada uno con KF asociado estable y score.
- Reproyeccion incremental por IDs afectados.
- Backfill completo al primer anchor, sin publicar submapas no anclados.
- Nube completa autoritativa por mensaje, aunque el calculo interno sea
  incremental.
- Frustums de KFs coloreados deterministamente por submapa.
- MapPoints con gradiente rojo-amarillo-verde derivado del score en servidor.
- Nube y KFs con la misma revision y timestamp.
- Grafo web con eventos reales del flujo acordado.

## Preparacion para fases posteriores

3F crea las interfaces de invalidacion, pero solo activa las fuentes existentes:

- una pose de KF modificada marcara ese KF y recolocara sus MapPoints;
- un score modificado marcara esos MapPoints y actualizara su atributo;
- una fusion futura marcara miembros/tracks y sustituira solo esas entradas.

Las optimizaciones, score avanzado y fusion se implementan en sus subfases
propietarias. 3F no simula resultados que aun no existen.

## Lo anterior que estaba mal y no debe repetirse

- Recorrer todos los submapas y todos los MapPoints en cada `Build()`.
- Crear un snapshot global que copie raw, poses, scores y fusion bajo un mutex
  comun; se llegaron a medir capturas superiores a 27 segundos.
- Mantener `live_state_mutex_` durante transformacion, serializacion o publish.
- Crear timer/worker de publicacion que compita con el flujo principal.
- Hacer que un commit secundario publique, despierte al principal o espere un
  ACK de RViz2/web.
- Reconstruir toda la geometria porque se movio un solo KF o cambio un score.
- Publicar `world_T_local * p_local_mp` cuando no existe un KF observador con
  pose world utilizable.
- Mezclar nube y KFs de revisiones distintas.
- Calcular score o geometria dentro de `GlobalMapServer`.
- Publicar union bruta de todos los MapPoints, incluidos submapas no anclados.
- Aplazar `/global_keyframes` hasta 3U: se publica ya en 3F; 3U solo audita y
  endurece la visualizacion acumulada.

## Fuera de alcance

- Full snapshots y reconciliacion completa de 3G.
- Revisitas fiduciales de 3H.
- BoW, matching, RANSAC y loops.
- Optimizacion fiducial o de loop.
- Fusion real de landmarks.
- Politica avanzada de score de 3S.
- Snapshots periodicos, publication worker o ACK visual.
- Cambios en `ORB_SLAM3`, `orbslam3_ros2` u `orbslam3_msgs`.
- GUI operacional de Fase 7.
- Ground truth para score, mapa o pose final.

## Archivos probables

```text
orbslam3_multi/include/orbslam3_multi/landmark_score_manager.hpp
orbslam3_multi/src/landmark_score_manager.cpp
orbslam3_multi/include/orbslam3_multi/global_sparse_point.hpp
orbslam3_multi/include/orbslam3_multi/global_map_builder.hpp
orbslam3_multi/src/global_map_builder.cpp
orbslam3_multi/include/orbslam3_multi/sparse_global_backend.hpp
orbslam3_multi/src/sparse_global_backend.cpp
orbslam3_server/src/global_map_server.cpp
orbslam3_server/launch/*.launch.py
simulacion_dron/launch/pipeline_flow_visualizer.launch.py
simulacion_dron/pipeline_flow_visualizer/*
simulacion_dron/config/rviz/*.rviz
```

Las rutas exactas se confirmaran contra el arbol vigente antes de implementar.
