# `CovisibilityDatabase`

## Rol

`CovisibilityDatabase` es la capa de `3M` que conserva relaciones confirmadas
entre `KeyFrames`. No sustituye los datos locales de `RawMapDatabase` ni las
poses globales de `GlobalPoseStore`.

Una arista existe únicamente si procede de covisibilidad nativa de ORB-SLAM3 o
de un loop confirmado por geometría. No almacena candidatos BoW, rechazos ni
estados pendientes.

Esta frontera se mantiene en la correccion de rendimiento acordada. Los
rechazos repetibles por revision deben vivir en una estructura separada, por
ejemplo `LoopPairAttemptDatabase`; no deben contaminar
`CovisibilityDatabase`. Un par confirmado continua canonizado aqui y cancela
trabajo posterior, mientras que un rechazo solo es reutilizable si coinciden
las revisiones materiales de ambos KFs.

## Archivos

```text
orbslam3_multi/include/orbslam3_multi/covisibility_database.hpp
orbslam3_multi/src/covisibility_database.cpp
orbslam3_multi/src/test_covisibility_database.cpp
orbslam3_multi/CMakeLists.txt
```

## Datos y API

`CovisibilityEdge` usa extremos `RawKeyFrameId`, por lo que conserva siempre
`(drone_id, map_epoch, local_kf_id)`. Los extremos se normalizan a orden
canónico para que `A-B` y `B-A` no dupliquen una relación.

La arista guarda soporte absoluto y relativo, cobertura 2D/3D, fuente,
`relative_pose_measured` inmutable, `relative_pose_current` actualizable,
`information_weight`, `arrival_id` y revisión. Una arista confirmada puede ser
débil; solo `HasStrongEdge` permite omitir la validación geométrica completa.
Las APIs disponibles son:

- `ImportOrbslam3Native` para importar conexiones de `OrbKeyFrame`;
- `ImportOrbslam3NativeForKeyFrames` para limitar la revisión a KFs nuevos o
  con conexiones modificadas;
- `AddConfirmedLoopEdge` para una relación ya validada por geometría;
- `HasConfirmedEdge`, `HasStrongEdge`, `GetEdge`, `GetNeighbors` y
  `GetEdgesForWindow` para consumidores de loops y optimización;
- `UpdateRelativePoseCurrent`, que no modifica la medición original;
- `Clear` y `GetStats` para replay y observabilidad.

`PoseGraphBuilder` recibe opcionalmente esta base y añade aristas
`SoftConsistency` solo entre controles que ya están en la ventana: no crea
vértices ni reemplaza las aristas temporales de `3I`.

## Integración y logs

`global_map_server` importa covisibilidad después de cada delta y full snapshot
y reinicia la base al iniciar replay. Toda conexión ORB-SLAM3 de peso positivo
se conserva confirmada. `f1m_covisibility_min_weight=15.0` permanece como
umbral de consumidores, por ejemplo al construir una ventana candidata, pero
ya no elimina evidencia durante el almacenamiento.

Desde `3P`, `LoopDecisionManager` usa `AddConfirmedLoopEdge` para cada
`FUSION_CANDIDATE`. La fuente es `ServerLoopGeometric`, el soporte procede de
los inliers y se guardan separadas la relación medida derivada de RANSAC y la
relación current calculada desde `GlobalPoseStore`. Este registro no construye
un grafo; solo queda disponible para consumidores futuros.

Los logs esperados son:

```text
[F1M-COVIS-IMPORT]
[F1M-COVIS-EDGE-ADD]
[F1M-COVIS-EDGE-UPDATE]
[F1M-COVIS-QUERY]
[F1M-COVIS-SUMMARY]
[F1P-COVIS-EDGE-CONFIRMED]
[F1P-COVIS-EDGE-ALREADY-KNOWN]
```

## Estado

Implementado, compilado y validado. El target `orbslam3_multi` debe incluir
`covisibility_database.cpp` y `loop_detector.cpp`, porque `PoseGraphBuilder` y
`global_map_server` enlazan esas APIs. También instala
`test_covisibility_database`, que cubre importación nativa en orden inverso,
aristas débiles/fuertes, loop geométrico sintético, actualización de
`relative_pose_current` y revisiones raw por etapa.

El bloqueo live del 2026-07-17/2026-07-19 quedó resuelto restaurando/buildando
`ORB_SLAM3`. La caída posterior del servidor venía de Eigen al hacer
`matrix = matrix.inverse()` durante la canonización de aristas invertidas; las
inversas se evalúan ahora en un temporal con `.eval()`. Validación final del
2026-07-20: build `orbslam3_multi orbslam3_server` OK,
`test_covisibility_database` OK, `prueba_1` con `SIM-EXIT-CODE 0`,
`confirmed_edges=3025`, `orbslam3_native=3025` y
`[F1M-COVIS-QUERY] returned_edges=63`.

En `prueba_48`, 3P añade `91` aristas geométricas: `58` inter-dron y `33`
intra-dron. El estado final es `3075` aristas, `2984` nativas y `91` de loop
geométrico.

En `prueba_62`, la clasificación fuerte/débil permite `117` confirmaciones por
fast path sin convertir aristas de soporte local en veto absoluto. El estado
final contiene `12372` aristas, de ellas `295` geométricas.
