# `SubcloudLoopVerifier`

## Rol

`SubcloudLoopVerifier` es la pieza de `3O` que convierte un candidato BoW en
evidencia geométrica preliminar.

Recibe un `LoopCandidate`, consulta `RawMapDatabase`, `GlobalPoseStore` y
`CovisibilityDatabase`, y devuelve un `LoopVerificationResult`. No escribe en
ninguna de esas bases, no fusiona, no optimiza y no usa GT.

Desde la corrección de carga de `3P`, la API permite separar el snapshot de
entrada del cálculo costoso:

```text
CaptureCandidate(...) -> CapturedLoopVerification
PrepareCapturedCandidate(...) -> PreparedLoopVerification
VerifyPreparedCandidate(...) -> LoopVerificationResult
```

`CapturedLoopVerification` conserva una copia acotada de los KFs, MapPoints,
poses world y scores necesarios para query y ventana candidata.
`PreparedLoopVerification` conserva las subnubes ya construidas. El servidor
ejecuta solo `CaptureCandidate` con acceso consistente al estado live; la
construcción de subnubes, matching, reducción y RANSAC se realizan en el worker
sin mantener `live_state_mutex_`. `PrepareCandidate(...)` y
`VerifyCandidate(...)` se conservan como envoltorios síncronos compatibles.

Desde la revisión incremental de `3P` también ofrece dos rutas previas:

- `MatchNewMapPointsAgainstConfirmedNeighbors`: compara solo asociaciones
  KF-MapPoint nuevas con vecinos cuya covisibilidad ya está confirmada;
- `FindUnknownAlignedOverlaps`: confirma una relación desconocida mediante
  matches estrictos uno-a-uno, cobertura 2D/3D, transformación rígida pequeña y
  residual acotado; después expande correspondencias guiándose por esa
  transformación.

La búsqueda espacial usa un hash 3D. Antes de Hamming consulta identidad
`RawMapPointId` y pertenencia al mismo track de `FusedLandmarkManager`. Para KFs
del mismo submapa, una identidad raw compartida se resuelve directamente y no
ejecuta matching cartesiano.

Desde el rediseño del 2026-08-04, `FindUnknownAlignedOverlaps` tambien evalua
relaciones intra-submapa desconocidas. Los `RawMapPointId` compartidos aportan
soporte estricto solo cuando estan distribuidos en imagen y espacio 3D; se
tratan como una identidad ya comun y nunca crean pares de fusion. Tras confirmar
la relacion, la expansion por posicion, descriptor y unicidad solo emite para
fusion IDs distintos que todavía no pertenezcan al mismo track. La evidencia
ya conocida sí cuenta para soporte y cobertura. Si el soporte no basta, se conserva el fast path
posicion/descriptor y el fallback 3N/3O.

## Archivos

```text
orbslam3_multi/include/orbslam3_multi/subcloud.hpp
orbslam3_multi/include/orbslam3_multi/loop_verification_result.hpp
orbslam3_multi/include/orbslam3_multi/subcloud_loop_verifier.hpp
orbslam3_multi/src/subcloud_loop_verifier.cpp
```

## Flujo

1. Si el par ya tiene una arista fuerte en `CovisibilityDatabase`, devuelve
   `ALREADY_CONFIRMED_COVISIBILITY` y el servidor loggea
   `[F1O-SUBCLOUD-SKIP-CONFIRMED-COVIS]`.
2. `CaptureCandidate` valida ambas poses, selecciona la ventana candidata y
   copia solo el estado necesario desde las bases live.
3. `PrepareCapturedCandidate` construye `query_subcloud` desde MapPoints
   observados por el KF query.
4. La captura construye una ventana candidata alrededor del `candidate_seed`: seed,
   covisibles, parent/children, vecinos temporales y vecinos espaciales.
5. La preparación construye `candidate_subcloud` desde los MapPoints observados por esa ventana,
   deduplicados por `RawMapPointId`.
6. `VerifyPreparedCandidate` ejecuta matching ORB por Hamming con ratio test y cross-check opcional.
7. El cálculo reduce la nube candidata con una caja robusta percentil 10-90 derivada de los
   matches; si faltan matches o la reducción queda degenerada, usa fallback.
8. Ejecuta matching refinado y RANSAC 3D-3D determinista.
9. Si RANSAC confirma geometría, calcula `relative_pose_measured`, residuales,
   cobertura 2D/3D, `error_t`, `error_yaw`, `error_rot` y `loop_confidence`, y conserva los
   pares `(query RawMapPointId, candidate RawMapPointId)` clasificados como
   inliers.

## Decisiones

`LoopGeometryDecision` puede ser:

```text
REJECT
HOLD_INSUFFICIENT_EVIDENCE
FUSION_CANDIDATE
LOOP_OPTIMIZATION_CANDIDATE
ALREADY_CONFIRMED_COVISIBILITY
```

En `3O` esas decisiones son solo evidencia. Desde `3P`, `FUSION_CANDIDATE`
inserta covisibilidad y fusiona solo pares distintos; un
`LOOP_OPTIMIZATION_CANDIDATE` registra el loop confirmado como handoff
`OPTIMIZATION_PENDING`, sin fusionar ni modificar poses. El verificador sigue
sin escribir estado.

## Configuración

El servidor expone parámetros `loop_verify_*` para tamaños mínimos de subnube,
ventana candidata, matching ORB, reducción por caja de matches, RANSAC y
umbrales de aceptación. Los defaults están en `SubcloudLoopVerifierConfig`.

## Logs

Marcadores principales:

```text
[F1N-SUBCLOUD-VERIFY-START]
[F1N-SUBCLOUD-QUERY-BUILD]
[F1N-SUBCLOUD-CANDIDATE-SEED]
[F1N-SUBCLOUD-CANDIDATE-WINDOW]
[F1N-SUBCLOUD-CANDIDATE-BUILD]
[F1N-SUBCLOUD-MATCH]
[F1N-SUBCLOUD-CANDIDATE-REDUCE-MATCH-BOX]
[F1N-SUBCLOUD-CANDIDATE-REDUCE-STATS]
[F1N-SUBCLOUD-MATCH-REFINED]
[F1N-SUBCLOUD-RANSAC]
[F1N-SUBCLOUD-ERROR]
[F1N-SUBCLOUD-DECISION]
```

`[F1N-SUBCLOUD-ERROR]` es el log técnico del error de pose calculado, no un
error ROS.

## Validación 3O

El 2026-07-21 se validó con build y tres pruebas:

```text
build orbslam3_multi orbslam3_server simulacion_dron: BUILD-EXIT-CODE 0
prueba_6 live con prueba_tipica_rodeo_edificio_dos_fiduciales.yaml: success=true, 459 verificaciones, 147 aceptados con error, 312 rechazos geométricos, 3060 skips por covisibilidad ORB-SLAM3
prueba_6 inter-dron: 321 verificados, 130 aceptados, 191 rechazados, 129 LOOP_OPTIMIZATION_CANDIDATE con error de pose
prueba_6 fiduciales: 2 tareas creadas, dry-run success=true, sin apply por cost_not_reduced
prueba_2 replay real: success=true, 298 verificaciones, 71 geometry_confirmed=true
prueba_3 replay real: success=true, 223 reducciones robustas sin fallback
errores graves finales: ninguno en ORB-SLAM3/SubcloudLoopVerifier; un `[ERROR]` de Gazebo o `generador_URDF` puede aparecer durante cleanup después de `SIM-DONE success=true`
```

La revalidación integrada del 2026-07-28 no requirió cambios de código:

```text
prueba_46 con prueba_tipica_anclaje_diferencial.yaml: success=true
680 candidatos BoW; 136 intentos por limite de uno por query; 544 sin evaluar
54 intentos sin pose world pre-anchor; 82 comparaciones evaluables
77 geometry_confirmed=true; 5 low_inlier_ratio
49 confirmados inter-dron; 28 intra-dron; 19 near_same_submap
76 FUSION_CANDIDATE; 1 LOOP_OPTIMIZATION_CANDIDATE
0 fusiones, 0 tareas/applies de loop y 0 mutaciones de RawMapDatabase
```

La integración 3P añade una cola en el servidor para no perder candidatos
rechazados temporalmente por falta de pose. No los reevalúa en cada delta: solo
los vuelve a entregar a 3O cuando query y candidate tienen pose world. En
`prueba_48` se aplazan `46`, se reintentan en dos bloques (`2` y `44`) y la cola
termina vacía.

La captura acotada se valida en `prueba_53`: `capture_ms` tiene media/máximo
`26.1/48.5 ms` y `prepare_ms` baja desde `1082.4/23832.4 ms` en `prueba_51` a
`47.3/86.3 ms`. El tiempo de espera del mutex queda medido por separado:
`lock_wait_ms` alcanza `1885.9 ms` de media y `13937.0 ms` de máximo porque los
callbacks del servidor todavía mantienen el mutex durante operaciones más
amplias.

El patron de captura inmutable ya se extiende al fast path y BoW desde el
servidor: cada resultado conserva revisiones raw y poses capturadas, y solo se
aplica si siguen vigentes. La falta de pose/anchor continua siendo diferible y
no se registra como rechazo geometrico definitivo.

En `prueba_58`, `579` resúmenes incrementales producen `326` pares directos y
una confirmación alineada con `155` matches estrictos y `210` expandidos. Los
`455` workers restantes mantienen `prepare_ms=51.429/94.411` y
`compute_ms=445.124/853.017` de media/máximo; la deuda dominante sigue siendo
el volumen admitido y `lock_wait_ms`.

Los tests del rediseño cubren soporte intra-submapa por IDs raw compartidos,
cobertura distribuida, consulta de fused track previa al descriptor y expansion
exclusiva sobre IDs distintos. En `prueba_62` se ejecutan `513` queries
alineadas, se confirman `117` relaciones y se resuelven `122170` identidades
raw sin descriptor. El worker mantiene RANSAC fuera del mutex y los resultados
stale no se reencolan de inmediato.
