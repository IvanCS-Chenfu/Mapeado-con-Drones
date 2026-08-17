# Subfase 3Q - Implementacion acordada

## Principio

Generalizar la ruta 3I-3L; no crear una ruta paralela:

```text
OptimizationInput
-> PoseGraphBuilder
-> OptimizationManager
-> OptimizationValidator
-> PosePatch/DerivedPatch
-> commit coordinado
```

Los adaptadores fiducial/loop construyen constraints distintas, pero comparten
problema, solver, validacion, commit, continuidad y telemetria.

## Tipos

Crear o adaptar tipos equivalentes a:

```text
OptimizationTaskKind
  FIDUCIAL_ABSOLUTE
  LOOP_RELATIVE

OptimizationInput
  task_id / source_arrival_id / kind
  endpoint KFs y submapas
  target absoluto o medidas relativas
  evidencia geometrica viva
  autoridades/revisiones

PoseGraphConstraintType
  TEMPORAL
  COVISIBILITY_NATIVE
  SERVER_GEOMETRIC
  LOOP_RELATIVE
  SOFT_ANCHOR_DEPENDENCY
  FIDUCIAL_HARD
  FIDUCIAL_TARGET
  GAUGE_FIXED
```

`PoseGraphProblem` deja de contener obligatoriamente una
`FiducialOptimizationTask`. Las APIs fiduciales publicas actuales pueden
mantenerse como wrappers para conservar compatibilidad y tests.

Cada edge conserva transformacion medida, informacion/peso, soporte, fuente y
revisiones. No se deriva una constraint relativa copiando ciegamente la pose
world actual.

## Store de constraints

Crear `OptimizationConstraintStore` o equivalente con metadata ligera:

- observaciones hard y controles;
- segmentos temporales por submapa;
- constraints loop/fusion aceptadas;
- dependencias soft y linaje;
- revisiones y estado soft/confirmado/retirado.

No guarda poses duplicadas. Debe poder reconstruirse deterministicamente en
replay a partir de commits/eventos persistidos. Sus consultas devuelven vistas
acotadas por IDs/componente.

## `LoopPipeline`

Modificar la decision de error alto para:

1. conservar grupos de geometria compatibles;
2. acumular una hipotesis compacta por primera query;
3. exigir segundo apoyo independiente;
4. resolver ambiguedad entre grupos;
5. producir `LoopOptimizationInput` con las constraints dominantes;
6. no terminar en `SameSubmapDiagnostic` solo por compartir dron/submapa.

Las guardas causal/temporal permanecen. `SameSubmap` sigue apareciendo en logs,
pero no decide fusion frente a optimizacion.

## `PoseGraphBuilder`

Reutilizar la clase y separar estas etapas:

1. resolver endpoints y autoridades;
2. consultar el subgrafo minimo de constraints;
3. capturar snapshots raw/pose/covisibilidad solo de IDs necesarios;
4. materializar intervalos temporales de cada submapa;
5. seleccionar controles base con cobertura 3D/temporal del 30 %;
6. insertar controles obligatorios por hard/loop/fusion/soft/covisibilidad;
7. añadir aristas y priors con procedencia explicita;
8. crear propagacion por intervalo y tail por submapa;
9. producir revisiones consumidas y resumen de cobertura.

Las aristas de covisibilidad no expanden recursivamente todo el mapa. Pueden
formar parte de los caminos minimos y promover controles adicionales.

Para optimizacion fiducial, el target absoluto entra como prior. Para loop, la
constraint principal sigue siendo relativa aunque el solver use una referencia
de gauge internamente.

## Pesos y robustez

Reutilizar la configuracion fiducial y ampliarla con pesos por familia:

```text
hard fiducial          -> inamovible
target fiducial        -> absoluto fuerte
loop actual            -> relativo fuerte verificado
fusion/loop anterior   -> relativo soft inicialmente
covisibilidad server   -> soporte/residual
covisibilidad ORB      -> soporte nativo
temporal               -> continuidad local
```

Normalizar por densidad para que una zona con muchas aristas no domine por
conteo. Aplicar perdida robusta a constraints no hard. Todos los parametros son
configurables y sus valores iniciales se revisaran con residuales Gazebo; no
usar GT para afinarlos.

## `OptimizationManager`

Mantener una unica clase y adaptar el codigo SE(3) existente. Reutilizar las
partes estables de la optimizacion fiducial y las ideas utiles de `legacy2`,
sin portar su infraestructura, snapshots globales, dumps ni locks largos.

El solver debe:

- fijar hard/gauge;
- optimizar simultaneamente controles de varios submapas;
- evaluar todas las familias de constraints;
- mover ambos extremos de un loop cuando la autoridad lo permite;
- devolver propuesta privada, costes y residuales;
- evitar NaN/divergencia y tener iteraciones acotadas;
- mantener la API comun para fiducial y loop.

No crear `LoopOptimizationManager` ni enlazar el backend a un segundo
optimizador.

## `OptimizationValidator`

Generalizar la validacion comun:

- cobertura completa de controles obligatorios;
- hard/gauge inmoviles;
- coste final inferior al inicial;
- target fiducial dentro de umbral o loop dentro del umbral de fusion;
- residual por aristas temporal/covisible/loop finito;
- continuidad y epochs validos;
- propuesta/revisiones completas;
- diagnostico de degradacion de fusiones anteriores.

En la primera implementacion loop solo devuelve:

```text
ACCEPT_FULL
REJECT(reason)
STALE(reason)
```

No compromete parciales. Fiducial puede conservar compatibilidad con su
resultado parcial anterior hasta que la generalizacion demuestre una sustituta
equivalente.

## Preparacion de patches

Antes del lock coordinador preparar sobre una vista candidata:

- `PosePatch` por submapa y su rollback acotado;
- continuidad/tail y linaje;
- actualizacion de `relative_pose_current`;
- metadata del constraint aceptado;
- fusion 3P, covisibilidad y score opcionales;
- dirty IDs exactos.

`FusedLandmarkManager::PrepareFusion()` debe poder consultar poses candidatas
u overrides sin escribir live. Reutiliza los inliers de `LoopGeometryResult`.

Si la fusion no queda lista por dispersion/no-op/evidencia insuficiente, se
registra `fusion_skipped_after_optimization` y el patch de pose puede seguir.

## Commit coordinado

Bajo una seccion breve:

1. revalidar revisiones realmente consumidas;
2. integrar KFs tardios compatibles;
3. validar que ningun hard se mueve;
4. aplicar poses/continuidad/constraints;
5. aplicar fusion/covisibilidad/score si estan listas;
6. publicar una revision derivada coherente;
7. marcar KFs, tracks, raws ocultos y scores dirty;
8. liberar locks.

Si una etapa inesperada falla, se revierten solo los patches ya aplicados y el
intento termina stale/rollback. El servidor encola despues una `LoopTask` BAJA
fresca mediante la politica 3P; no existe recursion ni retry dentro del solver.

`RawMapDatabase` no se escribe. El secundario no llama a `BuildGlobalMap()`.

## Continuidad

El commit calcula un `ContinuationRecord` por cada submapa con tail abierto:

```text
world_T_kf = world_T_last_control_accepted
           * inverse(raw_T_last_control_current)
           * raw_T_kf_current
```

- KFs tardios del intervalo se interpolan entre controles;
- KFs posteriores siguen el ultimo control optimizado;
- un hard posterior detiene el tail;
- hijos soft siguen el delta de su apoyo si quedan fuera del grafo;
- si entran en el grafo reciben poses propias del mismo commit;
- KFs futuros usan la continuidad nueva.

## Primer fiducial de hijo soft

Modificar la rama transitoria de 3H/3O:

- dentro de umbral: promocion hard atomica, conservar poses y constraint loop;
- fuera de umbral: crear tarea MAX fiducial covisible;
- tras `ACCEPT_FULL`: promocionar hard, cortar propagacion soft y conservar el
  constraint historico;
- si reject/stale: no cortar dependencia ni reanclar parcialmente.

El first anchor de un submapa realmente no colocado no cambia.

## Servidor, prioridad y `stop_drones`

En el worker secundario:

```text
LoopTask BAJA activa
-> decision OptimizationEvidence aceptada
-> set loop_optimization_active
-> recompute stop_drones/backpressure
-> ejecutar grafo/solver/validator/commit/fusion
-> clear loop_optimization_active en todo camino de salida
```

La guarda RAII o equivalente debe liberar el flag ante excepcion. El flag dura
hasta terminar la fusion directa y la tarea completa. No cambia prioridad ni
interrumpe la tarea. Una MAX pendiente empieza inmediatamente despues.

La presion total sigue siendo OR de watermarks principal/secundario y cualquier
optimizacion fiducial/loop activa, con la histeresis existente.

## Grafo web y logs

Añadir rutas reales, sin pulsos artificiales:

```text
LoopDecision --opt_loop--> PoseGraphBuilder
Validation --fusion after opt--> FusedLandmarkManager
Validation/commit -> GlobalPoseStore
```

El mismo `task_id` permanece iluminado desde la decision hasta task end.

Marcadores minimos:

```text
F3Q-BRANCH-BEGIN
F3Q-WINDOW
F3Q-CONSTRAINTS
F3Q-GRAPH
F3Q-SOLVER
F3Q-VALIDATION
F3Q-FUSION-PREPARE
F3Q-COMMIT-BEGIN/END
F3Q-STALE / F3Q-ROLLBACK
F3Q-TASK-END
```

Incluir `task_id`, kind, KFs/submapas, hard/soft, vertices, edges por fuente,
errores, residuales, IDs movidos, tails, tiempos y estado de stop.

## Archivos probables

```text
orbslam3_multi/include/orbslam3_multi/{loop_pipeline,pose_graph_problem}.hpp
orbslam3_multi/include/orbslam3_multi/{pose_graph_builder,optimization_manager}.hpp
orbslam3_multi/include/orbslam3_multi/{optimization_validator,global_pose_store}.hpp
orbslam3_multi/include/orbslam3_multi/{covisibility_database,fused_landmark_manager}.hpp
orbslam3_multi/include/orbslam3_multi/sparse_global_backend.hpp
orbslam3_multi/src/{loop_pipeline,pose_graph_builder,optimization_manager}.cpp
orbslam3_multi/src/{optimization_validator,global_pose_store}.cpp
orbslam3_multi/src/{covisibility_database,fused_landmark_manager}.cpp
orbslam3_multi/src/sparse_global_backend.cpp
orbslam3_server/src/global_map_server.cpp
orbslam3_server/include/orbslam3_server/secondary_queue.hpp
simulacion_dron/web/pipeline_flow/{graph_definition.js,app.js}
```

Crear archivos nuevos solo para tipos/store con ownership real. No tocar
`ORB_SLAM3`, `orbslam3_ros2` ni `orbslam3_msgs` sin una necesidad nueva,
explicada y autorizada.
