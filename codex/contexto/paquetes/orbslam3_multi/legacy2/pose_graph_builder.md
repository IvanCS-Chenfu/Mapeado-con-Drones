# `PoseGraphBuilder`

## Rol actual

`PoseGraphBuilder` construye `PoseGraphProblem` para tareas fiduciales y loops.
No ejecuta el solver ni escribe poses. `OptimizationManager` consume su salida;
el servidor valida y compromete el candidato sobre estado privado.

La ruta fiducial distingue dos conceptos:

- primer fiducial de submapa: crea el anchor rigido inicial;
- primer fiducial de optimizacion: fiducial previo al fiducial recien observado.

Una vez que el submapa esta anclado, cualquier fiducial posterior al anchor
puede crear una tarea si su error 6D supera umbral. No es una logica limitada a
la segunda observacion del submapa.

## Archivos

```text
orbslam3_multi/include/orbslam3_multi/pose_graph_builder.hpp
orbslam3_multi/src/pose_graph_builder.cpp
```

## API

```cpp
PoseGraphBuildResult BuildForFiducialTask(
    const FiducialOptimizationTask& task,
    const RawMapDatabase& raw_db,
    const GlobalPoseStore& pose_store,
    const CovisibilityDatabase* covisibility_db = nullptr) const;

PoseGraphBuildResult BuildForLoopTask(
    const LoopOptimizationTask& task,
    const RawMapDatabase& raw_db,
    const GlobalPoseStore& pose_store,
    const CovisibilityDatabase* covisibility_db = nullptr) const;
```

## Grafo de loop

`BuildForLoopTask` no reutiliza un target fiducial falso. Construye una ventana
conservadora del lado query y añade una arista explicita
`PoseGraphEdgeType::LoopRelative` entre query y candidate con la medida
relativa verificada por `3O`.

Para loops entre submapas, la ventana candidata se incorpora como referencia
fija. Para loops del mismo submapa, el seed candidato queda fijo. Los priors
suaves conservan la pose actual, pero no existe `FiducialTarget` asociado al
loop. Los hard fiducials siguen siendo fijos y la propagacion solo afecta al
lado query permitido por los controles existentes.

La seleccion de vertices de loop limita por defecto la ventana variable al
`10 %`, separada del ratio fiducial. El servidor exige soporte geometrico
previo y suprime pares temporales cercanos de error alto antes de llamar al
builder.

## Ventana fiducial

1. Obtiene los KFs raw del submapa, los ordena y conserva los que tienen pose
   world.
2. Busca el hard fiducial anterior de la optimizacion, dentro del mismo
   `(drone_id, map_epoch)`, y distinto del target.
3. Define el intervalo cerrado entre el KF anchor y el KF target.
4. Mantiene ambos extremos como vertices obligatorios.
5. Calcula el numero de controles como
   `ceil(vertex_selection_ratio * window_keyframes)`, con minimo de dos.
6. Reparte el resto con `balanced_coverage_sample`: parte iterativamente los
   mayores huecos relativos de la ventana y elige un control dentro del hueco.
   La coordenada de cobertura mezcla distancia acumulada 3D e indice temporal,
   para que tambien reciban controles las zonas con muchos KFs aunque el mapa
   estime poco desplazamiento.
7. Reserva un porcentaje par de los controles para proteger la vecindad de los
   dos fiduciales; por defecto se toma el `20 %` de los vertices objetivo,
   mitad cerca del fiducial previo y mitad cerca del target.
8. Conecta los controles seleccionados en orden temporal.
9. Crea un `PropagationPlan` para los KFs no seleccionados.

El muestreo es determinista. Las esquinas 3D/SE(3) son candidatas preferentes
al escoger dentro de un hueco, pero no se insertan todas como obligatorias. Esto
evita racimos de vertices pegados en una esquina y conserva cupo para tramos
con KFs repartidos por la ventana.

No se permiten umbrales absolutos para decidir si hay optimizacion: si el error
fiducial supera umbral, el builder debe intentar construir un grafo del tramo.
Los umbrales absolutos de distancia maxima entre vertices, gap de ID, longitud
de ventana o escala del mundo no deben actuar como rechazo.

Los vertices de esquina no se definen solo por yaw. La seleccion activa combina
el angulo 3D entre tramos consecutivos de traslacion y el cambio rotacional 3D
`SO(3)` entre KFs vecinos. Una esquina seleccionada emite
`corner_3d_vertex`; una muestra de cobertura normal emite
`balanced_coverage_sample`.

## Extremos obligatorios

El grafo siempre contiene:

```text
previous_fiducial_anchor: fixed=true, hard_fiducial=true
target_fiducial_error: fixed o prior absoluto 6D objetivo
```

Si no existe un hard fiducial previo valido en el mismo submapa, devuelve:

```text
success=false
reason=previous_fiducial_anchor_missing
```

No se permite construir el tramo cruzando `map_epoch`.

## Cobertura y propagacion

Para `N` KFs y ratio `r`:

```text
control_vertices = max(2, ceil(r * N))
temporal_edges = control_vertices - 1
propagation = N - control_vertices
coverage.reason = balanced_coverage_sample
```

Las aristas unen controles seleccionados consecutivos y conservan el soporte de
los KFs intermedios reales. No existe limite de distancia recorrida, separacion
entre controles, gap de ID ni cantidad maxima de vertices.

La vecindad protegida de ambos fiduciales debe mantener su pose relativa local
respecto al fiducial correspondiente. Puede implementarse como vecinos fijos
por pose inducida o como restricciones relativas de peso muy alto; si se usa
una formulacion blanda, ese peso debe ser claramente superior al de una arista
temporal normal.

## Configuracion activa

`PoseGraphBuilderConfig` incluye:

```text
vertex_selection_ratio=0.30
fiducial_neighborhood_vertex_ratio=0.20
min_vertices=2
include_temporal_edges=true
```

Ya no incluye:

```text
max_vertices
max_path_length
max_temporal_edge_kf_gap
max_temporal_edge_length_m
```

`fiducial_neighborhood_vertex_ratio` selecciona vertices protegidos cerca de
ambos fiduciales, repartidos de forma par. `fiducial_neighborhood_radius_m`
queda como compatibilidad/diagnostico para etiquetar controles cercanos, pero
no limita ventana, aristas ni decide si hay optimizacion.

La politica de pesos de aristas temporales queda provisional. La variante a
probar debe usar soporte de KFs intermedios, cercania relativa a fiduciales y
confianza odometrica como señales internas, pero debe revisarse tras ver los
HTML/dumps y varias pruebas live/replay. No usar GT para pesos.

## Covisibilidad

La API conserva el argumento opcional para una decision futura. La integracion
actual usa `pose_graph_use_covisibility_edges=false`, pasa `nullptr` y produce
solo aristas `F1I_TEMPORAL_WINDOW`. `CovisibilityDatabase` sigue activa para
diagnostico y loops fuera de esta ruta.

## Logs

```text
[F1I-GRAPH-BUILDER-CONFIG]
[F1I-GRAPH-WINDOW]
[F1I-GRAPH-VERTEX-COVERAGE]
[F1I-GRAPH-VERTEX-SELECT]
[F1I-GRAPH-EDGES]
[F1I-GRAPH-PROPAGATION-PLAN]
[F1I-GRAPH-BUILD-SUMMARY]
```

La configuracion publica:

```text
vertex_policy=balanced_coverage_sample
vertex_selection_ratio=0.300
vertex_limit=none
edge_length_limit_m=none
mandatory_fiducial_vertices=2
covisibility_edges=false
fiducial_neighborhood_vertex_ratio=0.200
corner_3d_threshold_rad=0.524
```

## Evidencia disponible

Build de `orbslam3_multi`, `orbslam3_server` y `simulacion_dron`: correcto.

Live `prueba_28`: tras limitar las esquinas por cobertura, el caso corto
fiducial 2 -> 1 termina con `SIM-DONE success=true` y `SIM-EXIT-CODE 0`. En el
grafo de `drone_2` (`task_id=4`) la ventana queda `85 -> 26` vertices, con `5`
`corner_3d_vertex`, `15` `balanced_coverage_sample`, `2` vecinos target, `2`
vecinos previos y extremos obligatorios. El dry-run corrige el target
`10.051937 m -> 0`; offline sobre `f3l_graph_task_4.tsv` baja la ventana GT
`mean_before=3.96151 -> mean_after=0.653821`,
`max_before=10.0466 -> max_after=1.68853` y genera
`codex/archivos_auxiliares/html/f3l_offline_graph_task_4_prueba_28_3d.html`.
En esa ejecución, el apply live de `task_id=4` aún se rechazó por
`global_map_check_failed`. El 2026-07-23 esa causa se reclasificó a `3K` y se
corrigió en `GlobalMapBuilder` con publicación por cobertura de KFs corregidos;
la distribución de vértices de `3I` mejora, pero siguen pendientes los pesos,
aristas y propagación de grafos largos.

Replay `prueba_26`: con replay raw, tarea debug forzada y
`pose_graph_vertex_selection_ratio=0.30`, el builder genera
`window_keyframes=168`, `vertices=51`, `edges=50`, `propagation=117`, `5`
vecinos protegidos por cada fiducial y `17` vertices `corner_3d_vertex`. El
servidor guarda la ventana reproducible en
`codex/archivos_auxiliares/repeticiones/f3i_window_task_9000000001.tsv` y el
grafo en `codex/archivos_auxiliares/repeticiones/f3l_graph_task_9000000001.tsv`.
El HTML diagnostico activo es 3D navegable:
`codex/archivos_auxiliares/html/f3l_debug_animation_task_9000000001.html`.
Replay offline sobre el TSV genera tambien
`codex/archivos_auxiliares/html/f3l_offline_graph_task_9000000001_3d.html`.
El target se corrige `2.071121 m -> 0`, el coste baja
`596033.269207 -> 13310.040467` y el resultado queda `useful=true`.

Replay `prueba_24`: `window_keyframes=168`, `vertices=51`, `edges=50`,
`propagation=117`, extremos obligatorios presentes y cero aristas de
covisibilidad. El solver termina en 8 iteraciones, reduce coste
`371528.383852 -> 73.967050`, no mueve el hard fiducial y genera
`f3l_debug_animation_task_9000000001.html`.

Live `prueba_25`: los grafos reales cumplen `106 -> 32`, `151 -> 46` y
`86 -> 26` vertices, con los dos extremos obligatorios y propagacion del resto.
Los tres ejecutan solver, HTML y apply. La tarea `4` queda aceptada tras reducir
coste `20305.421992 -> 823.276936` y error fiducial `0.444541 m -> 0`, sin
mover el hard fiducial ni perder puntos publicados.

## Estado actual

Revisión `3I`: `CONSEGUIDA Y CERRADA` el 2026-07-28. Ya existen selección 3D de esquinas,
vecindad protegida por porcentaje, dump de ventana y HTML 3D. Desde
`prueba_28`, las esquinas ya no acaparan el cupo: quedan como candidatos con
bonus dentro de una cobertura equilibrada. La pérdida de MapPoints post-apply
fue corregida en `3K` con `prueba_31`; `prueba_41-44` y la validación RViz2
cierran la calidad requerida. Pesos, aristas, priors y `PropagationPlan`
pueden revisarse si una regresión futura aporta evidencia concreta.

La ruta `BuildForLoopTask` queda validada por
`test_loop_optimization_task`: arista relativa presente, candidato fijo,
ausencia de prior world absoluto y dry-run sin mutacion del pose store.
