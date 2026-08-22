# Pipeline Fase 3 - Mapa sparse global multi-dron

Resumen de entrada:

```text
codex/pipeline/fase_3_sparse_global/pipeline_fase_3_RESUMEN.md
```

## Estado

```text
Fase 3: CONSEGUIDA
3B-3Q: CONSEGUIDAS
3R: CONSEGUIDA; scoring
3S: CONSEGUIDA; observabilidad y debug
3T: CONSEGUIDA; limpieza y handoff
Fase actual: Fase 2 - separacion servidor/dron/simulacion
```

La planificacion activa comprende `subfase_3A.md` a `subfase_3T.md`. Los
archivos de subfase son contratos ejecutables; la evidencia cronologica vive
en `historial/por_subfase/`.

## Objetivo

Obtener un mapa sparse global multi-dron coherente en `world`, con submapas
identificados por `(drone_id, map_epoch)`, anchors fiduciales, tracks fusionados,
loops geometricamente validados, optimizacion segura y publicacion continua.
El resultado debe preparar la estimacion de pose global y la nube densa sin
usar ground truth para construir el mapa o estimar la pose final.

## Principio de ejecucion

La arquitectura separa dos recorridos con responsabilidades distintas.

### Flujo principal siempre operativo

```text
OrbMap delta/full snapshot
  -> callback ROS ligero -> PrimaryQueue
  -> PrimaryWorker -> RawMapDatabase::Commit
  -> ChangeSet
  -> GlobalPoseStore / primer anchor
  -> enqueue DatabaseUpdateTask HIGH / LoopTask NORMAL si anchored
  -> dirty sets -> GlobalMapBuilder incremental
  -> /global_keyframes + /global_sparse_cloud
```

Por cada llegada:

1. el callback del servidor convierte el mensaje, asigna `arrival_id`, encola
   un DTO inmutable y retorna;
2. el `PrimaryWorker` hace el commit raw breve;
3. `RawMapDatabase` devuelve IDs y revisiones realmente modificados;
4. el servidor distribuye solo los cambios necesarios;
5. los KFs de un submapa anclado reciben pose world inmediatamente;
6. se encola `DatabaseUpdateTask` HIGH para covisibilidad/score derivados;
7. si el KF anclado habilita loops, se encola una `LoopTask` por KF;
8. el builder drena dirty sets y publica antes de terminar la entrada principal.

3C crea la `PrimaryQueue` FIFO y el unico `PrimaryWorker`; 3D ya extiende esa
tarea condicionalmente hasta poses y 3E hasta el primer anchor fiducial. 3F
continuará el mismo ciclo de vida hasta publicación. 3G incorpora `SnapshotInput` a
la misma cola. Ninguna de esas subfases crea una ruta principal paralela.

El primer fiducial de un submapa crea su anchor mediante
`FiducialAnchorManager` y `GlobalPoseStore`. Una revisita valida puede encolar
una optimizacion fiducial, pero no ejecuta el solver en el callback.

El flujo principal no espera trabajo geometrico, fusion, optimizacion ni
telemetria. Puede perder rendimiento por compartir CPU, pero no queda detenido
por una tarea secundaria.

### Flujo secundario serial y priorizado

Existe un `SecondaryWorker` persistente con una cola y una tarea activa como
máximo. `PrimaryWorker` y `SecondaryWorker` pueden progresar simultáneamente.

| Tipo | Prioridad | Alcance |
|---|---:|---|
| `FiducialOptimizationTask` | maxima | ventana fiducial, grafo, solver y commit de poses |
| `DatabaseUpdateTask` | alta | covisibilidad, score e índices derivados del `ChangeSet` |
| `LoopTask` | normal | BoW, filtros, verificacion, decision y fusion y/o optimizacion |

Reglas:

- la tarea activa no se interrumpe;
- al quedar libre el worker, se elige el fiducial pendiente mas antiguo;
- si no hay fiduciales, se elige el update HIGH más antiguo y después el loop;
- fusion y optimizacion por loop son desenlaces de la misma `LoopTask`;
- una tarea calcula con snapshots privados y versionados;
- los mutex solo protegen cambios breves de cola o commits;
- un resultado stale se descarta o reprograma de forma acotada;
- la tarea termina tras el commit o rechazo y no espera a RViz2.

## Propiedad de datos

| Dato | Autoridad | Escritores |
|---|---|---|
| KFs, MapPoints, descriptores, BoW y poses locales raw | `RawMapDatabase` | solo ingesta/reconciliacion |
| anchors y poses world | `GlobalPoseStore` | anclaje y commits de optimizacion |
| aristas ORB/geometricas y poses relativas | `CovisibilityDatabase` | flujo principal y commit de `LoopTask` |
| tracks fusionados | `FusedLandmarkManager` | rama de fusion de `LoopTask` |
| scores raw/fused | `LandmarkScoreManager` | eventos de ingesta y commits secundarios |
| cola y ciclo principal | `PrimaryQueue`/`PrimaryWorker` desde 3C | servidor/orquestador |
| cola y ciclo secundario | `SecondaryTaskQueue`/`SecondaryWorker` desde 3K | servidor/orquestador |
| revision publicable y caches | `GlobalMapBuilder` | solo flujo principal |

`RawMapDatabase` nunca se corrige como resultado de una fusion u optimizacion.
Las clases no se llaman entre si para ocultar escrituras: el servidor coordina
las transiciones y cada API devuelve un resultado estructurado.

## Poses y fiduciales

`GlobalPoseStore` conserva:

- anchor inicial por submapa;
- poses world aceptadas;
- lotes optimizados y propagados;
- ultimo control aceptado para KFs futuros;
- revisiones y rollback de cada commit.

Cuando llega un KF nuevo a un submapa anclado:

```text
T_world_new = T_world_control * inverse(T_raw_control) * T_raw_new
```

La operacion ocurre en el flujo principal. No existe una cola post-optimizacion
separada. Si un commit cambia el control, `GlobalPoseStore` recalcula de forma
coherente la cola derivada que ya posee.

## LoopTask

Una `LoopTask` mantiene un unico `task_id` y contexto desde el principio hasta
el commit:

```text
captura privada
  -> LoopDetector (BoW)
  -> filtros por covisibilidad/estado previo/pose
  -> SubcloudLoopVerifier cuando haga falta
  -> LoopDecisionManager
  -> FusionManager/FusedLandmarkManager
     o PoseGraphBuilder + OptimizationManager
     o optimizacion aceptada seguida de fusion
  -> validacion de revisiones
  -> commit atomico de bases derivadas
```

`3P` posee solo la rama de fusion. `3Q` posee la rama de optimizacion por loop y
reutiliza las capacidades de grafo/solver de `3I-3L`. No se crea una segunda
tarea al decidir el desenlace.

## Publicacion

`GlobalMapBuilder` mantiene caches e índices incrementales. Recibe dirty sets de
raw, poses, score y fusión, recalcula solo IDs/dependencias afectados y devuelve
mensajes completos coherentes. Solo se ejecuta dentro de una entrada principal.
Los commits secundarios no publican ni despiertan al principal; sus cambios se
acumulan hasta la siguiente llegada.

## Visualizador JavaScript incremental

`3B` crea la infraestructura web de diagnostico basada en Cytoscape.js:

- cada clase o componente es un nodo;
- cada transferencia real emite un pulso de arista;
- hover de nodo explica responsabilidad;
- hover de arista detalla los datos y metadatos transferidos;
- la topologia/textos viven en `graph_definition.js`;
- la conexión y render live por frame viven en `app.js`;
- la telemetria usa una cola acotada y no bloqueante;
- la simulacion puede abrir la pagina, pero el navegador no controla ROS.

En 3C el grafo muestra wrappers, servidor, cola, worker, raw y mission gate con
cinco aristas/eventos reales. Cada subfase posterior añade solo sus elementos y transferencias según
`CONTRATO_VISUAL_INCREMENTAL.md`. La auditoria visual absorbida verifico el
conjunto, el comportamiento live, la degradacion inocua y la correspondencia
final con RViz2.

RViz2 sigue siendo la vista espacial de KFs y nube. El diagrama muestra flujo y
actividad, no duplica la nube 3D.

## Backpressure incremental

3C crea `/global_mapping/backpressure_active` reliable/transient-local con
histeresis por pendientes principales (high=8, low=2). El goal activo termina
y `scenario_runner_node` retiene solo el siguiente lote hasta la liberacion.
3K conserva esta ruta y expone metricas secundarias. La politica final integra
ambas colas, edad, drenaje, optimizaciones activas, dwell, capacidad y
no-progreso.

## Subfases y estado

| Subfase | Responsabilidad | Estado operativo |
|---|---|---|
| 3A | baseline | conseguida; no rehacer |
| 3B | congelacion, servidor vacio y grafo base | conseguida |
| 3C | raw, journal/replay, flujo principal y backpressure basico | conseguida |
| 3D | poses globales/autoridad | conseguida |
| 3E | GT y primer anchor | conseguida |
| 3F | builder incremental/publicacion | conseguida |
| 3G | full snapshots/diff | conseguida |
| 3H | revisita fiducial MAX | conseguida |
| 3I-3J | grafo y propuesta privada | conseguidas |
| 3K | cola/worker secundarios, prioridad y commit | conseguida |
| 3L | validacion/no regresion | conseguida |
| 3M | covisibilidad MEDIA | conseguida |
| 3N-3O | ledger, BoW y verificación | conseguidas |
| 3P | fusión | conseguida |
| 3Q | optimizacion por loop | conseguida; mejora futura documentada |
| 3R | score | conseguida |
| 3S | observabilidad y perfil debug | conseguida |
| 3T | limpieza y handoff | conseguida |

Las auditorias provisionales de arquitectura, visualizador, regresion y
rendimiento se cerraron sobre capacidades ya implantadas. Sus contratos
independientes fueron retirados en 3T y sus historiales siguen disponibles en
`historial/absorbidas/`.

## Siguientes pasos

1. Usar `RESULTADO_FINAL_FASE_3.md` como handoff para Fase 2.
2. Consultar el historial 3Q si reaparecen optimizaciones incorrectas y evaluar
   entonces la evidencia adaptativa documentada.
3. Ejecutar tests deterministas antes de la simulación larga.
4. Conservar las regresiones absorbidas como referencia ante fallos futuros.
5. Mantener los tests que protegen la sincronizacion YAML durante Fase 2.

## Verificacion final ejecutada

```bash
./codex/herramientas/build_selected_packages.sh orbslam3_multi orbslam3_server simulacion_dron
```

Resultado: build 3/3, CTest 9/9 + 10/10 + 8/8 y prueba 196
`success=true`. Con el perfil 3S completamente false, el servidor proceso el
escenario, no aparecieron marcadores `[F3*]` y no se iniciaron RViz2, bridge ni
navegador.

Los logs completos se reducen antes de leerse. Cada ejecucion mantiene su
entrada historica y conclusion propia.

## Criterio de fin de Fase 3

- el flujo principal recibe y publica durante trabajo secundario lento;
- anchors y KFs futuros mantienen poses world coherentes;
- solo existe una tarea secundaria activa y se respeta la prioridad acordada;
- loops buenos fusionan y loops con error alto pueden optimizar;
- las bases derivadas cambian mediante commits breves y consistentes;
- raw permanece intacta;
- RViz2 muestra KFs y nube desde una revision coherente;
- el diagrama refleja eventos reales y puede fallar sin afectar al mapa;
- scoring, rollback, limites y shutdown estan validados;
- se eliminan rutas duplicadas solo tras probar sus sustitutas;
- la documentacion coincide con el codigo y conserva la evidencia historica.

## Evidencia retirada

Los contratos `12R-*`, `13`, `14` y `15` fueron retirados en 3T. Los
aprendizajes verificables permanecen en `historial/por_subfase/` y en Git, pero
ya no forman parte de la planificación ni del árbol documental vigente.
