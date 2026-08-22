# Subfase 3M - CovisibilityDatabase y DatabaseUpdateTask MEDIA

## Estado

```text
CONSEGUIDA; IMPLEMENTADA Y VALIDADA EL 2026-08-15
```

Este documento sustituye el contrato antiguo de 3M. La evidencia histórica se
conserva en `historial/por_subfase/historial_3M*.md`; 3T retiró la
implementación anterior después de validar su sustitución.

## Objetivo

Crear una base derivada de covisibilidad confirmada y activar la prioridad
MEDIA del unico flujo secundario. El flujo acordado es:

```text
RawMapDatabase commit
  -> ChangeSet con KFs materialmente afectados
  -> DatabaseUpdateTask MEDIA, si cambia covisibilidad
  -> preparar CovisibilityPatch fuera de lock
  -> commit breve, versionado e idempotente
  -> encolar LoopTask BAJA por cada KF elegible del ChangeSet
  -> terminar DatabaseUpdateTask
```

Si un KF necesita loop pero no cambia covisibilidad, el servidor encola su
`LoopTask` BAJA directamente. Un snapshot equivalente no crea tareas.

## Prioridades y scheduling

El unico `SecondaryWorker` utiliza la cola ya creada en 3H:

```text
MAXIMA -> FiducialOptimizationTask
MEDIA  -> DatabaseUpdateTask
BAJA   -> LoopTask
```

- una tarea activa nunca se interrumpe;
- al terminar se selecciona la prioridad mas alta pendiente;
- FIFO se conserva dentro de cada prioridad;
- la tarea MEDIA es una por `ChangeSet`, no una por KF;
- el payload contiene IDs y revisiones, nunca mapas o descriptores completos;
- la MEDIA encola una `LoopTask` independiente por KF y termina sin esperarla;
- el high watermark secundario aplica backpressure sin perder tareas.

Los nombres internos pueden conservar `Max`, `High` y `Normal`; su semantica
funcional es MAXIMA, MEDIA y BAJA respectivamente.

## CovisibilityDatabase

`CovisibilityDatabase` solo contiene relaciones confirmadas. No almacena
candidatos BoW, hipotesis ambiguas, rechazos ni componentes sin anchor.

### Identidad canonica

Una arista usa el par ordenado de IDs completos:

```text
RawKeyFrameId = (drone_id, map_epoch, local_kf_id)
CanonicalPair = min(kf_a, kf_b), max(kf_a, kf_b)
```

`A-B` y `B-A` son la misma relacion. Se rechazan self-edges, IDs invalidos,
revisiones incompatibles y medidas no finitas.

### Modelo minimo

```text
CovisibilityEdge
  kf_a
  kf_b
  source = ORBSLAM3_NATIVE | SERVER_LOOP_GEOMETRIC
  support
  information_weight
  relative_pose_measured
  relative_pose_current
  dependency_revisions
  created_arrival_id
  updated_revision
```

- `relative_pose_measured` es la medicion original y no se sobrescribe;
- `relative_pose_current` representa la relacion bajo las poses globales
  actuales y podra cambiar tras optimizaciones futuras;
- todas las aristas ORB positivas se conservan; los consumidores deciden que
  soporte consideran fuerte;
- una geometria rechazada nunca crea una arista.

### Fuentes

`ORBSLAM3_NATIVE` procede exclusivamente de connected KFs/weights raw. Sirve
para agrupar candidatos por region, construir ventanas y aportar topologia. No
autoriza por si sola una fusion ni confirma un loop server.

`SERVER_LOOP_GEOMETRIC` se comprometera en 3P despues de una fusion validada.
En 3M-3O solo se prepara la API; 3O no inserta esta fuente.

La fuente debe aparecer en logs y estadisticas para distinguir topologia ORB de
relaciones confirmadas por el servidor.

## CovisibilityPatch

La tarea MEDIA captura un input acotado por IDs y revisiones. Fuera de lock:

1. consulta solamente los KFs afectados en `RawMapDatabase`;
2. canoniza las relaciones ORB positivas;
3. calcula altas, actualizaciones e invalidaciones;
4. construye un `CovisibilityPatch` privado;
5. revalida las revisiones necesarias;
6. compromete el patch bajo un lock breve.

El commit debe ser:

- idempotente ante delta y snapshot equivalentes;
- versionado y diagnosticable;
- atomico respecto a los lectores;
- incapaz de modificar `RawMapDatabase`, poses o publicaciones;
- acotado a las relaciones afectadas, sin barrido global por llegada.

Los lectores reciben vistas inmutables y limitadas a pares, vecinos o ventanas
solicitadas. No se comparten referencias a contenedores live.

## Integracion con fases posteriores

- 3N consulta covisibilidad para agrupar candidatos BoW por region y evitar
  pares ya resueltos.
- 3O usa vecinos fuertes para construir `candidate_subcloud`.
- 3P insertara `SERVER_LOOP_GEOMETRIC` tras fusion confirmada.
- 3Q podra actualizar `relative_pose_current` tras un commit aceptado.
- `PoseGraphBuilder` tendra API de consulta, pero 3M no activa estas aristas en
  las optimizaciones fiduciales de 3H-3L.

## Cambios previstos

En `orbslam3_multi`:

- crear tipos de arista, snapshot y patch;
- crear `CovisibilityDatabase` con consultas acotadas;
- ampliar `SparseGlobalBackend` como fachada, sin trasladar el algoritmo al
  servidor;
- añadir tests unitarios de base y patches.

En `orbslam3_server`:

- sustituir el placeholder `DatabaseUpdate` por payload real;
- admitir una tarea MEDIA por `ChangeSet` relevante;
- ejecutar prepare/commit y encolar las `LoopTask` resultantes;
- conservar prioridad no expulsiva y backpressure existentes;
- emitir lifecycle y aristas reales al visualizador.

En `simulacion_dron`:

- añadir `CovisibilityDatabase` y `DatabaseUpdateTask` al grafo web;
- mantener encendida la ruta durante la tarea completa, no por cada subpaso;
- enviar solo IDs, revisiones, conteos, fuente y resultado.

## Archivos probables

```text
orbslam3_multi/include/orbslam3_multi/covisibility_database.hpp
orbslam3_multi/src/covisibility_database.cpp
orbslam3_multi/include/orbslam3_multi/database_update_task.hpp
orbslam3_multi/include/orbslam3_multi/sparse_global_backend.hpp
orbslam3_multi/src/sparse_global_backend.cpp
orbslam3_multi/test/test_covisibility_database.cpp
orbslam3_server/include/orbslam3_server/secondary_queue.hpp
orbslam3_server/src/global_map_server.cpp
orbslam3_server/test/test_secondary_queue.cpp
simulacion_dron/pipeline_flow_visualizer/*
```

No se modificaran `ORB_SLAM3`, `orbslam3_ros2` ni `orbslam3_msgs` salvo una
necesidad nueva, explicita y acordada.

## Pruebas

### Unitarias

1. canonicalizacion `A-B/B-A`;
2. rechazo de self-edge y medidas no finitas;
3. importacion incremental ORB solo para IDs afectados;
4. idempotencia delta/snapshot equivalente;
5. separacion de fuentes ORB/server;
6. inmutabilidad de `relative_pose_measured`;
7. actualizacion valida de `relative_pose_current`;
8. snapshot de lectura estable durante otro commit;
9. prioridad MAXIMA > MEDIA > BAJA y FIFO interno;
10. tarea activa no expulsada por otra prioridad;
11. una MEDIA por `ChangeSet` encola una BAJA por KF elegible;
12. un KF sin cambio de covisibilidad puede recibir BAJA directa.

### Integradas

- comprobar que la ingesta principal progresa durante una MEDIA ralentizada;
- comprobar que una MEDIA pendiente se ejecuta antes de loops pendientes;
- verificar ausencia de barridos globales y payloads pesados;
- comprobar lifecycle web continuo y sin pulsos engañosos.

## Criterios de cierre

3M sera `CONSEGUIDA` si compila, pasan las pruebas, la base contiene exactamente
las aristas ORB esperadas, no hay duplicados ni commits parciales, la prioridad
es correcta y raw/poses/publicaciones permanecen intactas.

Sera `PARCIAL` si la base y la cola funcionan pero faltan evidencia integrada o
visual. Sera `NO CONSEGUIDA` si bloquea el flujo principal, pierde tareas,
modifica raw, inserta candidatos no confirmados o activa covisibilidad en el
grafo fiducial antes de lo acordado.

## Evidencia de cierre

- build final de `orbslam3_multi`, `orbslam3_server` y `simulacion_dron`: 3/3;
- regresion final: 53/53 tests C++ y 9/9 web;
- replay 153: cola secundaria drenada, `pending=0`, `hard_failed=0`;
- live 154: updates MEDIA comprometidos y loops BAJOS derivados sin bloquear
  371 entradas principales; cierre secundario limpio.

## Politica de adaptacion

La arquitectura anterior es el objetivo principal, no una afirmacion de que
todos los detalles rindan bien al primer intento. Podran ajustarse, segun tests,
replay y logs, el formato compacto del patch, el tamaño de las vistas y la
instrumentacion. No podran cambiarse sin nuevo acuerdo los invariantes de un
worker, prioridades, raw inmutable, commit atomico, ausencia de publicacion
secundaria y no uso de covisibilidad en 3H-3L.
