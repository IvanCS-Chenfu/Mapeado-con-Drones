# Subfase 3O - Implementacion prevista

## Principio de integracion

3O es una etapa de la `LoopTask` BAJA. No crea otra tarea geometrica, otro
worker ni un callback. La tarea conserva `task_id`, query y revisiones desde 3N
hasta el resultado y, si procede, el commit de anchor.

El servidor orquesta. Las operaciones de dominio pertenecen a
`orbslam3_multi` y se exponen mediante `SparseGlobalBackend`.

## Tipos principales

### Geometria

```text
SubcloudPoint
Subcloud
CandidateRegionSnapshot
LoopGeometryInput
LoopVerificationResult
LoopRegionOutcome
LoopTaskDecision
```

`SubcloudPoint` conserva ID raw, posicion local/world opcional, descriptor,
observacion y revisiones. `LoopVerificationResult` incluye transformacion,
inliers, cobertura, residual, confianza, errores y dependencias.

### Hipotesis y anchors

```text
LoopHypothesisKey
LoopHypothesisObservation
LoopHypothesis
LoopHypothesisStore
LoopAnchorConstraint
LoopAnchorConstraintStore
LoopAnchorCommitInput
LoopAnchorCommitResult
```

`LoopHypothesisStore` agrupa observaciones por par de regiones/submapas y
transformacion compatible. No sustituye a la cache negativa ni a
`CovisibilityDatabase`.

`LoopAnchorConstraintStore` modela el grafo relativo de submapas y sus
componentes provisionales/activos.

## Flujo del SecondaryWorker

```text
dequeue LoopTask
  -> ledger ACTIVE
  -> capturar query/revisiones acotadas
  -> actualizar LoopBoWIndex
  -> FastOverlapVerifier, si query world
  -> si no hay fusion rapida: LoopDetector
  -> por cada region: SubcloudLoopVerifier
  -> LoopDecisionManager sobre el conjunto
  -> posible LoopAnchorManager
  -> actualizar caches/ledger
  -> done
```

Si durante la ejecucion cambia una revision material, el ledger registra como
maximo un `DIRTY_AFTER_RUN`. No se reinicia RANSAC dentro de un bucle infinito.

`LoopTaskRevision` separa scheduling de seguridad. La igualdad de cola usa
`appearance_revision`, `geometry_revision` semantica y `anchor_revision`.
`validation_revision` incluye pose, asociaciones, posiciones, descriptores y
covisibilidad exactos; se compara al dequeue y antes del anchor commit. La
huella semantica clasifica asociaciones como insuficientes, suficientes o
densa respecto a `min_query_mappoints`, y no cambia por cada refinamiento de
MapPoints.

## Captura acotada

Las APIs de raw, poses y covisibilidad deben permitir pedir:

- un KF concreto;
- sus MapPoints observados;
- una lista acotada de KFs/MPs;
- vecinos covisibles limitados;
- poses world de IDs concretos;
- revisiones sin copiar contenedores completos.

La captura se divide en prepare/compute/commit:

1. capturar IDs y datos pequeños bajo locks breves;
2. construir subnubes y calcular fuera de locks;
3. revalidar dependencias antes de registrar hipotesis o aplicar anchor.

## FastOverlapVerifier

Se ejecuta solo con query world valida:

1. construir `query_subcloud`;
2. consultar un indice espacial global acotado o API equivalente del builder;
3. filtrar identidad raw y tracks ya fusionados cuando exista 3P;
4. matching espacial + Hamming con unicidad;
5. medir cobertura 2D/3D y distribucion;
6. estimar transformacion/residual pequeño;
7. emitir pares distintos de MapPoints.

Si cumple los thresholds produce `FUSION_CANDIDATE`, conserva todos los pares
compatibles y omite la busqueda BoW. No modifica `GlobalMapBuilder` ni espera a
3P.

## SubcloudLoopVerifier

### Query subcloud

Usa exclusivamente MapPoints observados por query. Descarta bad, inexistentes,
no finitos y descriptores invalidos. Deduplica por ID raw.

### Candidate window

Parte del seed de la region y añade, con limites:

- miembros agrupados por 3N;
- covisibles ORB fuertes;
- parent/children disponibles;
- vecinos temporales;
- vecinos espaciales si existe world.

La nube final deduplica puntos y registra que KFs aportaron soporte.

### Matching y reduccion

1. matching Hamming inicial;
2. ratio test y cross-check configurable;
3. correspondencias uno a uno;
4. percentiles robustos iniciales 10-90 sobre la zona de matches;
5. fallback a la nube no reducida si hay pocos matches o degeneracion;
6. matching refinado sobre la region reducida.

Los valores parten de `legacy2`; no se copian ciegamente si los tipos o marcos
del runtime nuevo difieren.

### RANSAC

El RANSAC 3D-3D debe ser determinista para el mismo input/revisiones. Valida:

- minimo de correspondencias e inliers;
- ratio de inliers;
- residual medio y maximo;
- rango/condicion de la nube;
- cobertura distribuida;
- transformacion SE3 finita;
- ausencia de escala libre o reflexion;
- coherencia de gravedad/rotacion segun configuracion.

La medida se expresa con marcos documentados. Para relaciones entre submapas se
deriva `parent_local_T_child_local`; con ambos world se calcula la diferencia
entre relacion medida y relacion actual.

## LoopDecisionManager

Recibe todos los resultados de las regiones seleccionadas. No decide al primer
accept.

### Fusion dominante

Si existe al menos una fusion valida coherente con la pose actual:

- unir y deduplicar pares compatibles;
- registrar `FUSION_CANDIDATE` para 3P;
- no crear hipotesis de optimizacion de esa tarea;
- marcar candidatos de error alto incompatibles con la ubicacion confirmada
  como conflicto para esas revisiones, no como loop positivo.

### Hipotesis de anchor/optimizacion

Si no hay fusion:

- agrupar transformaciones compatibles;
- añadir una observacion por query independiente;
- comprobar baseline local `>= 0.20 m` o yaw `>= 5 grados`;
- exigir soporte de dos queries;
- si hay competidora incompatible, exigir ventaja de dos observaciones;
- devolver `DEFERRED` mientras no se alcance soporte/margen.

Ambos valores y thresholds de compatibilidad son parametros ROS iniciales.

### Casos de autoridad

```text
query world, candidate no world -> anclar candidate
query no world, candidate world -> anclar query
ambos world                   -> fusion o evidencia de optimizacion
ninguno world                 -> constraint relativa provisional/activa
```

La orientacion de la medida se invierte de forma explicita cuando sea necesario;
no se intercambian IDs sin transformar la SE3.

## Componentes no anclados

`LoopAnchorConstraintStore` mantiene un grafo por `RawSubmapId`.

Una primera observacion crea `PROVISIONAL`; dos queries independientes y
coherentes permiten `ACTIVE`. Una hipotesis incompatible mantiene el conjunto
`AMBIGUOUS` hasta alcanzar el margen acordado. Stale invalida solo evidencia que
dependa de la revision cambiada.

Cuando un nodo obtiene world:

1. obtener el componente conectado por edges activos;
2. excluir nodos ya anclados por otra autoridad world;
3. elegir un spanning tree determinista de mayor confianza;
4. usar edges adicionales para validar ciclos;
5. abortar si un ciclo supera los thresholds;
6. calcular anchors candidatos para todos los nodos;
7. preparar un unico batch de commit.

No se aplica parcialmente una cascada.

## Commit atomico de anchor

El batch incluye:

```text
component_revision
constraint revisions
raw submap revisions
pose store revision esperada
world_T_local por submapa
world_T_kf para todos los KFs actuales
dependencias parent/child
anchor_revision nueva
```

Antes del commit se reconsulta cada submapa en raw para incluir KFs llegados
durante BoW/RANSAC. El lock live se mantiene solo durante validacion final e
intercambio del batch.

El resultado es todo o nada:

- si aplica, devuelve KFs creados/movidos y submapas anclados;
- si una revision cambio, termina `STALE` sin escritura parcial;
- si una invariante falla, termina con error explicito;
- nunca modifica poses raw.

Tras aplicar:

- notificar IDs dirty a `GlobalMapBuilder`;
- no recalcular ni publicar desde el worker;
- incrementar `anchor_revision`;
- encolar una `LoopTask` coalescida por KF ahora colocado;
- el siguiente `PrimaryInput` transforma MPs y publica RViz2.

El anchor commit no activa `optimization_active`. La cola puede activar
backpressure por watermark.

## Dependencia y fiducial posterior

Mientras no exista fiducial hard en el hijo, un cambio del KF de apoyo del
padre propaga al componente hijo blando:

1. capturar `T_old` y `T_new` del KF de apoyo;
2. calcular `delta = T_new * inverse(T_old)`;
3. aplicar `delta` a todos los KFs de los descendientes blandos;
4. no modificar sus poses relativas internas;
5. revalidar revisiones y comprometer todo en un batch atomico;
6. notificar como dirty los KFs movidos para que el siguiente flujo principal
   recalcule sus MapPoints.

Esto es propagacion de anchor, no optimizacion con covisibilidad. Ningun
submapa con autoridad hard se mueve por esta via.

Al aceptar el primer fiducial del hijo:

1. obtener el GT fiducial y la pose local del KF observador;
2. calcular directamente el nuevo `world_T_local` absoluto;
3. reanclar rigidamente todos los KFs actuales del submapa como en first anchor;
4. marcar el KF fiducial hard aunque el error anterior fuese bajo;
5. convertirlo en `last_accepted_control_kf`;
6. cortar la propagacion rigida desde el padre;
7. conservar la constraint loop como evidencia inactiva para la futura
   optimizacion covisible;
8. dejar los fiduciales posteriores al flujo ordinario 3H-3L.

El reanchor directo y la propagacion rigida son politicas transitorias. Se
reemplazaran cuando el grafo de optimizacion incorpore covisibilidad y pueda
optimizar conjuntamente los submapas conectados entre fiduciales.

## Telemetria

Marcadores nuevos orientativos:

```text
[F3O-FAST-OVERLAP]
[F3O-SUBCLOUD-QUERY]
[F3O-CANDIDATE-WINDOW]
[F3O-MATCH]
[F3O-RANSAC]
[F3O-REGION-RESULT]
[F3O-DECISION-SET]
[F3O-HYPOTHESIS]
[F3O-AMBIGUOUS]
[F3O-ANCHOR-CONSTRAINT]
[F3O-COMPONENT-VALIDATE]
[F3O-ANCHOR-COMMIT]
[F3O-POSE-DIRTY]
```

Cada uno incluye `task_id`, IDs, revisiones, conteos y motivo. El visualizador
recibe metadatos, nunca nubes/descriptores. Una etapa permanece iluminada hasta
su final real.

## Archivos probables

```text
orbslam3_multi/include/orbslam3_multi/subcloud.hpp
orbslam3_multi/include/orbslam3_multi/loop_verification_result.hpp
orbslam3_multi/include/orbslam3_multi/subcloud_loop_verifier.hpp
orbslam3_multi/src/subcloud_loop_verifier.cpp
orbslam3_multi/include/orbslam3_multi/loop_hypothesis_store.hpp
orbslam3_multi/src/loop_hypothesis_store.cpp
orbslam3_multi/include/orbslam3_multi/loop_anchor_constraint_store.hpp
orbslam3_multi/src/loop_anchor_constraint_store.cpp
orbslam3_multi/include/orbslam3_multi/loop_anchor_manager.hpp
orbslam3_multi/src/loop_anchor_manager.cpp
orbslam3_multi/include/orbslam3_multi/global_pose_store.hpp
orbslam3_multi/src/global_pose_store.cpp
orbslam3_multi/include/orbslam3_multi/sparse_global_backend.hpp
orbslam3_multi/src/sparse_global_backend.cpp
orbslam3_server/src/global_map_server.cpp
simulacion_dron/pipeline_flow_visualizer/*
```

## Prohibiciones

- no copiar bases completas por comodidad;
- no ejecutar matching/RANSAC bajo mutex live;
- no aceptar BoW sin geometria;
- no fusionar por simple cercania o covisibilidad ORB;
- no aplicar una optimizacion en 3O;
- no insertar covisibilidad server antes de 3P;
- no mover un hard fiducial por propagacion de padre;
- no publicar o esperar confirmacion visual;
- no usar GT en ninguna decision.
