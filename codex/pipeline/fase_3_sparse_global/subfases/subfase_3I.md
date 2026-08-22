# Subfase 3I - Grafo temporal mono-submapa

## Estado

```text
Preparacion: CERRADA
Acuerdo cerrado: si
Autorizacion funcional: PENDIENTE PARA INTEGRACION POST-LOOP
Ejecucion: BASE MONO-SUBMAPA VALIDADA; EXCLUSION POST-LOOP PENDIENTE DE INTEGRAR
Dudas abiertas: ninguna
```

Este es el contrato vigente y ejecutado de 3I dentro de la entrega conjunta
3H-3L. El código y la documentación anteriores fueron referencia de un algoritmo
que funcionaba bien, no una arquitectura que debiera copiarse completa.

## Sucesion acordada en 3Q

3Q generalizara este mismo `PoseGraphBuilder`: la ventana mono-submapa temporal
seguira cubierta por sus tests, pero fiducial y loop podran construir el
subgrafo minimo multi-submapa con hard, dependencias soft, loops/fusiones y
covisibilidad confirmada. El 30 % pasa a densidad base ampliable por controles
obligatorios; no se crea otro builder.

## Objetivo

Construir un problema de pose graph privado, acotado y determinista para una
`FiducialOptimizationTask` que sobrevivio a la revalidacion de 3H.

En esta subfase la ventana contiene exclusivamente KFs del mismo submapa:

```text
last_accepted_control_kf ... target_fiducial_kf
```

No se usa covisibilidad. La ampliacion a varios submapas se realizara en fases
posteriores sin cambiar la identidad de tarea ni el contrato del solver.

Un primer fiducial observado por un submapa cuya unica autoridad era un anchor
loop no crea esta ventana: 3H/3O lo reanclan directamente como primer anchor
hard. 3I vuelve a intervenir en los fiduciales posteriores. Cuando se habilite
la optimizacion covisible multi-submapa, esta excepcion transitoria se
reemplazara por una ventana conectada entre autoridades fiduciales.

## Limites de la ventana

El inicio es `last_accepted_control_kf` del submapa. Puede ser:

- el KF del primer anchor;
- el primer KF coherente de una nueva visita fiducial;
- el target de una optimizacion fiducial ya aceptada.

Los KFs coherentes posteriores de la misma visita no desplazan el inicio. Al
entrar en una nueva visita, su primer KF coherente o posteriormente optimizado
se convierte en el nuevo control.

El final es el KF fiducial de la tarea. Debe pertenecer al mismo submapa y ser
posterior al control en el orden raw estable. Si no hay intervalo valido, la
tarea termina con un estado estructurado y no invoca el solver.

No existe un maximo absoluto de numero de KFs, distancia recorrida o separacion
temporal que pueda truncar silenciosamente la ventana. La reduccion de coste se
hace seleccionando controles, no eliminando el contrato entre extremos.

Los KFs intermedios que raw o `GlobalPoseStore` marquen inactivos se omiten sin
reactivarlos. Un hard control aceptado puede conservarse como frontera fija
aunque un snapshot posterior lo marque inactivo; el target debe seguir activo.

## Entrada inmutable

`PoseGraphBuilder` captura un `PoseGraphInput` versionado y autosuficiente:

```text
task_id
submap_id
control_keyframe_id
target_keyframe_id
target_world_T_kf
ordered_raw_keyframes[]
current_world_poses[]
hard/anchor metadata
raw_revision
pose_revision
fiducial_visit_id
```

La captura bajo lock debe ser breve. La seleccion de controles, las aristas y
el plan de propagacion se calculan fuera de locks sobre esta copia privada.

## Seleccion de controles

Se conserva el comportamiento final validado en la baseline anterior:

```text
control_ratio = 0.30
control_count = max(2, ceil(control_ratio * window_kf_count))
```

Los dos extremos son obligatorios. Los restantes se distribuyen con cobertura
equilibrada usando:

- distancia 3D acumulada a lo largo de la trayectoria;
- densidad temporal para evitar huecos grandes en zonas de poco movimiento;
- cambios de direccion/rotacion SE(3) como preferencia, no como requisito que
  pueda romper la cobertura;
- vecindades protegidas de los extremos.

Las vecindades protegidas usan inicialmente el reparto validado previamente:

```text
endpoint_neighborhood_ratio = 0.20
```

Ese porcentaje se reparte entre ambos extremos y se adapta de forma
determinista en ventanas pequenas. No se duplican controles y el orden final es
estrictamente temporal.

## Aristas del grafo

3I crea un grafo temporal. Cada arista conserva la transformacion relativa
SE(3) completa derivada de poses raw ORB-SLAM3:

```text
T_i_j_raw = inverse(T_raw_i) * T_raw_j
```

Reglas:

- el primer control es hard/fijo y conserva su pose world aceptada;
- el ultimo control recibe el objetivo absoluto `target_world_T_kf`;
- los controles intermedios se conectan respetando el orden temporal;
- las vecindades de ambos extremos pueden generar restricciones rigidas
  inducidas segun el algoritmo validado;
- no se añaden aristas de covisibilidad, loop ni GT entre KFs;
- no se convierten traslaciones 3D en aproximaciones planas.

La observacion fiducial absoluta del target es distinta de una arista loop y
debe conservar esa semantica.

## `PropagationPlan`

Los KFs no seleccionados como controles no desaparecen. El builder produce un
plan determinista que asigna cada KF de la ventana a sus controles temporales
vecinos y conserva los datos raw necesarios para reconstruir su pose.

El plan debe cubrir:

- todos los KFs originales de la ventana;
- el orden temporal;
- los intervalos entre controles;
- las vecindades inducidas de los extremos;
- los datos que 3K necesitara para incorporar KFs que aparezcan tarde dentro
  de la misma ventana.

La interpolacion/propagacion efectiva no modifica estado en 3I. Solo se define
el plan que consumiran 3J-3K.

## Salida

La salida `PoseGraphProblem` es inmutable y contiene, como minimo:

```text
task_id
control vertices and initial poses
fixed/hard vertices
temporal SE(3) edges
absolute target constraint
PropagationPlan
consumed revisions
window identity and diagnostics
```

No contiene referencias mutables a `RawMapDatabase` o `GlobalPoseStore`.

## Cambios a realizar

- adaptar o crear `PoseGraphInput`, `PoseGraphProblem` y `PropagationPlan` con
  ownership explicito y revisiones;
- implementar en `PoseGraphBuilder` la captura mono-submapa desde el ultimo
  control aceptado hasta el target;
- portar selectivamente de la baseline el 30 % de controles, cobertura por camino
  3D/tiempo, preferencias de esquinas y vecindades de extremos;
- generar aristas temporales SE(3) completas y restricciones absolutas de los
  extremos;
- producir diagnosticos ligeros de recuentos, cobertura y revisiones;
- conectar la salida al mismo `FiducialOptimizationTask` activo de 3H.

No se copiara el antiguo mutex global, snapshots completos del servidor,
publicacion secundaria ni ningun solver experimental.

## Invariantes

- `RawMapDatabase` no se modifica;
- no se mantiene ningun lock live durante la construccion costosa;
- el primer control y cualquier hard previo permanecen fijos;
- target, controles, aristas y plan pertenecen a la misma revision capturada;
- todo KF de la ventana queda cubierto exactamente una vez por el plan;
- el resultado es determinista para una entrada y parametros iguales.

## Contrato visual

Cuando la tarea supera la revalidacion debe activarse:

```text
SecondaryWorker --> PoseGraphBuilder
PoseGraphBuilder --graph_ready--> OptimizationManager
```

La telemetria expone `task_id`, submapa, extremos, numero de KFs, controles,
aristas, revision y duracion. No envia matrices ni el grafo completo.

## Pruebas

- ventanas minima, corta, larga, estacionaria y con giro fuerte;
- exactamente dos extremos obligatorios y numero de controles esperado;
- cobertura equilibrada por camino 3D y tiempo;
- ausencia de duplicados y orden temporal estricto;
- transformaciones relativas SE(3) correctas;
- primer control hard y target absoluto correctos;
- cobertura completa del `PropagationPlan`;
- misma entrada produce el mismo grafo;
- modificaciones posteriores del store no alteran el problema privado;
- ventana siempre mono-submapa y sin aristas de covisibilidad;
- fallo estructurado ante control/target ausente o revision incoherente.

## Criterios de cierre

3I queda conseguida cuando todas las tareas no `STALE` de 3H producen un grafo
privado correcto o un fallo estructurado, los tests sinteticos cubren las
reglas anteriores y replay/live muestran una ventana desde el ultimo control
aceptado hasta el nuevo fiducial sin bloquear el flujo principal.

## Fuera de alcance

- covisibilidad y ventanas multi-submapa;
- seleccion de loops o fusion de landmarks;
- mutacion de poses;
- aceptacion o commit de una solucion;
- publicacion ROS o visualizacion HTML del grafo como requisito funcional.
