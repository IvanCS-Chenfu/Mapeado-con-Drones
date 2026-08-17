# Subfase 3K - Commit atomico y propagacion de poses

## Estado

```text
Preparacion: CERRADA
Acuerdo cerrado: si
Autorizacion funcional: PENDIENTE PARA EXTENSION POST-LOOP
Ejecucion: COMMIT BASE VALIDADO; PROPAGACION DE HIJOS SOFT PENDIENTE
Dudas abiertas: ninguna
```

3K no crea la cola ni el worker secundario: ambos nacen en 3H. Esta subfase
añade al mismo flujo activo la preparacion y aplicacion atomica del cambio de
poses validado por 3L.

## Sucesion acordada en 3Q

El commit base se generalizara a `PosePatch` multi-submapa con continuidad por
cada tail, KFs tardios, dependencias soft y frontera hard. Poses, constraint
current y fusion/score opcionales se preparan fuera de locks y se comprometen
de forma coordinada con rollback acotado. El secundario sigue sin publicar.

## Objetivo

Convertir una `OptimizationProposal` valida en un unico patch coherente para
`GlobalPoseStore`. El patch debe cubrir:

1. controles optimizados;
2. KFs no control de la ventana original;
3. KFs que llegaron tarde pero pertenecen a esa ventana;
4. todos los KFs posteriores al target que llegaron mientras se calculaba;
5. el nuevo `last_accepted_control_kf` y sus metadatos hard/anchor cuando la
   validacion final entre en umbral;
6. los componentes hijos que sigan anclados de forma blanda a cualquier KF de
   apoyo movido por el patch;
7. la notificacion de KFs movidos a `GlobalMapBuilder`.

Estas partes no son tareas ni procesos separados. Se calculan y comprometen
dentro del mismo `FiducialOptimizationTask` y del mismo `task_id`.

## Propiedad del flujo

```text
SecondaryWorker
  -> PoseGraphBuilder
  -> OptimizationManager
  -> Validation
  -> GlobalPoseStore atomic commit
  -> GlobalMapBuilder pose_dirty notification
```

El worker mantiene la tarea activa hasta que se produce uno de estos finales:

- commit completo y error dentro de umbral;
- commit parcial valido seguido de otra pasada interna;
- `STALE` antes de calcular;
- fallo duro explicito sin commit;
- cierre ordenado.

El worker no se libera entre las etapas ni entre pasadas de refinamiento de la
misma tarea. Por tanto, ninguna tarea MAX/HIGH/NORMAL se intercala.

## Construccion del patch

### Controles optimizados

Se toman de la propuesta validada de 3J. El control inicial y cualquier hard
preexistente conservan exactamente su pose. El target adquiere la pose corregida
y solo pasa a ser el nuevo control del submapa cuando 3L emite `ACCEPT_FULL`.

### KFs no control de la ventana

Se calculan usando el `PropagationPlan` de 3I y las poses optimizadas de sus
controles temporales. La propagacion debe conservar movimiento SE(3), orden y
continuidad; no puede reducirse a una interpolacion planar si el movimiento raw
contiene roll, pitch o altura.

### KFs tardios dentro de la ventana

Antes del commit se vuelve a consultar el estado live. Un KF cuyo orden raw
este entre el control inicial y el target, pero que no existia en el snapshot de
3I, se incorpora al intervalo temporal correspondiente con la misma regla de
propagacion. No obliga a descartar una propuesta por el mero hecho de haber
llegado tarde.

Si cambio materialmente un KF raw que si fue consumido por el grafo, la revision
es incompatible y no se compromete el lote.

Un conflicto causado por el flujo principal mientras el solver trabajaba hace
obsoleta la propuesta, no el objetivo fiducial: el mismo worker revalida y
reconstruye de forma acotada. Solo si no converge dentro del limite termina en
fallo duro bloqueante.

### KFs posteriores al target

Todos los KFs del mismo submapa posteriores al target se reanclan respecto al
nuevo target aceptado:

```text
T_world_k = T_world_target_accepted
          * inverse(T_raw_target_current)
          * T_raw_k_current
```

Esto incluye los KFs que el flujo principal incorporo mientras el solver estaba
activo. Asi, los datos nuevos siguen publicandose durante el calculo y quedan
coherentes en el commit, sin detener el flujo principal.

### Hijos anclados por loop

Si el patch cambia la pose de un KF que sirve de apoyo a un submapa hijo soft,
el hijo debe conservar la relacion que le dio pose world:

```text
delta = T_world_support_new * inverse(T_world_support_old)
T_world_child_kf_new = delta * T_world_child_kf_old
```

El mismo delta se aplica a todos sus KFs y, recursivamente, a sus descendientes
que sigan siendo blandos. No se deforman sus trayectorias internas ni se añaden
aristas de covisibilidad. Una frontera con fiducial hard detiene la propagacion.
El patch del padre y toda esta propagacion se revalidan y comprometen en un
unico batch atomico.

## `last_accepted_control_kf`

Despues de un `ACCEPT_FULL`, el target fiducial se convierte en
`last_accepted_control_kf`. Los KFs posteriores se publicaran respecto a el.

Un `ACCEPT_PARTIAL_RETRY` actualiza atomicamente las poses y reancla la cola
posterior a la pose parcial del target, pero conserva como control al KF inicial
de la ventana. La siguiente pasada vuelve a optimizar ese mismo intervalo. El
target no se promociona ni adquiere semantica hard hasta quedar en umbral.

Si una observacion de una nueva visita ya era coherente y no necesitaba solver,
3H puede establecer ese primer KF como control. Los KFs posteriores coherentes
de la misma visita no lo desplazan.

La siguiente optimizacion siempre parte del ultimo control aceptado, haya sido
otro fiducial o el mismo fiducial en una visita posterior.

## Revisiones y atomicidad

El commit usa validacion de revisiones con alcance:

- cambios en raw o metadatos hard dentro del intervalo consumido invalidan el
  candidato;
- KFs nuevos despues del target no invalidan el candidato: se incorporan al
  patch de cola posterior;
- KFs tardios dentro de la ventana se incorporan si su relacion raw es
  compatible;
- un cambio concurrente de poses derivadas debe reconciliarse o producir
  conflicto explicito; nunca se sobrescribe silenciosamente.

`GlobalPoseStore` aplica todo el `PoseUpdateBatch` bajo una seccion critica
breve. Los lectores ven la revision anterior o la nueva, nunca un estado
parcial. `RawMapDatabase` no se modifica.

Antes del commit se construye y valida el candidato completo en memoria
privada. No existe un flujo normal de “aplicar, publicar y luego rollback”. Un
checkpoint de seguridad puede conservarse internamente, pero una propuesta
invalida no debe hacerse visible.

## Commit parcial y refinamiento

Si 3L determina que un candidato es finito, coherente y mejora de forma segura,
pero el target sigue fuera de umbral:

1. se compromete atomicamente el lote valido;
2. se mantiene activa la parada de drones;
3. se vuelve a capturar la misma ventana desde el control anterior al target;
4. se ejecuta otra pasada dentro de la misma tarea y con el mismo `task_id`;
5. se repite hasta entrar en umbral o alcanzar un fallo duro explicito.

No se descarta un error fiducial grande solo porque una pasada no lo elimine por
completo. Tampoco se compromete un candidato numericamente inseguro.

## Notificacion a `GlobalMapBuilder`

El commit genera un `PoseChangeSet` con exclusivamente IDs de KFs cuya pose world
cambio materialmente. No envia IDs de MPs ni transforma la nube desde el worker.

`GlobalMapBuilder` acumula esos KFs como dirty. En la siguiente ejecucion del
flujo principal:

- expande internamente KFs a MPs mediante sus indices inversos KF->MP;
- recalcula frustums y puntos afectados desde las poses relativas ya guardadas;
- construye y publica una revision coherente.

La notificacion secundaria no despierta al worker principal y no espera un ACK
de RViz2, web o ROS.

## Cambios a realizar

- definir `PoseUpdateBatch`/transaccion con revisiones, hard metadata,
  `last_accepted_control_kf` y lista de KFs movidos;
- implementar propagacion de controles, no controles, KFs tardios y cola
  posterior;
- expandir el patch con la propagacion rigida de dependencias loop blandas;
- añadir commit atomico corto en `GlobalPoseStore`;
- generar `PoseChangeSet` solo con KFs movidos;
- conectar la notificacion dirty ya soportada por `GlobalMapBuilder`;
- integrar las pasadas parciales dentro de la misma tarea activa;
- completar estados, metricas y telemetria de commit/conflicto/fallo;
- mantener parada de drones hasta el final real del flujo.

El anexo `subfase_3K_worker_secundario.md` describe el contrato transversal del
worker y la cola ya creados por 3H.

## Invariantes

- ningun hard fiducial previo se mueve; un anchor loop blando solo puede cambiar
  mediante la propagacion rigida de su dependencia;
- todas las poses del batch son finitas y pertenecen a una revision coherente;
- no falta ningun KF de ventana, late-window o cola posterior aplicable;
- `RawMapDatabase` conserva datos ORB-SLAM3 crudos;
- el commit es atomico y avanza una unica revision derivada;
- el builder recibe solo IDs de KFs movidos;
- el flujo principal nunca espera al commit salvo la seccion critica breve;
- no se publica desde el worker secundario.

## Contrato visual

La parte final visible es:

```text
Validation --accepted/full|partial--> GlobalPoseStore
GlobalPoseStore --pose_dirty(kf_ids)--> GlobalMapBuilder
```

Un evento dirty significa “estos KFs cambiaron”; no significa que el builder ya
haya recalculado la nube. La actividad real de builder y RViz2 solo aparece con
el siguiente `PrimaryInput`.

## Pruebas

- commit de ventana minima y larga;
- propagacion SE(3) de KFs no control;
- incorporacion de KF tardio dentro de ventana;
- reanclaje de varios KFs posteriores al target con la formula acordada;
- movimiento de un KF de apoyo propaga exactamente el mismo delta al hijo soft;
- una cadena de hijos soft se propaga atomicamente y una frontera hard no se
  mueve;
- llegada concurrente de KFs mientras el solver esta pausado artificialmente;
- revision raw incompatible provoca fallo sin commit;
- KFs nuevos posteriores no vuelven `STALE` la propuesta;
- ningun lector observa un batch parcial;
- commit parcial seguido de nueva pasada con igual `task_id`;
- hard/anchors inmutables y raw intacta;
- `PoseChangeSet` contiene KFs movidos y ningun MP;
- builder expande MPs solo en el siguiente flujo principal;
- replay reproduce la misma revision y lista dirty.

## Criterios de cierre

3K queda conseguida cuando una optimizacion validada modifica atomicamente toda
la region afectada, incluidos datos llegados durante el calculo, conserva raw y
hard, y deja al builder la informacion minima para recomputar la geometria en la
siguiente tarea principal.

## Fuera de alcance

- recalculo directo de MPs en el worker secundario;
- publicacion ROS secundaria o espera de ACK visual;
- optimizacion covisible y deformacion conjunta multi-submapa; se permite solo
  el batch multi-submapa derivado de propagacion rigida de anchors blandos;
- fusion de landmarks;
- rollback visible como estrategia normal de aceptacion.
