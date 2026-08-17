# Subfase 3L - Validacion, refinamiento y cierre fiducial

## Estado

```text
Preparacion: CERRADA
Acuerdo cerrado: si
Autorizacion funcional: PENDIENTE PARA EXTENSION POST-LOOP
Ejecucion: VALIDACION BASE CONSEGUIDA; COBERTURA DE HIJOS SOFT PENDIENTE
Dudas abiertas: ninguna
```

3L define la puerta de seguridad entre la propuesta privada de 3J y el commit
atomico de 3K. Aunque el orden documental sea 3J-3K-3L, en runtime la validacion
ocurre antes de cada commit y puede ordenar otra pasada interna.

## Sucesion acordada en 3Q

La misma puerta validara fiducial absoluto y loop relativo. Para loop, la
primera version solo acepta candidatos completos: coste/error bajan, hard no se
mueve, constraints son finitas y el loop termina bajo umbral de fusion. La
variacion de fusiones previas soft se mide sin convertirlas aun en hard.

## Objetivo

Garantizar que una optimizacion fiducial nunca haga visible un estado numerico,
geometrico o transaccionalmente incoherente, sin permitir que un error grande se
ignore simplemente porque una pasada no lo elimino por completo.

```text
OptimizationProposal -> ValidationDecision -> atomic commit or hard failure
```

La validacion se hace sobre un candidato privado completo: controles, KFs no
control, late-window, cola posterior y metadatos. No se valida despues de haber
publicado cambios parciales.

## Entrada

El validador recibe:

```text
FiducialOptimizationTask identity and target
PoseGraphProblem and consumed revisions
OptimizationProposal
complete candidate PoseUpdateBatch, incluidas dependencias loop blandas
current scoped revisions
current hard/anchor metadata
```

No usa GT de la trayectoria. Solo compara el KF fiducial target con la
observacion absoluta que origino la tarea.

## Comprobaciones obligatorias

### Finitud y SE(3)

- ninguna traslacion contiene NaN o Inf;
- todas las rotaciones son validas y normalizadas;
- no existen matrices degeneradas;
- el solver no ha producido un estado numericamente fallido.

### Hard y anchors

- el control inicial conserva exactamente su pose aceptada;
- ningun hard fiducial previo se mueve;
- no cambia la identidad del anchor inicial;
- un anchor loop blando solo se mueve mediante el delta del KF de apoyo del que
  depende;
- una dependencia que ya adquirio fiducial hard no recibe propagacion;
- el nuevo target solo pasa a control si el batch se acepta.

### Cobertura

- todos los controles tienen pose candidata;
- todos los KFs no control de la ventana estan propagados;
- todos los KFs late-window compatibles estan incluidos;
- todos los KFs posteriores al target del mismo submapa estan reanclados;
- todos los KFs de componentes hijos blandos afectados estan incluidos;
- no hay IDs duplicados, ausentes ni submapas ajenos a la ventana o a sus
  dependencias loop blandas.

### Geometria

- se preserva el orden temporal;
- no aparecen saltos relativos no explicados entre intervalos;
- las vecindades protegidas mantienen su rigidez acordada;
- las poses posteriores conservan su transformacion raw respecto al target;
- cada hijo soft conserva su geometria interna y recibe exactamente el delta de
  su KF de apoyo;
- cualquier tolerancia geometrica se apoya en el solver validado, no en GT.

### Revisiones y atomicidad

- las revisiones raw consumidas siguen siendo compatibles;
- hard/anchor metadata no cambio durante el calculo;
- los KFs nuevos posteriores se incorporaron en vez de invalidar sin motivo;
- el batch puede aplicarse como una unica revision de `GlobalPoseStore`;
- `RawMapDatabase` permanece sin mutaciones de optimizacion.

Una incompatibilidad de revision detectada antes del commit descarta la
propuesta completa y vuelve a revalidar/reconstruir dentro de la misma tarea,
hasta `fiducial_max_refinement_passes`. Nunca se aplica un lote stale.

### Error fiducial

Se recalculan respecto de `target_world_T_kf`:

```text
translation_error_m
rotation_error_rad
yaw_error_rad
```

Los defaults iniciales son los mismos de 3H: 0.35 m, 0.35 rad y 0.25 rad. El
comportamiento esperado de la variante final de `legacy2` es dejar un error
cercano a cero, pero el contrato de aceptacion exige estar dentro del umbral.

## Decisiones estructuradas

| Decision | Condicion | Accion |
|---|---|---|
| `STALE` | la revalidacion previa ya estaba en umbral | terminar sin grafo ni commit |
| `ACCEPT_FULL` | candidato seguro y target dentro de umbral | commit atomico y finalizar |
| `ACCEPT_PARTIAL_RETRY` | candidato seguro/mejor y target aun fuera de umbral | commit atomico y nueva pasada interna |
| `HARD_FAILURE` | candidato inseguro o revision incompatible | no commit; fallo bloqueante |
| `CANCELLED_SHUTDOWN` | cierre antes del commit | no commit y terminar ordenadamente |

Una propuesta parcial solo es aceptable si es completamente finita, conserva
hard/anchors, cubre el lote y representa progreso seguro. No se compromete una
pose simplemente porque reduzca un numero si rompe geometria o revisiones.
El commit parcial no promociona el target a `last_accepted_control_kf`: conserva
el control anterior para reconstruir la misma ventana en la siguiente pasada.
La promocion a control/hard ocurre exclusivamente con `ACCEPT_FULL`.

## Refinamiento

Con `ACCEPT_PARTIAL_RETRY`:

1. 3K compromete el batch completo y atomico;
2. el worker mantiene la tarea activa y la parada de drones;
3. se incrementa `pass_index` bajo el mismo `task_id`;
4. se vuelve a leer el estado comprometido y se reconstruye la misma ventana
   desde el control anterior hasta el target;
5. se repiten solver y validacion;
6. el flujo solo concluye al entrar en umbral o producir un fallo duro.

No se cede el worker entre pasadas. Los limites de seguridad frente a una falta
de convergencia deben producir un fallo explicito y diagnosticable, nunca un
loop infinito ni un falso exito.

## Fallos duros

Son, como minimo:

- NaN, Inf o pose SE(3) invalida;
- fallo numerico del solver sin candidato seguro;
- movimiento de hard/anchor;
- revision raw incompatible dentro de la ventana;
- batch incompleto, duplicado o geometricamente incoherente;
- imposibilidad persistente de converger tras las salvaguardas acordadas.

Un fallo duro no modifica `GlobalPoseStore`, no notifica dirty y no se convierte
en un descarte silencioso. Mantiene la parada de drones y expone una causa
estructurada para diagnostico/operacion.

## Despues del commit

El resultado registra revision, KFs movidos, errores finales y numero de
pasadas. `GlobalPoseStore` notifica solo los IDs de KFs movidos a
`GlobalMapBuilder`.

La comprobacion espacial ocurre cuando el siguiente `PrimaryInput` hace que el
builder expanda esos KFs a sus MPs, recalcule y publique. 3L puede verificar
despues que esa revision fue consumida, pero el worker secundario no espera un
ACK y el aspecto visual no forma parte de la decision runtime.

Las metricas GT externas pueden usarse para debug de simulacion, nunca para
aceptar o rechazar el mapa.

## Cambios a realizar

- definir `ValidationDecision`, codigos de fallo y diagnosticos estructurados;
- implementar validacion privada de finitud, hard, cobertura, geometria,
  revisiones y error fiducial;
- construir/validar el batch completo antes de invocar el commit de 3K;
- integrar `ACCEPT_PARTIAL_RETRY` como pasadas internas de la misma tarea;
- mantener parada de drones durante todas las pasadas y ante fallo bloqueante;
- registrar resultado final, revisiones, KFs movidos y errores;
- añadir telemetria real de validacion, commit parcial/completo y fallo;
- verificar en tests la recomputacion posterior del builder sin acoplarla al
  worker.

## Invariantes

- nada invalido se hace visible;
- un error fiducial grande no se acepta como resultado final;
- ningun hard/anchor se mueve;
- todo commit es completo y atomico;
- raw permanece intacta;
- una tarea conserva `task_id` durante refinamientos;
- el flujo principal continua durante calculo y solo comparte locks breves;
- la parada se libera unicamente al finalizar sin optimizacion activa y cumplir
  la histeresis global.

## Contrato visual

Debe distinguirse:

```text
OptimizationManager -> Validation
Validation --full--> GlobalPoseStore
Validation --partial/retry--> GlobalPoseStore -> nueva pasada
Validation --hard_failure--> estado bloqueante
GlobalPoseStore --pose_dirty(kf_ids)--> GlobalMapBuilder
```

El visualizador muestra decisiones ya tomadas, no decide ni bloquea el flujo.

## Pruebas

- aceptacion completa dentro de umbral;
- propuesta finita parcial, commit y segunda pasada bajo mismo `task_id`;
- error no decreciente alcanza fallo explicito sin loop infinito;
- NaN/Inf y rotacion invalida nunca hacen commit;
- hard movido nunca hace commit;
- hijo soft incompleto, delta incoherente o frontera hard movida nunca hacen
  commit;
- revision incompatible nunca hace commit;
- batch sin un KF de ventana/tail nunca hace commit;
- KFs posteriores nuevos se incluyen correctamente;
- raw antes/despues es identica;
- lectores concurrentes ven revision antigua o nueva completa;
- parada activa durante solver/refinamiento y liberacion con histeresis;
- siguiente entrada principal consume dirty KFs y actualiza MPs/RViz2;
- replay y live terminan en error fiducial dentro de umbral o fallo duro claro.

## Prueba integrada 3H-3L

La entrega se validara con:

- tests C++ sinteticos y deterministas de todas las ramas;
- replay completo usando `fiducial_visit_id` persistido;
- live de dos drones con trayectoria fiducial 2 -> fiducial 1 -> fiducial 2;
- umbrales normales; umbral casi cero solo si es necesario para forzar la rama
  `opt_fid`;
- logs reducidos con orden de cola, revalidacion, pasadas, commit y dirty KFs;
- grafo web con observacion, MAX, worker, graph, solver, validation y commit;
- RViz2 comprobado por el usuario tras la siguiente publicacion principal.

## Criterios de cierre

3L y la entrega conjunta 3H-3L quedan conseguidas solo si build, tests, replay y
live demuestran un flujo completo, atomico y determinista; el target termina en
umbral o se declara un fallo duro sin corromper estado; y las validaciones web y
RViz2 corresponden al runtime real.

## Fuera de alcance

- deformacion conjunta por covisibilidad; la propagacion rigida transitoria de
  hijos soft si forma parte del commit fiducial;
- loops, matching, RANSAC y fusion;
- GT como criterio de aceptacion global;
- publicacion desde el worker o espera de ACK visual;
- rollback visible como camino normal.
