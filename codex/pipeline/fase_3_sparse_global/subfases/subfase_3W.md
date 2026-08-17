# Subfase 3W - Rendimiento, limites y robustez runtime

## Estado vigente

```text
REHACER CONTRATO Y EJECUTAR TRAS 3B-3U
```

Esta subfase era pendiente, pero su contrato anterior no incorpora toda la
causalidad y backpressure necesarios. Se rehace documentalmente ahora y se
ejecutará después de las subfases propietarias.

### Trabajo que se conserva

- separación de presupuestos principal/secundario;
- métricas por etapa, profundidad y duración;
- principio de no degradar precisión para ganar velocidad.

### Implementación anterior incorrecta que no debe repetirse

- tratar la profundidad de la cola principal como politica final unica; en 3C
  es deliberadamente el bootstrap funcional;
- publicar el flag calculado siempre como `false`;
- cancelar o sustituir el goal de movimiento activo;
- acumular ticks de snapshot durante la pausa;
- permitir espera infinita si un worker muere o no drena.

### Contrato de reimplementación

3W hereda de 3C el topic, su publisher reliable/transient-local, el gate del
scenario runner y la histeresis principal high=8/low=2. No sustituye esa ruta:
la endurece. Backpressure será OR de high watermarks de ambas colas, edad
máxima, drenaje estimado y optimización fiducial/loop activa, con low
watermarks y dwell de liberación. El goal activo termina; el runner retiene el
siguiente movimiento.
No se solicitan snapshots periódicos durante presión y al liberar se pide como
máximo uno fresco por dron. Se medirán lock wait/hold, enqueue/start/end,
latencia de publicación, drops/coalescing web y fallo explícito por presión
excesiva o worker sin progreso.

## Estado histórico anterior

Las secciones posteriores se conservan como contrato/evidencia de la
implementación anterior. Si contradicen el bloque REHACER de esta cabecera,
prevalece el contrato de reimplementación nuevo.

```text
SIN HACER: contrato actualizado al flujo principal/secundario.
```

## Objetivo

Medir y acotar costes sin cambiar ownership ni precision para ganar velocidad.
La prioridad es que el flujo principal conserve progreso observable bajo carga.

## Presupuestos separados

### Flujo principal

Medir:

- recepcion -> commit raw;
- commit raw -> pose/covis/score principal;
- tiempo dentro de cada lock;
- creacion/tamaño de `ChangeSet`;
- captura de publicacion;
- build y publish de KFs/nube;
- periodo real entre publicaciones;
- KFs recibidos frente a visibles.

Ninguna metrica se mejora moviendo trabajo secundario de vuelta al callback.

### Worker secundario

Medir por `task_id`:

- espera en cola y prioridad;
- snapshot;
- `3N`, `3O`, decision, `3P`, `3I/3J/3L/3Q`;
- validacion de revisiones;
- commit;
- resultado y motivo;
- profundidad fiducial/loop;
- coalescing/stale/rechazos;
- `active_secondary_workers`, maximo uno.

No medir espera visual como parte de la tarea: termina en commit.

### Observabilidad

Medir:

- eventos producidos/publicados/descartados;
- tiempo de `TryEmit`;
- latencia evento -> navegador;
- CPU/memoria del puente y pagina;
- diferencia on/off en latencias funcionales.

## Limites

Parametros centralizados, con nombres finales tras auditar los actuales:

```text
primary_queue_high_watermark (heredado de 3C; default 8)
primary_queue_low_watermark (heredado de 3C; default 2)
primary_queue_max_age_ms
secondary_loop_queue_high_watermark
secondary_loop_queue_low_watermark
secondary_loop_queue_capacity
secondary_fiducial_queue_capacity
secondary_queue_max_age_ms
backpressure_release_dwell_ms
worker_no_progress_timeout_ms
secondary_max_coalesced_revisions_per_key
publication_max_pending_revisions
publication_snapshot_max_copy_items (solo diagnostico, no truncar mapa)
flow_trace_queue_capacity
flow_trace_batch_size
flow_trace_ui_rate_hz
```

Una tarea fiducial valida no se descarta por llegar loops. Si su cola alcanza el
limite por duplicados, se coalescen generaciones canonicas y se activa error
diagnostico.

Las `LoopTask` pueden coalescer revisiones de la misma clave, descartar stale y
aplicar backpressure por high/low watermark. No se descarta estado raw ni se
impide al flujo principal publicar para reducir carga secundaria.

## Degradacion segura

Orden permitido bajo carga:

1. descartar telemetria visual;
2. agrupar revisiones de publicacion intermedias conservando la ultima;
3. coalescer `LoopTask` equivalentes;
4. rechazar trabajo stale antes de calculo;
5. activar backpressure del siguiente movimiento;
6. reducir debug/exports opcionales.

Nunca:

- bloquear subscriptions por una cola secundaria;
- omitir commits raw;
- publicar estado parcial;
- descartar una tarea fiducial valida sin error explicito;
- saltar validaciones geometricas para ahorrar tiempo;
- esperar RViz2/navegador;
- ampliar indefinidamente memoria/colas.

## Contencion

Registrar por base:

```text
lock_wait_us
lock_hold_us
items_copied
items_written
revision_before/after
```

Un p99 alto exige localizar el bloque exacto. No se acepta un unico
`live_state_mutex_` alrededor de raw, poses, loops y publicacion. Tampoco se
elimina sincronizacion sin snapshots/versiones equivalentes.

## Publicacion

La reconstruccion puede ser incremental/cacheada si mantiene el contrato de
vista completa y revision coherente. El timer heartbeat no debe reconstruir sin
cambios. El objetivo es frescura, no publicar mensajes identicos a alta tasa.

## Archivos permitidos

- parametros y metricas de componentes ya propietarios;
- limites/coalescing del scheduler unico;
- optimizacion mecanica de snapshots/commits;
- tests/benchmarks;
- configuracion de telemetria y debug;
- documentacion.

No crear managers solo para almacenar una metrica.

## Pruebas

### Benchmark determinista

Replay fijo con telemetria on/off. Comparar resultados/revisiones identicos y
latencias por etapa.

### Limites forzados

Usar capacidades pequeñas para probar coalescing, prioridad fiducial, stale,
backpressure y drops de telemetria sin perder raw ni bloquear publicacion.

### Simulacion larga

Ruta de dos fiduciales con RViz2/JS. Registrar percentiles, picos, colas,
publicaciones y memoria. Una ejecucion lenta conserva su propia conclusion; no
se reescribe al repetir.

### Desconexion

Cerrar navegador/rosbridge y comprobar que solo cambia telemetria. Reconectar y
continuar con eventos nuevos, sin exigir replay ilimitado.

## Objetivos iniciales

Son presupuestos de diagnostico revisables, no umbrales para ocultar trabajo:

- ningun solver/RANSAC bajo locks live;
- commit secundario de lote corto frente al tiempo de calculo;
- ingesta y pose principal con progreso continuo durante tareas;
- publicacion sin huecos de decenas de segundos causados por mutex;
- cola secundaria acotada y finalmente drenable;
- `active_secondary_workers <= 1`;
- `TryEmit` no bloqueante y drops contabilizados;
- overhead de telemetria pequeño frente a variabilidad normal.

## Exito

- todas las colas y revisiones tienen limites/metricas;
- la prioridad no se viola bajo saturacion;
- el flujo principal continua;
- no hay estados parciales;
- telemetria y debug son degradables;
- benchmark on/off conserva resultados;
- no hay deadlock, crecimiento ilimitado o retry autosostenido.

## Parcial/fallo

`PARCIAL` si el sistema es correcto pero persisten picos medidos o no se alcanza
una muestra suficiente.

`NO CONSEGUIDA` si se gana velocidad bloqueando/omitiendo el flujo principal,
saltando validaciones, perdiendo tareas fiduciales, exponiendo estados parciales
o haciendo que observabilidad afecte al resultado.

## Documentacion

Actualizar docs de cada componente medido, historial `3W`, estado y pipeline.
Los datos largos viven en historial/artefactos reducidos, no en este contrato.

## Limites Visuales Obligatorios

Aplicar `../CONTRATO_VISUAL_INCREMENTAL.md`. Medir rafagas, drops, gaps,
reconexion y latencia hasta render sin que el visualizador altere throughput,
colas o resultado. Solo añadir un vertice de backpressure si existe como
componente runtime real.
