# Subfase 3S - Scoring de raw MPs y fused tracks

## Estado vigente

```text
REHACER INTEGRACION; CONSERVAR POLITICA DE SCORE
```

`LandmarkScoreManager` se conserva como autoridad numérica, pero sus cambios
raw deben salir del callback principal y producir cambios incrementales para
`3F`.

### Trabajo que se conserva

- score único para MPs raw y fused tracks;
- eventos estructurados y prohibición de escribir score desde otras clases;
- integración de score fused en el commit de `3P`.
- eventos positivos/negativos de geometria y visibilidad introducidos en `3P`,
  sin quitar a `LandmarkScoreManager` la autoridad numerica.

### Implementación anterior incorrecta que no debe repetirse

- actualizar o copiar todos los scores bajo `live_state_mutex_`;
- bloquear publicación esperando cálculo secundario;
- usar snapshots completos en builder/loops;
- publicar score de una tarea rechazada o stale.

### Contrato de reimplementación

El score ORB base de MPs afectados se calcula en el flujo principal. Cualquier
calculo derivado no imprescindible se integra en la `DatabaseUpdateTask` MEDIA
existente, sin crear una prioridad nueva. La preparacion ocurre fuera de lock y
el commit breve devuelve `ScoreChangeSet`. La fusion integra su patch de score
en la misma transaccion logica. `GlobalMapBuilder` consume IDs/revisiones dirty
y nunca escribe score.

`3P` adelanta solo la evidencia inseparable de una fusion: inlier `+0.04`, miss
visible `-0.01` y contradiccion foreground `-0.03`, todos configurables,
simetricos e idempotentes. `3S` conserva la propiedad de la politica general y
podra refinar formula, eventos y calibracion sin cambiar el flujo.

## Estado histórico anterior

Las secciones posteriores se conservan como contrato/evidencia de la
implementación anterior. Si contradicen el bloque REHACER de esta cabecera,
prevalece el contrato de reimplementación nuevo.

```text
INTEGRADA EN RUNTIME; cierre de politica y pruebas especificas PARCIAL.
```

## Propiedad

`LandmarkScoreManager` es la unica autoridad de score:

- el flujo principal actualiza score base desde el `ChangeSet` raw;
- una `LoopTask` actualiza score derivado dentro del mismo commit de fusion;
- `GlobalMapBuilder` solo lee el snapshot de score.

El builder publica todos los puntos independientemente del score. La GUI de
Fase 7 sera quien permita filtrarlos visualmente.

No existe una cola, worker o transaccion visual propios de scoring.

## Subdocumentos

- `subfase_3S_especificacion.md`: eventos, ownership e invariantes.
- `subfase_3S_implementacion.md`: APIs y commits.
- `subfase_3S_testing.md`: pruebas.
- `subfase_3S_criterios.md`: exito/fallo.

## Incremento Visual Obligatorio

Aplicar `../CONTRATO_VISUAL_INCREMENTAL.md`. Añadir
`LandmarkScoreManager`, eventos/patches de score raw y fused y la notificacion
dirty al builder. Mostrar cantidades y revision; no enviar scores completos.
