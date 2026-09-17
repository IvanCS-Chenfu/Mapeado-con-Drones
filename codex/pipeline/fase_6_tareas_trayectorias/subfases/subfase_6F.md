# Subfase 6F - Integracion de depth y continuaciones correlacionadas

## Estado

`PARCIAL`. La infraestructura de resultados y fuentes por KF existe, pero debe
migrarse al contrato de inspeccion de 6H/6I.

## Objetivo

Convertir cada resultado local de una orden autonoma en evidencia reversible
relativa a KF, sin bloquear al servidor ni decidir el siguiente movimiento
antes de que `VoxelMapBuilder` la haya aplicado.

## Contrato

El dron acepta una orden y ejecuta localmente; al terminar llama una unica vez
a `ReportAutonomousResult`. El callback solo valida IDs, deduplica
`command_id`, persiste el resultado y encola `DEPTH_INTEGRATION`; responde de
inmediato. Ningun callback remoto espera movimiento, giro o captura.

`DepthIntegrationWorker` escribe fuentes locales en `EvidenceDatabase` y
registra la continuacion con sus `source_id`. No toca el mapa mundial ni activa
la U directamente:

- `MOVE_AND_CAPTURE` / `vista_pared`: rayos FREE y, solo al mirar frontalmente
  una pared y dentro de la puerta geometrica acordada, impacto directo
  `OCCUPIED=1`;
- `LOOK_AND_CAPTURE` / `vista_unknown`: exclusivamente evidencia FREE. Nunca
  crea `OCCUPIED` depth;
- `VIEW_ADVANCE` / `free_prefix`: incluye tanto el prefijo FREE obtenido por
  D* como la pose FREE de respaldo tras una `vista_unknown`. Comparte la puerta
  geometrica de superficie frontal de `vista_pared`, por lo que aporta FREE y
  puede aportar OCCUPIED directo fiable. Una fuente frontal `depth_occupied`
  conserva su geometria local, su pose de captura relativa al KF y el contexto
  de coverage para que 6G derive claims reversibles; no se interpreta como una
  llegada normal a fachada;
- `TRACKING_RISK`: conserva su depth y su contexto para que la continuacion
  resuelva como se interrumpio la orden; no desecha evidencia por ser aborto.

Un `STOP` puede reemplazar una trayectoria y hacer que el terminal normal se
reporte como `REJECTED`. Si ese terminal incluye observaciones depth validas,
`DepthIntegrationWorker` las integra igual que las de `COMPLETED` o `ABORTED`,
espera a `sources_applied` y solo entonces encola la continuacion. La orden
cancelada nunca se reanuda: tras la evidencia materializada se vuelve a la
seleccion correspondiente.

Cuando `VoxelMapBuilder` confirma todos los `source_id`, la continuacion se
libera. Una `vista_pared` y un `VIEW_ADVANCE` vuelven a `POINT_SELECTION`; una
`vista_unknown` revalida la misma pose en 6H. Si no hay depth utilizable, se
trata como un corredor que no pudo confirmarse: el dron avanza solo por prefijo
FREE o 6H elige otro objetivo. No se inventa FREE ni se mantiene una espera
activa.

La actualizacion de coverage no es una continuacion separada: 6G la deriva de
las fuentes materializadas en la misma revision. Toda fuente frontal
`depth_occupied` de `vista_pared` o `VIEW_ADVANCE` puede reclamar la seccion
geometrica donde cae y su vecindario continuo de la U. Sus voxeles
`OCCUPIED=1` reclaman tambien cualquier seccion U que contengan
espacialmente. Los claims conservan procedencia por `source_id` y KF, por lo
que una optimizacion, tombstone o reproyeccion puede retirarlos. Un resultado
valido `SIN_FACHADA` de `vista_pared` conserva un claim propio para su seccion.

## Validacion y exito

Un resultado de `vista_pared` puede dar FREE y OCCUPIED directo; uno de
`vista_unknown` solo FREE. Ninguna continuacion se ejecuta antes de la
revision materializada y resultados duplicados o tardios no despiertan dos
movimientos.
