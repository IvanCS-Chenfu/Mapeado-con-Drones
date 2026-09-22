# 5.5 - Datos recibidos en `RawMapDatabase`

## Objetivo

Representar la evolución de los contadores que `RawMapDatabase` conserva tras
cada inserción recibida por el flujo principal. La prueba reutiliza el perfil
GT de dos drones con pérdida y reanclaje, incluidos los snapshots normales.

## Medición

El servidor emite `[F3A-RAW-STATS]` únicamente cuando
`raw_stats_telemetry_enabled=true`. El marcador se produce inmediatamente
después de `InsertDelta`/`InsertFullSnapshot`, usando el `RawInsertResult` ya
obtenido por el worker; con el flag apagado no hay construcción de cadenas ni
telemetría adicional.

Las series son: `submaps`, `keyframes`, `mappoints`, `delta_entries` y
`fiducial_observations` y `snapshot_count` acumulado. `journal_entries` no se
repite porque actualmente es idéntico a `delta_entries`; `last_arrival_id` se
conserva como identificador de la muestra en el CSV. No se incluyen poses
world, scores, tracks fused, asociaciones, covisibilidad ni nube publicada,
pues no son contadores nativos agregados de `RawMapDatabase`.

## Artefactos

- `datos_raw.csv`: una fila por inserción raw y sus contadores acumulados.
- `grafica_datos_raw.png`: series acumuladas respecto al tiempo desde la
  primera inserción.
- `resumen.json`: muestras, duración, fuente, snapshots y valores finales.

La gráfica usa cuatro paneles con escalas propias para que la magnitud de los
MapPoints no oculte la evolución de KFs o submapas. En el panel inferior
derecho, deltas y observaciones fiduciales usan el eje izquierdo; los snapshots
acumulados se representan en azul con el eje derecho.

## Ejecuciones anteriores

La primera ejecución `c5_5_5_raw_stats_two_drones_v1` contenía snapshots y se
descartó entonces porque el alcance inicial pedía exclusivamente inserciones
incrementales. La ejecución `v4_gui` repitió la misión sin snapshots y produjo
artefactos válidos para ese alcance; el usuario pidió sustituirlos por la
medición normal actual. Los intentos se conservan en los logs, pero no se usan
como resultados de esta sección.

## Ejecución válida

La ejecución `c5_5_5_raw_stats_two_drones_v5_snapshots` reutiliza el mismo
perfil GT de dos drones de la prueba visual, con Gazebo y el GUI global
abiertos. El escenario completó sus diez pasos con `success=true`; la
telemetría raw estuvo activa, Fase 6 y la máscara física de MPs permanecieron
desactivadas. No se aplicó ningún override de snapshots, por lo que el perfil
ejecutó su comportamiento normal.

Se almacenaron 217 inserciones durante 174,339 s desde la primera inserción
raw. Todas tuvieron `source=live` y 9 fueron snapshots. Al finalizar,
`RawMapDatabase` contenía 4 submapas, 120 KFs, 13.438 MapPoints, 217 entradas
delta y 30 observaciones fiduciales. Los valores exactos por inserción están en
`datos_raw.csv`; `resumen.json` conserva estas cifras para referencia del
informe.
