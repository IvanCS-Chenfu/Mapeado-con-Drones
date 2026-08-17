# Subfase 3P - Pruebas acordadas

## Estado

```text
PRUEBA 161 AUTOMATICA CONSEGUIDA; REVISION VISUAL PENDIENTE
```

## Build

```text
orbslam3_multi
orbslam3_server
simulacion_dron si cambia el grafo visual
```

Usar exclusivamente `build_selected_packages.sh`. Un build correcto es
obligatorio pero no suficiente.

## Tests deterministas de fusion

1. Dos raws distintos crean un track y ocultan dos miembros publicables.
2. El mismo raw ID no crea track artificial.
3. Dos miembros del mismo track producen no-op.
4. Add y merge son transitivos y conservan el menor ID existente.
5. Orden distinto de pares produce el mismo resultado.
6. Evidencia/revision repetida no infla soporte ni score.
7. Una evidencia materialmente nueva puede reforzar un track existente.
8. MP inexistente, bad, no finito o stale no deja escritura parcial.
9. La dispersion superior a `0.50 m` rechaza solo el par incompatible.
10. Fallo interno aplica inverse patch y restaura miembros/representantes.
11. Un cambio de pose que separa miembros marca `degraded` sin perder identidad.
12. El representante ponderado y descriptor medoid son deterministas.

## Tests de RANSAC, visibilidad y score

1. RANSAC rechazado no cambia ningun score.
2. Inlier aceptado aplica `+0.04` a ambos MPs una sola vez.
3. Outlier ambiguo o aislado solo genera diagnostico.
4. Hard outlier sin contradiccion visible no penaliza.
5. `EXPECTED_VISIBLE_MISS` aplica `-0.01` solo al MP responsable.
6. `FOREGROUND_CONTRADICTION` aplica `-0.03` solo al MP responsable.
7. Ocluido, fuera de imagen, detras o incierto no cambia.
8. La evaluacion query-candidate es simetrica.
9. Toda evidencia negativa elegible se evalua sin corte temporal; regiones y
   subnubes siguen respetando sus limites estructurales.
10. Repetir el mismo evento/revision es no-op.
11. Tarea stale o rama de error alto no deja score.

## Tests de commit e incrementalidad

- `FusionUsesBoundedInput`: ninguna base completa se captura.
- `AtomicFusionPatch`: tracks, covisibilidad y score positivo aparecen todos o
  ninguno bajo revision compatible.
- `FusionChangeSetIsExact`: created/updated/retired y hidden/released coinciden
  con el estado real.
- `ScoreChangeSetIsExact`: solo incluye entidades modificadas.
- `SecondaryFusionDoesNotPublish`: solo deja dirty sets.
- `GlobalMapBuilderPublishesAllScores`: ningun punto se filtra por umbral.
- `GlobalMapBuilderUsesOneRepresentative`: conteo exacto:

```text
published = valid_raw - unique_hidden_raw_members + active_fused_tracks
```

- Un cambio solo de score no reproyecta XYZ.
- `RawMapDatabase` y `GlobalPoseStore` permanecen logicamente iguales.
- Un stale precommit no escribe y, tras completar la tarea anterior, encola una
  tarea fresca con revisiones nuevas.
- Un fallo de score posterior a tracks/covisibilidad ejecuta rollback completo
  y encola el mismo retry fresco.
- Dos stale consecutivos no crean duplicados: existe como maximo un retry
  equivalente pendiente, cada uno con `task_id` nuevo.
- Un KF inactivo o `bad` no produce retry.

## Concurrencia y scheduler

Mantener artificialmente lenta la preparacion y comprobar que:

- el flujo principal sigue insertando y publicando KFs/MPs;
- hay como maximo un worker secundario activo;
- cada intento conserva un unico `task_id`; un stale/rollback termina antes de
  que un retry fresco vuelva al final de BAJA;
- una fiducial MAX llegada durante fusion espera a que termine la tarea activa
  y precede a loops BAJA pendientes;
- no hay espera de RViz2, web o publication ACK;
- backpressure sigue usando la politica existente `64/16`.

## Simulacion live

Prueba principal:

```text
codex/archivos_auxiliares/trayectorias/prueba_tipica_rodeo_edificio_dos_fiduciales.yaml
ros2 launch simulacion_dron multi_dron.launch.py
```

Debe observarse al menos un commit real de fusion. Si la prueba tipica no
produce suficientes solapes fusionables, crear un escenario dirigido de dos
pasadas con overlap controlado. No relajar RANSAC solo para forzar conteos.

Validar:

- misma `LoopTask` desde BoW hasta commit;
- todas las regiones compatibles agrupadas;
- nube posterior sin duplicar miembros de tracks;
- KFs nuevos siguen apareciendo mientras fusion trabaja;
- score cambia segun evidencia y no filtra puntos;
- cola drena o activa backpressure conforme al contrato;
- RViz2 y grafo web son coherentes con los conteos automaticos.

## Metricas obligatorias

```text
pares/inliers/outliers por clase
tracks creados/extendidos/merged/degraded/no-op
raws hidden/released y puntos publicados
score positivo/negativo/no-op
visibilidad: regiones iniciadas/completadas, puntos, evidencia negativa y avg/max ms
prepare/commit avg/max, solo como telemetria sin umbral de aceptacion
retries fresh por causa, task anterior/nueva y coalescencia
secondary pending/max_active/backpressure
latencia principal durante fusion
PSS/RAM y PSI
```

## Logs

Reducir con patrones `F3K-TASK|F3N-|F3O-|F3P-|F3M-COVIS|F3F-GLOBALMAP|FLOW-EVENT|BACKPRESSURE|ERROR|FATAL|Segmentation fault|Killed|SIM-DONE`.
Nunca abrir el log completo; ampliar el reducido si falta evidencia.

La repeticion principal del ajuste sera la prueba tipica de dos fiduciales y
debe validar tanto optimizaciones fiduciales como fusion/score/retry. La revision
humana sigue siendo obligatoria para el cierre visual.
