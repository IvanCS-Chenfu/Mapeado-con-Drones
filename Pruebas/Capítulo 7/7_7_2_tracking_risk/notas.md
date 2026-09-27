# Prueba 7.7.2 - Tracking risk durante trayectoria ORB

## Estado agregado

**CONSEGUIDA en `run_09` y repetida con éxito en `run_10`.** Ambas ejecuciones produjeron `TRACKING_RISK` durante el giro ORB gestionado, STOP prioritario y reorientación local completada; los intentos anteriores se conservan como diagnóstico.

## Ejecuciones

| Run | Resultado | Evidencia / causa |
|---|---|---|
| `run_01` | No válida | Configuración inicial anterior al uso de la trayectoria normal gestionada por `task_manager`. |
| `run_02` | No conseguida | El escenario no solicitaba de forma explícita el cambio de fuente a ORB después del tramo GT. |
| `run_03` | No conseguida | El primer goal GT fue rechazado: `F5B-GOAL-REJECT decision=reject_local_invalid`. |
| `run_04` | No conseguida | La nueva puerta `wait_for_navigation_ready` esperó 90 s y agotó timeout. ORB mantuvo `DYNAMIC_BASE_NOT_READY`, `gravity_valid=false`, `map_epoch=0` y `ref_kf=0`; por tanto no se emitió el tramo GT ni el giro ORB. |
| `run_05` | No conseguida | El tramo GT completó, inicializó gravedad y confirmó la pose; la solicitud ORB quedó con `deferred=true` mientras seguía activo el bloqueo de fuente de esa trayectoria, por lo que `orb_authority_confirmed` agotó su espera. |
| `run_06` | No conseguida | El handoff GT -> ORB fue correcto (`effective=orb`, salto de continuidad nulo y autoridad confirmada), pero `managed_yaw` agotó 90 s sin `task_id`: `mission_mode: autonomous` fuerza `execution_enabled_on_start=false`, por lo que Fase 6 no asigna tareas. |
| `run_07` | No conseguida | El gate F6I se abrió, pero se hizo tras pasar a ORB. `task_server` fue lanzado con fuente GT y rechaza ORB para elegibilidad; no hubo `[F6B-ASSIGNED]` y `managed_yaw` agotó su espera de tarea. |
| `run_08` | No conseguida | La tarea se asignó bajo GT y el giro ORB fue aceptado por `task_manager`, pero el worker autónomo lanzó una subtarea `autonomous:point:...` al abrir F6I y preemptó el giro; no se alcanzó riesgo visual. |
| `run_09` | Conseguida | Tarea asignada bajo GT, gate cerrado antes del handoff, giro ORB aceptado; `TRACKING_RISK` en frame 1319, STOP completado y reorientación local completada con éxito. |
| `run_10` | Conseguida | Repetición de `run_09` con GUI que distingue autoridad global pendiente y fracción direccional `0.50`; seis muestras `global_pending=true` sin pérdida, `TRACKING_RISK` en frame 1151, STOP y reorientación completados sin `TRACKING_LOST`. |
| `run_11` | No conseguida | Toma para grabación: GT -> ORB y GUI pendiente correctos, pero el gate se cerró antes de `[F6B-ASSIGNED]`; `managed_yaw` agotó 90 s sin `task_id`, sin giro ni `TRACKING_RISK`. |
| `run_12` | Conseguida | Toma válida para grabación: tarea asignada, giro ORB, riesgo en frame 1224, STOP y reorientación a 56.587 grados completados. |

## Configuración de `run_04`

- Un dron, spawn `(1, -10, 1, 90 deg)`.
- Trayectoria acordada: GT `(1, -10, 1, 90) -> (0, -10, 1, 90)`, cambio explícito a ORB y giro normal gestionado por `task_manager` hacia `(0, -10, 1, 0)`.
- Sin override de parámetros, persistencia ni umbrales de riesgo visual.
- Gazebo, `multidron_gui`, visor de depuración visual y registrador pasivo C7 activos; sin RViz2.

## Evidencia reproducible

- Log bruto: `codex/archivos_auxiliares/logs/prueba_c7_7_2_tracking_risk_orb_managed_yaw_ready_gate.log`.
- Log reducido: `codex/archivos_auxiliares/logs/prueba_c7_7_2_tracking_risk_orb_managed_yaw_ready_gate.reduced.log`.
- Marcador terminal: `[SCENARIO-RUNNER-NAV-READY-TIMEOUT]` a los 90 s, seguido de `[SIM-EXIT-CODE] 1`.

## Diagnóstico confirmado

El fallo primario es de inicialización de fuente, no del detector de riesgo ni de `task_manager`. Las pruebas históricas que sí arrancaron, como la 742, lanzaron `phase5_navigation_source:=gt`. En cambio, `run_03` y `run_04` cargan un perfil con `navigation_source: orb`, por lo que el mux publica ORB inválido durante el arranque no anclado.

El escenario intenta corregirlo por goal: el servicio `set_navigation_source_gt` responde que la fuente queda preparada, pero ese servicio no publica un estado GT ni espera uno. El mux publica `POSE_SOURCE_GT_FORCED` solo al recibir el siguiente mensaje de pose GT. En `run_03` el runner recibió la respuesta a las `...3083.304626`, envió el goal a las `...3083.313664` y `gen_tray` lo rechazó a las `...3083.315238`: unos 9 ms entre preparación y envío, antes de una nueva muestra GT. La action leyó el último `NavigationState` ORB inválido (`local_valid=false`, `velocity_valid=false`, `sample=1516`).

La puerta añadida en `run_04` quedó antes de solicitar GT, por lo que exige que ORB sea localmente válido antes de ejecutar precisamente el tramo GT que debía crear movimiento, fiducial y autoridad. La base dinámica requiere gravedad en O; esta solo se inicializa tras una pose global ORB autoritativa. Por ello `wait_for_navigation_ready` no puede completarse en ese punto y agota 90 s.

La solución coherente es arrancar la misión con fuente global `GT`, como la prueba histórica, ejecutar el único tramo GT, y solo entonces solicitar ORB y esperar `orb_authority_confirmed` antes del giro normal. La alternativa equivalente es solicitar GT como paso separado y esperar explícitamente una muestra canónica `POSE_SOURCE_GT_FORCED` válida antes de enviar el goal. Ninguna de las dos modifica la lógica ni los umbrales de `TRACKING_RISK`.

## Diagnóstico de `run_06`

La solución de fuente quedó validada: tras liberar explícitamente el bloqueo de la trayectoria GT, el mux aplicó ORB, publicó continuidad de traslación y rotación igual a cero y confirmó `orb_authority_confirmed`. El siguiente requisito del giro no se cumplió: el `task_manager` necesita una tarea `ASSIGNED` o `RUNNING`, pero el perfil `mission_mode: autonomous` hace que `multi_dron.launch.py` arranque el `task_server` con `execution_enabled_on_start=false`. Con el gate cerrado no se encola `TASK_ASSIGNMENT`, no aparece `[F6B-ASSIGNED]` y el runner agota `[SCENARIO-RUNNER-MANAGED-YAW-TASK-TIMEOUT]` a los 90 s. No hubo dispatch de `ExecuteTrajectory`, `TRACKING_RISK`, STOP ni reorientación. Para repetir el mismo giro normal gestionado hay que abrir ese gate, previsiblemente usando un modo de misión no autónomo o un override que no sea anulado por el launch; esa decisión queda pendiente de confirmación funcional.

## Resultado vigente: `run_09`

- Escenario y wrapper: código `0`; `[SIM-DONE] success=true`.
- Tarea: `map_section_level_0_AB` asignada bajo GT; tras cerrar el gate no hubo trayectoria autónoma competidora.
- Giro ORB: `scenario_managed_yaw_1790285594267869058` aceptado y ejecutado por `task_manager`.
- Riesgo: primer `TRACKING_RISK` en frame `1319`, `risk_mask=2`; STOP completado `5.110 s` después y reorientación local completada `5.080 s` más tarde, con `success=true`.
- Evidencia: 1.816 muestras y cuatro eventos en `run_09/raw/tracking_risk_timeline_raw.csv`; serie procesada y resumen en `run_09/processed/`.
- El giro original no se completó ni reanudó: quedó sustituido por la reorientación local a `38.353 deg`.

### Aclaración visual

El rótulo `PERDIDO/NO DISPONIBLE` de `multidron_gui` no equivale necesariamente a `TRACKING_LOST` de ORB-SLAM3. En `run_09`, el CSV C7 registra 1.816 muestras con `tracking_state=OK`, `local_valid=true`, `local_continuity_valid=true` y `map_epoch=0`: cero muestras `RECENTLY_LOST/LOST` y cero transiciones de época. De las 883 muestras con fuente ORB, 396 llevaban `global_valid=false`.

La causa es semántica y transitoria: para ORB, `global_valid` exige que la pose global del keyframe de referencia activo haya sido confirmada por el backend. Cuando cambia ese keyframe, el estimador pasa la autoridad global a estado provisional y solicita la actualización; el log reducido registra 83 solicitudes de keyframe y 30 recuperaciones de autoridad global. Mientras la respuesta no llega, el tracking local sigue correcto, pero la GUI agrupa esa pose global pendiente con una pérdida real bajo `lost_or_unavailable`, conserva la última pose y muestra `PERDIDO`. El flujo visual de esta prueba, con giro y degradación intencionada, provoca más cambios de referencia que un recorrido ORB estable. El criterio ORB de la GUI ya requería `global_valid` antes de esta prueba; no es una regresión nueva de esa condición.

## Repetición `run_10`: GUI pendiente y fracción `0.50`

- Configuración: un dron, misma secuencia GT -> ORB y giro gestionado de `run_09`; `phase6_visual_risk_empty_region_fraction=0.50`, persistencia `3`, máximo `3` inliers y sin RViz2 ni barridos autónomos.
- GUI: seis publicaciones con `global_pending=true`, `stale=false` y pose disponible durante autoridad global pendiente. El bridge dejó `lost_or_unavailable=false`, por lo que tarjetas, escena e inspector muestran `POSE GLOBAL PENDIENTE`/`[GLOBAL PENDIENTE]`, no `PERDIDO`.
- Riesgo: primer sector pobre en frame `1151`, máscara `2`, con fuente ORB, `TRACKING_OK`, continuidad local válida y `global_valid=false`. STOP completado en `5.030 s`; reorientación local a `78.782 deg` completada `5.060 s` después.
- Cierre: `scenario_runner` y wrapper código `0`, `[SIM-DONE] success=true`; 1.572 muestras C7, cuatro eventos y ningún `RECENTLY_LOST/LOST` desde el riesgo hasta el final. Serie, resumen y figura en `run_10/processed/`.
