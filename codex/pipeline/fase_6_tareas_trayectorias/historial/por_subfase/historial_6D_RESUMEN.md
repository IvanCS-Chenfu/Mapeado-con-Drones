# Resumen - 6D

Estado: PARCIAL. La evidencia historica reversible se conserva como antecedente,
pero el contrato vigente de 6D es runtime local: `task_manager` acepta una
orden correlacionada, la ejecuta con su action ordinaria y devuelve un unico
terminal sin retener al servidor. `TRACKING_RISK` mantiene STOP/reorientacion
local y devuelve `RESULT_ABORTED` sin perder depth valido. La interfaz incorpora
`MOVE_AND_WATCH_FIDUCIAL` y `fiducial_seen`, dejando la deteccion/identidad a
Fase 4. Build de las dos replicas `mission_msgs`, `task_manager` y
`task_server` correcto; CTest de `task_server` 9/9. Falta que
`POINT_SELECTION` produzca ordenes reales y una prueba Gazebo integrada. La
prueba 771 repitio el flujo GT real de fiducial 2 y coverage durante 180 s con
ambas puertas de depth habilitadas: `SIM-DONE success=true`, sin
`depth_disabled` y con actualizacion `x500_depth` de la capa navegable. La
revision posterior confirma que no hubo `CaptureDepth`, `DEPTH_INTEGRATION` ni
terminal autonomo: no se valido depth de fachada. El primer intento
de 771 queda conservado como fallo mecanico de configuracion porque ambas
puertas estaban inicialmente en `false`; no fue un bloqueo de workers. Esta
prueba es regresion de flujo legacy, no la validacion directa del productor
runtime 6D pendiente.

La prueba 605 observo snapshots voxel reales en GUI y termino con exit 0. La
620 confirmo en Gazebo que las revisiones globales de KF reintegran `FREE`; no
valido aun una ruta D* visual completa hacia el objetivo solicitado.

La optimizacion 624/625 confirma ademas la capa navegable derivada por perfil:
la pasada inicial recalculaba transitabilidad de forma repetida por arista y
tardo 16.060 ms para 24 cambios raw/1.056 celdas. La pasada doble vigente
calcula transitabilidad una vez y luego compone las conexiones 26, reduciendo
un caso equivalente a 110.169 ms para 24 cambios/1.122 celdas. Los lotes
grandes siguen siendo locales, aunque el mayor observado (988 cambios/34.703
celulas) tardo 2.633 s; queda como referencia para futuras optimizaciones de
coalescencia, no como reconstruccion global.

Optimizacion posterior pendiente de prueba integrada: cada perfil conserva
macro-voxeles derivados de `4x4x4` celdas finas por defecto. Sus deltas se
actualizan localmente junto a la capa navegable y se entregan al planificador
como guia no autoritativa; la seguridad sigue resolviendose en la malla fina.
La prueba 629 lo ejercio con GT y GUI F7: los deltas gruesos se propagaron y D1
acepto un plan remoto sin reconstruccion global. La capa macro queda CONSEGUIDA;
el estado agregado sigue PARCIAL solo por depth metrico real aplazado.

La prueba 664 valida una mejora posterior: el worker recibe delta global
coherente, agrupa 100 ms y aplica fuentes sparse sin reset. Sus conteos inversos
de influencia redujeron 828 cambios/50.886 celdas a 2.143 s; lotes de 2--12
quedaron entre 14 y 89 ms, frente a los 13--45 s observados antes en rafagas
similares. Falta una resincronizacion snapshot ejercida y depth metrico real.

Actualización 678: `ReversibleVoxelMap` exige cuatro identidades de MapPoint
distintas cualificadas antes de materializar `OCCUPIED`. El índice sparse es
reversible ante repetición, movimiento, delete o cambio de score; `task_lib`
pasó 9/9 y `task_server` 7/7. La simulación GT propagó
`min_occupied_mappoints=4` y terminó `SIM-DONE success=true`. El filtro queda
CONSEGUIDO; el estado agregado conserva `PARCIAL` por depth métrico y resync
global real pendientes.

Decisión posterior: el filtro actual (`score >= 0.2` y cuatro identidades) es
transitorio. La siguiente recalibración deberá hacerse coordinadamente con 3R:
el score seguirá siendo confianza individual de MapPoint y 6D sumará de forma
reversible las contribuciones dentro de cada voxel para obtener un
`voxel_occupancy_score` con umbral propio. No se modifica código ni parámetros
en esta iteración; la migración exigirá pruebas de suma e inversión exactas.
