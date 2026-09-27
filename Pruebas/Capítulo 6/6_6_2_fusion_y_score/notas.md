# Prueba 6.6.2 - Fusion de MapPoints y evolucion del score

## Configuracion

- Un dron con fuente `GT` alterna entre `(0,-10,90)` y
  `(-10,-10,90)` durante cuatro visitas al eje sur/oeste, elevando Z `0.3 m`
  en cada pasada: `1.0`, `1.3`, `1.6`, `1.9`, `2.2`, `2.5` y `2.8 m`.
- La subida progresiva de Z es la variacion geometrica acordada tras observar
  que la trayectoria plana generaba demasiados pocos KeyFrames/fusiones; se
  conserva como variante reproducible sin cambiar score ni fusion.
- La repeticion valida debe arrancar un unico dron real en runtime
  (`SIM_DRONE_COUNT_OVERRIDE=1`); los intentos donde el launch levante `dron_2`
  no se procesan como resultado experimental.
- Gazebo y `multidron_gui` activos; sin RViz2, Fase 6 ni protocolo de perdida.
- La mascara/esfera de vecinos esta desactivada. Se mantiene el criterio normal
  de score por distancia fuera del rango de 1 a 5 m.
- Para esta repeticion se duplican solo por launch los parametros de recompensa
  de fusion: `fusion_score_inlier_reward=0.08` y
  `fusion_score_member_bonus=0.08`.
- La repeticion procesable debe activar `debug_fase3_logs_terminal:=true` para
  registrar los marcadores `[F3P-*]` y `[F3R-*]` necesarios.

## Datos esperados

- `[F3P-FUSION]`: actividad de fusion, tracks y miembros raw ocultos.
- `[F3R-FUSED-SCORE-COMMIT]` y `[F3R-RAW-SCORE-COMMIT]`: evidencia y cambios
  de score.
- `[F3R-SCORE-STATS]`: distribucion del score y clasificacion de landmarks.

## Resultado

CONSEGUIDA en la ejecucion
`c6_6_2_fusion_y_score_z_escalonada_1dron_scorex2_f3logs`.

- Trayectoria completada con un unico dron real: el reducido no contiene
  actividad `dron_2`; los 11 pasos del escenario finalizaron con
  `[SCENARIO-RUNNER-DONE]` y `success=true`.
- Z escalonada aplicada: llegada/primera ida a `1.0 m`, vuelta a `1.3 m`, y
  pasadas sucesivas a `1.6`, `1.9`, `2.2`, `2.5` y `2.8 m`.
- Score de fusion duplicado solo por launch: `fusion_score_inlier_reward=0.08`
  y `fusion_score_member_bonus=0.08`.
- Evidencia procesada: 20 eventos de fusion, 19 committed y 1 no-op; 1151 pares
  procesados, 130 tracks creados, 39 actualizados, 2 retirados y 282 raw members
  ocultados.
- Evidencia de score: positiva acumulada `676`, negativa `0`, raw dirty `670`,
  raw updated `236372`.
- Estadistico final de score: revision `379`, tracked `8651`, bad `5799`,
  anchored `2852`, near `7`, far `2303`, min/media/max `0.0/0.146/1.0`.

Intentos conservados pero no usados como resultado: `z_escalonada` levanto
`dron_2`; `1dron_scorex2` completo con un solo dron pero sin logs F3P/F3R por
faltar `debug_fase3_logs_terminal=true`.
