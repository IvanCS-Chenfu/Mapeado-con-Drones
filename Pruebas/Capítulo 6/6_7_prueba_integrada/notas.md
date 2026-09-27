# Prueba 6.7 - Ejecucion integrada

Dos drones reales completaron dos vueltas simultaneas al edificio en sentidos
contrarios. `dron_1` recorrio oeste-norte-este-sur (antihorario) a `z=1.0 m`;
`dron_2` recorrio este-norte-oeste-sur (horario) a `z=1.3 m`.

La ejecucion usa control GT solo para seguir la trayectoria. El backend de
mapa conserva su configuracion normal: no se cambian algoritmos ni umbrales de
fiduciales, loops, fusion, score, colas, backpressure u optimizacion.

## Ejecucion `c6_7_dos_vueltas_sentidos_contrarios`

- GUI acordada: Gazebo y `multidron_gui`; RViz2 desactivado. La revision de
  video/fotos es una validacion humana separada.
- Instrumentacion pasiva: `debug_fase3_logs_terminal=true` y
  `chapter6_queue_telemetry_enabled=true`.
- Cierre operativo: `[SCENARIO-RUNNER-DONE] success=true`, 24 pasos completos,
  34 goals enviados y `[SIM-DONE] success=true` con codigo `0`.
- Incidencias controladas: 11 eventos `F3L-HARD-FAILURE`, todos con
  `reason=loop_submap_interval_too_small` y `action=continue`; no abortaron la
  ejecucion ni el runner.

## Valores exclusivos de 6.7 para la Tabla 6.3

Esta tabla contiene **solo** valores de
`c6_7_dos_vueltas_sentidos_contrarios`; no incorpora resultados de 6.2.4,
6.3.3, 6.4.x, 6.5.2 ni 6.6.2.

| Validacion de la Tabla 6.3 | Valores medidos en 6.7 | Resultado para redactar |
| --- | --- | --- |
| Cierre visual multidron | 24 pasos y 34 goals completados; video/fotos por validar por observacion humana. | Correcto tecnicamente; validacion visual humana pendiente. |
| Colas y backpressure | 1.109 muestras C6; backpressure activo en 162 muestras; 317 transiciones `F3C-BACKPRESSURE`. | Conseguida. |
| Optimizacion fiducial | 123 observaciones; 4 anchors creados; 10 grafos y 10 optimizaciones; 8 commits atomicos y 2 commits stale. | Conseguida. |
| Anclaje por loop | 1.175 tareas de loop; 1 anchor por loop; 1.328 candidatos BoW; 1.511 RANSAC, 1.205 aceptados y 306 rechazados. | Conseguida. |
| Optimizacion por loop | 137 optimizaciones, con 137 inicios y 137 cierres; 15 barreras de optimizacion. | Conseguida. |
| Concurrencia / stale | Worker secundario: 1.620 tareas procesadas, 582 stale, 1.027 committed y 352 propagaciones de KFs futuros. | Conseguida; 11 rechazos controlados por intervalo demasiado pequeno. |
| Fusion y score | 108 fusiones, 34 reintentos; 8.949 pares; tracks creados/actualizados/retirados: 1.302/1.463/68. Score final min/media/max: 0.0/0.2452/1.0. | Conseguida. |
| Ensayo integrado multidron | F3/C6: 9.425 eventos; fiduciales, loops, anclajes, optimizaciones, fusiones, score, colas y backpressure observados en una ejecucion. | **CONSEGUIDA**. |

El valor de evidencia positiva acumulada del procesador de score es 22.224,
con evidencia negativa 0; se interpreta como suma de los marcadores de score,
no como numero de landmarks unicos.

## Correccion del analisis inicial

El primer reducido usaba una expresion unica que incluia `ERROR` y el helper la
cortaba a las primeras 2.000 coincidencias. Esos errores tempranos ocultaron
los eventos F3 posteriores y produjeron los conteos falsos de 4 loops, 3
RANSAC, 0 optimizaciones y 0 fusiones. El reducido vigente usa un patron por
familia de eventos, con limite alto y sin cola del log bruto. Los valores de
esta pagina proceden exclusivamente de ese reducido corregido.

Fuentes: `datos_procesados/summary.json`,
`datos_procesados/normalizados_f3/summary.json`,
`datos_procesados/fusion_score/summary.json` y el log reducido preservado en
`datos_brutos/`.
