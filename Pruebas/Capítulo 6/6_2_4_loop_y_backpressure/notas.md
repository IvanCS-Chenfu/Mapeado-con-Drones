# Prueba 6.2.4: loop, RANSAC y backpressure

## Objetivo y configuración

Se ejecutaron dos drones bajo fuente de navegacion GT con la misma ruta
horizontal y alturas distintas: D1 a `z=1.0 m` y D2 a `z=1.3 m`.

```text
(0, -10, Z, 90 deg) -> (-10, -10, Z, 90 deg)
-> (-10, 0, Z, 0 deg) -> (-10, 10, Z, 0 deg)
```

La configuracion exacta esta en `configuracion/`. Se activo exclusivamente la
instrumentacion pasiva `chapter6_queue_telemetry_enabled=true`, con periodo de
500 ms. Fase 6 y el protocolo de perdida ORB estuvieron desactivados. Se uso
Gazebo GUI y `multidron_gui`; RViz2 no se lanzo.

El SHA de Git de la ejecucion fue `5e7ab9c1a02606f09422b5d183dfced5c726a376`.
El arbol de trabajo no estaba limpio y su estado se conserva literalmente en
`datos_brutos/metadata_v3.json`.

## Ejecucion valida

La ejecucion valida `c6_2_4_loop_backpressure_v3` termino correctamente con
codigo 0 y `success=true`. Su duracion registrada por el runner fue de
`309.0 s`. La guarda de recursos no intervino: la memoria disponible minima fue
`3002.9 MiB` y el RSS maximo del servidor fue `212.2 MiB`.

El log bruto canonico no se duplica en esta carpeta para evitar una segunda
copia. Se conserva en:

```text
codex/archivos_auxiliares/logs/prueba_c6_2_4_loop_backpressure_v3.log
SHA-256: 65ca0eaab5a10a3e8976c1e87fdf521ed7dc8398ff84777f36e7f3af98d74f60
```

`datos_brutos/eventos_f3_reducidos_v3.log` contiene la reduccion de marcadores
F3 y C6 usada por el parser, y `datos_brutos/recursos_v3.csv` conserva las
medidas del runner. Los CSV normalizados y las figuras se regeneran con los
comandos de `comandos.md`.

## Resultado medido

El pipeline produjo 350 tareas de loop finalizadas, 344 candidatos BoW
acumulados y 170 regiones candidatas. Se realizaron 170 verificaciones RANSAC:
79 aceptadas y 91 rechazadas. Las decisiones relevantes fueron cuatro
`fusion_candidate` y una `optimization_committed`; se iniciaron cinco
optimizaciones, y cada una termino con `committed=true`.

Tambien se conservaron decisiones no aceptadas: 44 `geometry_rejected`, 22
`same_submap_diagnostic`, 19 `waiting_independent_support` y 259 `stale` por
cambio de revision de score. No se eliminan de los datos porque son parte del
comportamiento observado del pipeline.

La telemetria contiene 517 muestras. El maximo de la cola primaria fue 58,
frente a watermark alto/bajo de 8/2. La cola secundaria critica alcanzo 60,
sin superar su watermark alto de 64. El backpressure estuvo activo en 336
muestras. Sus tres activaciones comenzaron con la cola primaria en 8; la
primera coincidio ademas con 16 tareas criticas secundarias. Se desactivo al
caer la primaria a 2 y vaciarse la secundaria. Las 36 muestras con una
optimizacion activa tuvieron asimismo backpressure activo.

Por tanto, esta ejecucion demuestra que el backpressure se correlaciona con la
presion primaria definida por el codigo y se mantiene durante optimizacion. No
demuestra una activacion causada exclusivamente por rebasar el watermark alto
de la cola secundaria, porque su maximo observado fue 60 de 64.

## Observacion de latencia del flujo principal

La visualizacion mostro que los KeyFrames aparecian despues de que los drones
hubieran recorrido el tramo correspondiente. Los marcadores del flujo primario
lo confirman: entre `F3C-PRIMARY-ENQUEUE` y `F3C-PRIMARY-START` hubo una espera
media de `12.353 s`, mediana de `7.923 s`, percentil 95 de `40.873 s` y maximo
de `44.355 s`. Por tanto, el retraso visible ocurre principalmente antes de
que el unico worker principal pueda atender el delta, no en el transporte ROS
de `MarkerArray`.

Una vez iniciado un input, `RAW-COMMIT` hasta `GLOBALMAP-PUBLISH` duro en media
`0.129 s` (mediana `0.087 s`), aunque hubo dos picos de `4.943 s` y `5.668 s`
en el periodo de optimizaciones. El procesamiento completo de un input tuvo
media `0.682 s`, p95 `2.962 s` y maximo `6.653 s`.

La configuracion efectiva mantenia `score_drone_body_mask_enabled=true`. En
`InsertDelta()`, mientras se posee `state_commit_mutex_`, se ejecutan
`RefreshGeometryScores()` y `RefreshDroneBodyMasks()`. El primero expande los
voxeles vecinos y recalcula el score de los MapPoints afectados. Esta ejecucion
registro una media de 896 `input_updated` y 828 `updated` por delta, con picos
de 3856 y 3795 respectivamente. Despues esos cambios se marcan dirty para el
builder. Es la causa observada de que la FIFO primaria no se drene en tiempo
real; la cola secundaria no es esperada explicitamente por el worker primario,
pero sus commits comparten el mutex de estado y explican de forma compatible
los picos durante optimizacion.

La prueba sigue siendo valida para el pipeline loop/RANSAC/backpressure, pero
no debe presentarse como evidencia de publicacion de KeyFrames en tiempo real.
Para aislar el Capitulo 6 en una repeticion se debe lanzar con
`score_drone_body_mask_enabled:=false`. Eso evita este calculo de score y la
esfera fisica, sin cambiar BoW, RANSAC, colas ni optimizacion. La correccion
arquitectonica permanente seria diferir o acotar el recalculo de score fuera de
la ruta critica de `InsertDelta()`; no se ha aplicado porque altera el alcance
funcional de las pruebas actuales.

## Figuras y capturas

- `figuras/pipeline_loop.{png,pdf}`: acumulados de candidatos BoW y RANSAC.
- `figuras/pipeline_decisiones.{png,pdf}`: regiones en el panel superior y
  decisiones relevantes/optimizaciones en el inferior, con escalas legibles e
  independientes de las series de alto volumen.
- `figuras/colas_y_optimizacion.{png,pdf}`: colas, watermarks y periodos de
  optimizacion.
- `figuras/backpressure.{png,pdf}`: estado binario de backpressure.

Para una captura manual del mapa global, los instantes de interes de la
ejecucion, medidos desde el primer marcador F3, fueron aproximadamente: primer
RANSAC rechazado `37.65 s`, primera optimizacion `237.74 s`, y final de la
ultima optimizacion `294.16 s`.

## Intentos anteriores

- `c6_2_4_loop_backpressure`: no conseguida. El perfil se paso con una ruta
  relativa y el launch lo resolvio desde otro directorio antes de iniciar
  Gazebo. No genero datos experimentales.
- `c6_2_4_loop_backpressure_v2`: cancelada por peticion del usuario a los
  113 s, porque abria RViz2. Sus datos no se procesan ni respaldan conclusiones.

La prueba 6.2.4 se considera conseguida: hay candidatos, RANSAC aceptado y
rechazado, decisiones de loop, optimizaciones y una serie temporal densa de
colas y backpressure. La posible optimizacion por loop de regiones ya ancladas
se analizara especificamente como evidencia reutilizable en 6.4.2.
