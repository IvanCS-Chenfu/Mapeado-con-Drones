# Subfase 4D - Configuracion remota, worker fiducial y estimacion `camera_T_tag`

## Estado

```text
CONSEGUIDA
Preparacion: cerrada
Acuerdo cerrado: si
Autorizacion funcional: concedida y consumida
```

## Detalle largo

```text
subfases/detalle/subfase_4D_DETALLE.md
```

## Dependencias

`4A` y `4C` conseguidas.

## Objetivo

Implementar la percepcion visual en Dron sin bloquear el flujo principal:
configuracion minima Server->Dron, cola acotada, worker unico, deteccion
AprilTag, PnP/IPPE y metricas de calidad por tag, produciendo observaciones
internas todavia previas al contrato ROS final de 4E.

## Decisiones activas

- la configuracion del detector llega mediante servicio del Servidor;
- los KFs anteriores a `READY` no se reprocesan;
- se clona la imagen (`cv::Mat::clone()`) antes de encolarla;
- la cola es acotada y descarta trabajo viejo segun la politica acordada;
- defaults operativos: retry `1 s`, timeout `2 s`, capacidad `4` y
  `drop oldest / keep newest`;
- el wrapper solo conoce `tag_id -> size_m` y parametros de detector;
- baseline: `APRILTAG_36H11`, `SUBPIX`, `IPPE_SQUARE` y reproyeccion maxima
  `3.0 px`;
- IDs desconocidos no producen observaciones funcionales;
- superar `max_reprojection_error_px` invalida la observacion visual;
- el rango configurable `1-5 m` no filtra en Dron: 4G decidira en Servidor;
- `quality_score = clamp(1 - error_px / max_error_px, 0, 1)` como formula
  inicial explicable, conservando las metricas originales;
- no se usa GT para seleccionar soluciones PnP.

## Debug visual acordado

- flag `debug_fiducial_visualization`, default `false`;
- una ventana por dron en un proceso ROS `fiducial_visualizer` separado;
- el wrapper publica `orbslam/fiducial_debug/image` con QoS latest-only y no
  contiene HighGUI;
- muestra el KF mas reciente durante `debug_fiducial_display_seconds=5.0` y
  reinicia el contador si llega otro;
- dibuja todos los tags decodificados: validos en verde y
  rechazados/desconocidos en rojo con motivo y `tag_id`;
- excluye candidatos no decodificados, que no tienen `tag_id`;
- fallo headless, cierre normal o muerte forzada de HighGUI quedan aislados de
  `stereo` y del SLAM;
- con debug apagado no se crea publisher, proceso visual ni anotacion especifica.

## Archivos probables al ejecutar

- interfaces minimas de configuracion en `orbslam3_msgs` y replica controlada;
- nodo/servicio de configuracion en Servidor;
- wrapper y componente `FiducialDetector` en Dron;
- launch/config para pasar el perfil correcto;
- tests y metadata de `system_architecture`.

## Prohibido

- ejecutar deteccion sincronamente dentro de `GrabStereo()`;
- hacer hot-reload en Fase 4;
- agrupar tags por cubo en el wrapper;
- publicar todavia el contrato final de observaciones de 4E;
- esconder candidatos malos o elegir con GT.

## Pruebas requeridas

Servicio ausente/timeout/configuracion vacia, misma config para dos drones, KF
antes de `READY`, tag frontal, multiples tags, ausencia de tag, ID desconocido,
tamanos distintos, cola llena, flujo de deltas durante carga, IPPE con
candidatos, intrinsecos efectivos, debug visual valido/rechazado/headless y
guardas de `orbslam3_msgs`/`system_architecture`. La prueba runtime usa la
trayectoria tipica revisada, Gazebo/RViz2 y `system_architecture` activo; despues
se ejecuta un smoke corto con el grafo apagado.

## Criterio de exito

El wrapper procesa solo KFs validos en worker no bloqueante y obtiene por tag
`camera_T_tag`, `quality_score`, `reprojection_error_px`, `tag_area_px2` y
metricas trazables, con build/pruebas/logs/documentacion sincronizados.

## Resultado

Servicio, cliente, worker, detector, PnP y metricas quedan implementados y
validados. HighGUI se aislo en un proceso separado tras los fallos 205/206. La
prueba 208 completa confirma publicaciones, recepcion, timeouts y continuidad
de ambos wrappers; el usuario acepta el resultado y da la subfase por cerrada.
Evidencia: `historial/por_subfase/historial_4D_RESUMEN.md`.
