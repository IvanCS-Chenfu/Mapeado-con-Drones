# Resultado de la prueba 5.1: configuración de cámaras

## Identificación

- **Prueba:** `c5_5_1_camera_info_v2`
- **Fecha:** 21/09/2026
- **Dron:** `dron_1`
- **Mundo:** `empty`
- **Propósito:** comprobar los mensajes `CameraInfo` y contrastarlos con el
  modelo `.xacro`, la calibración y el YAML de ORB-SLAM3.
- **Código del proyecto modificado:** no.
- **Build realizado:** no era necesario; la prueba es documental.

## Ejecución

Se inició una simulación mínima con un solo dron. Se desactivaron ORB-SLAM3,
el servidor global, la Fase 6, RViz y las interfaces gráficas para aislar la
publicación de las cámaras. Se consultaron los tópicos:

```text
/dron_1/sensor/camara_izq/camera_info
/dron_1/sensor/camara_der/camera_info
/dron_1/sensor/camara_izq/image_raw
/dron_1/sensor/camara_der/image_raw
```

La frecuencia observada en `image_raw` fue aproximadamente `20 Hz` en ambas
cámaras. Las dos `CameraInfo` se recibieron con resolución `480x360`,
intrínsecas idénticas y distorsión nula. Los mensajes completos conservados son
`camera_info_izq.yaml` y `camera_info_der.yaml`.

## Resultado

**CONSEGUIDA.** La configuración efectiva de las dos cámaras coincide con el
YAML de ORB-SLAM3 seleccionado por el launch de `dron_individual`:

- `fx=fy=286.02185016085167`;
- `cx=240.5`, `cy=180.5`;
- resolución `480x360`;
- frecuencia `20 Hz`;
- distorsión nula;
- línea base `0.057 m`, coherente con `Camera.bf`.

La prueba también deja documentada una observación: el `frame_id` publicado por
`CameraInfo` es `cuerpo`, aunque el plugin del `.xacro` declara como `frameName`
el frame óptico de cada cámara. No se ha cambiado nada del proyecto; queda como
posible punto de revisión posterior si se necesita verificar la semántica TF.

## Incidencias

El primer intento no llegó a iniciar Gazebo porque el ejecutor tenía una ruta
por defecto antigua para `mission_profile`. Se repitió la misma prueba con la
ruta actual explícita y terminó correctamente. Durante el apagado del segundo
intento, `fiducial_spawner` informó de un contexto ROS invalidado; no afectó a
la publicación de cámaras ni a los datos recogidos.

## Evidencia

- Log completo: `codex/archivos_auxiliares/logs/prueba_c5_5_1_camera_info_v2.log`
- Log reducido: `codex/archivos_auxiliares/logs/prueba_c5_5_1_camera_info_v2.reduced.log`
- Mensaje izquierdo: `camera_info_izq.yaml`
- Mensaje derecho: `camera_info_der.yaml`
