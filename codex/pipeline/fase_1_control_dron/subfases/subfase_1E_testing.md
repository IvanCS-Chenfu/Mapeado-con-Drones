# Subfase 1E — Build, pruebas y criterios

Este archivo complementa `subfase_1E.md`.

## Estado de la subfase

```text
realizado
```

## Paquetes a compilar

```bash
./codex/herramientas/build_selected_packages.sh simulacion_dron dron_individual
```

Si falla:

```bash
./codex/herramientas/reduce_build_log.sh
```

## Prueba 1 — Ground Truth de un dron

YAML:

```yaml
sensores.ground_truth.publish_rate: 50.0
fisico.brazos.numero: 4
```

Arranque:

```bash
./codex/herramientas/run_simulation.sh \
  --prueba fase1_1E_gt \
  --launch "ros2 launch simulacion_dron multi_dron.launch.py" \
  --post-scenario-wait-sec 10
```

Comprobar:

```bash
ros2 topic info /dron_1/sensor/GT/pose -v
ros2 topic info /dron_1/sensor/GT/vel -v
ros2 topic info /dron_1/sensor/GT/acc -v
ros2 topic hz /dron_1/sensor/GT/pose
ros2 topic echo --once /dron_1/sensor/GT/pose
```

Resultado esperado:

- stamps de simulación crecientes;
- `frame_id=world` con `frame_propio=false`;
- pose y velocidad finitas;
- aceleración disponible después de la primera muestra;
- frecuencia próxima a la configurada, tolerando carga de simulación.

## Prueba 2 — Cámara mono en RViz2

YAML:

```yaml
sensores.camara.porcentaje_estereo: 0.0
sensores.camara.publish_rate: 30.0
sensores.camara.mostrar_gazebo: "false"
```

Comprobar:

```bash
ros2 topic hz /dron_1/sensor/camara_mono/image_raw
ros2 topic echo --once /dron_1/sensor/camara_mono/camera_info
```

En RViz2:

1. añadir display `Image`;
2. elegir `/dron_1/sensor/camara_mono/image_raw`;
3. confirmar imagen actualizada y no congelada;
4. registrar frame, resolución y frecuencia observada.

## Prueba 3 — Cámaras estéreo en RViz2

YAML:

```yaml
sensores.camara.porcentaje_estereo: 0.3
sensores.camara.publish_rate: 30.0
```

Abrir dos displays `Image` para izquierda y derecha. Deben verse dos puntos de vista distintos y ambos `camera_info` deben existir.

No declarar calibración estéreo: esta prueba solo valida generación y transporte de imágenes.

## Prueba 4 — Aislamiento con dos drones

Con `dron.numero: 2`, comprobar que existen topics separados para `dron_1` y `dron_2`. RViz2 debe poder seleccionar una imagen de cada dron sin mezclar nombres.

## Prueba 5 — Cambio de frecuencia

Cambiar temporalmente `publish_rate` de GT o cámara a otro valor seguro y comprobar que el topic responde al YAML. Restaurar el baseline después.

## Patrones de reducción de logs

```text
PluginSensorGroundtruth|Publicando pose|Publicando velocidad|Publicando aceleración|camera_controller|camara_|image_raw|camera_info|rate|no encuentro link|ERROR|FATAL|Segmentation fault|Killed
```

## Criterio de éxito

1. Build devuelve `0`.
2. Las cinco pruebas obligatorias se ejecutan.
3. GT tiene frame y timestamps coherentes.
4. Las frecuencias responden al YAML.
5. Mono y estéreo publican los topics esperados.
6. Las imágenes son visibles en RViz2.
7. Dos drones están aislados por namespace.
8. No aparecen errores graves no explicados.
9. El historial futuro registra la observación visual.
10. La documentación indica que GT será sustituido en Fase 5.

## Criterio de fallo o parcial

- `NO CONSEGUIDA`: plugin no carga, faltan topics, imágenes vacías/no visibles o mezcla entre drones.
- `PARCIAL`: GT funciona pero cámaras no, o viceversa; o falta una configuración mono/estéreo.
- `BLOQUEADA`: falta el plugin de cámara de Gazebo ROS o no hay entorno gráfico disponible para la validación visual.

## Evidencia a guardar al ejecutar

```text
comandos exactos
YAML exacto
frecuencias medidas
un mensaje de cada interfaz
capturas RViz2
resultado del build
log reducido
conclusión CONSEGUIDA/PARCIAL/NO CONSEGUIDA/BLOQUEADA
```

Nunca copiar el log completo al contexto.
