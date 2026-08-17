# Subfase 4L — Validación con cámara real y cierre de Fase 4

## Estado

```text
sin hacer
```

## Dependencia

`4K`.

## Objetivo técnico

Validar que la parte visual diseñada para Gazebo no depende de artefactos del simulador: generar/imprimir tags físicos con los mismos IDs y dimensiones, observarlos con una cámara real calibrada y comprobar que el detector produce IDs/poses razonables bajo variaciones de distancia, ángulo e iluminación.

No se exige construir un dron físico completo. La prueba mínima es de cámara real + fiducial físico. Si existe una cámara estéreo compatible con el wrapper, puede ampliarse a la cadena completa; si no existe hardware disponible, esta subfase debe quedar `BLOQUEADA` en esa parte y no sustituirse por evidencia simulada fingiendo validación física.

## Comportamiento esperado

- Los mismos assets/IDs de 4A/4B pueden imprimirse a escala física conocida.
- La calibración usada por PnP corresponde a la cámara real.
- El detector reconoce tags físicos sin cambiar la lógica principal.
- Las poses relativas cambian de forma coherente con movimientos medidos de cámara/tag.
- El cierre de fase no introduce GT como sustituto de medición visual.

## Contexto obligatorio a leer


```text
AGENTS.md
codex/contexto/00_CONTEXTO_COMPACTACION.md
codex/contexto/CONTEXTO_MINIMO_ACTUAL.md
codex/contexto/08_POLITICA_TOKENS_DOCUMENTACION.md
codex/pipeline/PIPELINE_MAESTRO.md
codex/pipeline/fase_4_fiducial_real/pipeline_fase_4_RESUMEN.md
codex/pipeline/fase_4_fiducial_real/pipeline_fase_4.md
```


Además:

```text
subfases/subfase_4A.md ... subfase_4K.md
config/calibración del wrapper o herramienta de cámara real
scripts de generación de tags de 4B
documentación de cámara disponible, si existe
```

## Diagnóstico de partida

Hasta 4K toda la evidencia puede proceder de Gazebo. Una textura perfecta y una cámara simulada no reproducen desenfoque, exposición, impresión, reflexión o distorsión real. El propósito de 4L es comprobar transferibilidad del detector, no certificar un sistema de vuelo físico.

## Archivos permitidos a modificar

```text
script generador de tags de 4B
config local del detector wrapper
config/calibración específica de cámara real
scripts de test/offline para cargar imágenes reales
codex/contexto/paquetes/<paquete_wrapper_real>/
codex/pipeline/fase_4_fiducial_real/
```

No modificar el detector solo para una foto concreta; cualquier cambio debe seguir pasando 4J/4K.

## Archivos prohibidos

```text
GT de Gazebo como evidencia física
ORB_SLAM3 tracking policy
worlds/modelos para maquillar resultados reales
logs históricos inventados
```

## Funciones, clases o nodos que hay que localizar

```text
FiducialDetector de 4D
generador de texturas/tags de 4B
carga de K/distorsión del wrapper
entrada de cámara real disponible
```

Si no existe una ruta para alimentar imágenes offline al detector sin arrancar ORB-SLAM3, se puede crear un test ejecutable pequeño que reutilice exactamente la misma clase detector, no una implementación paralela.

## Cambios requeridos

1. Generar archivos imprimibles de al menos los tags usados en la prueba física desde la misma familia/ID que simulación.
2. Imprimir/montar el tag con tamaño físico medido; registrar la medida real utilizada por PnP.
3. Obtener o realizar calibración de cámara real (`fx,fy,cx,cy` y distorsión) de forma reproducible.
4. Reutilizar el mismo `FiducialDetector` de 4D sin fork específico para la prueba.
5. Probar frontal, varias distancias, varias oblicuidades y al menos dos condiciones de iluminación razonables.
6. Probar dos tags simultáneos si físicamente es sencillo, para validar la salida `0..N` fuera de Gazebo.
7. Registrar `tag_id`, `camera_T_tag`, reprojection error y coste sin usar una pose GT artificial.
8. Cuando exista una distancia física medible con cinta/regla, compararla como métrica externa y documentar tolerancia observada; no usar esa medida para corregir la pose runtime.
9. Si existe cámara estéreo compatible, ejecutar opcionalmente el wrapper completo y confirmar un KF con observación real. No convertir esta ampliación en requisito si el hardware acordado es solo monocular para validar detector.
10. Reejecutar los smoke/regresiones de 4J/4K tras cualquier ajuste de detector derivado de la cámara real.
11. Actualizar `pipeline_fase_4_RESUMEN.md` y estado global solo con evidencia real.

## Cambios prohibidos

- No cambiar `tag_size_m` hasta que “salga” la distancia esperada; debe ser la medida física real.
- No seleccionar manualmente corners en las imágenes de validación.
- No reemplazar el detector por otro solo para la prueba física sin repetir regresiones.
- No declarar validación física si solo se usaron screenshots de Gazebo.
- No requerir Bluetooth/telemetría física del dron: esta subfase valida visión fiducial/cámara, no comunicaciones de hardware.

## Paquetes a compilar

Compilar cualquier paquete tocado por ajustes finales. Como baseline:

```bash
./codex/herramientas/build_selected_packages.sh \
  orbslam3_msgs orbslam3_multi orbslam3_server simulacion_dron <paquete_wrapper_real>
```

Para un test offline del detector, compilar solo el wrapper/componente que lo contiene antes de la regresión completa.

## Pruebas requeridas

### Prueba 1 — Cámara real frontal

Capturar/usar stream real con el tag de tamaño conocido. Debe detectarse ID correcto y pose con Z positiva/finita.

### Prueba 2 — Distancia y oblicuidad

Mover cámara/tag a varias distancias/ángulos medibles. Registrar error/reproyección y detectar la envolvente donde deja de ser fiable.

### Prueba 3 — Iluminación

Repetir bajo al menos dos iluminaciones razonables sin cambiar la familia/ID.

### Prueba 4 — Multi-tag real

Presentar dos tags simultáneamente. El detector debe devolver dos observaciones independientes en el mismo frame de prueba.

### Prueba 5 — Regresión simulada final

Repetir los tests críticos de 4J y la prueba integral 4K después de cualquier ajuste real.

## Patrones de reducción de logs

```text
FID-DETECT|FID-POSE|tag_id|reprojection|detect_ms|REAL-CAMERA|calibration|4K|SCENARIO-RUNNER|ERROR|FATAL|Segmentation fault|Killed
```

## Criterio de éxito

1. El detector reconoce correctamente tags físicos con la misma familia/IDs.
2. `camera_T_tag` es finita y coherente con movimientos/distancias externas razonables.
3. Se conoce la calibración real usada.
4. La salida multi-tag funciona con cámara real cuando se prueba.
5. Cualquier ajuste derivado de real mantiene las regresiones simuladas.
6. No se usa GT como input funcional.
7. Documentación e historial contienen evidencia real y limitaciones.
8. Solo entonces Fase 4 puede marcarse `realizado` si las subfases anteriores también están conseguidas.

## Criterio de fallo o parcial

- `NO CONSEGUIDA`: detector solo funciona en Gazebo, escala física incoherente o regresión simulada tras ajustes.
- `PARCIAL`: validación real básica funciona pero falta parte de la matriz acordada.
- `BLOQUEADA`: no hay cámara/impresión/calibración física disponible. En ese caso no inventar evidencia ni sustituirla por simulación.

## Riesgos

- impresión a escala incorrecta;
- calibración de otra resolución;
- autofocus/motion blur;
- reflejos y baja exposición;
- ajustar el detector demasiado a una única cámara.

## Documentación a actualizar

```text
codex/contexto/00_CONTEXTO_COMPACTACION.md
codex/contexto/01_ESTADO_ACTUAL.md
codex/contexto/paquetes/<paquete_wrapper_real>/
codex/contexto/paquetes/orbslam3_server/
codex/contexto/paquetes/orbslam3_multi/
codex/contexto/paquetes/simulacion_dron/
codex/pipeline/fase_4_fiducial_real/pipeline_fase_4_RESUMEN.md
codex/pipeline/fase_4_fiducial_real/historial/por_subfase/historial_4L.md
codex/pipeline/fase_4_fiducial_real/historial/por_subfase/historial_4L_RESUMEN.md
```
