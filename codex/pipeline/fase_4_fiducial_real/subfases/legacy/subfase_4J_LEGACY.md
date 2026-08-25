# LEGACY — Antigua Subfase 4J — Rechazos, inconsistencias y funcionamiento degradado seguro

Este contrato ya no pertenece al flujo activo de Fase 4. La revision importada
desde `Fase_4_completa_4A_4I_muy_detallada.zip` distribuye la robustez en las
subfases propietarias 4A-4H y deja 4I como regresion final con perfil
ESP32-CAM simulado. Se conserva solo como referencia historica.

## Estado

```text
legacy
```

## Dependencia

`4I`.

## Objetivo técnico

Endurecer la cadena visual para que detecciones pobres, tags desconocidos, ambigüedades planares, timestamps/epochs incorrectos o contradicciones entre varias caras/cubos no produzcan anchors/optimizaciones falsas y nunca bloqueen ORB-SLAM3 ni el transporte sparse-global.

Esta subfase caracteriza límites y añade rechazos basados en la propia observación visual/configuración, no en GT.

## Comportamiento esperado

Una observación puede clasificarse como:

```text
accepted
rejected_unknown_tag
rejected_partial_or_geometry
rejected_reprojection
rejected_pose_ambiguity
rejected_timestamp
rejected_epoch
rejected_multiface_inconsistent
rejected_multiobject_inconsistent
```

El resto del batch debe poder continuar cuando un tag aislado es inválido y existe evidencia suficiente para separar el outlier.

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
subfases/subfase_4D.md ... subfase_4I.md
config detector del wrapper
config fiducial del servidor
logs/reductores de Fase 3
```

## Diagnóstico de partida

OpenCV puede detectar correctamente la mayoría de tags bien observados, pero un marcador planar es sensible a tamaño aparente, desenfoque, oblicuidad, oclusión y precisión de esquinas. La arquitectura multi-cara/multi-cubo añade una ventaja: observaciones simultáneas pueden comprobarse entre sí, pero también pueden revelar configuraciones o frames mal definidos.

No existe todavía una envolvente medida de distancia/ángulo/iluminación para este proyecto.

## Archivos permitidos a modificar

```text
wrapper detector/config fiducial
src/orbslam3_server/config/fiducials.yaml
src/orbslam3_server/src/global_map_server.cpp
src/orbslam3_multi/src/fiducial_anchor_manager.cpp        # solo reglas de rechazo ya normalizadas
src/simulacion_dron/config/fiducials.yaml
src/simulacion_dron/launch/multi_dron.launch.py
codex/archivos_auxiliares/trayectorias/                    # pruebas específicas
codex/contexto/paquetes afectados/
```

## Archivos prohibidos

```text
ORB_SLAM3/Tracking policy
loop detector para compensar fiduciales
GT como filtro runtime
legacy no relacionado
```

## Funciones, clases o nodos que hay que localizar

```text
FiducialDetector / equivalente de 4D
callback batch de 4F
agrupación de 4G
RegisterFiducialObservation
scenario_runner_node
```

## Cambios requeridos

1. Fijar y documentar filtros mínimos de detector: tag completo, cuatro esquinas válidas, área mínima, PnP válido, Z positiva y `reprojection_error_px` máximo.
2. Medir error y tasa de detección antes de fijar thresholds definitivos; no elegir valores para “hacer pasar” un escenario concreto.
3. Para IPPE/planar ambiguity, evaluar las soluciones disponibles cuando sea necesario y rechazar una solución espejo/incompatible en vez de aceptarla por defecto.
4. Rechazar `tag_id` desconocido sin invalidar otros tags válidos del mismo KF.
5. Rechazar mismatch de timestamp/KF o epoch sin reasociar.
6. Si dos caras del mismo cubo producen `world_T_camera` incompatibles, marcar el grupo como inconsistente; no fusionar ni elegir una por error de reproyección si la discrepancia geométrica excede el límite.
7. Si cubos distintos del mismo KF producen poses globales incompatibles, usar política robusta acordada: no crear anchor con un conjunto contradictorio; conservar diagnóstico por objeto.
8. Añadir counters por razón de rechazo y latencia del detector.
9. Verificar que una excepción, imagen vacía o detector sin tags no impide `TrackStereo`, `pose_local`, `orb_map_delta` ni el servicio de snapshot.
10. Añadir modo debug opcional de imagen anotada (topic o archivo temporal) con corners/ID/ejes, desactivado por defecto y fuera del contrato funcional.
11. Mantener GT exclusivamente para comparar el error final en un proceso de métricas externo al filtro funcional.
12. Caracterizar al menos: frontal, oblicuo, lejos, oclusión parcial, iluminación distinta, motion blur razonable y múltiples tags.
13. Probar falsos positivos/IDs incorrectos con superficies no fiduciales y tags deliberadamente no configurados.
14. Medir memoria de pendientes y asegurar que el detector no genera backlog si los KFs aparecen más rápido de lo esperado.

## Cambios prohibidos

- No usar GT para decidir si una detección “es buena”.
- No aceptar una pose solo porque está cerca de la esperada por ORB.
- No aumentar el tamaño físico configurado del tag para compensar errores de escala.
- No ignorar transformaciones multi-cara inconsistentes.
- No bloquear el callback hasta obtener una detección.
- No publicar imágenes debug permanentemente por defecto.

## Paquetes a compilar

```bash
./codex/herramientas/build_selected_packages.sh orbslam3_msgs orbslam3_multi orbslam3_server <paquete_wrapper_real> simulacion_dron
```

Reducir la lista si una iteración concreta solo toca un subconjunto.

## Pruebas Gazebo requeridas

### Prueba 1 — Frontal y oblicua

Observar el mismo tag desde una vista frontal y varias oblicuas crecientes. Registrar tasa de detección, error de reproyección y pose.

### Prueba 2 — Distancia

Alejar el dron progresivamente. Determinar hasta qué rango la detección sigue siendo utilizable con el tamaño/resolución actuales.

### Prueba 3 — Oclusión parcial

Ocultar parte del tag. Una detección incompleta/ambigua no debe crear anchor falso.

### Prueba 4 — Varias caras/cubos incoherentes

Introducir de forma controlada una orientación/pose errónea en una copia de YAML de prueba. El servidor debe detectar la contradicción y rechazar el grupo afectado.

### Prueba 5 — Pérdida total del detector

Deshabilitar detector o presentar solo escena sin tags durante una trayectoria. ORB-SLAM3/mapa deben continuar.

### Prueba 6 — Falso ID/no configurado

Mostrar un tag válido de la familia pero no presente en la configuración. Debe aparecer `unknown_tag` y no anchor.

## Patrones de reducción de logs

```text
FID-REJECT|FID-DETECT|FID-OBJECT|unknown_tag|reprojection|ambigu|partial|inconsistent|detect_ms|orb_map_delta|tracking_state|ERROR|FATAL|Segmentation fault|Killed
```

## Criterio de éxito

1. Todos los rechazos críticos tienen razón explícita y contador.
2. Ninguna detección inválida conocida crea anchor/task.
3. Tags válidos del mismo batch sobreviven a un outlier aislado cuando la política permite separarlo.
4. Multi-cara/multi-cubo incoherente no se fusiona silenciosamente.
5. La pérdida total del detector no bloquea SLAM ni mapa.
6. La envolvente de detección queda medida y documentada.
7. GT no participa en filtros funcionales.

## Criterio de fallo o parcial

- `NO CONSEGUIDA`: falsos anchors, detector puede bloquear tracking o no hay trazabilidad de rechazos.
- `PARCIAL`: robustez básica funciona pero falta una parte de la matriz de pruebas/medición.
- `BLOQUEADA`: el simulador no permite reproducir una condición crítica y se necesita un recurso externo no disponible.

## Riesgos

- sobreajustar thresholds a Gazebo;
- confundir error de configuración con ruido del detector;
- latencia creciente por debug images;
- aceptar la solución IPPE espejo.

## Documentación a actualizar

```text
codex/contexto/paquetes/<paquete_wrapper_real>/
codex/contexto/paquetes/orbslam3_server/
codex/contexto/paquetes/orbslam3_multi/
codex/contexto/paquetes/simulacion_dron/
codex/pipeline/fase_4_fiducial_real/historial/por_subfase/historial_4J.md
codex/pipeline/fase_4_fiducial_real/historial/por_subfase/historial_4J_RESUMEN.md
```
