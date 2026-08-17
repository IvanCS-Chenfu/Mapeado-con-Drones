# Subfase 4K — Integración multi-dron y validación completa en simulación

## Estado

```text
sin hacer
```

## Dependencia

`4J`.

## Objetivo técnico

Validar de extremo a extremo la Fase 4 con al menos dos drones, varios KeyFrames, varios cubos y observaciones simultáneas de uno o más tags. Debe demostrarse que la identidad de cada observación permanece aislada por `(drone_id, map_epoch, local_keyframe_id)` y que los submapas pueden anclarse/corregirse utilizando solo visión fiducial y calibración conocida.

GT puede registrarse únicamente para métricas externas posteriores.

## Comportamiento esperado

- Cada dron mantiene su epoch/KF sin contaminación cruzada.
- Dos drones pueden observar el mismo cubo en KFs diferentes.
- Un KF puede ver dos caras del mismo cubo y/o tags de cubos distintos.
- Primeras observaciones válidas anclan submapas.
- Revisitas pequeñas no optimizan; revisitas con error real generan tasks.
- El worker secundario continúa respetando prioridades y no bloquea la publicación.
- El mapa sparse-global permanece estable durante la prueba completa.

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


Además, antes de ejecutar:

```text
subfases/subfase_4A.md ... subfase_4J.md
historial/resúmenes reales de Fase 3 relevantes
codex/contexto/paquetes/simulacion_dron/
codex/contexto/paquetes/orbslam3_server/
codex/contexto/paquetes/orbslam3_multi/
codex/contexto/paquetes/<paquete_wrapper_real>/
```

## Diagnóstico de partida

La Fase 3 ya tiene una arquitectura sparse-global estable y una prueba integrada multi-dron. La Fase 4 introduce una fuente visual nueva para fiduciales y debe demostrar que no rompe aislamiento de submapas, deltas, optimizer, worker ni visualización.

La prueba típica de cierre debe recorrer el entorno suficientemente para que ambos drones generen múltiples KFs y observen fiduciales desde direcciones distintas; una vuelta alrededor del edificio es una geometría adecuada si el escenario actual lo permite.

## Archivos permitidos a modificar

```text
codex/archivos_auxiliares/trayectorias/tray_fase4_4K_*.yaml
src/simulacion_dron/config/fiducials.yaml
src/simulacion_dron/launch/multi_dron.launch.py
config local del wrapper detector
src/orbslam3_server/config/fiducials.yaml
launch/config de orbslam3_server
scripts de métricas/reducción necesarios
codex/contexto/paquetes afectados/
```

Solo corregir código funcional de subfases anteriores si la prueba demuestra una regresión dentro de su alcance; registrar cada corrección en el historial correspondiente.

## Archivos prohibidos

```text
GT como input de anchor/corrección
cambios de arquitectura no relacionados para mejorar la prueba
límites aumentados sin causa
legacy no usado
```

## Funciones, clases o nodos que hay que localizar

```text
multi_dron.launch.py
scenario_runner_node
wrapper de cada dron
orbslam/orb_map_delta
orbslam/fiducial_keyframe_observations
global_map_server
secondary worker / optimization manager
```

## Cambios requeridos

1. Crear un escenario reproducible con dos drones y varios cubos fiduciales alrededor/adyacentes al recorrido.
2. Asegurar que los dos wrappers usan la misma familia/tamaño físico y namespaces aislados.
3. Configurar servidor con los mismos IDs/geometría global necesarios, sin leer el YAML de simulación directamente.
4. Incluir al menos una situación donde dos tags del mismo cubo sean visibles en un KF.
5. Incluir al menos una situación donde un KF pueda ver tags de cubos distintos, si la geometría del escenario lo permite.
6. Hacer que ambos drones observen al menos un cubo común en momentos/KFs distintos.
7. Generar suficientes KFs para una primera observación y una revisita real por dron o por submapa.
8. Deshabilitar la ruta funcional GT de fiduciales durante la prueba.
9. Registrar GT únicamente en un namespace/log de métricas externas si se desea calcular error absoluto.
10. Reducir logs con IDs de dron/epoch/KF/tag/object y tareas de optimización.
11. Comprobar que no existen anchors cruzados entre drones ni observaciones asociadas a otro epoch.
12. Verificar que el mapa global y publicadores siguen activos durante detecciones y optimizaciones.
13. Mantener el escenario suficientemente largo para observar estado estable después de los eventos fiduciales.

## Cambios prohibidos

- No usar un único dron para declarar cierre multi-dron.
- No usar GT como fallback si un detector no ve el tag.
- No mover los cubos en runtime para que entren en la cámara.
- No aceptar IDs duplicados entre cubos.
- No desactivar loops/optimizer/worker salvo que la prueba específica lo justifique y se documente.
- No declarar éxito solo por ver tags en Gazebo; debe verificarse la cadena completa hasta servidor/backend.

## Paquetes a compilar

Antes del cierre, compilar todos los paquetes ROS modificados de Fase 4:

```bash
./codex/herramientas/build_selected_packages.sh \
  orbslam3_msgs orbslam3_multi orbslam3_server simulacion_dron <paquete_wrapper_real>
```

La librería ORB-SLAM3 modificada en 4C debe estar recompilada y enlazada por el wrapper real.

## Pruebas Gazebo requeridas

### Prueba 1 — Dos drones, recorrido completo

YAML propuesto:

```text
codex/archivos_auxiliares/trayectorias/tray_fase4_4K_dos_drones.yaml
```

Secuencia:

1. arrancar Gazebo/servidor/wrappers;
2. confirmar cubos spawneados;
3. lanzar los dos goals;
4. ambos drones realizan el recorrido acordado alrededor del edificio;
5. esperar las primeras detecciones/anchors;
6. continuar hasta revisitas;
7. esperar a que terminen tasks activas;
8. dejar ventana final para publicación estable.

Comando base:

```bash
./codex/herramientas/run_simulation.sh \
  --prueba fase4_4K_dos_drones \
  --launch "ros2 launch simulacion_dron multi_dron.launch.py" \
  --post-scenario-wait-sec 30
```

### Prueba 2 — Mismo cubo por dos drones

Aislar/verificar que ambos drones observan un `object_id` común y que cada observación queda ligada a su propio submapa/KF.

### Prueba 3 — Métrica externa GT

Si se usa GT, ejecutarlo solo después/por fuera de la decisión funcional para calcular error de pose. Desconectar ese proceso no debe cambiar anchors/tasks.

## Patrones de reducción de logs

```text
SCENARIO-RUNNER|GOAL|RESULT|success|KF-EVENT|FID-BATCH|FID-SYNC|FID-OBJECT|FID-VIS-ANCHOR|FID-REVISIT|FID-TASK|drone_id|map_epoch|keyframe_id|ERROR|FATAL|Segmentation fault|Killed
```

Si el reducido no contiene evidencia suficiente, ampliar patrones y regenerar el reducido antes de abrir el log completo.

## Criterio de éxito

1. Build global seleccionado devuelve 0.
2. Ambos drones completan los goals requeridos con `success=true`.
3. Se detectan tags en KFs exactos de ambos drones.
4. El servidor agrupa correctamente caras/cubos.
5. Cada submapa se ancla sin GT funcional.
6. Revisitas cumplen la política de residual/task.
7. No existen asociaciones cruzadas dron/epoch/KF.
8. El detector no bloquea SLAM/mapa.
9. No aparecen errores graves inexplicados.
10. La métrica GT, si se ejecuta, es externa y no cambia el resultado funcional.
11. Historial/documentación quedan actualizados con evidencia real.

## Criterio de fallo o parcial

- `NO CONSEGUIDA`: un dron no completa prueba, hay anchor cruzado, GT funcional, crash o mapa inestable.
- `PARCIAL`: cadena visual funciona en ambos drones pero falta una condición multi-tag/revisit o una tarea no finaliza correctamente.
- `BLOQUEADA`: recurso externo imprescindible del escenario no está disponible y no puede sustituirse sin cambiar el objetivo.

## Riesgos

- carga combinada de dos detectores y servidor;
- IDs/namespaces mezclados;
- cubos colocados fuera del FOV de las trayectorias;
- prueba demasiado corta para revisit;
- interpretar GT de métricas como dependencia funcional.

## Documentación a actualizar

```text
codex/contexto/00_CONTEXTO_COMPACTACION.md
codex/contexto/01_ESTADO_ACTUAL.md
codex/contexto/paquetes/ de todos los paquetes modificados
codex/pipeline/fase_4_fiducial_real/pipeline_fase_4_RESUMEN.md
codex/pipeline/fase_4_fiducial_real/historial/por_subfase/historial_4K.md
codex/pipeline/fase_4_fiducial_real/historial/por_subfase/historial_4K_RESUMEN.md
```
