# Subfase 8R — Rendimiento, memoria, red, colas y decisión de compresión


## Estado

```text
sin hacer
```


## Objetivo técnico

Medir y acotar el coste real de la Fase 8 en dron/red/servidor. Mantener al dron ligero, garantizar que dense no bloquea sparse/control, dimensionar colas y mapas, y decidir con evidencia si hace falta compresión de imágenes/transporte u otras optimizaciones.


## Invariantes y decisiones cerradas

- No elegir compresión antes de medir tráfico real.
- La optimización de red no puede trasladar reconstrucción pesada al dron.
- El pipeline principal debe seguir operativo bajo backlog dense.
- Los mapas coarse/fine y selección de DenseKF son herramientas principales para controlar memoria.


## Contexto obligatorio a leer

```text
AGENTS.md
codex/contexto/00_CONTEXTO_COMPACTACION.md
codex/contexto/CONTEXTO_MINIMO_ACTUAL.md
codex/contexto/08_POLITICA_TOKENS_DOCUMENTACION.md
codex/contexto/01_ESTADO_ACTUAL.md
codex/pipeline/PIPELINE_MAESTRO.md
codex/pipeline/fase_8_nube_densa/pipeline_fase_8_RESUMEN.md
codex/pipeline/fase_8_nube_densa/pipeline_fase_8.md
```

Antes de modificar código, leer también los resúmenes/contratos **reales ya ejecutados** de las fases anteriores de las que dependa esta subfase y los MD vigentes de cada paquete afectado en `codex/contexto/paquetes/`. En particular, Fase 8 depende funcionalmente de Fase 3 (sparse global), Fase 5 (poses globales), Fase 6 (tareas/trayectorias/obstáculos) y Fase 7 (GUI). Si el workspace posterior a esas fases usa nombres o rutas distintos a los de este contrato, localizar primero la fuente canónica y reutilizarla; no crear una segunda interfaz solo para satisfacer el nombre documental.



Además, leer específicamente:

Leer política de worker/backpressure de Fase 3 y herramientas de logs. Revisar también cualquier QoS real fijado por Fases 2/6.


## Diagnóstico de partida

Imágenes estéreo y clouds pueden saturar red/CPU/RAM si se envían todos los KFs o se acumulan tareas HQ. Tras implementar funcionalidad completa hay que medir tasa de KFs, MB/s, latencia de disparity/registro, bloques voxel y backlog antes de escoger compresión o límites.


## Archivos permitidos a modificar

```text
src/servidor/dense_map_multi/
src/servidor/dense_map_server/
src/dron/orbslam3_ros2/                    # solo transporte/QoS/compresión si la evidencia lo exige
src/dron/orbslam3_msgs/
src/servidor/orbslam3_msgs/
config/YAML de servidor/dron correspondiente
codex/herramientas/                         # solo si hace falta ampliar reductores/medición de forma general
```


Las rutas nuevas indicadas son de **contrato propuesto**. Antes de crearlas, comprobar el árbol real posterior a Fases 2–7 y reutilizar componentes equivalentes si existen.


## Archivos prohibidos

```text
build/                                      # no modificar manualmente
install/                                    # no modificar manualmente
log/                                        # no modificar manualmente salvo limpieza mínima autorizada por herramientas
src/dron/ORB_SLAM3/                         # salvo necesidad explícita, demostrada y nuevo acuerdo
legacy/ o *_antiguo.*                       # no tocar como solución de Fase 8
paquetes ajenos a la subfase                # salvo dependencia real localizada y justificada
```


## Funciones, clases, nodos o interfaces que hay que localizar

```text
colas de imágenes KF
worker(s) dense_map_server
DenseKeyFrameSelector
DenseFusionMap allocated blocks
OccupancyGrid3D memory
QoS/image_transport real
métricas de CPU/RAM disponibles en entorno
```



No inventar nombres de interfaces previas. Si alguno no existe con ese nombre, localizar el componente equivalente mediante documentación y búsqueda estática antes de implementar. Los nombres nuevos propuestos pueden materializarse con una estructura equivalente si conserva el ownership acordado.


## Cambios requeridos

1. Instrumentar tasa de imágenes/KFs, bytes/s por dron, tamaño de mensaje, cola, drops, latencia end-to-end y edad de datos.
2. Medir CPU/RAM servidor por disparity, quality, voxel integration, registration, reintegration y análisis de cobertura.
3. Medir carga adicional del dron exclusivamente por copia/serialización/transporte; no reconstrucción.
4. Definir colas acotadas y política de drop/prioridad: control/sparse no espera dense; captura HQ puede tener semántica distinta de DenseKF oportunista.
5. Medir memoria de DenseKeyFrameDatabase, occupancy y DenseFusionMap en prueba larga.
6. Solo si la red es un cuello de botella, probar compresión/transporte (por ejemplo mecanismo ROS compatible) y comparar calidad/CPU/latencia antes/después; si no aporta, mantener baseline sencillo.
7. Ajustar frecuencia/selección/resoluciones con parámetros y presupuesto documentado, sin hardcodes invisibles.
8. Añadir markers `DENSE-PERF`, `DENSE-NET`, `DENSE-QUEUE`, `DENSE-MEM`, `DENSE-DROP`.


## Cambios prohibidos

- No usar Ground Truth para calcular disparity/depth, colocar la nube densa, fusionar, corregir poses, refinar MapPoints, decidir ocupación o validar online una trayectoria. GT solo puede aparecer como métrica externa de simulación.
- No modificar datos raw de ORB-SLAM3 en `RawMapDatabase`.
- No devolver MapPoints corregidos al ORB-SLAM3 que corre en el dron.
- No ejecutar reconstrucción densa pesada en el dron: el dron se limita a capturar y enviar información.
- No convertir `orbslam3_server` ni `dense_map_server` en un backend algorítmico monolítico; los algoritmos densos pertenecen a `dense_map_multi`.
- No bloquear ingesta sparse, pose, control, GUI o ejecución de tareas mientras se calcula disparity, registro, voxelización, fusión o reintegración.
- No almacenar imágenes L/R permanentemente como parte de `DenseKeyFrameDatabase`; si una zona queda mal, la estrategia acordada es volver a observarla/recapturarla.
- No implementar por adelantado subfases posteriores salvo infraestructura mínima estrictamente necesaria y documentada.
- No limpiar legacy ni cambiar paquetes ajenos como efecto colateral.
- No crear historiales con resultados ficticios. Las carpetas se entregan vacías y los MD de historial nacen solo tras ejecuciones reales.
- No reducir resolución/calidad hasta ocultar fallos geométricos sin medir impacto.
- No hacer colas ilimitadas.
- No bloquear `GrabStereo` esperando procesamiento dense.
- No mover OpenCV/Open3D al dron para ahorrar red.


## Puerta de validación hacia fases anteriores

Fase 8 no puede maquillar fallos de las fases previas. Si durante la preparación o prueba aparece una incoherencia cuyo dato de entrada ya es incorrecto:

1. identificar la fuente propietaria (sparse/KFs Fase 3, poses Fase 5, tareas/obstáculos Fase 6, visualización Fase 7);
2. detener la implementación de Fase 8 que dependa de ese dato;
3. registrar el bloqueo/diagnóstico sin inventar una compensación local;
4. volver a la fase propietaria, corregirla y revalidarla según `AGENTS.md`;
5. repetir después la prueba de Fase 8.

No se aceptan offsets, deformaciones, filtros visuales o copias de estado cuyo único objetivo sea ocultar un error previo.


## Paquetes a compilar

```bash
./codex/herramientas/build_selected_packages.sh dense_map_multi dense_map_server orbslam3 orbslam3_msgs
```

Si la separación de Fase 2 está ya ejecutada, respetar sus builds por grupo (`dron`, `servidor`, `simulacion`) y la sincronización de las dos copias de `orbslam3_msgs`. Los comandos listados son el conjunto lógico esperado; el implementador debe usar el helper vigente y registrar en historial el comando real.


## Pruebas Gazebo requeridas

### Prueba 1 — Prueba larga multi-dron

Ejecutar misión representativa con al menos dos drones, DenseKF oportunistas, registros y algunas capturas HQ. Medir red, CPU, RAM, backlog y drops.

### Prueba 2 — Consumidor dense ralentizado

Introducir carga controlada/retraso y comprobar que sparse/control siguen funcionando y la cola no crece sin límite.

### Prueba 3 — A/B de compresión si procede

Solo si la medición muestra cuello de botella de red: repetir mismo escenario con baseline y alternativa comprimida; aceptar solo si reduce tráfico sin degradación inaceptable/CPU excesiva.

No arrancar Gazebo artificialmente para una prueba puramente unitaria/de componente. Cuando la subfase necesite integración visual, temporal o multi-nodo, usar `run_simulation.sh` y registrar el comando exacto solo en el historial real.


## Patrones de reducción de logs

```text
DENSE-PERF|DENSE-NET|DENSE-QUEUE|DENSE-MEM|DENSE-DROP|backlog|bytes|latency|RSS|CPU|ERROR|FATAL|Killed
```

Los logs completos solo son entrada de `reduce_*`/`split_*`. Si el reducido no contiene evidencia suficiente, regenerarlo con patrones más específicos o crear un sublog temático; no abrir ni volcar el log completo.


## Criterio de éxito

La subfase se considera `CONSEGUIDA` solo si:

1. Existen métricas reproducibles de red/CPU/RAM/colas para el escenario largo.
2. Las colas son acotadas y una saturación dense no detiene sparse/control.
3. El dron no asume procesamiento denso adicional.
4. La decisión “comprimir/no comprimir” queda respaldada por A/B o por demostrar que la red no es cuello de botella.
5. El mapa se mantiene dentro del presupuesto acordado en la prueba.


## Criterio de fallo o parcial

- `NO CONSEGUIDA` si el backlog dense bloquea tracking/control o la memoria crece sin límite.
- `PARCIAL` si la funcionalidad es correcta pero el presupuesto todavía no se cumple.


## Riesgos

- Comprimir puede ahorrar red y consumir CPU en dron; medir ambas.
- Logs demasiado verbosos pueden distorsionar la propia prueba de rendimiento.


## Documentación a actualizar

Al ejecutar realmente la subfase, actualizar únicamente documentación respaldada por cambios/evidencia real:

```text
codex/pipeline/fase_8_nube_densa/historial/por_subfase/historial_8R.md
codex/pipeline/fase_8_nube_densa/historial/por_subfase/historial_8R_RESUMEN.md
codex/pipeline/fase_8_nube_densa/pipeline_fase_8_RESUMEN.md
codex/contexto/01_ESTADO_ACTUAL.md                 # si cambia el estado real
codex/contexto/paquetes/<paquete_afectado>/
```

Si se modifica código, el MD del paquete debe reflejar clases/funciones, topics/services/actions, parámetros, markers, ownership, limitaciones y estado de validación vigente. No marcar la subfase como `realizado` sin cumplir su criterio de éxito.


## Dudas funcionales de contrato

```text
ninguna en el acuerdo actual
```

Si durante preparación/ejecución aparece una alternativa funcional material no cubierta por este contrato, suspender la autorización y consultarla al usuario conforme a `AGENTS.md`.
