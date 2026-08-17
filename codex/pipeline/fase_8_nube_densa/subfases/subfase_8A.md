# Subfase 8A — Calibración, frames, estéreo y base del backend denso


## Estado

```text
sin hacer
```


## Objetivo técnico

Cerrar la geometría métrica y el ownership de la reconstrucción densa antes de generar mapas: inventariar cámaras estéreo, intrínsecos, baseline, rectificación, extrínsecos y frames; validar que la profundidad reconstruida tiene escala correcta; y crear la infraestructura mínima de servidor `dense_map_multi` + `dense_map_server` sobre la que se desarrollará el resto de la fase. Todo el procesamiento denso se ejecuta en el servidor.


## Invariantes y decisiones cerradas

- El dron realiza el mínimo procesamiento posible: captura y transporte de imágenes/datos; disparity, depth, point cloud, voxelización, registro y fusión se calculan en el servidor.
- `dense_map_multi` es una librería algorítmica C++ análoga en responsabilidad a `orbslam3_multi`; no es el nodo ROS.
- `dense_map_server` es el adaptador/coordinador ROS 2 de la parte densa y usa clases/métodos de `dense_map_multi`.
- La nube final no usa GT. Las dimensiones conocidas del mundo/Gazebo pueden usarse solo como métrica externa de prueba.
- No fijar todavía compresión de transporte: se mide en 8R.


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

```text
codex/pipeline/fase_2_separacion_paquetes/pipeline_fase_2_RESUMEN.md
codex/pipeline/fase_3_sparse_global/pipeline_fase_3_RESUMEN.md
codex/contexto/paquetes/orbslam3_ros2/stereo_slam_node.md
codex/contexto/paquetes/dron_individual/vision_experimental.md
```
Revisar los experimentos legacy de profundidad/ICP/TSDF solo como fuente de ideas y parámetros; no copiarlos ciegamente al pipeline nuevo.


## Diagnóstico de partida

La guía maestra exige RGB+disparidad/depth, escala métrica, OpenCV/Open3D y coherencia con poses globales, pero el pipeline activo todavía no posee un backend denso de servidor. El wrapper ya carga intrínsecos/baseline y rectifica para ORB-SLAM3; existen scripts experimentales antiguos de profundidad/TSDF, pero no son arquitectura vigente. Antes de continuar debe demostrarse una cadena estéreo reproducible y documentada.


## Archivos permitidos a modificar

```text
src/servidor/dense_map_multi/                    # nuevo paquete/librería propuesto
src/servidor/dense_map_server/                   # nuevo paquete/nodo propuesto
src/servidor/orbslam3_server/                    # solo lectura/integración mínima si hace falta descubrir frames/poses
src/dron/orbslam3_ros2/                          # solo si hace falta exponer calibración ya disponible; no extraer imágenes de KF todavía
src/dron/dron_individual/config/                 # parámetros de cámara si Fase 2 los mantiene aquí
src/simulacion/simulacion_dron/                  # solo recursos/pruebas/calibración simulada
codex/archivos_auxiliares/                       # artefactos de test reducidos, no datos funcionales
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
StereoSlamNode::GrabStereo
StereoSlamNode::LoadCameraInfoFromSettings
parámetros Camera.fx/fy/cx/cy/bf y/o LEFT/RIGHT
camera/left
camera/right
frame camera/base_link/world posterior a Fase 5
configuración de cámara en simulación
```
Nuevas clases sugeridas, solo como contrato de responsabilidad:
```text
dense_map_multi::StereoCalibration
dense_map_multi::StereoDepthProcessor     # interfaz mínima; algoritmo real se completa en 8C
dense_map_server::DenseMapServer          # nodo/adaptador ROS
```



No inventar nombres de interfaces previas. Si alguno no existe con ese nombre, localizar el componente equivalente mediante documentación y búsqueda estática antes de implementar. Los nombres nuevos propuestos pueden materializarse con una estructura equivalente si conserva el ownership acordado.


## Cambios requeridos

1. Inventariar de extremo a extremo resolución, encoding, sincronización, intrínsecos, baseline y rectificación de la cámara estéreo real/simulada; documentar una única convención.
2. Definir explícitamente los frames de entrada/salida del denso (`camera_left` o equivalente, `base_link`, `world`) y las transformaciones conocidas; no duplicar TF si ya existe.
3. Crear `dense_map_multi` como librería C++ de servidor, sin dependencia ROS innecesaria en el núcleo algorítmico siempre que el código real lo permita.
4. Crear `dense_map_server` como nodo ROS 2 ligero que recibirá imágenes/poses y delegará algoritmos en `dense_map_multi`; todavía sin lógica de mapa global compleja.
5. Configurar dependencias mínimas de OpenCV y Open3D en servidor según el entorno real y comprobar versiones instaladas antes de fijarlas.
6. Implementar una prueba métrica mínima de estéreo sobre un plano/objeto conocido para validar signo de disparity, `Z = f*B/d` o formulación equivalente, ejes y unidades.
7. Añadir markers `DENSE-CALIBRATION-READY`, `DENSE-STEREO-SCALE` y errores explícitos para calibración incompleta/no finita.


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
- No implementar todavía captura exacta de imágenes de KF (8B).
- No crear todavía DenseKeyFrameDatabase funcional (8D).
- No elegir TSDF como representación definitiva antes de 8H.


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
./codex/herramientas/build_selected_packages.sh dense_map_multi dense_map_server
```
Si se toca el wrapper, añadir el paquete ROS real `orbslam3` y consumidores afectados.

Si la separación de Fase 2 está ya ejecutada, respetar sus builds por grupo (`dron`, `servidor`, `simulacion`) y la sincronización de las dos copias de `orbslam3_msgs`. Los comandos listados son el conjunto lógico esperado; el implementador debe usar el helper vigente y registrar en historial el comando real.


## Pruebas Gazebo requeridas

### Prueba 1 — Test de componente de calibración

Sin arrancar Gazebo si no es necesario, cargar el YAML real y comprobar intrínsecos, baseline, tamaño, frames y errores de configuración con un test determinista.

### Prueba 2 — Plano/objeto de escala conocida en Gazebo

Usar la cámara estéreo del dron frente a una superficie cuya geometría se conoce **solo como referencia externa de evaluación**. El servidor recibe L/R y reconstruye una profundidad inicial. Comparar escala/signo/ejes; GT no entra en el cálculo.

Comando base si se requiere simulación:

```bash
./codex/herramientas/run_simulation.sh \
  --prueba fase_8_8A \
  --launch "ros2 launch simulacion_dron multi_dron.launch.py" \
  --post-scenario-wait-sec 20
```

No arrancar Gazebo artificialmente para una prueba puramente unitaria/de componente. Cuando la subfase necesite integración visual, temporal o multi-nodo, usar `run_simulation.sh` y registrar el comando exacto solo en el historial real.


## Patrones de reducción de logs

```text
DENSE-CALIBRATION|DENSE-STEREO-SCALE|camera/left|camera/right|baseline|fx|fy|ERROR|FATAL|Segmentation fault|Killed
```

Los logs completos solo son entrada de `reduce_*`/`split_*`. Si el reducido no contiene evidencia suficiente, regenerarlo con patrones más específicos o crear un sublog temático; no abrir ni volcar el log completo.


## Criterio de éxito

La subfase se considera `CONSEGUIDA` solo si:

1. `dense_map_multi` y `dense_map_server` compilan en el grupo Servidor sin depender de Gazebo.
2. La calibración y frames quedan definidos sin ambigüedad y coinciden con el wrapper/cámara reales.
3. La prueba de escala produce profundidad finita, con signo y orden de magnitud correctos frente a la referencia externa.
4. No existe procesamiento denso funcional en el dron.
5. Los markers de calibración/escala permiten diagnosticar fallos y la documentación de paquetes queda actualizada.


## Criterio de fallo o parcial

- `NO CONSEGUIDA` si baseline/intrínsecos/frames no pueden reconciliarse o la escala sale invertida/incoherente.
- `PARCIAL` si compila y lee calibración pero falta validar la escala con imágenes estéreo reales/simuladas.
- `BLOQUEADA` solo si falta una dependencia externa imprescindible o una calibración que no puede obtenerse del proyecto.


## Riesgos

- Confundir `bf` con baseline físico o usar unidades distintas.
- Duplicar rectificación y degradar imágenes ya rectificadas.
- Introducir Open3D/Gazebo dentro del grupo Dron.


## Documentación a actualizar

Al ejecutar realmente la subfase, actualizar únicamente documentación respaldada por cambios/evidencia real:

```text
codex/pipeline/fase_8_nube_densa/historial/por_subfase/historial_8A.md
codex/pipeline/fase_8_nube_densa/historial/por_subfase/historial_8A_RESUMEN.md
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
