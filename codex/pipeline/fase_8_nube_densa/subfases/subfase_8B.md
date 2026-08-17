# Subfase 8B — Asociación exacta KeyFrame ↔ par estéreo del wrapper


## Estado

```text
sin hacer
```


## Objetivo técnico

Modificar el wrapper para que, cuando ORB-SLAM3 cree un KeyFrame, pueda emitir al servidor el par izquierda/derecha exacto utilizado por el `TrackStereo` que originó ese KF, con identidad `(drone_id, map_epoch, local_kf_id)` y timestamp coherentes. No se guardan imágenes de forma persistente.


## Invariantes y decisiones cerradas

- El wrapper sí puede obtener las imágenes de un KF; esta subfase debe implementarlo.
- La asociación debe ser exacta, no por “KF más cercano” ni por una ventana temporal aproximada.
- Las imágenes se transportan al servidor y se descartan cuando ya se ha construido/aceptado la subnube; no forman parte de la DB persistente.
- Fase 4 ya requiere una asociación exacta imagen-KF para fiducial; reutilizar esa infraestructura si existe tras ejecutar Fase 4.


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
codex/pipeline/fase_4_fiducial_real/pipeline_fase_4_RESUMEN.md
codex/contexto/paquetes/orbslam3_ros2/stereo_slam_node.md
codex/contexto/decisiones/ADR_0005_wrapper_y_mensajes_estables.md
```
Inspeccionar físicamente el wrapper posterior a Fase 4 antes de diseñar un segundo mecanismo.


## Diagnóstico de partida

`StereoSlamNode::GrabStereo` recibe las dos imágenes y llama a `TrackStereo`, pero el contrato actual exporta KFs mediante `BuildOrbMap` y no conserva/transporta necesariamente las imágenes exactas asociadas a un KF. La Fase 8 necesita esa correspondencia para que la subnube local comparta el frame del KF y sobreviva a optimizaciones.


## Archivos permitidos a modificar

```text
src/dron/orbslam3_ros2/
src/dron/orbslam3_msgs/
src/servidor/orbslam3_msgs/          # copia canónica/guardas de Fase 2
src/servidor/dense_map_server/
src/servidor/dense_map_multi/        # tipos auxiliares si no dependen de ROS
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
StereoSlamNode::BuildOrbMap
StereoSlamNode::FillKeyFrameMsg
UpdateMapEpochFromCurrentMap
m_SLAM->GetAllKeyFrames() o API equivalente
mecanismo de Fase 4 que detecta el KF exacto para fiducial
orbslam3_msgs/msg/OrbKeyFrame.msg
```
Localizar además el QoS/sincronizador real de `camera/left` + `camera/right`.



No inventar nombres de interfaces previas. Si alguno no existe con ese nombre, localizar el componente equivalente mediante documentación y búsqueda estática antes de implementar. Los nombres nuevos propuestos pueden materializarse con una estructura equivalente si conserva el ownership acordado.


## Cambios requeridos

1. Detectar de forma determinista qué KF nuevo aparece como consecuencia del `TrackStereo` actual, respetando `map_epoch` y resets.
2. Asociar ese KF al `cv::Mat`/mensaje left y right exactos del callback que lo produjo; copiar solo lo imprescindible para sobrevivir al scope del callback.
3. Definir/reutilizar un contrato ROS para enviar al servidor identidad del KF + par estéreo + calibración/referencia necesaria, manteniendo sincronizadas las copias de `orbslam3_msgs` si se amplían.
4. Evitar almacenamiento histórico de imágenes en el dron; implementar cola acotada/backpressure seguro para no bloquear tracking si servidor/red se retrasa.
5. En `dense_map_server`, recibir el evento y verificar unicidad de `(drone_id,map_epoch,kf_id)` y coherencia de timestamps antes de pasarlo a 8C.
6. Añadir markers `DENSE-KF-STEREO-EMIT`, `DENSE-KF-STEREO-RECV`, `DENSE-KF-STEREO-DROP` y razones de descarte.


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
- No modificar la biblioteca `ORB_SLAM3` si el wrapper puede detectar el nuevo KF con APIs ya disponibles.
- No publicar todas las imágenes continuamente por este contrato de KF; eso pertenece a los topics normales usados en 8O/8Q.
- No usar el timestamp como sustituto de la identidad exacta del KF.


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
./codex/herramientas/build_selected_packages.sh orbslam3 orbslam3_msgs dense_map_server dense_map_multi
```
Compilar ambas copias/grupos de `orbslam3_msgs` según la estrategia de Fase 2.

Si la separación de Fase 2 está ya ejecutada, respetar sus builds por grupo (`dron`, `servidor`, `simulacion`) y la sincronización de las dos copias de `orbslam3_msgs`. Los comandos listados son el conjunto lógico esperado; el implementador debe usar el helper vigente y registrar en historial el comando real.


## Pruebas Gazebo requeridas

### Prueba 1 — Creación controlada de KFs

Mover un dron por un tramo que genere varios KFs. Para cada evento emitido, comprobar que `kf_id/map_epoch` coincide con el KF exportado y que el par L/R proviene del mismo callback `TrackStereo`.

### Prueba 2 — Reset/map_epoch

Provocar o reproducir un cambio de `map_epoch` y verificar que no se asocian imágenes de un epoch al KF de otro.

### Prueba 3 — Red/consumidor lento

Retrasar el consumidor dense y comprobar que el wrapper no bloquea el tracking; los drops deben ser explícitos y acotados.

```bash
./codex/herramientas/run_simulation.sh \
  --prueba fase_8_8B \
  --launch "ros2 launch simulacion_dron multi_dron.launch.py" \
  --post-scenario-wait-sec 20
```

No arrancar Gazebo artificialmente para una prueba puramente unitaria/de componente. Cuando la subfase necesite integración visual, temporal o multi-nodo, usar `run_simulation.sh` y registrar el comando exacto solo en el historial real.


## Patrones de reducción de logs

```text
DENSE-KF-STEREO|map_epoch|kf_id|TrackStereo|ORB_MAP|DROP|queue|ERROR|FATAL|Segmentation fault|Killed
```

Los logs completos solo son entrada de `reduce_*`/`split_*`. Si el reducido no contiene evidencia suficiente, regenerarlo con patrones más específicos o crear un sublog temático; no abrir ni volcar el log completo.


## Criterio de éxito

La subfase se considera `CONSEGUIDA` solo si:

1. Cada par L/R recibido por servidor se asocia inequívocamente a un KF real del mismo `map_epoch`.
2. No aparecen asociaciones aproximadas por tiempo ni cruces entre epochs/drones.
3. El tracking/flujo sparse continúa aunque el consumidor dense se retrase.
4. No se conserva un historial de imágenes en el dron ni en `DenseKeyFrameDatabase`.
5. Build y prueba de reset/cola pasan sin errores graves.


## Criterio de fallo o parcial

- `NO CONSEGUIDA` si no puede demostrarse qué par exacto originó el KF.
- `NO CONSEGUIDA` si el envío dense puede bloquear `GrabStereo`/tracking.
- `PARCIAL` si la asociación normal funciona pero no está validado reset/epoch o cola lenta.


## Riesgos

- El delta de KFs puede reenviar KFs antiguos; no confundir “aparece en delta” con “se creó en este frame”.
- Copiar imágenes grandes sin límites puede aumentar memoria del dron.
- Modificar mensajes sin sincronizar las dos copias de Fase 2.


## Documentación a actualizar

Al ejecutar realmente la subfase, actualizar únicamente documentación respaldada por cambios/evidencia real:

```text
codex/pipeline/fase_8_nube_densa/historial/por_subfase/historial_8B.md
codex/pipeline/fase_8_nube_densa/historial/por_subfase/historial_8B_RESUMEN.md
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
