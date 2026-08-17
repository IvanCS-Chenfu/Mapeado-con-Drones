# Subfase 8E — Baseline end-to-end con todos los KeyFrames y visualización en GUI


## Estado

```text
sin hacer
```


## Objetivo técnico

Conectar por primera vez todo el flujo: cada KF creado produce temporalmente una subnube densa, se guarda en `DenseKeyFrameDatabase`, se coloca en `world` usando la pose global vigente del propio KF y se visualiza en la GUI. Probar extensamente con drones en movimiento y demostrar que, cuando una optimización cambia la pose de un KF, su subnube se recoloca. Esta subfase debe ofrecer una primera visión real de la arquitectura antes de optimizar selección/calidad/fusión.


## Invariantes y decisiones cerradas

- En 8E todos los KFs válidos se intentan convertir en DenseKF para observar el comportamiento sin filtro de redundancia.
- La nube `world` mostrada es una vista derivada en tiempo de lectura; la DB sigue guardando puntos locales al KF.
- La GUI de Fase 7 ya debe tener `DenseMapLayer`/infraestructura equivalente; Fase 8 alimenta datos reales, no crea una segunda GUI.
- Debe probarse movimiento lento, normal, giros y traslación; no asumir de antemano que las nubes móviles son buenas o malas.
- La recolocación simple de una subnube al cambiar su KF se implementa aquí; la reintegración de mapas fusionados queda en 8I.


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
codex/pipeline/fase_7_gui/pipeline_fase_7_RESUMEN.md
codex/contexto/paquetes/orbslam3_multi/global_pose_store.md
codex/contexto/paquetes/orbslam3_multi/global_map_builder.md
```
Leer además el contrato real de optimización/loops/fiduciales de Fase 3 que pueda provocar cambios de poses.


## Diagnóstico de partida

Hasta 8D existen piezas aisladas, pero aún no sabemos si la reconstrucción desde KFs en movimiento es utilizable ni si la relación `DenseSubcloud local ↔ KF ↔ world` se comporta correctamente al optimizar. Esta es la prueba de arquitectura más importante antes de añadir filtros.


## Archivos permitidos a modificar

```text
src/servidor/dense_map_multi/
src/servidor/dense_map_server/
src/servidor/multidron_gui/              # solo consumir/renderizar producto dense real si Fase 7 lo dejó preparado
src/servidor/orbslam3_multi/              # solo integración de lectura de poses/revisiones; no cambiar solver todavía
src/servidor/orbslam3_server/             # integración mínima si la fuente canónica de revisiones está aquí
src/simulacion/simulacion_dron/           # escenarios de prueba
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
DenseKeyFrameDatabase
GlobalPoseStore::Get*KeyFramePose o equivalente
revision de pose/publicación de Fase 3
multidron_gui::DenseMapLayer o componente equivalente
mecanismo de snapshot/caches GUI de Fase 7
```
Nueva clase sugerida:
```text
DenseWorldViewBuilder / DenseSubcloudWorldTransformer
```



No inventar nombres de interfaces previas. Si alguno no existe con ese nombre, localizar el componente equivalente mediante documentación y búsqueda estática antes de implementar. Los nombres nuevos propuestos pueden materializarse con una estructura equivalente si conserva el ownership acordado.


## Cambios requeridos

1. Implementar consulta `T_world_kf` vigente por clave DenseKF y transformar puntos locales solo al construir la vista/salida.
2. Publicar un producto ligero/stream compatible con la GUI sin duplicar permanentemente todas las subnubes en RAM si no aporta valor.
3. Alimentar la capa dense de la GUI y permitir inspeccionar/activar/desactivar subnubes para diagnóstico.
4. Escuchar/reaccionar a revisión de poses: si cambia `T_world_kf`, la siguiente vista debe mostrar la misma subnube local en la nueva pose.
5. Ejecutar recorridos alrededor del entorno con varios regímenes de movimiento y registrar métricas básicas por DenseKF: velocidad si está disponible, densidad, disparity válida, spread/planitud, etc.
6. Crear una versión mínima de información voxel/cobertura solo si es necesaria para visualizar/medir; la representación madura se implementa en 8H.
7. Añadir markers `DENSE-WORLD-VIEW`, `DENSE-KF-POSE-REV`, `DENSE-GUI-PUBLISH`, `DENSE-MOTION-SAMPLE`.


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
- No filtrar KFs redundantes todavía; 8F necesita el baseline “todos”.
- No mover KFs para hacer encajar nubes; eso empieza en 8J/8K.
- No ocultar nubes malas en GUI para que el resultado parezca mejor.


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
./codex/herramientas/build_selected_packages.sh dense_map_multi dense_map_server multidron_gui orbslam3_multi orbslam3_server
```

Si la separación de Fase 2 está ya ejecutada, respetar sus builds por grupo (`dron`, `servidor`, `simulacion`) y la sincronización de las dos copias de `orbslam3_msgs`. Los comandos listados son el conjunto lógico esperado; el implementador debe usar el helper vigente y registrar en historial el comando real.


## Pruebas Gazebo requeridas

### Prueba 1 — Recorrido de movimiento variado

Mover un dron por los alrededores de la casa con tramos lentos, normales, giros y traslaciones. Todos los KFs válidos generan DenseKF. Revisar en GUI planitud, bordes, ruido, dobles superficies y degradación durante giros.

### Prueba 2 — Dos drones

Ejecutar dos drones en zonas con solape para comprobar identidad multi-dron y visualización simultánea; todavía no exigir fusión geométrica avanzada.

### Prueba 3 — Optimización durante/tras captura

Provocar una corrección real ya disponible en Fase 3/4 y verificar que la subnube de cada KF afectado cambia de `world` sin mutar los puntos locales de DB.

```bash
./codex/herramientas/run_simulation.sh \
  --prueba fase_8_8E \
  --launch "ros2 launch simulacion_dron multi_dron.launch.py" \
  --post-scenario-wait-sec 30
```

No arrancar Gazebo artificialmente para una prueba puramente unitaria/de componente. Cuando la subfase necesite integración visual, temporal o multi-nodo, usar `run_simulation.sh` y registrar el comando exacto solo en el historial real.


## Patrones de reducción de logs

```text
DENSE-WORLD-VIEW|DENSE-KF-POSE-REV|DENSE-GUI|DENSE-MOTION|SERVER_OPTIMIZATION|FIDUCIAL|LOOP|map_epoch|ERROR|FATAL|Segmentation fault|Killed
```

Los logs completos solo son entrada de `reduce_*`/`split_*`. Si el reducido no contiene evidencia suficiente, regenerarlo con patrones más específicos o crear un sublog temático; no abrir ni volcar el log completo.


## Criterio de éxito

La subfase se considera `CONSEGUIDA` solo si:

1. El flujo KF→L/R→subnube→DB→pose KF→GUI funciona end-to-end.
2. Todos los KFs de la prueba se intentan densificar sin colisiones de identidad.
3. Una optimización mueve las subnubes en GUI según las nuevas poses de sus KFs y la DB local permanece intacta.
4. Se obtiene evidencia visual/métrica suficiente sobre nubes en movimiento, incluidos giros y zonas de baja textura.
5. No se bloquea el flujo sparse ni la GUI.


## Criterio de fallo o parcial

- `NO CONSEGUIDA` si una subnube queda congelada tras cambiar la pose de su KF.
- `NO CONSEGUIDA` si las identidades multi-dron/epoch se mezclan.
- `PARCIAL` si end-to-end funciona pero no puede evaluarse todavía una optimización real o la GUI no muestra datos suficientes.
- Si las nubes en movimiento son extremadamente malas, no inventar una solución: conservar evidencia y continuar hacia 8G/estrategia estacionaria según el acuerdo.


## Riesgos

- El volumen de datos puede ser enorme porque todos los KFs son dense; es intencional y temporal.
- Confundir una mala pose sparse con un mal depth; comparar subnube local antes de culpar al global.


## Documentación a actualizar

Al ejecutar realmente la subfase, actualizar únicamente documentación respaldada por cambios/evidencia real:

```text
codex/pipeline/fase_8_nube_densa/historial/por_subfase/historial_8E.md
codex/pipeline/fase_8_nube_densa/historial/por_subfase/historial_8E_RESUMEN.md
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
