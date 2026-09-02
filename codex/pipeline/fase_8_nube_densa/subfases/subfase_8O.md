# Subfase 8O — Captura densa estacionaria de alta calidad desde topics de cámara


## Estado

```text
sin hacer
```


## Objetivo técnico

Implementar la ejecución de una captura dense HQ cuando una tarea de Fase 6 pide densificar/reparar: el dron llega, se detiene/estabiliza, y el servidor consume directamente varios pares L/R de los topics normales —sin pasar por la creación de KF— para producir una subnube/captura local de alta calidad mediante combinación robusta.


## Invariantes y decisiones cerradas

- El procesamiento de las imágenes y promedio/filtrado ocurre en servidor.
- La captura HQ puede existir aunque ORB-SLAM3 no cree un KF en ese instante.
- Se usan varios pares si mejora la calidad; no se presupone un promedio simple si produce ghosting.
- Las imágenes se descartan al terminar el producto de captura; no se guardan permanentemente.
- La integración al mapa/KFs se hace en 8P, no aquí.


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

Leer el contrato real de tareas de Fase 6 y localizar cómo un dron reporta llegada/estabilización y cómo el servidor accede a sus topics estéreo.


## Diagnóstico de partida

Las DenseKF móviles pueden ser insuficientes en zonas concretas o no existir donde hace falta densidad. El mecanismo correctivo debe capturar directamente desde cámara con el dron parado, independientemente de si ORB decide crear un KF.


## Archivos permitidos a modificar

```text
src/servidor/dense_map_multi/
src/servidor/dense_map_server/
src/servidor/<paquete_tareas_fase_6>/
src/dron/dron_individual/                  # solo señal/estado de tarea ya acordado; no procesamiento dense
src/simulacion/simulacion_dron/            # escenarios
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
task_server / task_manager de Fase 6
tipo de tarea/acción de captura definido por Fase 7/6 (`CAPTURE_DENSE` o equivalente real)
camera/left + camera/right topics normales
pose global/estado de movimiento de Fase 5
StereoDepthProcessor / DenseQualityEvaluator
```



No inventar nombres de interfaces previas. Si alguno no existe con ese nombre, localizar el componente equivalente mediante documentación y búsqueda estática antes de implementar. Los nombres nuevos propuestos pueden materializarse con una estructura equivalente si conserva el ownership acordado.


## Cambios requeridos

1. Definir/reutilizar el lifecycle de tarea dense: llegar a viewpoint, detenerse, confirmar estabilidad, capturar N pares, procesar, devolver resultado.
2. En `dense_map_server`, suscribirse/seleccionar los pares del dron objetivo durante la ventana de captura sin afectar el stream usado por ORB.
3. Generar depth/subnubes por par y combinar de forma robusta (mediana/consenso/fusión local) solo si la evidencia de prueba demuestra mejora.
4. Rechazar frames con movimiento, sincronización mala o calidad insuficiente y prolongar/fallar la captura según el contrato de tarea.
5. Crear `DenseHQCapture` con frame de captura, pose/timestamp de referencia y geometría local; todavía no asignarla a KFs.
6. Eliminar buffers de imágenes al terminar/cancelar.
7. Añadir markers `DENSE-HQ-CAPTURE-START`, `DENSE-HQ-FRAME`, `DENSE-HQ-CAPTURE-DONE`, `DENSE-HQ-CAPTURE-FAIL`.


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
- No forzar a ORB_SLAM3 a crear un KF para poder capturar.
- No calcular disparity/depth en el dron.
- No integrar todavía la captura a varios KFs; 8P.


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
./codex/herramientas/build_selected_packages.sh dense_map_multi dense_map_server task_server task_manager dron_individual
```

Si la separación de Fase 2 está ya ejecutada, respetar sus builds por grupo (`dron`, `servidor`, `simulacion`) y la sincronización de las dos copias de `orbslam3_msgs` y `mission_msgs`. Los comandos listados son el conjunto lógico esperado; el implementador debe usar el helper vigente y registrar en historial el comando real.


## Pruebas Gazebo requeridas

### Prueba 1 — Captura parada de pared/plano

Comparar un par único frente a varios pares procesados; la estrategia multi-par debe aportar mejora o, si no, quedar documentado y simplificarse.

### Prueba 2 — Rechazo de movimiento

Iniciar captura antes de estabilizar: frames con movimiento no deben entrar como HQ.

### Prueba 3 — Sin KF nuevo

Mantener el dron parado en una pose donde ORB no crea KF; la tarea HQ debe poder completarse igualmente desde topics.

### Prueba 4 — Cancelación/fallo

Cancelar o perder imágenes durante la captura y verificar limpieza de buffers y resultado explícito.

No arrancar Gazebo artificialmente para una prueba puramente unitaria/de componente. Cuando la subfase necesite integración visual, temporal o multi-nodo, usar `run_simulation.sh` y registrar el comando exacto solo en el historial real.


## Patrones de reducción de logs

```text
DENSE-HQ-CAPTURE|TASK-STATE|camera/left|camera/right|stable|motion|quality|success|ERROR|FATAL|Killed
```

Los logs completos solo son entrada de `reduce_*`/`split_*`. Si el reducido no contiene evidencia suficiente, regenerarlo con patrones más específicos o crear un sublog temático; no abrir ni volcar el log completo.


## Criterio de éxito

La subfase se considera `CONSEGUIDA` solo si:

1. La captura HQ funciona desde topics normales aunque no aparezca KF nuevo.
2. El dron no procesa depth/nube y permanece detenido durante frames aceptados.
3. El producto HQ mejora o al menos no empeora la referencia de un solo frame según métricas 8G.
4. Las imágenes/buffers se liberan tras terminar y no se guardan en DB.
5. Cancelación/fallo no deja estado parcial.


## Criterio de fallo o parcial

- `NO CONSEGUIDA` si depende de forzar un KF o si procesa en dron.
- `PARCIAL` si captura varios pares pero la combinación no mejora y debe simplificarse.


## Riesgos

- Pequeños movimientos entre pares pueden crear ghosting; alinear/rechazar antes de promediar.
- Esperar demasiado puede bloquear misión; respetar timeout de tarea.


## Documentación a actualizar

Al ejecutar realmente la subfase, actualizar únicamente documentación respaldada por cambios/evidencia real:

```text
codex/pipeline/fase_8_nube_densa/historial/por_subfase/historial_8O.md
codex/pipeline/fase_8_nube_densa/historial/por_subfase/historial_8O_RESUMEN.md
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
