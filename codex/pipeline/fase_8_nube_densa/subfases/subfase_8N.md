# Subfase 8N — Análisis global de cobertura/calidad y necesidades de densificación


## Estado

```text
sin hacer
```


## Objetivo técnico

Analizar el ROI después/durante la misión sparse+dense para localizar regiones sin cobertura, poco densas, de baja confianza, con dobles superficies o incoherencias, y producir necesidades geométricas estructuradas que el gestor de tareas de Fase 6 convertirá/asignará como tareas. `dense_map_multi` detecta el problema; no decide qué dron lo ejecuta.


## Invariantes y decisiones cerradas

- La primera misión sparse también genera dense oportunista si la calidad móvil lo permite.
- No se repite toda la casa por defecto: las tareas posteriores se concentran en zonas deficientes.
- La librería dense no asigna drones ni gestiona lifecycle de tareas.
- Occupancy y DenseFusionMap aportan información distinta al análisis.


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
codex/pipeline/fase_6_tareas_trayectorias/pipeline_fase_6_RESUMEN.md
```
Localizar `task_server`, `task_lib` y los contratos reales de task submission
en `mission_msgs` posteriores a Fase 6.


## Diagnóstico de partida

Tras 8M existe un mapa dense corregible, pero todavía no hay una política para decidir dónde merece la pena volver. El ROI debe convertirse en una fuente explícita de “necesidades dense” para completar huecos o reparar errores sin una segunda pasada completa obligatoria.


## Archivos permitidos a modificar

```text
src/servidor/dense_map_multi/
src/servidor/dense_map_server/
src/servidor/<paquete_tareas_fase_6>/       # solo contrato de necesidad→tarea, no mover ownership a dense
src/servidor/multidron_gui/                 # visualización de zonas/needs si ya existe infraestructura genérica
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
mapping_roi
OccupancyGrid3D
DenseFusionMap
DenseQualityMetrics
DenseKeyFrameDatabase
task_server / task_lib / mission_msgs de Fase 6
```
Nuevos componentes sugeridos:
```text
DenseCoverageAnalyzer
DenseRegionNeed
DenseRepairReason
```



No inventar nombres de interfaces previas. Si alguno no existe con ese nombre, localizar el componente equivalente mediante documentación y búsqueda estática antes de implementar. Los nombres nuevos propuestos pueden materializarse con una estructura equivalente si conserva el ownership acordado.


## Cambios requeridos

1. Definir métricas por celda/región del ROI: observado, occupancy confidence, densidad/superficie, calidad, número de vistas y consistencia.
2. Detectar al menos: `UNCOVERED`, `LOW_DENSITY`, `LOW_QUALITY`, `INCONSISTENT_SURFACE` y `BAD_DENSE_KF` o equivalentes.
3. Agrupar voxels vecinos en regiones accionables para no crear una tarea por voxel.
4. Para un DenseKF concreto de mala calidad, emitir necesidad de recaptura referida a ese KF; para huecos sin KF, emitir región/objetivo geométrico.
5. Publicar/entregar `DenseRegionNeed` al gestor de tareas; el gestor decide prioridad, dron, cola y lifecycle según Fase 6.
6. Mostrar en GUI las regiones deficientes si la capa/overlay de Fase 7 lo permite sin convertir la GUI en autoridad.
7. Añadir markers `DENSE-COVERAGE-ANALYSIS`, `DENSE-REGION-NEED`, `DENSE-RECAPTURE-NEED`.


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
- No asignar drones dentro de `dense_map_multi`.
- No declarar “cubierto” solo porque existe un KF sparse.
- No usar GT para detectar huecos o errores online.


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
./codex/herramientas/build_selected_packages.sh dense_map_multi dense_map_server task_server multidron_gui
```

Si la separación de Fase 2 está ya ejecutada, respetar sus builds por grupo (`dron`, `servidor`, `simulacion`) y la sincronización de las dos copias de `orbslam3_msgs` y `mission_msgs`. Los comandos listados son el conjunto lógico esperado; el implementador debe usar el helper vigente y registrar en historial el comando real.


## Pruebas Gazebo requeridas

### Prueba 1 — ROI parcialmente cubierto

Finalizar una misión sparse+dense dejando intencionadamente una zona sin observar; debe producir una región de densificación, no una segunda misión completa.

### Prueba 2 — DenseKF malo

Marcar/obtener una subnube BAD de 8G: debe producir necesidad específica de recaptura de ese KF.

### Prueba 3 — Inconsistencia/doble superficie

Introducir una zona con fusión/registro de baja calidad y comprobar que se clasifica como reparación/inspección.

### Prueba 4 — Ownership

Verificar que `dense_map_multi` no elige `drone_id`; `task_server` recibe la
necesidad y hace la asignación.

No arrancar Gazebo artificialmente para una prueba puramente unitaria/de componente. Cuando la subfase necesite integración visual, temporal o multi-nodo, usar `run_simulation.sh` y registrar el comando exacto solo en el historial real.


## Patrones de reducción de logs

```text
DENSE-COVERAGE-ANALYSIS|DENSE-REGION-NEED|DENSE-RECAPTURE-NEED|TASK-ALLOC|mapping_roi|LOW_|UNCOVERED|INCONSISTENT|ERROR|FATAL|Killed
```

Los logs completos solo son entrada de `reduce_*`/`split_*`. Si el reducido no contiene evidencia suficiente, regenerarlo con patrones más específicos o crear un sublog temático; no abrir ni volcar el log completo.


## Criterio de éxito

La subfase se considera `CONSEGUIDA` solo si:

1. El analizador detecta huecos/zonas malas con datos del mapa, no GT.
2. Las necesidades se agrupan en regiones útiles y no explotan en miles de microtareas.
3. Un DenseKF BAD produce recaptura específica; una zona sin KF produce densificación de región.
4. La asignación de drones permanece en `task_server` de Fase 6.
5. GUI puede inspeccionar el motivo/calidad si está conectada.


## Criterio de fallo o parcial

- `NO CONSEGUIDA` si la librería dense empieza a asignar/ejecutar drones directamente.
- `NO CONSEGUIDA` si huecos claros del ROI se marcan cubiertos por mera presencia sparse.
- `PARCIAL` si detecta regiones pero aún no se conectan al gestor de tareas.


## Riesgos

- ROI grueso puede producir tareas demasiado grandes; agrupar/segmentar con parámetros.
- Ruido aislado no debe provocar recapturas infinitas.


## Documentación a actualizar

Al ejecutar realmente la subfase, actualizar únicamente documentación respaldada por cambios/evidencia real:

```text
codex/pipeline/fase_8_nube_densa/historial/por_subfase/historial_8N.md
codex/pipeline/fase_8_nube_densa/historial/por_subfase/historial_8N_RESUMEN.md
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
