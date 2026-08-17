# Subfase 8K — Constraints dense y optimización del grafo sparse


## Estado

```text
sin hacer
```


## Objetivo técnico

Convertir únicamente medidas dense fiables de 8J en restricciones del problema de grafo de Fase 3 y reutilizar el mismo mecanismo de optimización/validación/commit para corregir poses de KFs. Tras el commit, DenseKF y MapPoints derivados se mueven por esas poses y 8I reintegra el mapa dense.


## Invariantes y decisiones cerradas

- La autoridad de poses sigue siendo `GlobalPoseStore` y el optimizador global de Fase 3.
- `dense_map_multi` detecta/describe constraints; no mantiene un optimizador paralelo que escriba poses.
- Por defecto se reutiliza `PoseGraphBuilder`/`OptimizationManager` o equivalentes reales.
- No crear una librería nueva de optimización de grafos preventivamente. Si el código real está tan acoplado que extraer una librería compartida es una refactorización material, parar y consultar al usuario antes de hacerlo.
- Hard fiducials siguen protegidos.


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
codex/contexto/paquetes/orbslam3_multi/pose_graph_builder.md
codex/contexto/paquetes/orbslam3_multi/pose_graph_problem.md
codex/contexto/paquetes/orbslam3_multi/optimization_manager.md
codex/contexto/paquetes/orbslam3_multi/global_pose_store.md
```
Leer el historial/contrato vigente de 3Q y la lógica de apply/rollback antes de tocar el grafo.


## Diagnóstico de partida

8J ya puede medir que dos DenseKF encajan con una transformación distinta a la pose sparse actual. Si esa corrección se aplicara directamente moviendo una nube se rompería la relación con KFs/MPs; por eso debe entrar como restricción al mismo grafo que ya resuelve loops/fiduciales.


## Archivos permitidos a modificar

```text
src/servidor/dense_map_multi/
src/servidor/dense_map_server/
src/servidor/orbslam3_multi/              # integración de nuevo tipo de constraint/edge y reutilización del solver
src/servidor/orbslam3_server/             # coordinación/worker si el ownership real lo requiere
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
PoseGraphBuilder
PoseGraphProblem / edge types
OptimizationManager::RunDryRun
OptimizationManager::ApplyUsefulResult / ApplyCandidateResult
GlobalPoseStore
worker secundario/colas de Fase 3
DenseRelativeConstraintCandidate (8J)
```



No inventar nombres de interfaces previas. Si alguno no existe con ese nombre, localizar el componente equivalente mediante documentación y búsqueda estática antes de implementar. Los nombres nuevos propuestos pueden materializarse con una estructura equivalente si conserva el ownership acordado.


## Cambios requeridos

1. Definir representación de edge/constraint dense en el grafo con medida relativa e information/covariance derivada de 8J.
2. Agregarla al builder solo si pasa validación de revisión, KFs existentes, calidad y compatibilidad básica con constraints actuales.
3. Ejecutar dry-run con el optimizador vigente y validar coste/residuales/movimientos/hard fiducials exactamente con las garantías de Fase 3.
4. Aplicar commit por el mecanismo existente; nunca mutar `RawMapDatabase`.
5. Después de apply aceptado, notificar revisión para 8I y 8M; la publicación sparse usa poses nuevas y 8L correcciones derivadas si existen.
6. Integrar el trabajo pesado en el worker/cola apropiada sin bloquear ingesta; respetar prioridad fiducial frente a dense si el diseño de Fase 3 lo exige.
7. Añadir markers `DENSE-GRAPH-CONSTRAINT`, `DENSE-GRAPH-DRYRUN`, `DENSE-GRAPH-COMMIT`, `DENSE-GRAPH-REJECT`.


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
- No escribir poses directamente desde `dense_map_multi`.
- No fijar o mover hard fiducials para satisfacer dense.
- No crear un segundo `GlobalPoseStore`.
- No extraer una nueva librería de grafo sin necesidad demostrada y acuerdo si el cambio es material.


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
./codex/herramientas/build_selected_packages.sh dense_map_multi dense_map_server orbslam3_multi orbslam3_server
```

Si la separación de Fase 2 está ya ejecutada, respetar sus builds por grupo (`dron`, `servidor`, `simulacion`) y la sincronización de las dos copias de `orbslam3_msgs`. Los comandos listados son el conjunto lógico esperado; el implementador debe usar el helper vigente y registrar en historial el comando real.


## Pruebas Gazebo requeridas

### Prueba 1 — Constraint dense útil

Usar un registro 8J fiable que corrige una deriva visible. El dry-run debe proponer poses coherentes; apply aceptado mueve KFs y activa 8I.

### Prueba 2 — Hard fiducial

El constraint dense entra en conflicto parcial con un anchor duro. El hard fiducial no puede moverse; la solución debe adaptarse o rechazarse.

### Prueba 3 — Constraint malo/stale

Modificar revisión o inyectar una medida incompatible. Debe rechazarse sin mutación parcial.

### Prueba 4 — Regresión sparse

Loops/fiduciales de Fase 3 siguen funcionando y el nuevo tipo de edge no cambia su semántica.

No arrancar Gazebo artificialmente para una prueba puramente unitaria/de componente. Cuando la subfase necesite integración visual, temporal o multi-nodo, usar `run_simulation.sh` y registrar el comando exacto solo en el historial real.


## Patrones de reducción de logs

```text
DENSE-GRAPH|SERVER_OPTIMIZATION|POSE-GRAPH|FIDUCIAL|hard|stale|dry_run|commit|reject|ERROR|FATAL|Killed
```

Los logs completos solo son entrada de `reduce_*`/`split_*`. Si el reducido no contiene evidencia suficiente, regenerarlo con patrones más específicos o crear un sublog temático; no abrir ni volcar el log completo.


## Criterio de éxito

La subfase se considera `CONSEGUIDA` solo si:

1. Una medida dense válida puede entrar al mismo grafo y producir un apply coherente.
2. Hard fiducials y raw data permanecen invariantes.
3. Una medida mala/stale no muta poses.
4. Tras commit, sparse y dense se recolocan mediante la nueva revisión y 8I limpia contribuciones antiguas.
5. No existe un segundo optimizador de poses independiente.


## Criterio de fallo o parcial

- `NO CONSEGUIDA` si dense puede escribir poses saltándose el optimizador/GlobalPoseStore.
- `NO CONSEGUIDA` si se mueve un hard fiducial o RawMapDatabase.
- `PARCIAL` si el edge se construye/dry-run funciona pero apply/reintegración no está validado.


## Riesgos

- Una constraint dense incorrecta puede deformar gran parte del mapa; mantener gates fuertes.
- El solver actual puede estar acoplado a tipos de edge existentes; no duplicar solver como atajo.


## Documentación a actualizar

Al ejecutar realmente la subfase, actualizar únicamente documentación respaldada por cambios/evidencia real:

```text
codex/pipeline/fase_8_nube_densa/historial/por_subfase/historial_8K.md
codex/pipeline/fase_8_nube_densa/historial/por_subfase/historial_8K_RESUMEN.md
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
