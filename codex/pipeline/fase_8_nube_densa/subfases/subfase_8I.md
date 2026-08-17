# Subfase 8I — Reintegración incremental tras cambios de pose de KeyFrames


## Estado

```text
sin hacer
```


## Objetivo técnico

Mantener correctos `OccupancyGrid3D` y `DenseFusionMap` cuando una optimización sparse cambia poses de KFs. 8E ya hace que la subnube visual siga al KF; 8I debe retirar/inutilizar su contribución antigua en los productos fusionados y reintegrarla con la pose nueva, preferiblemente solo en bloques afectados.


## Invariantes y decisiones cerradas

- La DB local no se muta por una optimización.
- No reconstruir toda la casa por defecto ante un cambio pequeño; usar tracking de contribuciones/bloques sucios.
- La reintegración debe ser revision-aware y atómica desde el punto de vista de lectores.


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
codex/contexto/paquetes/orbslam3_multi/global_pose_store.md
codex/contexto/paquetes/orbslam3_multi/optimization_manager.md
```
Localizar cómo Fase 3 expone revisiones/commits y qué KFs cambian en cada apply.


## Diagnóstico de partida

Después de 8H, mover `DenseSubcloud_20` en la GUI no elimina automáticamente los puntos/pesos/rayos que `KF_20` había integrado antes en voxels. Sin reintegración pueden quedar dobles paredes o occupancy contradictoria congelada en la pose vieja.


## Archivos permitidos a modificar

```text
src/servidor/dense_map_multi/
src/servidor/dense_map_server/
src/servidor/orbslam3_multi/              # solo integración mínima para consumir eventos/revisiones, no cambiar solver
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
GlobalPoseStore revision/apply notifications
OptimizationApplyResult o ChangeSet equivalente
DenseKeyFrameDatabase revision
DenseFusionMap contributors
OccupancyGrid3D contributors/evidence
```
Nuevos componentes sugeridos:
```text
DenseReintegrationManager
DenseDirtyBlockSet
DenseIntegrationContribution
```



No inventar nombres de interfaces previas. Si alguno no existe con ese nombre, localizar el componente equivalente mediante documentación y búsqueda estática antes de implementar. Los nombres nuevos propuestos pueden materializarse con una estructura equivalente si conserva el ownership acordado.


## Cambios requeridos

1. Mantener por DenseKF la información mínima para identificar qué bloques/voxels derivaron de su integración.
2. Al detectar cambio de pose/revisión, calcular conjunto de DenseKF afectados y capturar snapshot consistente de nuevas poses.
3. Retirar o invalidar contribuciones antiguas de esos DenseKF sin destruir evidencia de otros KFs.
4. Reintegrar subnubes/rayos con nueva `T_world_kf`, actualizar pesos/confianza y marcar nueva revisión.
5. Procesar cálculo pesado fuera de locks y hacer commit breve/atómico de bloques recalculados.
6. Si llega otra revisión mientras se calcula, detectar resultado stale y recomputar/rechazar sin aplicar estado parcial.
7. Añadir markers `DENSE-REINTEGRATE-START`, `DENSE-REINTEGRATE-COMMIT`, `DENSE-REINTEGRATE-STALE`, `dirty_blocks`.


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
- No borrar la DB raw para “recrear desde cero”.
- No aceptar una revisión nueva a medias en el mapa fusionado visible.
- No recalcular KFs no afectados salvo fallback documentado por simplicidad inicial y coste acotado.


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
./codex/herramientas/build_selected_packages.sh dense_map_multi dense_map_server orbslam3_multi
```

Si la separación de Fase 2 está ya ejecutada, respetar sus builds por grupo (`dron`, `servidor`, `simulacion`) y la sincronización de las dos copias de `orbslam3_msgs`. Los comandos listados son el conjunto lógico esperado; el implementador debe usar el helper vigente y registrar en historial el comando real.


## Pruebas Gazebo requeridas

### Prueba 1 — Corrección controlada de un KF

Integrar una pared, cambiar la pose global del KF mediante un mecanismo de test/optimización real y verificar que desaparece la contribución vieja y aparece la nueva.

### Prueba 2 — Varios contribuyentes

Dos DenseKF observan la misma región; mover solo uno. La contribución del otro debe permanecer.

### Prueba 3 — Revisión stale

Disparar dos cambios de pose cercanos y comprobar que un cálculo basado en revisión antigua no sobrescribe la nueva.

### Prueba 4 — GUI

No debe aparecer una doble pared persistente tras concluir la reintegración.

No arrancar Gazebo artificialmente para una prueba puramente unitaria/de componente. Cuando la subfase necesite integración visual, temporal o multi-nodo, usar `run_simulation.sh` y registrar el comando exacto solo en el historial real.


## Patrones de reducción de logs

```text
DENSE-REINTEGRATE|dirty_blocks|stale|revision|commit|SERVER_OPTIMIZATION|ERROR|FATAL|Killed
```

Los logs completos solo son entrada de `reduce_*`/`split_*`. Si el reducido no contiene evidencia suficiente, regenerarlo con patrones más específicos o crear un sublog temático; no abrir ni volcar el log completo.


## Criterio de éxito

La subfase se considera `CONSEGUIDA` solo si:

1. Las contribuciones antiguas de KFs movidos se retiran/recalculan sin afectar KFs no movidos.
2. No queda geometría/occupancy persistente en la pose vieja tras commit.
3. Resultados stale no mutan el mapa live.
4. El coste se limita a bloques/KFs afectados en el escenario de prueba o existe un fallback claramente acotado y documentado.
5. Readers obtienen revisiones coherentes.


## Criterio de fallo o parcial

- `NO CONSEGUIDA` si quedan dobles paredes por contribuciones antiguas.
- `NO CONSEGUIDA` si una revisión obsoleta puede sobrescribir una nueva.
- `PARCIAL` si la corrección es coherente pero todavía reconstruye todo el mapa y el coste no cumple presupuesto.


## Riesgos

- Retirar pesos TSDF/voxels sin provenance suficiente puede ser difícil; diseñar contribuciones desde 8H.
- Reintegración concurrente puede generar lecturas híbridas si no hay snapshot/commit.


## Documentación a actualizar

Al ejecutar realmente la subfase, actualizar únicamente documentación respaldada por cambios/evidencia real:

```text
codex/pipeline/fase_8_nube_densa/historial/por_subfase/historial_8I.md
codex/pipeline/fase_8_nube_densa/historial/por_subfase/historial_8I_RESUMEN.md
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
