# Subfase 8D — DenseKeyFrameDatabase y contratos de estado denso


## Estado

```text
sin hacer
```


## Objetivo técnico

Crear la base canónica de subnubes densas por KeyFrame y las interfaces de datos que usarán las subfases posteriores. Cada entrada debe contener el KF de referencia y geometría local, no una copia de la nube global ni las imágenes originales. Definir además las interfaces —aún básicas— de las dos representaciones derivadas futuras: occupancy gruesa y fusión geométrica fina.


## Invariantes y decisiones cerradas

- Clave canónica de DenseKF: `(drone_id, map_epoch, kf_id)`.
- El `kf_id`/referencia al KF forma parte de la entrada; no es metadata opcional.
- La DB persiste subnube local y metadata necesaria; no guarda L/R.
- `DenseCoverage/Occupancy` y `DenseFusionMap` son productos derivados, no fuente de verdad de subnubes.
- La occupancy y la geometría fina tendrán escalas/roles diferentes para no duplicar información.


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
codex/contexto/paquetes/orbslam3_multi/raw_map_database.md
codex/contexto/paquetes/orbslam3_multi/global_pose_store.md
```
Tomar como referencia de diseño la separación raw/derivado de Fase 3, no copiar APIs sin necesidad.


## Diagnóstico de partida

8C puede construir subnubes locales, pero sin una base canónica sería fácil perder ownership, duplicar KFs o hornear poses `world`. Fase 8 necesita poder recolocar, sustituir y reintegrar cada contribución de forma independiente.


## Archivos permitidos a modificar

```text
src/servidor/dense_map_multi/include/dense_map_multi/
src/servidor/dense_map_multi/src/
src/servidor/dense_map_multi/test*/
src/servidor/dense_map_server/
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
DenseSubcloud (8C)
identidad de KF de orbslam3_multi/RawMapDatabase
GlobalPoseStore lookup por (drone_id,map_epoch,kf_id)
```
Nuevas clases sugeridas:
```text
DenseKeyFrameRecord
DenseKeyFrameDatabase
DenseKeyFrameStatus
DenseMapRevision / DenseDataRevision
OccupancyGrid3D interface            # mínima
DenseFusionMap interface             # mínima
```



No inventar nombres de interfaces previas. Si alguno no existe con ese nombre, localizar el componente equivalente mediante documentación y búsqueda estática antes de implementar. Los nombres nuevos propuestos pueden materializarse con una estructura equivalente si conserva el ownership acordado.


## Cambios requeridos

1. Implementar inserción/idempotencia/reemplazo controlado de `DenseKeyFrameRecord` por clave completa.
2. Guardar subnube local, KF, timestamp, calibración/versionado mínimo, quality placeholder y estado activo; no guardar imágenes.
3. Detectar duplicado exacto, KF inexistente, epoch obsoleto y subnube inválida antes de commit.
4. Exponer snapshots/lecturas thread-safe para builder/GUI/registro sin mantener locks durante cálculos pesados.
5. Definir revisiones de DB para que 8I/8M puedan detectar estado obsoleto.
6. Definir interfaces mínimas separadas para occupancy gruesa y fusion map fino, sin implementar todavía su algoritmo final.
7. Añadir markers `DENSE-DB-INSERT`, `DENSE-DB-REPLACE`, `DENSE-DB-REJECT`, `DENSE-DB-REVISION`.


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
- No almacenar una segunda copia concatenada de todos los puntos.
- No implementar todavía la fusión voxel completa de 8H.
- No mantener imágenes por si “algún día” se recalcula disparity.


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

Si la separación de Fase 2 está ya ejecutada, respetar sus builds por grupo (`dron`, `servidor`, `simulacion`) y la sincronización de las dos copias de `orbslam3_msgs`. Los comandos listados son el conjunto lógico esperado; el implementador debe usar el helper vigente y registrar en historial el comando real.


## Pruebas Gazebo requeridas

### Prueba 1 — Unitarios de DB

Insertar KFs de dos drones/epochs con IDs repetidos localmente, reemplazar una entrada y verificar aislamiento de claves/revisión.

### Prueba 2 — Snapshot concurrente

Simular inserciones mientras un reader toma snapshot; comprobar coherencia y ausencia de locks largos.

### Prueba 3 — Integración mínima

Recibir varios KFs reales de 8C y comprobar que solo queda una entrada activa por clave, sin imágenes persistentes.

No arrancar Gazebo artificialmente para una prueba puramente unitaria/de componente. Cuando la subfase necesite integración visual, temporal o multi-nodo, usar `run_simulation.sh` y registrar el comando exacto solo en el historial real.


## Patrones de reducción de logs

```text
DENSE-DB|revision|duplicate|epoch|REJECT|REPLACE|ERROR|FATAL|Segmentation fault|Killed
```

Los logs completos solo son entrada de `reduce_*`/`split_*`. Si el reducido no contiene evidencia suficiente, regenerarlo con patrones más específicos o crear un sublog temático; no abrir ni volcar el log completo.


## Criterio de éxito

La subfase se considera `CONSEGUIDA` solo si:

1. La DB distingue correctamente drones, epochs y KFs.
2. Cada record contiene su KF y nube local, sin depender de pose `world` almacenada.
3. No hay imágenes persistentes ni nube global concatenada duplicada.
4. Snapshots son coherentes y aptos para cálculo fuera de locks.
5. Las interfaces derivadas quedan preparadas sin adelantar 8H.


## Criterio de fallo o parcial

- `NO CONSEGUIDA` si un cambio de pose exige mutar la subnube local almacenada.
- `NO CONSEGUIDA` si colisionan IDs entre drones/epochs.
- `PARCIAL` si la DB funciona pero no tiene revisiones/snapshot coherente.


## Riesgos

- Guardar `world` dentro del record como autoridad crearía geometría congelada.
- Mantener mutex durante Open3D/voxelización bloquearía ingesta.


## Documentación a actualizar

Al ejecutar realmente la subfase, actualizar únicamente documentación respaldada por cambios/evidencia real:

```text
codex/pipeline/fase_8_nube_densa/historial/por_subfase/historial_8D.md
codex/pipeline/fase_8_nube_densa/historial/por_subfase/historial_8D_RESUMEN.md
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
