# Subfase 8L — Refinamiento dense→MapPoints sin modificar raw


## Estado

```text
sin hacer
```


## Objetivo técnico

Permitir que superficies densas fiables refinen la posición publicada de MapPoints sparse —por ejemplo, acercando MPs ruidosos de una pared a su superficie— mediante una base de correcciones derivadas. `RawMapDatabase` permanece intacta y `GlobalMapBuilder` consulta estas correcciones al construir la nube sparse global.


## Invariantes y decisiones cerradas

- No se devuelven MPs corregidos a ORB-SLAM3.
- La corrección vive en servidor y se aplica a la vista/mapa global derivado.
- Siempre que sea posible, MP y evidencia dense se expresan respecto al mismo KF de referencia para que una optimización global los mueva coherentemente.
- La corrección no sustituye el tracking ni crea MapPoints nuevos.


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
codex/contexto/paquetes/orbslam3_multi/global_map_builder.md
codex/contexto/paquetes/orbslam3_multi/raw_map_database.md
codex/contexto/paquetes/orbslam3_multi/global_pose_store.md
```
Respetar la regla vigente de `GlobalMapBuilder`: publicar MPs desde un KF utilizable con pose world coherente.


## Diagnóstico de partida

El mapa sparse puede tener ruido normal a superficies, especialmente en zonas planas. La nube dense aporta geometría más rica, pero modificar la posición raw rompería la separación de Fase 3 y podría interferir con ORB local.


## Archivos permitidos a modificar

```text
src/servidor/dense_map_multi/
src/servidor/orbslam3_multi/include/orbslam3_multi/
src/servidor/orbslam3_multi/src/global_map_builder.cpp
src/servidor/orbslam3_server/                  # publicación/telemetría si corresponde
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
GlobalMapBuilder
RawMapDatabase MapPoint + reference/observer KFs
DenseFusionMap / superficies fiables
DenseRegistration/quality
```
Nuevas clases sugeridas:
```text
SparseDenseCorrection
SparseDenseCorrectionDatabase
SparseDenseRefiner
```



No inventar nombres de interfaces previas. Si alguno no existe con ese nombre, localizar el componente equivalente mediante documentación y búsqueda estática antes de implementar. Los nombres nuevos propuestos pueden materializarse con una estructura equivalente si conserva el ownership acordado.


## Cambios requeridos

1. Definir corrección por identidad raw `(drone_id,map_epoch,local_mappoint_id)` + KF de referencia + fuente dense + revisión/confianza.
2. Encontrar correspondencias MP↔superficie solo con criterios geométricos y visibilidad/observación suficientes; rechazar casos ambiguos.
3. Calcular offset/posición refinada en el frame del KF de referencia cuando sea posible, sin mutar el raw.
4. Implementar DB derivada thread-safe, con invalidación/revisión para 8M.
5. Modificar `GlobalMapBuilder` para consultar una corrección válida y usarla al producir `GlobalSparsePoint`; si no existe/está stale, usar la lógica sparse vigente.
6. Exponer en GUI/debug diferencia raw→refinada para poder revisar que no se “aplana” geometría incorrectamente.
7. Añadir markers `DENSE-MP-REFINE`, `DENSE-MP-REFINE-REJECT`, `DENSE-MP-CORRECTION-DB`.


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
- No escribir `MapPoint.position` en RawMapDatabase.
- No modificar descriptores/observaciones ORB para forzar una asociación.
- No proyectar todo MP al plano más cercano sin verificar correspondencia/soporte.


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
./codex/herramientas/build_selected_packages.sh dense_map_multi orbslam3_multi orbslam3_server
```

Si la separación de Fase 2 está ya ejecutada, respetar sus builds por grupo (`dron`, `servidor`, `simulacion`) y la sincronización de las dos copias de `orbslam3_msgs`. Los comandos listados son el conjunto lógico esperado; el implementador debe usar el helper vigente y registrar en historial el comando real.


## Pruebas Gazebo requeridas

### Prueba 1 — Pared plana

Seleccionar MPs observados por KFs con DenseSubcloud/superficie fiable. Comparar distancia a plano antes/después en la **salida derivada**, no en raw.

### Prueba 2 — Borde/esquina

Asegurar que MPs cerca de cambio de plano no se proyectan al plano incorrecto.

### Prueba 3 — Optimización del KF

Mover el KF con el optimizador: MP refinado y superficie local asociada deben transformarse coherentemente; RawMapDatabase debe conservar coordenadas originales.

### Prueba 4 — Fallback

Invalidar la corrección: GlobalMapBuilder vuelve a la posición sparse vigente sin crash.

No arrancar Gazebo artificialmente para una prueba puramente unitaria/de componente. Cuando la subfase necesite integración visual, temporal o multi-nodo, usar `run_simulation.sh` y registrar el comando exacto solo en el historial real.


## Patrones de reducción de logs

```text
DENSE-MP-REFINE|DENSE-MP-CORRECTION|GLOBAL-MAP-BUILDER|raw|refined|stale|ERROR|FATAL|Killed
```

Los logs completos solo son entrada de `reduce_*`/`split_*`. Si el reducido no contiene evidencia suficiente, regenerarlo con patrones más específicos o crear un sublog temático; no abrir ni volcar el log completo.


## Criterio de éxito

La subfase se considera `CONSEGUIDA` solo si:

1. RawMapDatabase es byte/lógicamente inalterada por el refinamiento.
2. GlobalMapBuilder publica la corrección solo cuando es válida y vuelve a sparse si no lo es.
3. En una pared fiable disminuye el ruido normal sin deformar bordes/esquinas.
4. Optimizar el KF no rompe la relación local MP↔dense.
5. No se crean nuevos MapPoints ni se toca tracking ORB.


## Criterio de fallo o parcial

- `NO CONSEGUIDA` si se modifica raw o se envían correcciones al dron.
- `NO CONSEGUIDA` si el refinamiento degrada bordes/esquinas sistemáticamente.
- `PARCIAL` si la DB/corrección funciona pero GlobalMapBuilder aún no la consume.


## Riesgos

- Asociación errónea de MP a superficie puede “pegar” puntos a paredes incorrectas.
- Un MP observado por varios KFs requiere escoger referencia estable según reglas vigentes del builder.


## Documentación a actualizar

Al ejecutar realmente la subfase, actualizar únicamente documentación respaldada por cambios/evidencia real:

```text
codex/pipeline/fase_8_nube_densa/historial/por_subfase/historial_8L.md
codex/pipeline/fase_8_nube_densa/historial/por_subfase/historial_8L_RESUMEN.md
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
