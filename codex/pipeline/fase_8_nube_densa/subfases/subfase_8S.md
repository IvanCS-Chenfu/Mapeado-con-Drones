# Subfase 8S — Persistencia, exportación y metadatos del mapa denso


## Estado

```text
sin hacer
```


## Objetivo técnico

Definir y validar cómo se guarda/exporta el producto final de Fase 8 y la metadata mínima para reproducir qué revisión/calibración/parámetros lo generaron, sin convertir imágenes originales en parte del almacenamiento permanente ni duplicar innecesariamente raw+global.


## Invariantes y decisiones cerradas

- Producto mínimo final: nube/representación densa global exportable; malla/textura no son requisito de cierre.
- No guardar imágenes L/R.
- La exportación debe indicar revisión de sparse/dense y calibración/parámetros relevantes.
- La DB canónica sigue siendo subnubes/patches locales + relaciones; el export puede ser un snapshot global derivado.


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

Leer herramientas/formato de salida ya existentes en el proyecto si Fase 7/otros paquetes ofrecen PLY/PCD/serialización; reutilizar cuando sea adecuado.


## Diagnóstico de partida

El mapa se puede visualizar en vivo, pero para evaluación/TFG hace falta exportar un snapshot coherente y saber con qué calibración/revisión/filtros se obtuvo. Guardar solo un PLY sin metadata haría difícil reproducir o comparar pruebas.


## Archivos permitidos a modificar

```text
src/servidor/dense_map_multi/
src/servidor/dense_map_server/
config/YAML dense
codex/archivos_auxiliares/                  # solo resultados de prueba, no fuente de verdad runtime
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
DenseKeyFrameDatabase snapshot
DenseFusionMap snapshot
OccupancyGrid3D snapshot
GlobalPoseStore revision
calibración/param config
```
Nuevos componentes sugeridos:
```text
DenseMapExporter
DenseMapMetadata
```



No inventar nombres de interfaces previas. Si alguno no existe con ese nombre, localizar el componente equivalente mediante documentación y búsqueda estática antes de implementar. Los nombres nuevos propuestos pueden materializarse con una estructura equivalente si conserva el ownership acordado.


## Cambios requeridos

1. Definir snapshot coherente de exportación que capture revisiones sparse/dense compatibles.
2. Exportar nube/superficie dense en formato estándar disponible (PLY/PCD u otro justificado) con color si existe.
3. Exportar metadata separada: timestamp, map revision, calibración/versiones, voxel sizes, filtros, cantidad de DenseKF/patches y parámetros esenciales.
4. Permitir exportar occupancy/cobertura como producto separado si aporta valor a pruebas/planificación, sin mezclarlo con puntos de superficie.
5. Evitar duplicar permanentemente el mapa global en RAM: construir/streaming snapshot fuera de locks cuando sea posible.
6. Añadir markers `DENSE-EXPORT-START`, `DENSE-EXPORT-DONE`, `DENSE-EXPORT-REVISION`.


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
- No hacer de mesh/textura un requisito obligatorio.
- No exportar una revisión híbrida mientras hay reintegración/commit parcial.
- No guardar imágenes por defecto.


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

### Prueba 1 — Export snapshot

Tras una misión, exportar el mapa, cerrar el visualizador y abrir el fichero con herramienta externa/GUI offline si existe. Verificar escala, color y continuidad.

### Prueba 2 — Revisión durante export

Provocar una nueva optimización mientras se prepara un export. El resultado debe pertenecer a una revisión coherente o reintentarse; no mezclar estados.

### Prueba 3 — Metadata

Comprobar que puede identificarse qué calibración, voxel sizes, filtros y revisiones generaron el resultado.

No arrancar Gazebo artificialmente para una prueba puramente unitaria/de componente. Cuando la subfase necesite integración visual, temporal o multi-nodo, usar `run_simulation.sh` y registrar el comando exacto solo en el historial real.


## Patrones de reducción de logs

```text
DENSE-EXPORT|revision|snapshot|points|voxels|metadata|ERROR|FATAL|Killed
```

Los logs completos solo son entrada de `reduce_*`/`split_*`. Si el reducido no contiene evidencia suficiente, regenerarlo con patrones más específicos o crear un sublog temático; no abrir ni volcar el log completo.


## Criterio de éxito

La subfase se considera `CONSEGUIDA` solo si:

1. El mapa final puede exportarse en una revisión coherente y abrirse posteriormente.
2. La metadata permite identificar calibración, parámetros y revisiones esenciales.
3. No se guardan imágenes L/R ni se duplica una nube global en RAM sin necesidad.
4. Occupancy, si se exporta, permanece conceptualmente separada de la geometría dense.


## Criterio de fallo o parcial

- `NO CONSEGUIDA` si el export mezcla revisiones o pierde escala/frame.
- `PARCIAL` si el fichero geométrico es correcto pero falta metadata reproducible.


## Riesgos

- Exportaciones grandes pueden bloquear servidor si se hacen bajo lock.
- Formatos distintos pueden perder color/precision; documentar.


## Documentación a actualizar

Al ejecutar realmente la subfase, actualizar únicamente documentación respaldada por cambios/evidencia real:

```text
codex/pipeline/fase_8_nube_densa/historial/por_subfase/historial_8S.md
codex/pipeline/fase_8_nube_densa/historial/por_subfase/historial_8S_RESUMEN.md
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
