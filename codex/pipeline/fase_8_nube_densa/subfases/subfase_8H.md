# Subfase 8H — Mapa voxel global: occupancy gruesa y fusión geométrica fina


## Estado

```text
sin hacer
```


## Objetivo técnico

Construir las dos representaciones globales derivadas del denso: (1) una rejilla/voxelización gruesa del ROI para `UNKNOWN/FREE/OCCUPIED`, evidencia y cobertura, actualizable incluso desde depth que no genera nube; y (2) un `DenseFusionMap` fino, sparse por bloques/voxels observados y apto para fusión tipo voxel/TSDF sin reservar todo el volumen fino del ROI.


## Invariantes y decisiones cerradas

- No se mantiene una PointCloud global que sea simplemente la concatenación de todas las subnubes.
- Occupancy y geometría fina tienen escalas distintas y roles distintos para evitar redundancia.
- Occupancy es evidencia estática acumulativa, no un timeout temporal: observaciones posteriores pueden aumentar o reducir confianza y cambiar OCCUPIED↔FREE.
- Un rayo depth que atraviesa voxels aporta evidencia FREE aunque ese frame no se convierta en DenseKF.
- El mapa fino puede empezar como voxel hash/blocks ponderados; TSDF es una evolución permitida, no obligación inicial.


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

Leer el contrato real de ROI/flight_bounds de Fase 6. `mapping_roi` define el volumen que interesa observar; no confundirlo con los límites de vuelo.


## Diagnóstico de partida

La DB de 8D conserva subnubes, pero no responde eficientemente a “qué espacio está libre/ocupado?”, “qué zona está poco observada?” o “cómo fusiono superficies sin concatenar puntos?”. Además, planificación necesita saber espacio libre incluso sin puntos de superficie.


## Archivos permitidos a modificar

```text
src/servidor/dense_map_multi/
src/servidor/dense_map_server/
config/YAML dense del servidor
src/servidor/<paquete_tareas_fase_6>/       # solo lectura de ROI/contrato; integración funcional se hace en 8Q
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
mapping_roi / flight_bounds de Fase 6
DenseKeyFrameDatabase
DenseQualityMetrics
DepthResult / ray geometry
```
Nuevas clases sugeridas:
```text
OccupancyVoxel
OccupancyGrid3D
OccupancyIntegrator
DenseFusionVoxel / DenseVoxelBlock
DenseFusionMap
DenseFusionIntegrator
```



No inventar nombres de interfaces previas. Si alguno no existe con ese nombre, localizar el componente equivalente mediante documentación y búsqueda estática antes de implementar. Los nombres nuevos propuestos pueden materializarse con una estructura equivalente si conserva el ownership acordado.


## Cambios requeridos

1. Definir resolución gruesa de occupancy y resolución fina de geometría como parámetros independientes del servidor.
2. Implementar occupancy del ROI con estado, confianza, `observed_count`, evidencia free/occupied y metadata mínima de revisión; no almacenar puntos redundantes.
3. Integrar rayos depth: voxels atravesados válidamente reciben evidencia FREE; superficie final recibe evidencia OCCUPIED. Una observación posterior que atraviesa un voxel antes ocupado debe reducir su confianza y poder cambiarlo a FREE con evidencia suficiente.
4. Implementar `DenseFusionMap` fino con asignación sparse por bloques/voxel hash únicamente donde hay superficie/observaciones útiles.
5. Fusionar puntos de DenseKF con peso derivado de calidad y conservar contribuyentes/revisión suficientes para 8I.
6. Permitir que depth directo de navegación actualice occupancy sin obligar a crear DenseKF ni geometría fina persistente.
7. Exponer productos para GUI/consulta: occupancy/cobertura resumida y superficie densa derivada; no duplicar todos los puntos raw.
8. Añadir markers `DENSE-OCCUPANCY-UPDATE`, `DENSE-FUSION-INTEGRATE`, `DENSE-VOXEL-STATS`.


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
- No reservar una matriz fina completa para todo el ROI si el coste crece con volumen vacío.
- No considerar FREE un voxel que nunca ha sido atravesado/observado.
- No usar ausencia de punto como evidencia FREE si el sensor no tenía visibilidad/rango válido.


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

### Prueba 1 — Raycast FREE/OCCUPIED

Con depth sintético/controlado, comprobar que los voxels hasta una pared quedan observados/libres y el voxel de superficie ocupado; fuera del frustum permanece UNKNOWN.

### Prueba 2 — Evidencia contradictoria

Marcar un voxel OCCUPIED con varias observaciones y después atravesarlo repetidamente sin retorno; verificar descenso de confianza y transición coherente a FREE cuando la evidencia nueva domina.

### Prueba 3 — ROI y mapa fino sparse

Recorrer una parte de la casa. El coverage grid representa todo el ROI a baja resolución, mientras el mapa fino solo asigna bloques observados. Medir memoria y comprobar que no se reserva el ROI fino completo.

### Prueba 4 — Depth sin DenseKF

Emitir una comprobación lateral/normal de depth que no se almacena como nube. Debe actualizar occupancy y no crear DenseKeyFrameRecord.

No arrancar Gazebo artificialmente para una prueba puramente unitaria/de componente. Cuando la subfase necesite integración visual, temporal o multi-nodo, usar `run_simulation.sh` y registrar el comando exacto solo en el historial real.


## Patrones de reducción de logs

```text
DENSE-OCCUPANCY|DENSE-FUSION-INTEGRATE|DENSE-VOXEL-STATS|UNKNOWN|FREE|OCCUPIED|confidence|allocated_blocks|ERROR|FATAL|Killed
```

Los logs completos solo son entrada de `reduce_*`/`split_*`. Si el reducido no contiene evidencia suficiente, regenerarlo con patrones más específicos o crear un sublog temático; no abrir ni volcar el log completo.


## Criterio de éxito

La subfase se considera `CONSEGUIDA` solo si:

1. Occupancy distingue UNKNOWN/FREE/OCCUPIED con evidencia observable y puede corregir una creencia antigua.
2. Depth no persistido puede actualizar occupancy sin crear puntos/nubes en DB.
3. DenseFusionMap ocupa memoria principalmente en regiones observadas y no duplica una concatenación raw.
4. Las dos resoluciones son independientes, parametrizadas y tienen ownership claro.
5. GUI/consumidores pueden consultar cobertura/superficie sin bloquear integradores.


## Criterio de fallo o parcial

- `NO CONSEGUIDA` si ausencia de puntos se interpreta automáticamente como FREE.
- `NO CONSEGUIDA` si se reserva el grid fino completo y rompe el presupuesto de memoria en el ROI de prueba.
- `PARCIAL` si occupancy funciona pero la fusión fina sigue siendo una concatenación temporal sin estructura.


## Riesgos

- Raycast con frame/pose incorrectos corrompe gran volumen de occupancy.
- Resolución gruesa demasiado grande puede ocultar pasillos/obstáculos; parametrizar y validar.


## Documentación a actualizar

Al ejecutar realmente la subfase, actualizar únicamente documentación respaldada por cambios/evidencia real:

```text
codex/pipeline/fase_8_nube_densa/historial/por_subfase/historial_8H.md
codex/pipeline/fase_8_nube_densa/historial/por_subfase/historial_8H_RESUMEN.md
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
