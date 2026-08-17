# Subfase 8Q — Uso de occupancy+dense+sparse en planificación y comprobación local de profundidad


## Estado

```text
sin hacer
```


## Objetivo técnico

Integrar el mapa sparse, la geometría dense y la occupancy voxel en la planificación de trayectorias de Fase 6, manteniendo además comprobaciones locales de depth durante el vuelo para detectar obstáculos aún no mapeados. Si el dron se desplaza lateralmente hacia una dirección desconocida, debe orientar la cámara/comprobar profundidad antes de asumir que el corredor está libre, reutilizando el mecanismo ya existente en Fase 6 cuando esté implementado.


## Invariantes y decisiones cerradas

- Planificación global usa mapa vigente; seguridad local sigue usando depth reciente.
- Un voxel FREE puede existir sin punto dense porque un rayo lo atravesó.
- Depth de seguridad puede actualizar occupancy aunque no genere DenseKF.
- Fase 8 no reimplementa desde cero obstacle avoidance/yaw/look-side si Fase 6 ya lo hace; amplía su fuente de mapa y valida regresión.


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
Leer especialmente subfases reales de depth local, yaw perceptivo, replanning y reservas. Si falta el comportamiento acordado de mirar lateralmente/evitar obstáculos, corregir Fase 6 antes de continuar.


## Diagnóstico de partida

Fase 6 ya usa depth local como autoridad de obstáculos y sparse como ayuda. Tras Fase 8 existe información volumétrica mucho más útil: espacio observado libre, ocupado y geometría de superficies. Debe aprovecharse sin convertir un mapa viejo en sustituto del sensor local.


## Archivos permitidos a modificar

```text
src/servidor/dense_map_multi/                 # consultas occupancy/fusion
src/servidor/dense_map_server/
src/servidor/<planner_task_manager_fase_6>/
src/dron/dron_individual/                     # solo comportamiento local ya perteneciente a Fase 6
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
planner/replanning de Fase 6
obstacle avoidance/depth local de Fase 6
yaw/side-look logic de Fase 6
OccupancyGrid3D query
DenseFusionMap query
SparseGlobalMap query
TaskManager/reservas
```



No inventar nombres de interfaces previas. Si alguno no existe con ese nombre, localizar el componente equivalente mediante documentación y búsqueda estática antes de implementar. Los nombres nuevos propuestos pueden materializarse con una estructura equivalente si conserva el ownership acordado.


## Cambios requeridos

1. Crear API de consulta de occupancy/dense eficiente y snapshot-consistent para el planner, sin darle acceso mutable a las DBs.
2. Incluir OCCUPIED/superficies dense como obstáculos globales y FREE observado como información de corredor; UNKNOWN conserva incertidumbre y no se trata como libre garantizado.
3. Reutilizar el planificador/replanning de Fase 6 para trayectorias cortas y reservas Dron-Dron; dense no crea una ruta paralela.
4. Mantener comprobaciones depth cada intervalo/evento acordado durante el vuelo; cualquier observación válida actualiza occupancy con raycast de 8H.
5. Para desplazamiento lateral con lateral desconocido, reutilizar comportamiento de orientar/mirar y verificar depth antes de continuar; si ya existe un KF/mapa fiable de esa dirección, puede usarlo como información previa sin omitir seguridad si la política real exige sensor reciente.
6. Si aparece obstáculo no mapeado, detener/cancelar/liberar/replanificar según Fase 6 y conservar la nueva evidencia occupancy.
7. Añadir markers `DENSE-PLAN-QUERY`, `OCCUPANCY-PLAN`, `LOCAL-DEPTH-OCCUPANCY`, `SIDE-LOOK-CHECK`.


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
- No considerar UNKNOWN como FREE por comodidad.
- No eliminar comprobación local de depth porque exista mapa global.
- No meter planificación dentro de `dense_map_multi`.
- No usar ocupación del propio dron/otros drones como pared estática si el sistema de reservas ya los trata aparte.


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
./codex/herramientas/build_selected_packages.sh dense_map_multi dense_map_server <paquetes_fase_6_afectados> dron_individual
```

Si la separación de Fase 2 está ya ejecutada, respetar sus builds por grupo (`dron`, `servidor`, `simulacion`) y la sincronización de las dos copias de `orbslam3_msgs`. Los comandos listados son el conjunto lógico esperado; el implementador debe usar el helper vigente y registrar en historial el comando real.


## Pruebas Gazebo requeridas

### Prueba 1 — Trayectoria por espacio conocido

Planificar usando sparse+dense+occupancy y comprobar que atraviesa voxels FREE y evita OCCUPIED dentro de márgenes de Fase 6.

### Prueba 2 — UNKNOWN lateral

Ordenar desplazamiento lateral donde no hay información suficiente. El dron debe realizar la comprobación de vista/depth acordada antes de invadir el corredor.

### Prueba 3 — Obstáculo nuevo

Introducir/usar un obstáculo no presente en el mapa previo. Depth local lo detecta, actualiza occupancy y dispara el comportamiento seguro/replanning de Fase 6.

### Prueba 4 — Depth sin nube

La comprobación local actualiza FREE/OCCUPIED aunque no cree DenseKF ni DenseHQCapture.

No arrancar Gazebo artificialmente para una prueba puramente unitaria/de componente. Cuando la subfase necesite integración visual, temporal o multi-nodo, usar `run_simulation.sh` y registrar el comando exacto solo en el historial real.


## Patrones de reducción de logs

```text
DENSE-PLAN-QUERY|OCCUPANCY-PLAN|LOCAL-DEPTH-OCCUPANCY|SIDE-LOOK|LOCAL-OBSTACLE|REPLAN|TRAJ-CONFLICT|ERROR|FATAL|Killed
```

Los logs completos solo son entrada de `reduce_*`/`split_*`. Si el reducido no contiene evidencia suficiente, regenerarlo con patrones más específicos o crear un sublog temático; no abrir ni volcar el log completo.


## Criterio de éxito

La subfase se considera `CONSEGUIDA` solo si:

1. El planner consume sparse+dense+occupancy sin duplicar ownership.
2. UNKNOWN no se trata como libre garantizado y OCCUPIED se evita.
3. Depth local sigue protegiendo frente a obstáculos nuevos y actualiza occupancy.
4. El caso lateral desconocido usa la comprobación perceptiva acordada o demuestra que Fase 6 ya la resolvía correctamente.
5. No se introducen colisiones Dron-Dron fuera del sistema de reservas.


## Criterio de fallo o parcial

- `NO CONSEGUIDA` si el planner ignora OCCUPIED o trata UNKNOWN como FREE.
- `NO CONSEGUIDA` si se elimina/reduce la seguridad local de depth.
- `BLOQUEADA` si Fase 6 no implementó el comportamiento perceptivo acordado; volver a esa fase y revalidar.


## Riesgos

- Mapa estático incorrecto puede sesgar planner; sensor local sigue siendo última defensa.
- Occupancy gruesa requiere margen compatible con tamaño del dron.


## Documentación a actualizar

Al ejecutar realmente la subfase, actualizar únicamente documentación respaldada por cambios/evidencia real:

```text
codex/pipeline/fase_8_nube_densa/historial/por_subfase/historial_8Q.md
codex/pipeline/fase_8_nube_densa/historial/por_subfase/historial_8Q_RESUMEN.md
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
