# Subfase 8G — Calidad de disparity, depth y subnubes densas


## Estado

```text
sin hacer
```


## Objetivo técnico

Mejorar de forma medible la calidad de cada DenseSubcloud antes de fusionarla: estudiar disparity/depth, poca textura, movimiento, bordes, oclusiones, ruido y outliers; introducir filtros y una estimación de confianza que permita distinguir subnubes buenas, provisionales o rechazables sin inventar geometría.


## Invariantes y decisiones cerradas

- La calidad debe evaluarse en cada nivel: imagen/disparity/depth/nube, no solo por “cómo se ve” al final.
- Poca textura puede perjudicar tanto ORB como estéreo pasivo; no rellenar superficies sin evidencia.
- Una nube móvil puede usarse si pasa calidad; una captura estacionaria posterior puede reemplazarla en 8P.
- Los thresholds de calidad deben ser parametrizables y justificarse con comparación contra el baseline 8E/8C.


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

Leer los scripts experimentales de visión solo para identificar pruebas históricas de planos/ICP/TSDF y comparar ideas de filtrado.


## Diagnóstico de partida

8E muestra nubes reales en movimiento y 8F reduce redundancia, pero siguen existiendo ruido en planos, bordes incorrectos, zonas sin disparity y diferencias por velocidad/giro. Antes de permitir fusión/correcciones sparse se necesita calidad explícita.


## Archivos permitidos a modificar

```text
src/servidor/dense_map_multi/
src/servidor/dense_map_server/
config/YAML dense del servidor
src/simulacion/simulacion_dron/          # escenarios de superficies variadas
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
StereoDepthProcessor
DenseSubcloudBuilder
DenseKeyFrameRecord quality placeholder
DenseKeyFrameSelector
```
Nuevos componentes sugeridos:
```text
DenseQualityEvaluator
DenseQualityMetrics
DepthFilterPipeline
```



No inventar nombres de interfaces previas. Si alguno no existe con ese nombre, localizar el componente equivalente mediante documentación y búsqueda estática antes de implementar. Los nombres nuevos propuestos pueden materializarse con una estructura equivalente si conserva el ownership acordado.


## Cambios requeridos

1. Calcular métricas de disparity/depth: proporción válida, rango, discontinuidades, inconsistencias y huecos.
2. Calcular métricas de nube: finitud, densidad útil, outliers, ruido local, planitud donde sea detectada de forma fiable y continuidad de bordes.
3. Incorporar estado de movimiento/velocidad lineal-angular disponible desde Fase 5 como metadata, no como sentencia automática de rechazo.
4. Aplicar filtros locales de rango, voxelización ligera si procede, outliers y filtros edge-aware/consistencia que la evidencia justifique.
5. Definir calidad/confianza y estado (`GOOD/PROVISIONAL/BAD` o equivalente) sin fijar nombres si ya existe un enum real.
6. Repetir comparación parado/lento/normal/giro tras los filtros y documentar si el modo híbrido móvil es viable o si debe apoyarse mucho más en 8O/8P.
7. Añadir markers `DENSE-QUALITY`, `DENSE-FILTER`, `DENSE-REJECT-QUALITY`.


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
- No “arreglar” ruido moviendo KFs todavía.
- No usar la nube sparse como máscara autoritaria para borrar dense; puede aportar contexto después, pero no sustituye evidencia de depth.
- No aceptar una nube únicamente porque tenga muchos puntos.


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

### Prueba 1 — Plano, esquina y bordes

Comparar métricas antes/después de filtros, con visualización GUI y error externo de geometría conocida.

### Prueba 2 — Textura alta vs baja

Medir disparity válida y comportamiento del filtro. Una zona sin evidencia debe quedar como desconocida/baja confianza, no como superficie inventada.

### Prueba 3 — Movimiento

Repetir parado, lento, normal y giro; comparar calidad. Si el movimiento normal sigue siendo inservible, registrar el resultado y priorizar tareas estacionarias en 8N–8P.

No arrancar Gazebo artificialmente para una prueba puramente unitaria/de componente. Cuando la subfase necesite integración visual, temporal o multi-nodo, usar `run_simulation.sh` y registrar el comando exacto solo en el historial real.


## Patrones de reducción de logs

```text
DENSE-QUALITY|DENSE-FILTER|DENSE-REJECT-QUALITY|valid_ratio|outlier|planar|motion|depth|ERROR|FATAL|Killed
```

Los logs completos solo son entrada de `reduce_*`/`split_*`. Si el reducido no contiene evidencia suficiente, regenerarlo con patrones más específicos o crear un sublog temático; no abrir ni volcar el log completo.


## Criterio de éxito

La subfase se considera `CONSEGUIDA` solo si:

1. Los filtros mejoran o mantienen las métricas geométricas respecto al baseline sin borrar sistemáticamente superficies válidas.
2. Cada DenseKF obtiene calidad/confianza reproducible y razón de rechazo cuando aplica.
3. Poca textura/oclusiones no generan geometría ficticia.
4. La decisión sobre uso de nubes móviles queda respaldada por evidencia comparativa.
5. No se utiliza GT en ninguna decisión funcional.


## Criterio de fallo o parcial

- `NO CONSEGUIDA` si el filtrado empeora de forma sistemática la geometría o inventa superficies.
- `PARCIAL` si la calidad se mide pero aún no permite separar nubes utilizables/malas.


## Riesgos

- Sobre-filtrado puede borrar bordes reales.
- Un score único puede ocultar fallos distintos; conservar métricas componentes.


## Documentación a actualizar

Al ejecutar realmente la subfase, actualizar únicamente documentación respaldada por cambios/evidencia real:

```text
codex/pipeline/fase_8_nube_densa/historial/por_subfase/historial_8G.md
codex/pipeline/fase_8_nube_densa/historial/por_subfase/historial_8G_RESUMEN.md
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
