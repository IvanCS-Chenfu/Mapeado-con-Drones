# Subfase 8C — Disparidad, profundidad y DenseSubcloud local al KeyFrame


## Estado

```text
sin hacer
```


## Objetivo técnico

Implementar en `dense_map_multi` la cadena servidor L/R → disparity → depth → puntos XYZRGB y producir una `DenseSubcloud` canónica expresada en el frame de cámara/KF que originó esas imágenes, sin transformarla permanentemente a `world`.

## Relacion con la visibilidad sparse de 3P

Fase 3P usa un z-buffer sparse, simetrico y temporal sobre dos subnubes de loop
para distinguir contradicciones visibles de oclusiones antes de penalizar
scores. No produce un mapa de profundidad y descarta trabajo negativo cuando
agota su presupuesto.

8C debe releer el contrato y la implementacion real de 3P para:

- comparar convenciones de proyeccion, intrinsecos, frames y clasificacion de
  oclusion;
- reutilizar tipos o telemetria solo si reducen duplicacion real;
- evaluar si `DepthResult` por KF puede ofrecer en el futuro evidencia mas
  completa que el z-buffer sparse.

No se conectara automaticamente depth dense con 3P. Una realimentacion futura
debe volver a la preparacion de Fase 3 y medir memoria, latencia, revisiones y
disponibilidad temporal. El mapa sparse debe seguir funcionando cuando 8C no
exista o no conserve depth para ese KF.


## Invariantes y decisiones cerradas

- El cálculo ocurre siempre en servidor.
- La subnube densa de un KF es local a ese KF; su colocación global se obtiene después con la pose global vigente del KF.
- Las imágenes no se guardan una vez construida/aceptada la subnube.
- OpenCV se usa para la cadena estéreo; Open3D puede usarse para estructuras/filtros posteriores, sin obligar aún a TSDF.


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
codex/pipeline/fase_3_sparse_global/subfases/subfase_3P_especificacion.md
codex/pipeline/fase_3_sparse_global/subfases/subfase_3P_implementacion.md
```

Antes de modificar código, leer también los resúmenes/contratos **reales ya ejecutados** de las fases anteriores de las que dependa esta subfase y los MD vigentes de cada paquete afectado en `codex/contexto/paquetes/`. En particular, Fase 8 depende funcionalmente de Fase 3 (sparse global), Fase 5 (poses globales), Fase 6 (tareas/trayectorias/obstáculos) y Fase 7 (GUI). Si el workspace posterior a esas fases usa nombres o rutas distintos a los de este contrato, localizar primero la fuente canónica y reutilizarla; no crear una segunda interfaz solo para satisfacer el nombre documental.



Además, leer específicamente:

```text
codex/contexto/paquetes/dron_individual/vision_experimental.md
codex/contexto/paquetes/orbslam3_ros2/stereo_slam_node.md
```
Revisar `vision_experimental` para recuperar ideas de parámetros/filtros, pero reimplementar la cadena de forma limpia en C++ servidor.


## Diagnóstico de partida

Tras 8B el servidor recibe el par exacto de un KF, pero aún no existe un producto denso canónico. Los scripts experimentales previos prueban múltiples enfoques de profundidad/nube/TSDF, pero no son parte del pipeline y pueden contener supuestos incompatibles.


## Archivos permitidos a modificar

```text
src/servidor/dense_map_multi/include/dense_map_multi/
src/servidor/dense_map_multi/src/
src/servidor/dense_map_server/
src/servidor/dense_map_multi/config/             # si el paquete instala YAML propios
src/simulacion/simulacion_dron/                  # solo escenario de validación
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
dense_map_multi::StereoCalibration (8A)
dense_map_multi::StereoDepthProcessor (8A)
evento KF+L/R recibido por dense_map_server (8B)
parámetros de rectificación/Camera.bf reales
```
Nuevos componentes sugeridos:
```text
DisparityResult
DepthResult
DensePoint
DenseSubcloud
DenseSubcloudBuilder
```



No inventar nombres de interfaces previas. Si alguno no existe con ese nombre, localizar el componente equivalente mediante documentación y búsqueda estática antes de implementar. Los nombres nuevos propuestos pueden materializarse con una estructura equivalente si conserva el ownership acordado.


## Cambios requeridos

1. Calcular disparity con un algoritmo OpenCV configurable y determinista; no hardcodear parámetros de una única escena.
2. Validar disparity: no finita, <=0 cuando no sea válida, fuera de rango, inconsistencia estéreo y bordes/oclusiones según la capacidad del algoritmo elegido.
3. Convertir a depth métrico con calibración 8A y rechazar profundidades fuera del rango físico/configurado.
4. Retroproyectar a XYZ en el frame del KF/cámara y asociar color de la imagen izquierda; definir claramente ejes/unidades.
5. Crear `DenseSubcloud` con identidad `(drone_id,map_epoch,kf_id)`, puntos locales y metadatos básicos de calidad necesarios para 8G/8D.
6. Aplicar solo filtros mínimos necesarios para una nube utilizable; los filtros de calidad avanzados quedan en 8G.
7. Descartar las imágenes L/R de trabajo después de construir la subnube y completar el envío/commit correspondiente.
8. Añadir markers `DENSE-DISPARITY`, `DENSE-DEPTH`, `DENSE-SUBCLOUD-BUILT` con conteos y rangos, sin inundar logs.
9. Documentar comparacion con la proyeccion sparse de 3P y decidir si existe
   una abstraccion reutilizable; no crear una integracion runtime hacia score
   sin acuerdo y prueba especificos.


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
- No transformar ni almacenar todavía una nube global fusionada.
- No corregir MapPoints sparse todavía.
- No modificar scores/fusiones de 3P ni sustituir su buffer durante 8C.
- No aplicar ICP entre KFs todavía.


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

### Prueba 1 — Unitarios matemáticos

Casos sintéticos de disparity conocida → profundidad/XYZ conocida, disparity inválida, NaN/inf y rango máximo.

### Prueba 2 — Plano y esquina en Gazebo

Generar KFs observando una pared y una esquina. Mostrar/guardar solo artefacto de prueba y medir planitud, escala y orientación; la geometría conocida se usa como métrica externa.

### Prueba 3 — Bordes y poca textura

Observar zonas con textura alta/baja y discontinuidad de profundidad para registrar dónde falla la cadena, sin “rellenar” todavía artificialmente.

```bash
./codex/herramientas/run_simulation.sh \
  --prueba fase_8_8C \
  --launch "ros2 launch simulacion_dron multi_dron.launch.py" \
  --post-scenario-wait-sec 20
```

No arrancar Gazebo artificialmente para una prueba puramente unitaria/de componente. Cuando la subfase necesite integración visual, temporal o multi-nodo, usar `run_simulation.sh` y registrar el comando exacto solo en el historial real.


## Patrones de reducción de logs

```text
DENSE-DISPARITY|DENSE-DEPTH|DENSE-SUBCLOUD-BUILT|valid_pixels|invalid|depth_min|depth_max|ERROR|FATAL|Segmentation fault|Killed
```

Los logs completos solo son entrada de `reduce_*`/`split_*`. Si el reducido no contiene evidencia suficiente, regenerarlo con patrones más específicos o crear un sublog temático; no abrir ni volcar el log completo.


## Criterio de éxito

La subfase se considera `CONSEGUIDA` solo si:

1. Los tests matemáticos de disparity/depth/XYZ pasan.
2. Cada `DenseSubcloud` está expresada en el frame del KF y mantiene identidad completa.
3. Plano/esquina presentan escala y orientación coherentes sin GT funcional.
4. Los píxeles inválidos se rechazan de forma explícita y no generan puntos no finitos.
5. Las imágenes no quedan almacenadas tras completar la construcción.


## Criterio de fallo o parcial

- `NO CONSEGUIDA` si aparecen puntos no finitos, escala incorrecta o frame ambiguo.
- `PARCIAL` si la nube existe pero no se ha validado en bordes/poca textura.
- `BLOQUEADA` si no hay calibración fiable de 8A.


## Riesgos

- Errores de signo/ejes al convertir `Tcw/Twc` no deben mezclarse con XYZ local.
- Rellenar holes agresivamente puede inventar superficies.
- Parámetros estéreo sobreajustados a una única pared.


## Documentación a actualizar

Al ejecutar realmente la subfase, actualizar únicamente documentación respaldada por cambios/evidencia real:

```text
codex/pipeline/fase_8_nube_densa/historial/por_subfase/historial_8C.md
codex/pipeline/fase_8_nube_densa/historial/por_subfase/historial_8C_RESUMEN.md
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
