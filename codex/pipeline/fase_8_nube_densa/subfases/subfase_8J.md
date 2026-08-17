# Subfase 8J — Registro y fusión dense↔dense con medida relativa entre KeyFrames


## Estado

```text
sin hacer
```


## Objetivo técnico

Detectar solape entre subnubes densas de cualquier dron, registrar geométricamente pares fiables y obtener una medida relativa `KF_A ↔ KF_B` con calidad/covarianza/residual. La subfase fusiona en el producto dense derivado, pero todavía **no** permite que esa medida mueva el grafo sparse; esa autoridad se activa en 8K.


## Invariantes y decisiones cerradas

- La fusión considera todas las DenseKF de la DB independientemente del dron.
- No mover una subnube respecto a su KF para “hacerla encajar”: si el registro revela otra transformación, la conclusión es una posible restricción entre KFs.
- Debe existir rechazo fuerte de falsos registros; converger ICP no basta.
- Hard fiducials/poses sparse no se tocan en 8J.


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

Leer las estrategias de candidatos/verificación geométrica de Fase 3 para reutilizar principios de filtrado, no para mezclar loops ORB con constraints dense.


## Diagnóstico de partida

El voxel map puede mezclar contribuciones, pero si dos poses sparse tienen deriva aparecerán dobles superficies. Antes de optimizar el grafo se necesita demostrar que el registro dense puede medir transformaciones relativas fiables y rechazar pares ambiguos.


## Archivos permitidos a modificar

```text
src/servidor/dense_map_multi/
src/servidor/dense_map_server/
config/YAML dense
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
DenseKeyFrameDatabase
DenseQualityEvaluator
DenseFusionMap
candidatos espaciales por poses vigentes / overlap
```
Nuevos componentes sugeridos:
```text
DenseRegistrationCandidate
DenseRegistrationManager
DenseRegistrationResult
DenseRelativeConstraintCandidate
```



No inventar nombres de interfaces previas. Si alguno no existe con ese nombre, localizar el componente equivalente mediante documentación y búsqueda estática antes de implementar. Los nombres nuevos propuestos pueden materializarse con una estructura equivalente si conserva el ownership acordado.


## Cambios requeridos

1. Generar candidatos de solape usando pose/frustum/bloques observados sin comparar todos contra todos.
2. Registrar subnubes/superficies con Open3D o método C++ validado, usando inicialización sparse solo como punto de partida, no como evidencia independiente de corrección.
3. Calcular métricas de solape, residual, inliers/consistencia de normales/planos y degeneración geométrica.
4. Rechazar registros con poco soporte, soluciones no finitas, transformaciones incompatibles o geometría degenerada (por ejemplo, un único plano que no observa todos los DoF).
5. Producir una medida relativa entre KFs con confidence/information/covariance explícita y revisión de origen.
6. Usar el registro aceptado para fusionar la representación dense derivada; mantener raw subclouds separadas.
7. Validar intra-dron e inter-dron con la misma lógica.
8. Añadir markers `DENSE-REG-CANDIDATE`, `DENSE-REG-ACCEPT`, `DENSE-REG-REJECT`, `DENSE-RELATIVE-MEASUREMENT`.


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
- No llamar todavía a `OptimizationManager` para aplicar poses.
- No tratar automáticamente toda coincidencia de planos como pose 6-DoF completa; respetar observabilidad/ambigüedad.
- No usar GT para aceptar/rechazar un registro.


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

### Prueba 1 — Misma pared/esquina desde dos vistas

Comprobar registro correcto y una sola superficie derivada sin mover el grafo sparse.

### Prueba 2 — Dos drones

Dos drones observan la misma zona desde lados distintos; el registro debe ser independiente del `drone_id`.

### Prueba 3 — Par falso/ambigüedad

Nubes sin solape o con un único plano ambiguo deben rechazarse o producir constraint parcial/de baja información, nunca una corrección 6-DoF inventada.

### Prueba 4 — Deriva sparse simulada/real

Con poses globales ligeramente incoherentes, el registro debe medir una transformación relativa útil y registrarla sin aplicarla todavía.

No arrancar Gazebo artificialmente para una prueba puramente unitaria/de componente. Cuando la subfase necesite integración visual, temporal o multi-nodo, usar `run_simulation.sh` y registrar el comando exacto solo en el historial real.


## Patrones de reducción de logs

```text
DENSE-REG|DENSE-RELATIVE-MEASUREMENT|candidate|accept|reject|overlap|residual|degenerate|ERROR|FATAL|Killed
```

Los logs completos solo son entrada de `reduce_*`/`split_*`. Si el reducido no contiene evidencia suficiente, regenerarlo con patrones más específicos o crear un sublog temático; no abrir ni volcar el log completo.


## Criterio de éxito

La subfase se considera `CONSEGUIDA` solo si:

1. Pares realmente solapados generan medidas relativas coherentes con soporte geométrico.
2. Pares falsos/degenerados se rechazan de forma explícita.
3. La misma lógica funciona intra-dron e inter-dron.
4. Las subnubes raw no se mueven respecto a sus KFs; la medida se expresa entre KFs.
5. El grafo sparse/GlobalPoseStore no cambia en 8J.


## Criterio de fallo o parcial

- `NO CONSEGUIDA` si un falso registro se acepta sistemáticamente o si para fusionar se desacopla la nube de su KF.
- `PARCIAL` si registra bien casos simples pero no detecta degeneración/ambigüedad.


## Riesgos

- ICP puede converger a mínimos falsos en paredes repetitivas.
- La inicialización sparse puede crear un círculo vicioso; la aceptación debe apoyarse en geometría local medida.


## Documentación a actualizar

Al ejecutar realmente la subfase, actualizar únicamente documentación respaldada por cambios/evidencia real:

```text
codex/pipeline/fase_8_nube_densa/historial/por_subfase/historial_8J.md
codex/pipeline/fase_8_nube_densa/historial/por_subfase/historial_8J_RESUMEN.md
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
