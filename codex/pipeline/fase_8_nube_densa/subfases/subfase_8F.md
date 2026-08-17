# Subfase 8F — Selección de DenseKeyFrames y eliminación de redundancia temprana


## Estado

```text
sin hacer
```


## Objetivo técnico

Reducir coste y repetición evitando construir/guardar nubes densas para KFs que aportan prácticamente la misma vista que un DenseKF reciente, sin eliminar ni modificar los KFs sparse de ORB-SLAM3.


## Invariantes y decisiones cerradas

- Todos los KFs siguen existiendo en sparse. Solo se decide si además generan DenseKF.
- Baseline inicial: distancia + diferencia angular respecto a DenseKF(s) relevantes; métricas más complejas solo si hacen falta.
- La selección debe ser parametrizable y no romper cobertura de zonas nuevas.


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

Leer evidencia real de 8E para entender la tasa de KFs y la redundancia observada. No fijar thresholds copiando ejemplos del chat.


## Diagnóstico de partida

8E densifica todos los KFs y puede generar muchas subnubes casi idénticas en el mismo sitio, duplicando CPU/red/memoria y sesgando fusiones posteriores. Se necesita una selección barata antes de ejecutar disparity completa cuando sea posible.


## Archivos permitidos a modificar

```text
src/servidor/dense_map_multi/
src/servidor/dense_map_server/
config/YAML del backend dense dentro del grupo servidor
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
pose global/local de KFs
covisibilidad/observaciones disponibles en Fase 3 (solo si se necesita una mejora)
```
Nueva clase sugerida:
```text
DenseKeyFrameSelector
DenseKeyFrameSelectionDecision
```



No inventar nombres de interfaces previas. Si alguno no existe con ese nombre, localizar el componente equivalente mediante documentación y búsqueda estática antes de implementar. Los nombres nuevos propuestos pueden materializarse con una estructura equivalente si conserva el ownership acordado.


## Cambios requeridos

1. Implementar selector previo a la construcción pesada de subnube cuando la información disponible lo permita.
2. Comparar traslación y rotación con el/los DenseKF recientes del mismo `(drone_id,map_epoch)` y aceptar vistas suficientemente nuevas.
3. Parametrizar umbrales en YAML del servidor; derivar valores iniciales de la evidencia 8E y documentarlos, no enterrarlos en C++.
4. Registrar razón de `ACCEPT/REJECT` y métricas para poder comprobar si se pierden zonas.
5. Si distancia+ángulo no basta en pruebas, añadir de forma incremental overlap/covisibilidad/MapPoints compartidos sin convertir el selector en un SLAM nuevo.
6. Añadir markers `DENSE-KF-SELECT accept=... reason=...` y contadores de reducción.


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
- No borrar KFs sparse ni alterar la política de creación de KFs de ORB-SLAM3.
- No descartar una vista solo porque esté espacialmente cerca si la orientación/área observada es nueva.
- No usar GT para decidir novedad.


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

### Prueba 1 — Zona repetida

Mantener el dron alrededor de una región que genera varios KFs; comprobar reducción de DenseKF y ausencia de pérdida visual significativa.

### Prueba 2 — Misma posición, nueva orientación

Girar hacia una pared/zona nueva: debe poder aceptarse aunque la traslación sea pequeña.

### Prueba 3 — Recorrido completo de 8E

Repetir recorrido y comparar número de DenseKF, coste y cobertura visual frente a “todos los KFs”.

No arrancar Gazebo artificialmente para una prueba puramente unitaria/de componente. Cuando la subfase necesite integración visual, temporal o multi-nodo, usar `run_simulation.sh` y registrar el comando exacto solo en el historial real.


## Patrones de reducción de logs

```text
DENSE-KF-SELECT|accept=true|accept=false|reason=|distance|angle|coverage|ERROR|FATAL|Killed
```

Los logs completos solo son entrada de `reduce_*`/`split_*`. Si el reducido no contiene evidencia suficiente, regenerarlo con patrones más específicos o crear un sublog temático; no abrir ni volcar el log completo.


## Criterio de éxito

La subfase se considera `CONSEGUIDA` solo si:

1. Se reduce de forma medible el número de DenseKF/subnubes frente a 8E.
2. Vistas claramente nuevas siguen entrando aunque el dron esté cerca de una posición anterior.
3. No se modifica la creación de KFs ORB ni el mapa sparse.
4. La decisión es reproducible/parametrizada y los rechazos tienen razón.
5. La GUI no muestra una pérdida de cobertura atribuible al filtro en la prueba acordada.


## Criterio de fallo o parcial

- `NO CONSEGUIDA` si el selector elimina vistas necesarias o toca KFs sparse.
- `PARCIAL` si reduce redundancia pero necesita criterios adicionales para giros/solapes.


## Riesgos

- Umbral demasiado agresivo produce huecos; demasiado laxo no reduce coste.
- Comparar solo con el último DenseKF puede fallar en retornos; ampliar únicamente si la prueba lo demuestra.


## Documentación a actualizar

Al ejecutar realmente la subfase, actualizar únicamente documentación respaldada por cambios/evidencia real:

```text
codex/pipeline/fase_8_nube_densa/historial/por_subfase/historial_8F.md
codex/pipeline/fase_8_nube_densa/historial/por_subfase/historial_8F_RESUMEN.md
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
