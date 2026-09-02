# Subfase 8P — Registro de capturas HQ, recaptura de DenseKF y reparto por patches


## Estado

```text
sin hacer
```


## Objetivo técnico

Integrar de forma segura las `DenseHQCapture` de 8O en el mapa. Cubrir dos casos: (A) reparar un DenseKF malo volviendo a su pose global vigente, registrando una nueva captura estacionaria y sustituyendo atómicamente su subnube activa; (B) densificar una zona sin KF adecuado, registrar la captura contra sparse+dense y dividirla en patches rígidos asociados a KFs próximos, evitando asignación punto-a-punto que pueda “romper” una superficie tras optimizaciones.


## Invariantes y decisiones cerradas

- Recaptura de KF: objetivo es la pose global **vigente** del KF y orientación equivalente; no la pose histórica raw sin correcciones.
- La subnube antigua se mantiene solo hasta validar/commit de la nueva; después se elimina si no tiene uso real.
- No guardar imágenes antiguas/nuevas tras commit.
- Para zonas sin KF, asociar por patches/voxels/regiones rígidas, no cada punto al KF más cercano de forma independiente.
- Tras commit, no conservar una copia monolítica redundante si los patches canónicos contienen toda la geometría necesaria; si una asociación futura se invalida, puede generarse nueva tarea de recaptura.


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

Leer 8N para conocer `DenseRegionNeed`, 8O para el producto HQ, 8M para reglas de revalidación y Fase 6 para GO_TO/CAPTURE_DENSE reales.


## Diagnóstico de partida

Una captura estacionaria no tiene necesariamente un KF propio. Asociarla por una transformación arbitraria fija a un KF cercano puede degradarse con deformaciones del grafo; repartir cada punto entre KFs distintos puede rasgar una pared. Se necesita un commit explícito y geometría rígida por patches.


## Archivos permitidos a modificar

```text
src/servidor/dense_map_multi/
src/servidor/dense_map_server/
src/servidor/<paquete_tareas_fase_6>/
src/servidor/orbslam3_multi/              # lectura de KFs/poses; no tocar raw
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
DenseHQCapture
DenseKeyFrameDatabase
DenseRegistrationManager
GlobalPoseStore / KFs próximos
DenseRegionNeed
task_server/task_manager GO_TO/CAPTURE_DENSE
```
Nuevos componentes sugeridos:
```text
DenseRecaptureCommit
DensePatch
DensePatchAssignment
DenseHQMapIntegrator
```



No inventar nombres de interfaces previas. Si alguno no existe con ese nombre, localizar el componente equivalente mediante documentación y búsqueda estática antes de implementar. Los nombres nuevos propuestos pueden materializarse con una estructura equivalente si conserva el ownership acordado.


## Cambios requeridos

1. Caso A: generar tarea hacia `T_world_kf` vigente + orientación del KF, estabilizar/capturar en 8O y registrar la nueva nube contra la geometría del KF/zona.
2. Validar calidad y registro de la nueva subnube; mantener la antigua activa mientras la candidata no esté aceptada.
3. Hacer commit atómico: nueva subnube pasa a ACTIVE; antigua sale de mapas derivados y se elimina después del commit si no existe una necesidad concreta de rollback.
4. Caso B: registrar la captura HQ contra mapa sparse+dense actual y comprobar que la pose está suficientemente determinada; si es degenerada, pedir otra vista en vez de inventar 6-DoF.
5. Segmentar la captura aceptada en patches/voxels/regiones locales rígidas y asociar cada patch al KF utilizable más apropiado según proximidad/visibilidad/soporte, guardando geometría relativa a ese KF.
6. Insertar patches como contribuciones canónicas sin una segunda copia completa redundante; actualizar occupancy/fusion map y revisionar relaciones para 8M.
7. Si el registro HQ detecta una incoherencia de pose global, emitir candidato a 8J/8K, no deformar el patch local.
8. Añadir markers `DENSE-RECAPTURE-CANDIDATE`, `DENSE-RECAPTURE-COMMIT`, `DENSE-PATCH-ASSIGN`, `DENSE-HQ-INTEGRATE-REJECT`.


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
- No borrar el DenseKF antiguo antes de validar el nuevo.
- No asignar cada punto de una captura rígida a KFs distintos sin estructura de patch.
- No mover KFs directamente para hacer coincidir la captura; si el registro revela una corrección de pose, usar 8J/8K.


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
./codex/herramientas/build_selected_packages.sh dense_map_multi dense_map_server task_server task_manager orbslam3_multi
```

Si la separación de Fase 2 está ya ejecutada, respetar sus builds por grupo (`dron`, `servidor`, `simulacion`) y la sincronización de las dos copias de `orbslam3_msgs` y `mission_msgs`. Los comandos listados son el conjunto lógico esperado; el implementador debe usar el helper vigente y registrar en historial el comando real.


## Pruebas Gazebo requeridas

### Prueba 1 — Sustituir DenseKF malo

Seleccionar un KF BAD, volver a su pose global vigente, capturar parado y validar que la nueva subnube reemplaza a la antigua solo después de commit. La antigua desaparece de DB/mapa después del commit.

### Prueba 2 — Fallo de recaptura

Nueva captura peor o registro fallido: la antigua debe seguir ACTIVE.

### Prueba 3 — Densificar zona sin KF

Capturar una pared/zona sin DenseKF adecuado, registrar y dividir en patches asociados a KFs próximos. Comprobar continuidad visual.

### Prueba 4 — Optimización posterior

Mover de forma diferencial los KFs propietarios de varios patches; 8M debe detectar si la superficie deja de ser coherente y no mantener una asociación inválida silenciosamente.

No arrancar Gazebo artificialmente para una prueba puramente unitaria/de componente. Cuando la subfase necesite integración visual, temporal o multi-nodo, usar `run_simulation.sh` y registrar el comando exacto solo en el historial real.


## Patrones de reducción de logs

```text
DENSE-RECAPTURE|DENSE-PATCH|DENSE-HQ-INTEGRATE|ACTIVE|candidate|commit|reject|TASK-STATE|ERROR|FATAL|Killed
```

Los logs completos solo son entrada de `reduce_*`/`split_*`. Si el reducido no contiene evidencia suficiente, regenerarlo con patrones más específicos o crear un sublog temático; no abrir ni volcar el log completo.


## Criterio de éxito

La subfase se considera `CONSEGUIDA` solo si:

1. Una recaptura buena sustituye atómicamente a la mala y la antigua se elimina tras commit.
2. Una recaptura fallida no destruye el estado vigente.
3. Una zona sin KF puede integrarse mediante patches rígidos sin asignación punto-a-punto.
4. Las nuevas contribuciones participan en occupancy/fusion/revisión y pueden revalidarse.
5. No se guardan imágenes ni copias monolíticas redundantes innecesarias.


## Criterio de fallo o parcial

- `NO CONSEGUIDA` si un fallo de recaptura deja el KF sin nube activa.
- `NO CONSEGUIDA` si una superficie rígida se rasga por asociación punto-a-punto.
- `PARCIAL` si recaptura de KF funciona pero densificación sin KF sigue bloqueada por registro degenerado.


## Riesgos

- Volver a la pose de un KF puede no ser físicamente accesible exactamente; usar planner y tolerancia real, luego registro fino.
- Un patch demasiado pequeño se acerca al problema punto-a-punto; demasiado grande puede abarcar geometría no rígida.


## Documentación a actualizar

Al ejecutar realmente la subfase, actualizar únicamente documentación respaldada por cambios/evidencia real:

```text
codex/pipeline/fase_8_nube_densa/historial/por_subfase/historial_8P.md
codex/pipeline/fase_8_nube_densa/historial/por_subfase/historial_8P_RESUMEN.md
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
