# Subfase 8T — Integración final, regresión multi-dron y cierre de Fase 8


## Estado

```text
sin hacer
```


## Objetivo técnico

Validar de extremo a extremo la estrategia híbrida acordada: misión sparse que genera dense oportunista, optimizaciones que recolocan/reintegran, cooperación dense↔sparse, occupancy para navegación, análisis de huecos, tareas correctivas estacionarias y export final. Corroborar explícitamente que Fases 3–7 siguen correctas; si algo se ve mal por una fase anterior, volver a ella y arreglarla antes de cerrar.


## Invariantes y decisiones cerradas

- La prueba final es multi-dron.
- El mapa dense final no usa GT funcional; GT solo mide error externo.
- No se exige una segunda pasada completa: el servidor genera tareas correctivas solo donde hace falta. Si las nubes móviles resultaron inservibles según 8E/8G, el modo válido puede degenerar a sparse primero + densificación estacionaria, pero debe quedar documentado con evidencia.
- La GUI es herramienta de validación importante: no esconder dobles paredes o inconsistencias.


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

Leer todos los `historial_<ID>_RESUMEN.md` que ya existan para 8A–8S y los cierres reales de Fases 3–7. No ejecutar una “prueba final” si quedan subfases obligatorias bloqueadas sin decisión.


## Diagnóstico de partida

Las piezas pueden pasar tests aislados y aun fallar juntas por red, revisiones, tareas, fusión o planificación. El cierre exige una misión representativa y regresiones explícitas, no solo un screenshot bonito.


## Archivos permitidos a modificar

```text
src/servidor/dense_map_multi/
src/servidor/dense_map_server/
src/servidor/orbslam3_multi/
src/servidor/orbslam3_server/
src/servidor/multidron_gui/
src/servidor/<paquetes_fase_6>/
src/dron/orbslam3_ros2/
src/dron/dron_individual/
src/simulacion/simulacion_dron/
codex/archivos_auxiliares/trayectorias/
```
Solo corregir regresiones atribuibles a Fase 8. Para fallos de ownership previo, aplicar la puerta de validación y volver a la fase correspondiente.


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
launch multi-dron final
TaskManager/MissionManager Fase 6
multidron_gui
DenseMapServer / DenseMapMulti
GlobalMapServer / orbslam3_multi
scenario_runner_node
exporter 8S
```



No inventar nombres de interfaces previas. Si alguno no existe con ese nombre, localizar el componente equivalente mediante documentación y búsqueda estática antes de implementar. Los nombres nuevos propuestos pueden materializarse con una estructura equivalente si conserva el ownership acordado.


## Cambios requeridos

1. Preparar escenario reproducible de dos o más drones que cubra el ROI y genere sparse+dense oportunista.
2. Durante la misión, comprobar que DenseKF siguen poses de KFs y que loops/fiduciales/dense constraints pueden optimizar sin congelar geometría.
3. Comprobar que fusión dense-dense no deja dobles paredes persistentes en solapes normales y que MapPoints refinados siguen raw intacto.
4. Al terminar la primera misión, ejecutar análisis 8N y comprobar que genera solo necesidades de zonas malas/huecos.
5. Ejecutar al menos una recaptura de DenseKF malo y una densificación HQ de zona deficiente si el escenario las produce/puede inducirlas.
6. Usar occupancy+dense+sparse para trayectorias y una comprobación local de depth que actualice voxels sin crear nube.
7. Ejecutar una optimización posterior a datos HQ/patches y verificar 8I/8M.
8. Medir rendimiento/red/memoria según 8R y exportar snapshot final 8S.
9. Corroborar visual y funcionalmente Fases 3–7; cualquier fallo de dato fuente obliga a volver a la fase propietaria y repetir la prueba después.
10. Cerrar documentación con conclusión `CONSEGUIDA/PARCIAL/NO CONSEGUIDA/BLOQUEADA` respaldada por evidencia real.


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
- No bajar criterios/thresholds solo para que pase la prueba final.
- No usar GT para recolocar/corregir el mapa durante la prueba.
- No cerrar con dobles paredes persistentes o occupancy incoherente sin causa explicada y aceptada.


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
./codex/herramientas/build_selected_packages.sh dense_map_multi dense_map_server orbslam3_multi orbslam3_server multidron_gui <paquetes_fase_6> orbslam3 dron_individual simulacion_dron
```
Ajustar al conjunto real posterior a Fases 2–7 y hacer builds pequeños por grupo si los paquetes pesados lo requieren.

Si la separación de Fase 2 está ya ejecutada, respetar sus builds por grupo (`dron`, `servidor`, `simulacion`) y la sincronización de las dos copias de `orbslam3_msgs`. Los comandos listados son el conjunto lógico esperado; el implementador debe usar el helper vigente y registrar en historial el comando real.


## Pruebas Gazebo requeridas

### Prueba 1 — Misión final multi-dron de mapeo y corrección

Secuencia mínima:

1. arrancar sistema completo con dos o más drones y GUI;
2. ejecutar misión de mapping sparse del ROI;
3. generar DenseKF oportunistas según 8F/8G;
4. observar al menos una optimización/revisión y reintegración correcta;
5. completar/fusionar solapes dense intra/inter-dron;
6. analizar ROI con 8N;
7. ejecutar tareas correctivas HQ necesarias o inducidas para probar el camino;
8. planificar/mover usando occupancy+dense+sparse y comprobar depth local;
9. exportar mapa final;
10. revisar visualmente paredes, esquinas, huecos y coherencia sparse/dense.

```bash
./codex/herramientas/run_simulation.sh \
  --prueba fase_8_final \
  --launch "ros2 launch simulacion_dron multi_dron.launch.py" \
  --post-scenario-wait-sec 60
```

### Prueba 2 — Regresión de fases anteriores

Repetir las smoke/regresiones mínimas acordadas de sparse, pose, tareas/reservas/obstáculos y GUI. Fase 8 no puede declararse conseguida si introdujo regresiones materiales.

No arrancar Gazebo artificialmente para una prueba puramente unitaria/de componente. Cuando la subfase necesite integración visual, temporal o multi-nodo, usar `run_simulation.sh` y registrar el comando exacto solo en el historial real.


## Patrones de reducción de logs

```text
DENSE-|SERVER_OPTIMIZATION|FIDUCIAL|LOOP|GLOBAL-MAP|TASK-|TRAJ-|LOCAL-OBSTACLE|OCCUPANCY|GUI-|SCENARIO-RUNNER|RESULT|success|ERROR|FATAL|Segmentation fault|Killed
```

Los logs completos solo son entrada de `reduce_*`/`split_*`. Si el reducido no contiene evidencia suficiente, regenerarlo con patrones más específicos o crear un sublog temático; no abrir ni volcar el log completo.


## Criterio de éxito

La subfase se considera `CONSEGUIDA` solo si:

1. Todos los builds requeridos devuelven 0 y la simulación final completa los goals/scenario esperados.
2. Sparse y dense permanecen separados pero geométricamente coherentes; no quedan dobles paredes persistentes en escenarios normales.
3. Cambios de poses reintegran mapas derivados y no dejan contribuciones antiguas.
4. Dense puede generar constraints útiles sin romper hard fiducials/raw, y refinamientos de MPs solo afectan salida derivada.
5. Occupancy representa FREE/OCCUPIED/UNKNOWN y mejora planificación manteniendo depth local como seguridad.
6. El análisis del ROI genera tareas correctivas selectivas y las capturas HQ se integran/reemplazan correctamente.
7. El sistema multi-dron se mantiene dentro de presupuestos acordados y exporta un mapa final reproducible.
8. Fases 3–7 quedan corroboradas o, si hubo regresión previa, se corrigió en su fase y se repitió esta validación.


## Criterio de fallo o parcial

- `NO CONSEGUIDA` si el mapa final depende de GT, hay dobles paredes persistentes no explicadas, una optimización rompe dense o se modifican raw data.
- `NO CONSEGUIDA` si dense bloquea control/sparse o introduce colisiones por información de occupancy incorrecta.
- `PARCIAL` si el mapa es funcional pero quedan presupuestos/rutas correctivas obligatorias sin validar.
- `BLOQUEADA` solo si una dependencia externa o una fase anterior impide completar la prueba y no puede resolverse dentro del alcance autorizado.


## Riesgos

- La prueba final puede ser larga; registrar checkpoint antes de cada simulación y resultado inmediatamente después.
- No confundir calidad visual con corrección métrica; usar métricas externas además de GUI.


## Documentación a actualizar

Al ejecutar realmente la subfase, actualizar únicamente documentación respaldada por cambios/evidencia real:

```text
codex/pipeline/fase_8_nube_densa/historial/por_subfase/historial_8T.md
codex/pipeline/fase_8_nube_densa/historial/por_subfase/historial_8T_RESUMEN.md
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
