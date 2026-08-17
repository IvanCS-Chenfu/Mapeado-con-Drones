# Subfase 6T — Integración autónoma multi-dron, cobertura completa del ROI y cierre de Fase 6

## Estado

```text
sin hacer
```

## Dependencia

6A–6S completadas.

## Objetivo técnico

Integrar toda la misión de Fase 6 y demostrar que un `tarea_principal.yaml` con el ROI de la casa y N drones produce automáticamente el mapa sparse de todo el entorno accesible del ROI, sin goals manuales de mapeo, sin colisiones y sin GT funcional.

## Comportamiento esperado

Prueba oficial conceptual:

```text
tarea_principal.yaml
  -> servidor carga ROI / flight_bounds / N drones
  -> genera todas las MAP_SECTION de todos los niveles
  -> las encola y asigna por entrada/cercanía
  -> cada dron planifica y ejecuta cobertura adaptativa
       * depth local para obstáculos
       * ORB visual state para tracking
       * yaw perceptivo
       * trayectorias cortas
       * replanning online
  -> servidor serializa propuestas
       * reservas espaciales
       * conflictos Dron-Dron
       * hints A-C-B
  -> cada tarea termina y el dron pide otra
  -> MISSION-COMPLETE cuando todas las tareas iniciales completan
```

RViz2 debe mostrar una reconstrucción sparse global coherente de todo lo accesible dentro del ROI. Los MapPoints exteriores que aparezcan naturalmente pueden seguir publicados. Las zonas dentro del ROI físicamente inaccesibles no se convierten en una misión infinita.

El solape entre tareas/submapas debe ser visible/evidenciable: regiones comunes como B-C se recorren desde tareas vecinas para favorecer loops/unión global.

## Contexto obligatorio a leer

```text
AGENTS.md
codex/contexto/00_CONTEXTO_COMPACTACION.md
codex/contexto/CONTEXTO_MINIMO_ACTUAL.md
codex/contexto/08_POLITICA_TOKENS_DOCUMENTACION.md
codex/contexto/01_ESTADO_ACTUAL.md
codex/pipeline/PIPELINE_MAESTRO.md
codex/pipeline/fase_6_tareas_trayectorias/pipeline_fase_6_RESUMEN.md
codex/pipeline/fase_6_tareas_trayectorias/pipeline_fase_6.md
```

Antes de modificar código, Codex debe leer también el MD vigente de cada paquete afectado en `codex/contexto/paquetes/` y el contrato final de Fase 5 que exista en el workspace real. El snapshot usado para preparar esta Fase 6 no contiene la Fase 5 recién ejecutada por el usuario, por lo que no se deben inventar nombres nuevos de topics, frames o mensajes si Fase 5 ya proporciona un contrato equivalente.



## Diagnóstico de partida

Las subfases anteriores validan componentes. Falta demostrar que su composición funciona en una misión autónoma larga, que las reservas no bloquean el progreso y que la cobertura no se degrada a simples recorridos de borde en una geometría real.

## Invariantes y decisiones cerradas

- La prueba final arranca desde configuración de misión, no desde una secuencia manual de TrayAction que precalcule el vuelo.
- Debe haber al menos dos drones para validar coordinación real; N sigue siendo configurable y puede aumentarse según recursos.
- No se exige nube densa final en Fase 6.
- No se usa GT para pose/control, obstáculos, cobertura, asignación, colisiones ni cierre. GT solo puede aparecer en métricas externas si se decide medir error.
- No se declara `CONSEGUIDA` solo por terminar tasks si visualmente/logs muestran zonas accesibles relevantes sin mapear por un fallo claro del criterio de cobertura.
- Todas las reservas deben quedar liberadas al cierre.

## Archivos permitidos a modificar

```text
src/dron/dron_individual/  # correcciones de integración dentro del contrato ya cerrado
src/dron/lib_tray/
src/dron/orbslam3_ros2/
src/dron/orbslam3_msgs/
src/servidor/orbslam3_server/
src/servidor/orbslam3_msgs/
src/simulacion/simulacion_dron/config/  # escenario/selección del YAML de prueba
src/simulacion/simulacion_dron/launch/
src/simulacion/simulacion_dron/src/control_tray/scenario_runner_node.cpp  # solo adaptación de orquestación/evidencia, no plan de vuelo manual
codex/contexto/paquetes/
```

Las rutas marcadas como propuestas deben confirmarse contra el árbol real antes de crearlas. Si Fase 5 ya contiene un componente equivalente, se amplía/reutiliza en lugar de duplicarlo.

## Archivos prohibidos

```text
ORB_SLAM3/  # salvo regresión/bloqueo nuevo con autorización específica
src/servidor/orbslam3_multi/  # no retocar algoritmos sparse para esconder fallos de autonomía salvo defecto demostrado de Fase 3
implementación de nube densa global Fase 8
```

Además, no tocar legacy o paquetes ajenos a la subfase como limpieza colateral.

## Funciones, clases, nodos o interfaces que hay que localizar

```text
launch oficial `multi_dron.launch.py` posterior a Fase 2–5
modo de cargar `tarea_principal.yaml` en servidor
RViz2 sparse global vigente
logs/markers de 6A–6S
escenario/casa oficial del proyecto
herramientas de reducción de logs y métricas
```

Los nombres nuevos que aparezcan en este documento son nombres de contrato/propuesta. Si el workspace real ya posee una abstracción equivalente, reutilizarla y documentar la correspondencia antes de implementar.

## Cambios requeridos

1. Crear/ajustar un escenario de integración que arranque N drones y el servidor con un `tarea_principal.yaml` para la casa, sin enviar manualmente las rutas de mapeo.
2. Integrar launch/namespaces/configuración para que cada dron solicite y ejecute tareas del manager.
3. Forzar o elegir al menos un escenario donde dos propuestas entren en conflicto espacial y verificar rechazo/hint/replan sin colisión.
4. Comprobar que los drones ejecutan niveles en paralelo cuando hay trabajo y que los sobrantes pueden subir de nivel.
5. Comprobar que `MAP_SECTION` se adapta a geometría real, genera replans y no sigue literalmente el borde del ROI.
6. Comprobar que trayectorias completadas/canceladas liberan reservas con rapidez y que al final el registry está vacío salvo volúmenes de drones parados.
7. Obtener evidencia de solape entre tareas vecinas/submapas en zonas comunes y de anclaje/loops cuando correspondan.
8. Validar en RViz2 y logs que las zonas accesibles del ROI quedan cubiertas por sparse; documentar explícitamente cualquier región inaccesible y por qué no bloquea.
9. Repetir la prueba suficiente para descartar un éxito fortuito según recursos del proyecto y cerrar regresiones de Fases 3–5/Control.
10. Actualizar toda la documentación/handoff de Fase 6 y dejar Fase 7/8 como posteriores sin implementarlas.

## Cambios prohibidos

- No precalcular todas las trayectorias de la casa en el scenario runner para “pasar” la prueba.
- No desactivar detector de conflictos o aumentar márgenes/límites de trayectoria para evitar bloqueos sin diagnóstico.
- No usar GT para decidir que una zona ya está mapeada.
- No marcar la fase conseguida si una tarea queda `FAILED`/bloqueada o una reserva queda huérfana.
- No usar Ground Truth como entrada funcional para pose, asignación, navegación, obstáculos, autorización de trayectorias, cobertura o criterio de finalización.
- No modificar `ORB_SLAM3` como primera opción; ampliar el wrapper o reutilizar interfaces existentes siempre que sea suficiente.
- No implementar la nube densa global, TSDF/Open3D global ni la reconstrucción final de Fase 8.
- No convertir el servidor en planificador de paredes u obstáculos físicos: esa responsabilidad pertenece al dron.
- No borrar ni reescribir datos raw de ORB-SLAM3 para adaptar la navegación.
- No introducir un modo separado de navegación interior y otro exterior.
- No rellenar historiales con resultados ficticios: se crean únicamente cuando exista una ejecución real.

## Paquetes a compilar

Comando base esperado:

```bash
./codex/herramientas/build_selected_packages.sh dron_individual lib_tray orbslam3_ros2 orbslam3_msgs orbslam3_server orbslam3_multi simulacion_dron
```

Si la distribución física creada por Fase 2 requiere el helper por grupos, usar la herramienta vigente equivalente y registrar el comando exacto en historial. Añadir dependencias reales solo si el build demuestra que son necesarias.

## Pruebas Gazebo requeridas

Preparación común:

- Si la prueba necesita una secuencia reproducible, usar/crear un YAML de escenario dentro de `src/simulacion/simulacion_dron/config/scenarios/fase_6/` (ruta final a confirmar contra Fase 2), no en el grupo Dron/Servidor.
- No precalcular en el scenario runner la autonomía que precisamente se está validando; el runner solo prepara condiciones, inyecta eventos o espera resultados.
- Comando base para cualquier prueba Gazebo de esta subfase:

```bash
./codex/herramientas/run_simulation.sh   --prueba fase_6_6T   --launch "ros2 launch simulacion_dron multi_dron.launch.py"   --post-scenario-wait-sec 20
```

Si la prueba indicada es determinista/unitaria y no necesita Gazebo, no arrancarlo artificialmente; ejecutar el test del paquete y registrar el comando exacto en historial.

### Prueba 1 — Smoke autónomo multi-dron

ROI reducido/parte de la casa, al menos 2 drones y pocas bandas. Arrancar misión desde YAML, dejar que el servidor cree/asigne tareas y verificar un ciclo completo task→trayectorias→release→siguiente task sin rutas manuales.

### Prueba 2 — Prueba oficial: casa completa dentro del ROI

YAML:

```text
src/servidor/orbslam3_server/config/tarea_principal.yaml  # o ruta final acordada
```

Comando base:

```bash
./codex/herramientas/run_simulation.sh \
  --prueba fase_6_final \
  --launch "ros2 launch simulacion_dron multi_dron.launch.py" \
  --post-scenario-wait-sec 30
```

El launch debe iniciar la misión automáticamente o mediante una única orden de “start mission”; `scenario_runner_node` no debe mandar waypoints de cobertura manuales.

Observación esperada en RViz2: sparse global cubriendo el entorno accesible del ROI, drones recorriendo geometría real y solape suficiente entre submapas/tareas.

### Prueba 3 — Conflicto multi-dron durante misión

Durante la misión oficial o escenario reducido, obtener al menos un conflicto de corredores. Debe verse `TRAJ-CONFLICT`, rechazo/hint, nueva propuesta y progreso posterior sin colisión.

### Prueba 4 — Replanning por geometría nueva

Observar al menos un caso real donde un segmento se cancele/cambie por obstáculo/geometría descubierta y la misma `MAP_SECTION` continúe hasta completar.

## Patrones de reducción de logs

```text
MISSION-CONFIG|MISSION-TASKS-READY|TASK-ALLOC|TASK-EXEC|COVERAGE-|REPLAN-|TRAJ-LIFECYCLE|RESERVATION-|TRAJ-CONFLICT|TRAJ-HINT|SLAM-VISUAL-STATE|LOCAL-OBSTACLE|ANCHOR-TASK|MISSION-COMPLETE|success|ERROR|FATAL|Segmentation fault|Killed
```

El log completo se conserva como artefacto y se reduce antes de leerlo, según `AGENTS.md`. Si el reducido no contiene evidencia suficiente, regenerar un reducido con patrones más precisos; no usar el log completo como contexto directo.

## Criterio de éxito

La subfase se considera `CONSEGUIDA` solo si:

1. Build de todos los paquetes afectados devuelve 0 en los grupos correctos.
2. La misión arranca desde `tarea_principal.yaml` y crea automáticamente todas las tareas.
3. Todos los trabajos iniciales terminan `COMPLETED`; no quedan tareas fallidas ni reservas huérfanas.
4. El mapa sparse en RViz2 cubre razonablemente todo lo accesible dentro del ROI conforme a los criterios de cobertura, con solape real entre tareas/submapas.
5. Los drones evitan paredes/objetos mediante percepción local y entre sí mediante reservas servidor.
6. Existe evidencia de trayectorias cortas, release inmediato y replanning online.
7. No se usa GT funcional y no se implementa nube densa global.
8. La documentación/historial real queda sincronizada y la fase se marca según evidencia, no por intención.

Además, el build requerido debe devolver `0`, las pruebas obligatorias deben haberse ejecutado, los marcadores deben aparecer sin errores graves no explicados y la documentación/historial real debe quedar sincronizada.

## Criterio de fallo o parcial

- `PARCIAL` si la misión completa la mayoría de tareas pero queda una región accesible/tarea obligatoria sin resolver.
- `PARCIAL` si cobertura funciona pero la coordinación multi-dron obliga a ejecutar prácticamente en serie por un bug/limitación no acordada.
- `PARCIAL` si todos los tasks terminan pero falta evidencia visual/log de cobertura suficiente del ROI.

- `NO CONSEGUIDA` si no compila, una prueba obligatoria no se ejecuta, falta evidencia crítica o el comportamiento contradice el objetivo.
- `BLOQUEADA` solo ante dependencia/información externa realmente irresoluble con cambios mínimos.

## Riesgos

- Prueba larga y costosa que oculte un primer error simple; ejecutar smoke tests por bloques antes.
- Cobertura que termina formalmente pero deja huecos accesibles por criterio frontier deficiente.
- Conservadurismo espacial que reduzca paralelismo; mitigado por trayectorias cortas, no por introducir tiempo a última hora.
- CPU elevada por stereo + ORB + planner en varios drones.

## Documentación a actualizar

Al ejecutar realmente la subfase, actualizar solo la documentación que corresponda al código tocado, incluyendo:

```text
codex/pipeline/fase_6_tareas_trayectorias/historial/por_subfase/historial_6T.md
codex/pipeline/fase_6_tareas_trayectorias/historial/por_subfase/historial_6T_RESUMEN.md
codex/pipeline/fase_6_tareas_trayectorias/pipeline_fase_6_RESUMEN.md
codex/contexto/01_ESTADO_ACTUAL.md              # si cambia el estado real
codex/contexto/paquetes/<paquete_afectado>/
```

Los historiales anteriores **no existen en este ZIP a propósito**. Deben crearse solo tras una ejecución real. La documentación de paquete debe reflejar el estado actual del código, no limitarse a añadir una nota histórica.

## Dudas funcionales de contrato

```text
ninguna
```

Cualquier duda nueva que cambie el comportamiento acordado suspende la autorización funcional y debe discutirse antes de continuar, conforme a `AGENTS.md`.
