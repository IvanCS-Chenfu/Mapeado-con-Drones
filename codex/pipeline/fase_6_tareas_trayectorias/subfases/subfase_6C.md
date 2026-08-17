# Subfase 6C — Generación inicial de tareas `MAP_SECTION` con solape deliberado

## Estado

```text
sin hacer
```

## Dependencia

6A y 6B.

## Objetivo técnico

Crear, antes de empezar la misión, todas las tareas de mapeo de todos los niveles y definir su objetivo nominal A–B–C con solape explícito entre secciones vecinas.

## Comportamiento esperado

Para cada punto principal B se crea una `MAP_SECTION_B` cuyos dos extremos nominales son los vecinos A y C. La tarea expresa **cobertura**, no una ruta rígida.

Ejemplo:

```text
Task B: A -> B -> C
Task C: B -> C -> D
```

La región B→C se observa deliberadamente en ambas tareas. Esta redundancia es deseada para que distintos drones/submapas compartan información visual y aumente la probabilidad de loops, fusión y unión global.

Todas las tareas iniciales de todos los niveles se crean antes de iniciar el reparto. La misión finaliza cuando estas tareas han terminado; no se crean “tareas densas” ni una tarea perimetral especial.

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

No existe todavía un generador de trabajos autónomos. El sistema actual puede ejecutar trayectorias, pero no representa que una misma región deba ser observada por tareas vecinas ni que la tarea pueda desviarse de la geometría nominal para cubrir interiores/ramificaciones.

## Invariantes y decisiones cerradas

- Una `MAP_SECTION` conserva A, B, C y nivel como identidad/semilla, pero su ejecución real puede desviarse ampliamente.
- Las tareas vecinas deben conservar su tramo solapado aunque el mapa global ya indique que otro dron lo observó.
- La información global de cobertura puede evitar exploración redundante no necesaria, pero no eliminar el solape nominal obligatorio.
- Se crean tareas de todos los niveles desde el inicio; no hay barrera “terminar piso N antes de N+1”.
- La misión acaba por finalización de las tareas iniciales, no por alcanzar una coordenada final global.

## Archivos permitidos a modificar

```text
src/servidor/orbslam3_server/src/  # generador de misión
src/servidor/orbslam3_server/include/
src/servidor/orbslam3_server/test/
src/servidor/orbslam3_server/config/tarea_principal.yaml
codex/contexto/paquetes/orbslam3_server/
```

Las rutas marcadas como propuestas deben confirmarse contra el árbol real antes de crearlas. Si Fase 5 ya contiene un componente equivalente, se amplía/reutiliza en lugar de duplicarlo.

## Archivos prohibidos

```text
src/dron/dron_individual/
src/servidor/orbslam3_multi/
ORB_SLAM3/
src/simulacion/simulacion_dron/src/control_tray/scenario_runner_node.cpp  # no simular tasks manuales como solución
```

Además, no tocar legacy o paquetes ajenos a la subfase como limpieza colateral.

## Funciones, clases, nodos o interfaces que hay que localizar

```text
estructuras `MappingLevel`/puntos creadas en 6B
componente servidor que cargará la misión
identidad de tarea si Fase 5 o código actual ya posee una interfaz equivalente
```

Los nombres nuevos que aparezcan en este documento son nombres de contrato/propuesta. Si el workspace real ya posee una abstracción equivalente, reutilizarla y documentar la correspondencia antes de implementar.

## Cambios requeridos

1. Crear una estructura interna de tarea de mapeo con `task_id`, nivel, punto principal B, vecinos A/C, banda vertical, ROI y metadatos suficientes para asignación posterior.
2. Generar exactamente `num_levels * tasks_per_level` tareas iniciales para los casos baseline.
3. Mantener los dos sentidos A→B→C y C→B→A como alternativas equivalentes hasta la asignación de 6F.
4. Marcar en el contrato de cada tarea los dos tramos nominales de cobertura y su solape con vecinos, sin convertirlos en waypoints ejecutables.
5. Comprobar automáticamente que cada tramo entre puntos consecutivos queda incluido en dos `MAP_SECTION` vecinas cuando la topología es cíclica.
6. Emitir `MISSION-TASK-CREATED` con IDs/nivel/sección y un resumen final `MISSION-TASKS-READY count=...`.

## Cambios prohibidos

- No elegir todavía qué dron ejecutará cada tarea.
- No considerar completa una tarea únicamente porque otro dron haya observado su tramo solapado.
- No crear una trayectoria A-B-C en esta subfase.
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
./codex/herramientas/build_selected_packages.sh orbslam3_server
```

Si la distribución física creada por Fase 2 requiere el helper por grupos, usar la herramienta vigente equivalente y registrar el comando exacto en historial. Añadir dependencias reales solo si el build demuestra que son necesarias.

## Pruebas Gazebo requeridas

Preparación común:

- Si la prueba necesita una secuencia reproducible, usar/crear un YAML de escenario dentro de `src/simulacion/simulacion_dron/config/scenarios/fase_6/` (ruta final a confirmar contra Fase 2), no en el grupo Dron/Servidor.
- No precalcular en el scenario runner la autonomía que precisamente se está validando; el runner solo prepara condiciones, inyecta eventos o espera resultados.
- Comando base para cualquier prueba Gazebo de esta subfase:

```bash
./codex/herramientas/run_simulation.sh   --prueba fase_6_6C   --launch "ros2 launch simulacion_dron multi_dron.launch.py"   --post-scenario-wait-sec 20
```

Si la prueba indicada es determinista/unitaria y no necesita Gazebo, no arrancarlo artificialmente; ejecutar el test del paquete y registrar el comando exacto en historial.

### Prueba 1 — Misión 4 tareas por nivel

Con dos niveles, validar que se crean 8 tareas y que cada segmento nominal de borde pertenece a exactamente dos tareas vecinas.

### Prueba 2 — Misión 8 tareas por nivel

Validar el mismo contrato con 8 puntos: 8 tareas por nivel, vecinos correctos y solape de cada segmento entre tareas adyacentes.

## Patrones de reducción de logs

```text
MISSION-TASK-CREATED|MISSION-TASKS-READY|MAP_SECTION|overlap|level|section|duplicate|ERROR|FATAL|Segmentation fault|Killed
```

El log completo se conserva como artefacto y se reduce antes de leerlo, según `AGENTS.md`. Si el reducido no contiene evidencia suficiente, regenerar un reducido con patrones más precisos; no usar el log completo como contexto directo.

## Criterio de éxito

La subfase se considera `CONSEGUIDA` solo si:

1. El número y los IDs de tareas son deterministas.
2. Cada tarea contiene A/B/C y banda correctos.
3. Los dos sentidos permanecen disponibles.
4. El solape nominal queda explícito y comprobado.
5. No se genera ninguna trayectoria física ni se requiere conocer paredes reales.

Además, el build requerido debe devolver `0`, las pruebas obligatorias deben haberse ejecutado, los marcadores deben aparecer sin errores graves no explicados y la documentación/historial real debe quedar sincronizada.

## Criterio de fallo o parcial

- `PARCIAL` si las tareas se crean pero el solape no está validado.
- `PARCIAL` si solo se soporta una granularidad 4/8.

- `NO CONSEGUIDA` si no compila, una prueba obligatoria no se ejecuta, falta evidencia crítica o el comportamiento contradice el objetivo.
- `BLOQUEADA` solo ante dependencia/información externa realmente irresoluble con cambios mínimos.

## Riesgos

- Eliminar sin querer el solape al deduplicar tareas.
- Confundir “tarea generada” con “zona geométrica exclusiva”.
- Asignar IDs inestables que cambien entre ejecuciones del mismo YAML.

## Documentación a actualizar

Al ejecutar realmente la subfase, actualizar solo la documentación que corresponda al código tocado, incluyendo:

```text
codex/pipeline/fase_6_tareas_trayectorias/historial/por_subfase/historial_6C.md
codex/pipeline/fase_6_tareas_trayectorias/historial/por_subfase/historial_6C_RESUMEN.md
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
