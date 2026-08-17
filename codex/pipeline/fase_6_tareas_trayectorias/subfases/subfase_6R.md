# Subfase 6R — Sugerencias de desvío servidor `A→C→B` ante conflicto multi-dron

## Estado

```text
sin hacer
```

## Dependencia

6Q y LocalPlanner/6O en el dron.

## Objetivo técnico

Hacer que un rechazo Dron-Dron pueda incluir uno o varios waypoints intermedios que eviten los corredores reservados, sin convertir al servidor en planificador de paredes ni considerar la sugerencia una autorización de vuelo.

## Comportamiento esperado

Si el dron propone A→B y 6Q detecta una reserva conflictiva, el servidor puede buscar puntos C alrededor de la región reservada:

```text
A --------X-------- B
          reserva D1

sugerencia:
A ------ C ------ B
```

El servidor solo conoce coordinación multi-dron y `flight_bounds`. Por tanto C significa:

> “Respecto a otros drones/reservas, esta desviación parece libre.”

No significa que C sea físicamente navegable. El dron comprueba C con depth, tracking y su LocalPlanner. Si puede construir A-C-B (o una ruta más compleja), **envía esa trayectoria completa como una nueva propuesta**, que vuelve a 6P/6Q. El hint por sí mismo nunca da permiso.

El servidor puede sugerir desvío lateral y, en 3D, arriba/abajo si permanecen dentro de `flight_bounds`; debe minimizar razonablemente el desvío y no cruzar otra reserva.

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

Un simple `REJECTED` es seguro pero puede hacer esperar/reintentar sin información aunque el servidor conoce exactamente qué región multi-dron está bloqueada. A la vez, hacer que el servidor calcule un path completo contra paredes violaría el reparto de responsabilidades.

## Invariantes y decisiones cerradas

- C evita solo otros drones/reservas y `flight_bounds`; no garantiza obstáculos físicos.
- El dron es libre de rechazar todos los hints.
- El hint no crea reserva ni cambia la reserva existente.
- Toda ruta nueva debe volver a la cola y detector normales.
- La reserva ya aceptada que causó el conflicto no se mueve/cancela por la nueva petición.
- No usar tiempo/espera programada como requisito para resolver el cruce; esperar puede ser fallback del dron, no una ventana temporal de aceptación.

## Archivos permitidos a modificar

```text
src/servidor/orbslam3_server/include/  # ConflictResolutionHintGenerator
src/servidor/orbslam3_server/src/
src/servidor/orbslam3_server/config/
src/servidor/orbslam3_server/test/
src/servidor/orbslam3_msgs/
src/dron/orbslam3_msgs/
src/dron/dron_individual/src/control_tray/  # consumo/validación del hint
codex/contexto/paquetes/orbslam3_server/
codex/contexto/paquetes/dron_individual/
```

Las rutas marcadas como propuestas deben confirmarse contra el árbol real antes de crearlas. Si Fase 5 ya contiene un componente equivalente, se amplía/reutiliza en lugar de duplicarlo.

## Archivos prohibidos

```text
src/servidor/orbslam3_multi/
src/servidor/orbslam3_server/src/vision/  # no crear percepción de paredes en servidor
ORB_SLAM3/
```

Además, no tocar legacy o paquetes ajenos a la subfase como limpieza colateral.

## Funciones, clases, nodos o interfaces que hay que localizar

```text
detalles de conflicto producidos por 6Q
LocalPlanner/replanning 6O
contrato de respuesta de propuesta 6H/6P
`flight_bounds` 6A
representación de reservas activas 6P
```

Los nombres nuevos que aparezcan en este documento son nombres de contrato/propuesta. Si el workspace real ya posee una abstracción equivalente, reutilizarla y documentar la correspondencia antes de implementar.

## Cambios requeridos

1. Definir en el contrato de rechazo un array acotado de hints intermedios o un hint opcional con frame y razón.
2. Generar candidatos C alrededor del volumen conflictivo usando offsets laterales/verticales parametrizados por tamaño+margen; no asumir que `z` fijo siempre es mejor.
3. Filtrar C y los segmentos geométricos A-C/C-B contra `flight_bounds` y otras reservas usando 6Q antes de devolverlos.
4. Ordenar candidatos por coste simple (desvío adicional, margen, cambio vertical) sin introducir conocimiento de paredes.
5. En el dron, validar cada hint con 6K/6L y, si es útil, pedir al LocalPlanner una ruta segura que pase por C.
6. La ruta resultante recibe nuevo `trajectory_id` y vuelve a la cola; registrar `CONFLICT-HINT-RECEIVED/ACCEPTED/REJECTED` en el dron y `TRAJ-HINT` en servidor.
7. Si no existe hint razonable, devolver rechazo limpio y permitir que el dron espere/replantee por otra estrategia.

## Cambios prohibidos

- No enviar C directamente a `TrayAction` sin validación local.
- No hacer commit automático de A-C-B al generar el hint.
- No consultar la nube sparse para decidir si C atraviesa una pared desde servidor.
- No garantizar que siempre habrá una solución C única.
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
./codex/herramientas/build_selected_packages.sh orbslam3_server dron_individual orbslam3_msgs
```

Si la distribución física creada por Fase 2 requiere el helper por grupos, usar la herramienta vigente equivalente y registrar el comando exacto en historial. Añadir dependencias reales solo si el build demuestra que son necesarias.

## Pruebas Gazebo requeridas

Preparación común:

- Si la prueba necesita una secuencia reproducible, usar/crear un YAML de escenario dentro de `src/simulacion/simulacion_dron/config/scenarios/fase_6/` (ruta final a confirmar contra Fase 2), no en el grupo Dron/Servidor.
- No precalcular en el scenario runner la autonomía que precisamente se está validando; el runner solo prepara condiciones, inyecta eventos o espera resultados.
- Comando base para cualquier prueba Gazebo de esta subfase:

```bash
./codex/herramientas/run_simulation.sh   --prueba fase_6_6R   --launch "ros2 launch simulacion_dron multi_dron.launch.py"   --post-scenario-wait-sec 20
```

Si la prueba indicada es determinista/unitaria y no necesita Gazebo, no arrancarlo artificialmente; ejecutar el test del paquete y registrar el comando exacto en historial.

### Prueba 1 — Hint válido y reenvío

Crear un conflicto simple entre dos corredores con espacio lateral libre. El servidor debe devolver C; el dron valida y presenta una nueva trayectoria A-C-B; solo tras segunda validación puede ejecutarse.

### Prueba 2 — Hint bloqueado por pared local

Situar un obstáculo físico en el C sugerido sin que el servidor lo conozca. El dron debe rechazar el hint y no ejecutarlo. No debe considerarse fallo del detector servidor.

### Prueba 3 — Varios candidatos / sin solución

Comprobar que el servidor puede devolver otro candidato o un rechazo sin hint si todos los desvíos geométricos invaden reservas/flight bounds.

## Patrones de reducción de logs

```text
TRAJ-HINT|CONFLICT-HINT|TRAJ-CONFLICT|new_trajectory|hint_rejected|flight_bounds|RESERVATION|ERROR|FATAL|Segmentation fault|Killed
```

El log completo se conserva como artefacto y se reduce antes de leerlo, según `AGENTS.md`. Si el reducido no contiene evidencia suficiente, regenerar un reducido con patrones más precisos; no usar el log completo como contexto directo.

## Criterio de éxito

La subfase se considera `CONSEGUIDA` solo si:

1. Un conflicto simple produce un hint útil cuando existe espacio multi-dron.
2. El dron valida físicamente el hint antes de usarlo.
3. La nueva ruta se vuelve a proponer y no se autoautoriza.
4. Hints imposibles pueden rechazarse sin romper la tarea.
5. El servidor sigue sin planificar paredes/objetos.

Además, el build requerido debe devolver `0`, las pruebas obligatorias deben haberse ejecutado, los marcadores deben aparecer sin errores graves no explicados y la documentación/historial real debe quedar sincronizada.

## Criterio de fallo o parcial

- `PARCIAL` si el servidor genera C pero el dron lo ejecuta sin revalidar/reproponer.
- `PARCIAL` si solo funciona desvío en XY y un escenario baseline requiere 3D para resolver un conflicto simple.

- `NO CONSEGUIDA` si no compila, una prueba obligatoria no se ejecuta, falta evidencia crítica o el comportamiento contradice el objetivo.
- `BLOQUEADA` solo ante dependencia/información externa realmente irresoluble con cambios mínimos.

## Riesgos

- Hint geométricamente libre respecto a drones pero dentro de una pared.
- Oscilación entre dos hints rechazados repetidamente.
- Desvío vertical excesivo que salga de la banda de cobertura aunque siga dentro de flight bounds.

## Documentación a actualizar

Al ejecutar realmente la subfase, actualizar solo la documentación que corresponda al código tocado, incluyendo:

```text
codex/pipeline/fase_6_tareas_trayectorias/historial/por_subfase/historial_6R.md
codex/pipeline/fase_6_tareas_trayectorias/historial/por_subfase/historial_6R_RESUMEN.md
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
