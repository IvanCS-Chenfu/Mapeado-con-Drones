# Subfase 6H — Contrato de trayectorias locales por waypoints `(x,y,z,yaw)`

## Estado

```text
sin hacer
```

## Dependencia

6G y contratos de pose/frames de Fase 5.

## Objetivo técnico

Definir una representación compartida de trayectoria local propuesta al servidor y ejecutable por el dron, basada en una lista de waypoints con posición y `yaw`, identidad/revisión y límites suficientes para reserva y cancelación.

## Comportamiento esperado

La trayectoria baseline debe representar:

```text
trajectory_id
owner drone_id
task_id
frame = world
waypoints:
  - x,y,z,yaw
  - x,y,z,yaw
  - ...
```

La lista describe el recorrido que el dron pretende ejecutar. El servidor la usará más adelante para construir un corredor de seguridad, pero **no** usa su duración para permitir cruces temporales. El dron la convierte a referencias temporales mediante `lib_tray`/`TrayAction`.

El `yaw` forma parte del plan: la misma geometría XYZ mirando a una pared o a una zona sin textura no es equivalente para ORB-SLAM3.

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

`TrayAction.action` actual representa un único objetivo y tipos `pol3/veltrap/elipse`; no existe un contrato general de lista de waypoints ni una identidad explícita de reserva multi-dron.

## Invariantes y decisiones cerradas

- Todo waypoint contiene `x,y,z,yaw`.
- La trayectoria se expresa en el frame global vigente acordado en Fase 5; conversiones locales deben ser explícitas.
- `trajectory_id` es distinto de `task_id` y una misma tarea puede generar muchos IDs.
- La representación no presupone que la trayectoria sea larga; 6J impondrá límites de distancia/duración.
- Los timestamps/duración pueden existir para ejecución/diagnóstico, pero 6Q no los usará como permiso de cruce.

## Archivos permitidos a modificar

```text
src/servidor/orbslam3_msgs/msg/
src/servidor/orbslam3_msgs/srv/
src/servidor/orbslam3_msgs/action/
src/dron/orbslam3_msgs/  # réplica controlada
src/dron/dron_individual/action/TrayAction.action  # solo si el contrato acordado lo requiere
src/dron/dron_individual/CMakeLists.txt
codex/contexto/paquetes/orbslam3_msgs/
codex/contexto/paquetes/dron_individual/
```

Las rutas marcadas como propuestas deben confirmarse contra el árbol real antes de crearlas. Si Fase 5 ya contiene un componente equivalente, se amplía/reutiliza en lugar de duplicarlo.

## Archivos prohibidos

```text
src/dron/lib_tray/src/  # implementación 6I
src/servidor/orbslam3_multi/
ORB_SLAM3/
```

Además, no tocar legacy o paquetes ajenos a la subfase como limpieza colateral.

## Funciones, clases, nodos o interfaces que hay que localizar

```text
`dron_individual/action/TrayAction.action`
tipos geometry_msgs disponibles y convenciones yaw/quat del proyecto
guardas de `orbslam3_msgs` de Fase 2
interfaces de trayectoria si Fase 5 ya añadió alguna
```

Los nombres nuevos que aparezcan en este documento son nombres de contrato/propuesta. Si el workspace real ya posee una abstracción equivalente, reutilizarla y documentar la correspondencia antes de implementar.

## Cambios requeridos

1. Definir el tipo `Waypoint` o equivalente con posición 3D y yaw, reutilizando `Pose`/`PoseStamped` si mantiene semántica clara y evita campos redundantes.
2. Definir el mensaje de trayectoria propuesta con `trajectory_id`, `task_id`, propietario, frame/revisión necesaria y array de waypoints.
3. Definir estados/resultados de propuesta suficientes para 6P–6R: accepted/rejected y razones sin incorporar todavía el algoritmo de colisión.
4. Ampliar el contrato de `TrayAction` para recibir waypoints o crear un payload compatible que `gen_tray` pueda ejecutar en 6I, preservando los modos legacy.
5. Validar lista vacía, NaN, yaw no finito, IDs duplicados y waypoints fuera de formato.
6. Actualizar copias compartidas/guardas y añadir logs/test `TRAJ-CONTRACT`.

## Cambios prohibidos

- No eliminar `pol3`, `veltrap` o `elipse` por crear waypoints.
- No codificar obstáculos ni MapPoints dentro del mensaje de trayectoria.
- No introducir una representación temporal de reservas multi-dron en esta subfase.
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
./codex/herramientas/build_selected_packages.sh orbslam3_msgs dron_individual
```

Si la distribución física creada por Fase 2 requiere el helper por grupos, usar la herramienta vigente equivalente y registrar el comando exacto en historial. Añadir dependencias reales solo si el build demuestra que son necesarias.

## Pruebas Gazebo requeridas

Preparación común:

- Si la prueba necesita una secuencia reproducible, usar/crear un YAML de escenario dentro de `src/simulacion/simulacion_dron/config/scenarios/fase_6/` (ruta final a confirmar contra Fase 2), no en el grupo Dron/Servidor.
- No precalcular en el scenario runner la autonomía que precisamente se está validando; el runner solo prepara condiciones, inyecta eventos o espera resultados.
- Comando base para cualquier prueba Gazebo de esta subfase:

```bash
./codex/herramientas/run_simulation.sh   --prueba fase_6_6H   --launch "ros2 launch simulacion_dron multi_dron.launch.py"   --post-scenario-wait-sec 20
```

Si la prueba indicada es determinista/unitaria y no necesita Gazebo, no arrancarlo artificialmente; ejecutar el test del paquete y registrar el comando exacto en historial.

### Prueba 1 — Serialización de waypoints

Crear una trayectoria con varios waypoints 3D y yaw distintos, serializar/publicar y comprobar igualdad en receptor y frame correcto.

### Prueba 2 — Compatibilidad de `TrayAction`

Compilar los clientes/servidor existentes y confirmar que el nuevo contrato no rompe los casos legacy documentados, o proporcionar transición explícita y probada.

## Patrones de reducción de logs

```text
TRAJ-CONTRACT|trajectory_id|waypoint|yaw|TrayAction|legacy|invalid|ERROR|FATAL|Segmentation fault|Killed
```

El log completo se conserva como artefacto y se reduce antes de leerlo, según `AGENTS.md`. Si el reducido no contiene evidencia suficiente, regenerar un reducido con patrones más precisos; no usar el log completo como contexto directo.

## Criterio de éxito

La subfase se considera `CONSEGUIDA` solo si:

1. Existe una representación inequívoca de waypoints `(x,y,z,yaw)`.
2. Tarea, trayectoria y propietario están identificados.
3. Las copias de interfaces compartidas siguen consistentes.
4. Los casos legacy de trayectoria continúan compilando/funcionando o la migración queda cerrada explícitamente.
5. No se introduce todavía lógica temporal de colisión.

Además, el build requerido debe devolver `0`, las pruebas obligatorias deben haberse ejecutado, los marcadores deben aparecer sin errores graves no explicados y la documentación/historial real debe quedar sincronizada.

## Criterio de fallo o parcial

- `PARCIAL` si el mensaje existe pero `TrayAction` queda incompatible sin migración completa.
- `PARCIAL` si falta validación de datos inválidos.

- `NO CONSEGUIDA` si no compila, una prueba obligatoria no se ejecuta, falta evidencia crítica o el comportamiento contradice el objetivo.
- `BLOQUEADA` solo ante dependencia/información externa realmente irresoluble con cambios mínimos.

## Riesgos

- Duplicar pose con varias convenciones de yaw.
- Romper clientes legacy de `TrayAction`.
- Enviar una trayectoria enorme sin límites; 6J debe impedirlo funcionalmente.

## Documentación a actualizar

Al ejecutar realmente la subfase, actualizar solo la documentación que corresponda al código tocado, incluyendo:

```text
codex/pipeline/fase_6_tareas_trayectorias/historial/por_subfase/historial_6H.md
codex/pipeline/fase_6_tareas_trayectorias/historial/por_subfase/historial_6H_RESUMEN.md
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
