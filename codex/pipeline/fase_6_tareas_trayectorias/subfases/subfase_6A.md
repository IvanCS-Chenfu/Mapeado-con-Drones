# Subfase 6A — Contrato de `tarea_principal.yaml` y configuración de misión

## Estado

```text
sin hacer
```

## Dependencia

Fase 5 cerrada y contratos de pose global disponibles. Es la primera subfase de Fase 6.

## Objetivo técnico

Definir una única configuración de misión que el servidor cargue antes de comenzar la autonomía. En Fase 6 la misión principal es exclusivamente construir el mapa sparse de todo el entorno accesible dentro del ROI; no se crea todavía ninguna tarea de densificación.

## Comportamiento esperado

Antes de iniciar el reparto de trabajo, el servidor debe disponer de un YAML válido con los drones participantes, el ROI de mapeo, `flight_bounds`, histéresis, altura nominal de rebanada y número de tareas por nivel. El archivo describe **qué** se quiere mapear y límites globales de seguridad, pero no codifica trayectorias ni presupone la geometría real del edificio.

Semántica obligatoria:

```text
mapping_roi      -> volumen cuyo contenido accesible se quiere reconstruir
mapping_hysteresis -> tolerancia para seguir/observar geometría ligeramente fuera del ROI
flight_bounds    -> volumen duro para impedir que un dron se aleje demasiado
tasks_per_level  -> granularidad inicial de trabajos de cobertura
level_height     -> altura base con la que se dividen las bandas de mapeo
```

`mapping_roi` y `flight_bounds` son independientes. Un dron puede volar fuera del ROI si lo necesita para observarlo, siempre que permanezca dentro de `flight_bounds`.

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

El proyecto entregado no contiene todavía un contrato de misión autónoma de Fase 6. Los escenarios actuales pueden mandar goals de trayectoria, pero no existe una fuente de verdad que describa una misión completa de cobertura multi-dron antes del arranque del servidor.

## Invariantes y decisiones cerradas

- La misión de Fase 6 es mapeo sparse; la nube densa global pertenece a Fase 8.
- ROI y `flight_bounds` se expresan en `world` y no deben depender de GT.
- Los MapPoints que ORB-SLAM3 produzca fuera del ROI no se borran ni se ocultan obligatoriamente; simplemente no existe obligación de perseguir esa región.
- `mapping_hysteresis` no convierte el ROI ampliado en nueva zona obligatoria: evita cortes artificiales al seguir superficies próximas al borde.
- `tasks_per_level` se mantiene configurable. Baseline obligatorio de Fase 6: valores 4 y 8.
- La configuración de drones debe usar la identidad/namespaces reales establecidos por Fases 2–5; no crear un segundo sistema de IDs.
- El servidor debe disponer del tamaño/radio conservador y margen necesarios para 6Q mediante ownership coherente con Fase 2; `tarea_principal.yaml` puede referenciarlo o contener la parte de misión acordada, pero no debe duplicar silenciosamente parámetros físicos.

## Archivos permitidos a modificar

```text
src/servidor/orbslam3_server/config/tarea_principal.yaml  # ruta propuesta
src/servidor/orbslam3_server/launch/
src/servidor/orbslam3_server/src/  # parser/validador separado si hace falta
src/servidor/orbslam3_server/include/
src/servidor/orbslam3_server/CMakeLists.txt
src/servidor/orbslam3_server/package.xml
codex/contexto/paquetes/orbslam3_server/
```

Las rutas marcadas como propuestas deben confirmarse contra el árbol real antes de crearlas. Si Fase 5 ya contiene un componente equivalente, se amplía/reutiliza en lugar de duplicarlo.

## Archivos prohibidos

```text
src/dron/dron_individual/config/  # no duplicar la misión completa en el dron
src/simulacion/simulacion_dron/config/  # no convertir Simulación en propietaria del contrato de misión
src/servidor/orbslam3_multi/
ORB_SLAM3/
orbslam3_ros2/
```

Además, no tocar legacy o paquetes ajenos a la subfase como limpieza colateral.

## Funciones, clases, nodos o interfaces que hay que localizar

```text
global_orb_map_server.launch.py y launch real posterior a Fase 5
mecanismo vigente de carga de YAML en orbslam3_server
identidad/namespaces de drones definida por Fase 2/Fase 5
frames `world` y pose global de Fase 5
```

Los nombres nuevos que aparezcan en este documento son nombres de contrato/propuesta. Si el workspace real ya posee una abstracción equivalente, reutilizarla y documentar la correspondencia antes de implementar.

## Cambios requeridos

1. Crear el esquema de `tarea_principal.yaml` y documentar unidades, frame y obligatoriedad de cada clave.
2. Incluir una lista de drones con identidad suficiente para enlazar cada entrada con el namespace/pose global real; validar duplicados y drones no reconocidos. Resolver también cómo obtiene el Servidor el radio/tamaño de seguridad (campo de misión o réplica/config local declarada según Fase 2).
3. Definir `mapping_roi.min/max` y `flight_bounds.min/max` como cuboides axis-aligned en `world`; rechazar mínimos mayores que máximos, valores no finitos o volumen nulo.
4. Definir `mapping_hysteresis >= 0`, `level_height > 0` y `tasks_per_level`; validar inicialmente 4 u 8 y fallar explícitamente ante un valor no soportado.
5. Hacer que el servidor cargue/valide la configuración antes de crear tareas. Un YAML inválido debe impedir iniciar la misión autónoma, no provocar defaults silenciosos.
6. Añadir marcadores `MISSION-CONFIG loaded`, `MISSION-CONFIG invalid` y un resumen sin datos innecesarios para validar la carga.
7. Dejar explícita la política de parámetros: el YAML pertenece al grupo Servidor; Simulación puede tener escenarios que seleccionen un archivo, pero no una copia divergente de su semántica.

## Cambios prohibidos

- No almacenar waypoints de vuelo precomputados en `tarea_principal.yaml`.
- No hacer que el ROI actúe como barrera de vuelo.
- No usar `tasks_per_level=5` para crear una elipse/perímetro especial en el baseline acordado.
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
./codex/herramientas/run_simulation.sh   --prueba fase_6_6A   --launch "ros2 launch simulacion_dron multi_dron.launch.py"   --post-scenario-wait-sec 20
```

Si la prueba indicada es determinista/unitaria y no necesita Gazebo, no arrancarlo artificialmente; ejecutar el test del paquete y registrar el comando exacto en historial.

### Prueba 1 — Carga de misión válida

Arrancar el servidor con un YAML mínimo de dos drones, ROI y `flight_bounds` distintos, histéresis, `level_height` y `tasks_per_level=4`. No es necesario mover drones.

Resultado esperado: validación completa y marcador `MISSION-CONFIG loaded`; todavía no se exige generar tareas (6C).

### Prueba 2 — Rechazo de configuración inválida

Probar, sin alterar el baseline, al menos: ROI invertido, `level_height <= 0`, dron duplicado y `tasks_per_level` no soportado. El servidor debe rechazar el arranque de misión con razón explícita y sin crash.

## Patrones de reducción de logs

```text
MISSION-CONFIG|tarea_principal|mapping_roi|flight_bounds|tasks_per_level|level_height|duplicate|invalid|ERROR|FATAL|Segmentation fault|Killed
```

El log completo se conserva como artefacto y se reduce antes de leerlo, según `AGENTS.md`. Si el reducido no contiene evidencia suficiente, regenerar un reducido con patrones más precisos; no usar el log completo como contexto directo.

## Criterio de éxito

La subfase se considera `CONSEGUIDA` solo si:

1. El YAML tiene esquema y semántica documentados y se carga desde el grupo Servidor.
2. ROI y `flight_bounds` quedan diferenciados y validados en `world`.
3. Las identidades de drones reutilizan los contratos reales del proyecto.
4. Los valores 4 y 8 de `tasks_per_level` quedan aceptados por contrato.
5. Configuraciones inválidas fallan antes de crear/mover drones.
6. No se introduce ninguna tarea densa ni dependencia de Fase 8.

Además, el build requerido debe devolver `0`, las pruebas obligatorias deben haberse ejecutado, los marcadores deben aparecer sin errores graves no explicados y la documentación/historial real debe quedar sincronizada.

## Criterio de fallo o parcial

- `PARCIAL` si el esquema está definido pero falta validación de alguna condición crítica.
- `PARCIAL` si se carga el YAML pero todavía se depende de defaults ambiguos o de una ruta no instalada.

- `NO CONSEGUIDA` si no compila, una prueba obligatoria no se ejecuta, falta evidencia crítica o el comportamiento contradice el objetivo.
- `BLOQUEADA` solo ante dependencia/información externa realmente irresoluble con cambios mínimos.

## Riesgos

- Confundir ROI con zona permitida de vuelo.
- Duplicar parámetros de identidad ya existentes tras Fase 5.
- Aceptar configuraciones geométricamente inválidas y fallar mucho más tarde durante la planificación.

## Documentación a actualizar

Al ejecutar realmente la subfase, actualizar solo la documentación que corresponda al código tocado, incluyendo:

```text
codex/pipeline/fase_6_tareas_trayectorias/historial/por_subfase/historial_6A.md
codex/pipeline/fase_6_tareas_trayectorias/historial/por_subfase/historial_6A_RESUMEN.md
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
