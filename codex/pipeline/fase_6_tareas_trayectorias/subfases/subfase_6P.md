# Subfase 6P — Cola serializada y registro de reservas de trayectorias en el servidor

## Estado

```text
sin hacer
```

## Dependencia

6H y 6J. La detección geométrica se añade en 6Q.

## Objetivo técnico

Crear la autoridad de coordinación Dron-Dron del servidor: recibir propuestas de trayectoria en una cola, procesarlas secuencialmente, registrar una única versión aceptada como reserva activa y liberarla inmediatamente al terminar/cancelar.

## Comportamiento esperado

Cada dron propone la **trayectoria real que pretende ejecutar**. El servidor no debe comprobar dos peticiones sobre un mismo snapshot libre de forma concurrente:

```text
request D1 -> procesar -> commit reserva D1
request D2 -> procesar viendo ya reserva D1
request D3 -> ...
```

6P implementa lifecycle y serialización. 6Q añadirá `ValidateCandidate` espacial. La reserva pertenece a un `trajectory_id`, no a la tarea. Al recibir `COMPLETED/CANCELLED/FAILED`, desaparece inmediatamente del conjunto activo.

Además del registro de trayectorias activas, el servidor mantiene la última pose/volumen de seguridad vigente de cada dron: un dron parado sigue ocupando espacio aunque no tenga una ruta reservada.

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

No existe actualmente un registro de rutas activas ni una cola de autorización. Sin commit serializado, dos drones podrían comprobar simultáneamente “libre” y aceptar recorridos incompatibles.

## Invariantes y decisiones cerradas

- Las propuestas se procesan secuencialmente/atómicamente respecto al conjunto de reservas activas.
- Una reserva aceptada es visible antes de procesar la siguiente petición.
- Una reserva se identifica por `trajectory_id` y propietario; el `task_id` solo da contexto.
- Finalizar/cancelar una trayectoria libera su reserva inmediatamente.
- Un dron sin trayectoria mantiene un volumen ocupado en su pose actual.
- No se usa tiempo para permitir cruces; esa política se formaliza en 6Q.
- El registro de reservas pertenece al servidor coordinador, no a `orbslam3_multi`.

## Archivos permitidos a modificar

```text
src/servidor/orbslam3_server/include/  # TrajectoryReservationManager propuesto
src/servidor/orbslam3_server/src/
src/servidor/orbslam3_server/config/
src/servidor/orbslam3_server/test/
src/servidor/orbslam3_msgs/
src/dron/orbslam3_msgs/
codex/contexto/paquetes/orbslam3_server/
codex/contexto/paquetes/orbslam3_msgs/
```

Las rutas marcadas como propuestas deben confirmarse contra el árbol real antes de crearlas. Si Fase 5 ya contiene un componente equivalente, se amplía/reutiliza en lugar de duplicarlo.

## Archivos prohibidos

```text
src/servidor/orbslam3_multi/
src/dron/dron_individual/src/vision/
ORB_SLAM3/
```

Además, no tocar legacy o paquetes ajenos a la subfase como limpieza colateral.

## Funciones, clases, nodos o interfaces que hay que localizar

```text
contrato de trayectoria/propuesta 6H
releases `COMPLETED/CANCELLED` 6J
fuente de pose global actual de cada dron de Fase 5
nodo servidor y modelo de concurrencia/callback groups posterior a Fase 5
```

Los nombres nuevos que aparezcan en este documento son nombres de contrato/propuesta. Si el workspace real ya posee una abstracción equivalente, reutilizarla y documentar la correspondencia antes de implementar.

## Cambios requeridos

1. Crear un `TrajectoryReservationManager` o equivalente con cola FIFO/orden determinista de solicitudes y sección crítica/worker que impida validaciones concurrentes sobre estado obsoleto.
2. Modelar estados `PROPOSED/ACCEPTED(ACTIVE)/REJECTED/RELEASED` o equivalentes con transiciones explícitas y revisión.
3. Insertar una reserva solo tras validación; el hook de colisión de 6Q puede estar inicialmente en modo `no_conflict_detector` para tests de lifecycle, pero no usar ese modo en integración multi-dron final.
4. Procesar `COMPLETED`, `CANCELLED` y `FAILED` idempotentemente; una liberación duplicada no debe borrar otra reserva.
5. Mantener por dron la última pose global válida y parámetros de volumen de seguridad necesarios; resolver el ownership de tamaño/radio siguiendo la política YAML de Fase 2 (config local servidor o réplica declarada, no lectura directa de YAML del Dron).
6. Rechazar propuesta con propietario/ID incoherente o dos activas para el mismo dron si el lifecycle 6J no lo permite.
7. Añadir logs `RESERVATION-QUEUE`, `RESERVATION-COMMIT`, `RESERVATION-RELEASE`, `RESERVATION-STATE`.

## Cambios prohibidos

- No validar propuestas en paralelo y hacer commit después sin revalidación.
- No mantener la reserva hasta terminar la `MAP_SECTION`.
- No asumir que un dron parado deja de ocupar espacio.
- No introducir obstacle avoidance físico en el servidor.
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
./codex/herramientas/build_selected_packages.sh orbslam3_server orbslam3_msgs
```

Si la distribución física creada por Fase 2 requiere el helper por grupos, usar la herramienta vigente equivalente y registrar el comando exacto en historial. Añadir dependencias reales solo si el build demuestra que son necesarias.

## Pruebas Gazebo requeridas

Preparación común:

- Si la prueba necesita una secuencia reproducible, usar/crear un YAML de escenario dentro de `src/simulacion/simulacion_dron/config/scenarios/fase_6/` (ruta final a confirmar contra Fase 2), no en el grupo Dron/Servidor.
- No precalcular en el scenario runner la autonomía que precisamente se está validando; el runner solo prepara condiciones, inyecta eventos o espera resultados.
- Comando base para cualquier prueba Gazebo de esta subfase:

```bash
./codex/herramientas/run_simulation.sh   --prueba fase_6_6P   --launch "ros2 launch simulacion_dron multi_dron.launch.py"   --post-scenario-wait-sec 20
```

Si la prueba indicada es determinista/unitaria y no necesita Gazebo, no arrancarlo artificialmente; ejecutar el test del paquete y registrar el comando exacto en historial.

### Prueba 1 — Serialización de propuestas

Inyectar propuestas casi simultáneas de varios drones. Los logs deben mostrar un orden total de procesamiento y cada commit debe ver todas las reservas aceptadas anteriores.

### Prueba 2 — Liberación inmediata/idempotente

Aceptar una reserva, enviar `COMPLETED` o `CANCELLED` y comprobar que desaparece antes de procesar la siguiente propuesta. Repetir el release y verificar que no corrompe el estado.

### Prueba 3 — Dron parado

Sin trayectoria activa, actualizar la pose de un dron y confirmar que el manager conserva su volumen actual para que 6Q pueda consultarlo.

## Patrones de reducción de logs

```text
RESERVATION-QUEUE|RESERVATION-COMMIT|RESERVATION-RELEASE|RESERVATION-STATE|trajectory_id|duplicate|owner|ERROR|FATAL|Segmentation fault|Killed
```

El log completo se conserva como artefacto y se reduce antes de leerlo, según `AGENTS.md`. Si el reducido no contiene evidencia suficiente, regenerar un reducido con patrones más precisos; no usar el log completo como contexto directo.

## Criterio de éxito

La subfase se considera `CONSEGUIDA` solo si:

1. Las solicitudes se procesan en un orden serializado reproducible.
2. Un commit es visible para la solicitud siguiente.
3. Release es inmediato e idempotente.
4. El estado no permite dos reservas activas ambiguas del mismo dron.
5. La pose/volumen de drones parados queda disponible para 6Q.

Además, el build requerido debe devolver `0`, las pruebas obligatorias deben haberse ejecutado, los marcadores deben aparecer sin errores graves no explicados y la documentación/historial real debe quedar sincronizada.

## Criterio de fallo o parcial

- `PARCIAL` si el registro funciona pero las callbacks aún pueden validar concurrentemente.
- `PARCIAL` si release depende de un timeout en lugar de la notificación inmediata acordada.

- `NO CONSEGUIDA` si no compila, una prueba obligatoria no se ejecuta, falta evidencia crítica o el comportamiento contradice el objetivo.
- `BLOQUEADA` solo ante dependencia/información externa realmente irresoluble con cambios mínimos.

## Riesgos

- Race entre release y commit de una nueva propuesta del mismo dron.
- ID duplicado que libere la reserva equivocada.
- Bloqueo largo del callback ROS si el futuro detector se ejecuta dentro de un lock global.

## Documentación a actualizar

Al ejecutar realmente la subfase, actualizar solo la documentación que corresponda al código tocado, incluyendo:

```text
codex/pipeline/fase_6_tareas_trayectorias/historial/por_subfase/historial_6P.md
codex/pipeline/fase_6_tareas_trayectorias/historial/por_subfase/historial_6P_RESUMEN.md
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
