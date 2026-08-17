# Subfase 6J — Lifecycle de trayectorias locales cortas, cancelación y liberación

## Estado

```text
sin hacer
```

## Dependencia

6G, 6H y 6I.

## Objetivo técnico

Imponer que las tareas largas se ejecuten mediante trayectorias locales de horizonte acotado, con límites configurables de distancia/duración y un lifecycle claro que permita terminar/cancelar una trayectoria sin terminar la tarea.

## Comportamiento esperado

Una `MAP_SECTION` puede producir:

```text
trajectory_001 COMPLETED -> liberar
trajectory_002 COMPLETED -> liberar
trajectory_003 CANCELLED -> liberar
trajectory_004 COMPLETED -> liberar
...
```

Las trayectorias no deben ser largas ni ocupar grandes zonas durante mucho tiempo. Deben existir límites configurables como `max_trajectory_length` y `max_trajectory_duration` (nombres exactos a confirmar). Al finalizar o cancelar un segmento se notifica inmediatamente al servidor para que la reserva futura de 6P pueda desaparecer y no bloquee a otros drones.

Solo puede existir una trayectoria ejecutable activa por dron. La seguridad local puede frenar/cancelar sin esperar autorización del servidor.

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

`TrayAction` permite goals/cancelación, pero todavía no existe un contrato de “segmento reservado” ligado a una tarea larga ni límites de horizonte diseñados para coordinación multi-dron.

## Invariantes y decisiones cerradas

- Tarea y trayectoria tienen lifecycle independientes.
- Una trayectoria completada/cancelada debe producir notificación inmediata; no se espera al fin de `MAP_SECTION`.
- Antes de ejecutar otra trayectoria, la anterior debe quedar terminada/cancelada de forma inequívoca.
- Los límites de distancia/duración son parámetros de seguridad/coordinación, no valores hardcoded.
- La duración limita el horizonte local pero no se usa para permitir cruces Dron-Dron en 6Q.

## Archivos permitidos a modificar

```text
src/dron/dron_individual/src/control_tray/  # TaskExecutor/control_trayectorias
src/dron/dron_individual/config/
src/dron/dron_individual/launch/
src/servidor/orbslam3_server/src/  # receptor de estado/release mínimo
src/servidor/orbslam3_server/include/
src/servidor/orbslam3_msgs/
src/dron/orbslam3_msgs/
codex/contexto/paquetes/dron_individual/
codex/contexto/paquetes/orbslam3_server/
```

Las rutas marcadas como propuestas deben confirmarse contra el árbol real antes de crearlas. Si Fase 5 ya contiene un componente equivalente, se amplía/reutiliza en lugar de duplicarlo.

## Archivos prohibidos

```text
src/servidor/orbslam3_multi/
ORB_SLAM3/
src/dron/lib_tray/  # salvo ajuste mecánico demostrado tras 6I
```

Además, no tocar legacy o paquetes ajenos a la subfase como limpieza colateral.

## Funciones, clases, nodos o interfaces que hay que localizar

```text
TaskExecutor 6G
cliente/servidor de `TrayAction` 6I
contrato de trayectoria 6H
interfaz servidor para propuesta/release que se formalizará en 6P
```

Los nombres nuevos que aparezcan en este documento son nombres de contrato/propuesta. Si el workspace real ya posee una abstracción equivalente, reutilizarla y documentar la correspondencia antes de implementar.

## Cambios requeridos

1. Crear parámetros de horizonte máximo espacial y temporal con validación; determinar valores iniciales mediante pruebas, no por intuición oculta.
2. Antes de enviar a ejecución, rechazar/dividir una propuesta que exceda los límites de horizonte.
3. Asignar `trajectory_id` nuevo a cada segmento y enlazarlo con el `task_id` actual.
4. Propagar `COMPLETED`, `CANCELLED` y `FAILED` al servidor inmediatamente y conservar la tarea `RUNNING` cuando corresponda.
5. Implementar transición segura de replanning: frenar/hover -> cancelar vieja -> confirmar liberación/estado -> proponer nueva; 6O la disparará por geometría.
6. Rechazar una segunda trayectoria activa para el mismo dron salvo transición atómica explícita.
7. Añadir logs `TRAJ-LIFECYCLE`, `TRAJ-LIMIT`, `TRAJ-RELEASE` con IDs.

## Cambios prohibidos

- No aumentar muchísimo los límites para reducir la cantidad de propuestas.
- No mantener una reserva hasta terminar la tarea completa.
- No iniciar una segunda ruta mientras el estado de la anterior es ambiguo.
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
./codex/herramientas/build_selected_packages.sh dron_individual orbslam3_server orbslam3_msgs
```

Si la distribución física creada por Fase 2 requiere el helper por grupos, usar la herramienta vigente equivalente y registrar el comando exacto en historial. Añadir dependencias reales solo si el build demuestra que son necesarias.

## Pruebas Gazebo requeridas

Preparación común:

- Si la prueba necesita una secuencia reproducible, usar/crear un YAML de escenario dentro de `src/simulacion/simulacion_dron/config/scenarios/fase_6/` (ruta final a confirmar contra Fase 2), no en el grupo Dron/Servidor.
- No precalcular en el scenario runner la autonomía que precisamente se está validando; el runner solo prepara condiciones, inyecta eventos o espera resultados.
- Comando base para cualquier prueba Gazebo de esta subfase:

```bash
./codex/herramientas/run_simulation.sh   --prueba fase_6_6J   --launch "ros2 launch simulacion_dron multi_dron.launch.py"   --post-scenario-wait-sec 20
```

Si la prueba indicada es determinista/unitaria y no necesita Gazebo, no arrancarlo artificialmente; ejecutar el test del paquete y registrar el comando exacto en historial.

### Prueba 1 — División por límite

Solicitar desde el ejecutor un recorrido mayor que el máximo. Debe dividirse/rechazarse como una única trayectoria y generar segmentos dentro de límites, sin terminar la tarea global.

### Prueba 2 — Cancelación y siguiente segmento

Durante una tarea, cancelar un segmento. Verificar el orden `CANCELLED/RELEASE -> nueva propuesta`, `task_id` constante y `trajectory_id` distinto.

## Patrones de reducción de logs

```text
TRAJ-LIFECYCLE|TRAJ-LIMIT|TRAJ-RELEASE|task_id|trajectory_id|CANCELLED|COMPLETED|FAILED|ERROR|FATAL|Segmentation fault|Killed
```

El log completo se conserva como artefacto y se reduce antes de leerlo, según `AGENTS.md`. Si el reducido no contiene evidencia suficiente, regenerar un reducido con patrones más precisos; no usar el log completo como contexto directo.

## Criterio de éxito

La subfase se considera `CONSEGUIDA` solo si:

1. Ningún segmento aceptado supera los límites configurados.
2. Finalizar/cancelar un segmento no finaliza automáticamente la tarea.
3. El servidor recibe liberación inmediata y trazable.
4. No coexisten dos trayectorias activas del mismo dron.
5. El flujo permite generar otra trayectoria tras cancelación segura.

Además, el build requerido debe devolver `0`, las pruebas obligatorias deben haberse ejecutado, los marcadores deben aparecer sin errores graves no explicados y la documentación/historial real debe quedar sincronizada.

## Criterio de fallo o parcial

- `PARCIAL` si los límites existen pero la liberación no es inmediata/confirmable.
- `PARCIAL` si cancelación funciona pero puede existir una ventana con dos trayectorias activas.

- `NO CONSEGUIDA` si no compila, una prueba obligatoria no se ejecuta, falta evidencia crítica o el comportamiento contradice el objetivo.
- `BLOQUEADA` solo ante dependencia/información externa realmente irresoluble con cambios mínimos.

## Riesgos

- Liberación tardía que bloquee todo el entorno.
- Horizonte tan corto que genere overhead excesivo o tan largo que impida replanning.
- Carrera entre cancelación local y nueva propuesta.

## Documentación a actualizar

Al ejecutar realmente la subfase, actualizar solo la documentación que corresponda al código tocado, incluyendo:

```text
codex/pipeline/fase_6_tareas_trayectorias/historial/por_subfase/historial_6J.md
codex/pipeline/fase_6_tareas_trayectorias/historial/por_subfase/historial_6J_RESUMEN.md
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
