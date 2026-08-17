# Subfase 6F — Asignación de tareas por entrada, cercanía y continuidad

## Estado

```text
sin hacer
```

## Dependencia

6E y pose global fiable de Fase 5.

## Objetivo técnico

Sustituir la selección provisional de la cola por un asignador que elija tareas cercanas y un sentido de ejecución adecuado, usando la posición global del dron y las dos entradas posibles de cada `MAP_SECTION`.

## Comportamiento esperado

Para una tarea principal B con vecinos A y C:

```text
MAP_SECTION_B
  opción 1: A -> B -> C
  opción 2: C -> B -> A
```

El servidor **no** selecciona al dron por distancia a B. Evalúa el acceso a A y a C. Si un dron está cerca de A, B es una tarea natural para él aunque otro dron esté más cerca de B.

El coste debe favorecer continuidad espacial: al terminar una tarea, normalmente se ofrece otra cercana del mismo nivel o de un nivel próximo; cambiar de nivel está permitido en cualquier momento y los drones sobrantes pueden subir. La primera versión puede ser greedy/determinista, siempre que evite elecciones absurdamente lejanas y quede encapsulada para futuras mejoras.

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

6E mantiene la cola pero todavía no contiene el criterio funcional acordado. Una política FIFO o “más cerca de la esquina principal” puede mandar drones a recorridos largos, elegir el sentido equivocado y desperdiciar autonomía.

## Invariantes y decisiones cerradas

- La distancia/coste se calcula hacia las entradas A/C, no hacia B.
- La entrada seleccionada determina el sentido de `MAP_SECTION`.
- No hay una orientación “clockwise” obligatoria para todas las tareas.
- Cambiar de nivel es válido; la cercanía vertical puede formar parte del coste, no una barrera.
- El solape entre tareas vecinas se conserva y no debe penalizarse hasta eliminarlo.
- Solo se asignan drones con pose/estado suficientemente válido según Fase 5.

## Archivos permitidos a modificar

```text
src/servidor/orbslam3_server/include/  # TaskAllocator o equivalente
src/servidor/orbslam3_server/src/
src/servidor/orbslam3_server/config/  # pesos de coste si se parametrizan
src/servidor/orbslam3_server/test/
codex/contexto/paquetes/orbslam3_server/
```

Las rutas marcadas como propuestas deben confirmarse contra el árbol real antes de crearlas. Si Fase 5 ya contiene un componente equivalente, se amplía/reutiliza en lugar de duplicarlo.

## Archivos prohibidos

```text
src/dron/dron_individual/  # el servidor asigna, el dron planifica
src/servidor/orbslam3_multi/
ORB_SLAM3/
```

Además, no tocar legacy o paquetes ajenos a la subfase como limpieza colateral.

## Funciones, clases, nodos o interfaces que hay que localizar

```text
TaskManager de 6E
fuente de pose/estado de dron de Fase 5
representación A/B/C de 6C
cualquier política de prioridad ya existente tras Fase 5
```

Los nombres nuevos que aparezcan en este documento son nombres de contrato/propuesta. Si el workspace real ya posee una abstracción equivalente, reutilizarla y documentar la correspondencia antes de implementar.

## Cambios requeridos

1. Crear una función de coste explícita y testeable por `(drone, task, entry)` que compare A y C.
2. Usar distancia global como término base; añadir penalización por cambio vertical/lejanía y continuidad con la última tarea solo si los parámetros quedan documentados.
3. Filtrar drones sin pose global utilizable o con una tarea `RUNNING`.
4. Para cada asignación, guardar en la tarea la entrada elegida y el sentido de cobertura resultante.
5. Procesar múltiples drones de forma determinista para que una tarea ya elegida deje de estar disponible antes de evaluar la siguiente asignación.
6. Evitar concentrar drones en una misma entrada si existen alternativas de coste razonable; no eliminar el solape estructural entre tareas.
7. Añadir logs `TASK-ALLOC-CANDIDATE`, `TASK-ALLOC-SELECTED` con coste, entrada y sentido.

## Cambios prohibidos

- No usar GT para distancia/asignación.
- No imponer “completar un piso” antes de asignar el siguiente.
- No ejecutar un optimizador global complejo como requisito inicial si un greedy correcto satisface las pruebas.
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
./codex/herramientas/run_simulation.sh   --prueba fase_6_6F   --launch "ros2 launch simulacion_dron multi_dron.launch.py"   --post-scenario-wait-sec 20
```

Si la prueba indicada es determinista/unitaria y no necesita Gazebo, no arrancarlo artificialmente; ejecutar el test del paquete y registrar el comando exacto en historial.

### Prueba 1 — Entrada correcta

Colocar sintéticamente dos drones cerca de extremos distintos de una misma sección. Verificar que el coste se calcula a A/C y que el sentido elegido corresponde a la entrada más conveniente.

### Prueba 2 — Continuidad entre niveles

Con varias tareas pendientes y un dron que acaba de terminar, comprobar que se elige una tarea razonablemente cercana aunque esté en otro nivel, y que no se fuerza una espera por piso.

## Patrones de reducción de logs

```text
TASK-ALLOC-CANDIDATE|TASK-ALLOC-SELECTED|entry=|direction=|cost=|pose_invalid|ERROR|FATAL|Segmentation fault|Killed
```

El log completo se conserva como artefacto y se reduce antes de leerlo, según `AGENTS.md`. Si el reducido no contiene evidencia suficiente, regenerar un reducido con patrones más precisos; no usar el log completo como contexto directo.

## Criterio de éxito

La subfase se considera `CONSEGUIDA` solo si:

1. La selección usa A/C y no B como destino de proximidad.
2. El sentido de tarea queda fijado al asignarla.
3. Drones sin pose fiable no reciben trabajo normal.
4. La política permite transición entre niveles y produce elecciones deterministas.
5. El solape nominal de tareas no se elimina.

Además, el build requerido debe devolver `0`, las pruebas obligatorias deben haberse ejecutado, los marcadores deben aparecer sin errores graves no explicados y la documentación/historial real debe quedar sincronizada.

## Criterio de fallo o parcial

- `PARCIAL` si la selección es correcta para un dron pero falla con varios candidatos.
- `PARCIAL` si la entrada es correcta pero no se conserva continuidad espacial entre tareas.

- `NO CONSEGUIDA` si no compila, una prueba obligatoria no se ejecuta, falta evidencia crítica o el comportamiento contradice el objetivo.
- `BLOQUEADA` solo ante dependencia/información externa realmente irresoluble con cambios mínimos.

## Riesgos

- Coste mal escalado que favorezca saltos verticales absurdos.
- Asignar por B y romper la semántica acordada de entrada.
- Hacer el algoritmo no determinista al iterar contenedores sin orden.

## Documentación a actualizar

Al ejecutar realmente la subfase, actualizar solo la documentación que corresponda al código tocado, incluyendo:

```text
codex/pipeline/fase_6_tareas_trayectorias/historial/por_subfase/historial_6F.md
codex/pipeline/fase_6_tareas_trayectorias/historial/por_subfase/historial_6F_RESUMEN.md
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
