# Subfase 6B — Geometría del ROI, rebanadas verticales y puntos nominales de sección

## Estado

```text
sin hacer
```

## Dependencia

6A.

## Objetivo técnico

Transformar el ROI validado en niveles verticales y en un conjunto ordenado de puntos nominales por nivel que sirvan como semillas de cobertura, sin convertir las aristas del cuboide en trayectorias físicas obligatorias.

## Comportamiento esperado

La geometría de misión produce bandas verticales de cobertura. El dron debe mapear aproximadamente alrededor de la altura central de su banda, aunque puede desplazarse verticalmente por seguridad, obstáculos o mejor percepción.

La altura restante se **suma a la última banda**. Ejemplo:

```text
ROI z=[0,7], level_height=2
nivel 0 -> [0,2], z_nominal=1
nivel 1 -> [2,4], z_nominal=3
nivel 2 -> [4,7], z_nominal=5.5
```

Para `tasks_per_level=4` se usan las cuatro esquinas de la sección XY. Para `tasks_per_level=8` se intercalan esquinas y centros de lado. Cada punto debe conocer sus dos vecinos geométricos adyacentes, pero **no existe una dirección anterior/posterior global**: el sentido se decide al asignar la tarea.

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

La guía inicial hablaba de rebanadas y tareas por esquina, pero no define el resto vertical ni la variante de 8 tareas. Tampoco debe perpetuarse una interpretación rígida de “seguir el borde del cuboide”, porque la geometría real puede ser una L, interiores con pasillos o un laberinto.

## Invariantes y decisiones cerradas

- El último nivel absorbe el resto de altura; no se crea una rebanada final pequeña.
- La altura nominal es referencia de cobertura, no restricción exacta de vuelo.
- Los puntos nominales pertenecen a la sección del ROI, no a la pared real.
- Cada punto B tiene dos vecinos A y C; `MAP_SECTION_B` podrá ejecutarse A→B→C o C→B→A.
- La geometría nominal solo estructura el reparto. La ruta real se descubrirá en 6N/6O.

## Archivos permitidos a modificar

```text
src/servidor/orbslam3_server/src/  # módulo geométrico de misión
src/servidor/orbslam3_server/include/
src/servidor/orbslam3_server/test/  # si existe/creado para tests deterministas
codex/contexto/paquetes/orbslam3_server/
```

Las rutas marcadas como propuestas deben confirmarse contra el árbol real antes de crearlas. Si Fase 5 ya contiene un componente equivalente, se amplía/reutiliza en lugar de duplicarlo.

## Archivos prohibidos

```text
src/dron/dron_individual/  # todavía no ejecutar cobertura
src/servidor/orbslam3_multi/
ORB_SLAM3/
src/simulacion/simulacion_dron/worlds/
```

Además, no tocar legacy o paquetes ajenos a la subfase como limpieza colateral.

## Funciones, clases, nodos o interfaces que hay que localizar

```text
parser/configuración creada en 6A
convenciones `world` de Fase 5
estructura de tests unitarios vigente en orbslam3_server; si no existe, localizar patrón de tests del workspace
```

Los nombres nuevos que aparezcan en este documento son nombres de contrato/propuesta. Si el workspace real ya posee una abstracción equivalente, reutilizarla y documentar la correspondencia antes de implementar.

## Cambios requeridos

1. Crear una representación determinista `MappingLevel` o equivalente con `z_min`, `z_max` y `z_nominal`.
2. Implementar la división vertical absorbiendo el resto en el último nivel y cubrir casos `ROI_height < level_height` y múltiplo exacto.
3. Generar los cuatro puntos de esquina por nivel con convención documentada y sin asociar todavía un sentido de recorrido.
4. Para `tasks_per_level=8`, intercalar un punto en el centro de cada lado y construir la vecindad cíclica de los ocho puntos.
5. Cada punto nominal debe tener ID estable dentro del nivel y referencias a sus dos vecinos; no depender del orden en que se iteró un `map`/contenedor.
6. Crear tests deterministas con ROIs conocidos y logs `MISSION-LEVEL`/`MISSION-SECTION-POINT` para inspección.

## Cambios prohibidos

- No generar rutas de vuelo que sigan literalmente las aristas del ROI.
- No recortar el movimiento vertical del futuro planificador a `[z_min,z_max]`; esa banda describe cobertura.
- No crear una quinta tarea elíptica.
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
./codex/herramientas/run_simulation.sh   --prueba fase_6_6B   --launch "ros2 launch simulacion_dron multi_dron.launch.py"   --post-scenario-wait-sec 20
```

Si la prueba indicada es determinista/unitaria y no necesita Gazebo, no arrancarlo artificialmente; ejecutar el test del paquete y registrar el comando exacto en historial.

### Prueba 1 — División vertical con resto

Usar ROI `z=[0,7]`, `level_height=2`. Deben generarse exactamente tres bandas `[0,2]`, `[2,4]`, `[4,7]` y nominales `1`, `3`, `5.5`.

### Prueba 2 — Puntos 4 y 8

Con un ROI rectangular conocido, validar coordenadas y vecindad para `tasks_per_level=4` y `8`. Ningún punto puede carecer de dos vecinos ni duplicarse.

## Patrones de reducción de logs

```text
MISSION-LEVEL|MISSION-SECTION-POINT|z_nominal|neighbor|tasks_per_level|invalid|ERROR|FATAL|Segmentation fault|Killed
```

El log completo se conserva como artefacto y se reduce antes de leerlo, según `AGENTS.md`. Si el reducido no contiene evidencia suficiente, regenerar un reducido con patrones más precisos; no usar el log completo como contexto directo.

## Criterio de éxito

La subfase se considera `CONSEGUIDA` solo si:

1. La división vertical cubre el 100 % de la altura y el resto queda absorbido por la última banda.
2. Se generan 4 u 8 puntos deterministas por nivel con vecinos correctos.
3. No se asigna un sentido fijo de recorrido.
4. La salida geométrica es independiente de la geometría física real del edificio.

Además, el build requerido debe devolver `0`, las pruebas obligatorias deben haberse ejecutado, los marcadores deben aparecer sin errores graves no explicados y la documentación/historial real debe quedar sincronizada.

## Criterio de fallo o parcial

- `PARCIAL` si solo funciona el caso de 4 puntos o falla la división con resto.
- `PARCIAL` si la geometría es correcta pero no existe test determinista de vecindad.

- `NO CONSEGUIDA` si no compila, una prueba obligatoria no se ejecuta, falta evidencia crítica o el comportamiento contradice el objetivo.
- `BLOQUEADA` solo ante dependencia/información externa realmente irresoluble con cambios mínimos.

## Riesgos

- Off-by-one vertical que deje una franja sin tarea.
- Vecinos mal ordenados en la variante de 8 puntos.
- Confundir el punto nominal con una posición necesariamente libre/navegable.

## Documentación a actualizar

Al ejecutar realmente la subfase, actualizar solo la documentación que corresponda al código tocado, incluyendo:

```text
codex/pipeline/fase_6_tareas_trayectorias/historial/por_subfase/historial_6B.md
codex/pipeline/fase_6_tareas_trayectorias/historial/por_subfase/historial_6B_RESUMEN.md
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
