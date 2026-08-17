# Subfase 6Q — Detección espacial de conflictos Dron-Dron mediante corredores 3D

## Estado

```text
sin hacer
```

## Dependencia

6P, tamaños/márgenes de seguridad configurados y pose global de Fase 5.

## Objetivo técnico

Rechazar una propuesta cuando su volumen barrido/corredor de seguridad intersecta la trayectoria reservada o el volumen actual de otro dron, usando una política puramente espacial y conservadora, sin planificación temporal.

## Comportamiento esperado

Cada polilínea de waypoints se infla por el tamaño del dron y márgenes de seguridad. Baseline recomendado: aproximar cada dron por una esfera/cápsula conservadora para que la validación sea robusta y barata.

Para dos trayectorias A y B, si en cualquier par de segmentos la distancia mínima 3D es inferior a la separación requerida, existe conflicto:

```text
min_distance(segment_A, segment_B)
  < radius_A + radius_B + safety_margin (+ error_margin)
=> CONFLICT
```

**No se utiliza tiempo.** Si las curvas ocupan la misma región espacial, la segunda propuesta se rechaza aunque los drones pudieran pasar por allí en instantes distintos. Esta decisión evita dependencia de retrasos/relojes. El conservadurismo se compensa con trayectorias cortas y liberación inmediata de 6J/6P.

La propuesta también se comprueba contra el volumen de drones parados y contra `flight_bounds` como defensa global. Paredes/objetos físicos no se comprueban aquí.

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

El servidor dispone tras 6P de rutas activas pero aún no sabe si se interfieren. Comparar solo puntos/curvas de grosor cero permitiría pasar demasiado cerca; usar ventanas temporales contradiría la política acordada.

## Invariantes y decisiones cerradas

- Conflicto = intersección espacial de corredores, independientemente del tiempo.
- El primer commit aceptado conserva prioridad frente a propuestas posteriores; una propuesta nueva no expulsa una reserva activa.
- El tamaño/margen debe ser conservador y documentado.
- Drones detenidos también generan zona prohibida.
- `flight_bounds` se puede validar en servidor como defensa, pero no genera tareas.
- El servidor no consulta depth ni el mapa de paredes para esta decisión.

## Archivos permitidos a modificar

```text
src/servidor/orbslam3_server/include/  # ConflictDetector
src/servidor/orbslam3_server/src/
src/servidor/orbslam3_server/config/  # safety params locales
src/servidor/orbslam3_server/test/
codex/contexto/paquetes/orbslam3_server/
```

Las rutas marcadas como propuestas deben confirmarse contra el árbol real antes de crearlas. Si Fase 5 ya contiene un componente equivalente, se amplía/reutiliza en lugar de duplicarlo.

## Archivos prohibidos

```text
src/dron/dron_individual/src/vision/
src/servidor/orbslam3_multi/
ORB_SLAM3/
```

Además, no tocar legacy o paquetes ajenos a la subfase como limpieza colateral.

## Funciones, clases, nodos o interfaces que hay que localizar

```text
TrajectoryReservationManager 6P
representación de waypoints 6H
fuente/configuración de tamaño de dron y margen de seguridad
`flight_bounds` 6A
pose global de Fase 5
```

Los nombres nuevos que aparezcan en este documento son nombres de contrato/propuesta. Si el workspace real ya posee una abstracción equivalente, reutilizarla y documentar la correspondencia antes de implementar.

## Cambios requeridos

1. Definir el modelo geométrico de seguridad baseline (esfera/cápsula o bounding volume conservador) y documentar cómo se obtiene el radio/tamaño de cada dron sin duplicación no controlada de YAML.
2. Implementar distancia mínima segmento-segmento 3D o algoritmo equivalente y construir el corredor de una polilínea completa.
3. Comparar propuesta contra todas las reservas de otros drones y contra el volumen actual de drones sin reserva.
4. Rechazar cualquier waypoint/segmento fuera de `flight_bounds` con razón diferenciada `outside_flight_bounds`.
5. Devolver el primer conflicto útil y/o conjunto acotado: `other_drone_id`, `other_trajectory_id`, segmento/región/punto aproximado de mínima distancia y margen requerido.
6. No considerar conflicto con la propia reserva antigua durante la transición de cancelación si el lifecycle garantiza que ya fue liberada; si no, rechazar hasta resolver estado.
7. Añadir tests geométricos deterministas (cruce, paralelo seguro, vertical, touch al margen, dron parado) y logs `TRAJ-CONFLICT`/`TRAJ-CLEAR`.

## Cambios prohibidos

- No aceptar un cruce porque las duraciones estimadas sean diferentes.
- No reducir el margen para hacer pasar una prueba.
- No tratar MapPoints/paredes como parte del detector servidor.
- No expulsar la trayectoria ya aceptada en favor de una posterior de mayor prioridad en el baseline.
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
./codex/herramientas/run_simulation.sh   --prueba fase_6_6Q   --launch "ros2 launch simulacion_dron multi_dron.launch.py"   --post-scenario-wait-sec 20
```

Si la prueba indicada es determinista/unitaria y no necesita Gazebo, no arrancarlo artificialmente; ejecutar el test del paquete y registrar el comando exacto en historial.

### Prueba 1 — Cruce espacial en tiempos diferentes

Proponer y aceptar D1: A→B. Después proponer D2 con una curva que cruza el mismo corredor. Debe rechazarse **aunque** la duración/tiempo estimado no coincida.

### Prueba 2 — Trayectorias próximas pero seguras

Dos rutas paralelas separadas por más de la suma de radios+margen deben aceptarse. Repetir justo por debajo/encima del umbral.

### Prueba 3 — Dron parado

Con D1 sin trayectoria, proponer una ruta de D2 que atraviese su volumen actual: rechazo. Una ruta que lo rodea con margen: aceptación.

### Prueba 4 — `flight_bounds`

Propuesta sin conflicto Dron-Dron pero que sale del volumen permitido: rechazo específico.

## Patrones de reducción de logs

```text
TRAJ-CONFLICT|TRAJ-CLEAR|other_drone|other_trajectory|min_distance|safety|outside_flight_bounds|RESERVATION|ERROR|FATAL|Segmentation fault|Killed
```

El log completo se conserva como artefacto y se reduce antes de leerlo, según `AGENTS.md`. Si el reducido no contiene evidencia suficiente, regenerar un reducido con patrones más precisos; no usar el log completo como contexto directo.

## Criterio de éxito

La subfase se considera `CONSEGUIDA` solo si:

1. Cruces espaciales se rechazan independientemente del tiempo.
2. Rutas separadas por el margen requerido se aceptan.
3. Se considera el volumen de drones parados.
4. Se rechazan rutas fuera de `flight_bounds`.
5. El conflicto devuelve información suficiente para 6R sin consultar obstáculos físicos.

Además, el build requerido debe devolver `0`, las pruebas obligatorias deben haberse ejecutado, los marcadores deben aparecer sin errores graves no explicados y la documentación/historial real debe quedar sincronizada.

## Criterio de fallo o parcial

- `PARCIAL` si funciona ruta-ruta pero ignora drones parados.
- `PARCIAL` si detecta cruce pero no respeta correctamente tamaños/márgenes.

- `NO CONSEGUIDA` si no compila, una prueba obligatoria no se ejecuta, falta evidencia crítica o el comportamiento contradice el objetivo.
- `BLOQUEADA` solo ante dependencia/información externa realmente irresoluble con cambios mínimos.

## Riesgos

- Errores numéricos en segmentos degenerados/waypoints repetidos.
- Modelo de esfera demasiado conservador o demasiado pequeño.
- Coste O(N²) excesivo si se permiten rutas demasiado largas; 6J limita el horizonte.

## Documentación a actualizar

Al ejecutar realmente la subfase, actualizar solo la documentación que corresponda al código tocado, incluyendo:

```text
codex/pipeline/fase_6_tareas_trayectorias/historial/por_subfase/historial_6Q.md
codex/pipeline/fase_6_tareas_trayectorias/historial/por_subfase/historial_6Q_RESUMEN.md
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
