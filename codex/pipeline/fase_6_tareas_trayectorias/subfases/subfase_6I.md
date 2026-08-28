# Subfase 6I — Extensión de `lib_tray` y `TrayAction` para trayectorias multi-waypoint

## Estado

```text
sin hacer
```

## Dependencia

6H y la cadena de control validada en Fase 1/Fase 5.

## Objetivo técnico

Hacer ejecutables las listas de waypoints sin sustituir el controlador existente: `lib_tray` debe generar referencias continuas entre puntos y `gen_tray`/`TrayAction` debe evaluar el segmento correcto en cada instante, incluyendo `yaw`.

## Comportamiento esperado

Un goal con waypoints debe producir una trayectoria temporal completa y continua, pero compuesta por tramos internos. Debe conservar las propiedades que necesita el controlador actual:

```text
posición / velocidad / aceleración / jerk cuando proceda
yaw / yaw_rate ... según contrato vigente
```

El mecanismo puede reutilizar `GenTrayPol3`/`GenTrayVelTrap` por segmento o crear una abstracción de ruta multi-tramo, siempre que no introduzca discontinuidades peligrosas ni destruya los generadores existentes.

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

`lib_tray` entregado contiene `GenTrayPol3`, `GenTrayVelTrap` y `GenTrayElipse`, con tests de pol3/veltrap. `gen_tray.cpp` expone un `TrayAction` de objetivo único; no sabe recorrer una lista arbitraria de waypoints.

## Invariantes y decisiones cerradas

- Waypoints y yaw deben cumplirse en orden.
- Las uniones entre tramos no deben introducir saltos de posición ni cambios de referencia incompatibles con el controlador.
- La cancelación de action sigue funcionando y detiene la trayectoria completa.
- Los modos legacy siguen disponibles.
- No usar GT como condición inicial; tomar pose/velocidad de la fuente Fase 5.
- Al sustituir los goals externos por una trayectoria multi-waypoint, eliminar
  el mecanismo temporal Fase 5 de `GT_FALLBACK`, source lock entre goals,
  handshake de frontera, hold y handoff angular SO(3) por cambio de fuente. No
  dejar ramas muertas.
- Mantener el contrato útil: condición inicial atómica de una sola muestra,
  con primer setpoint `x0/v0/yaw0/yaw_rate0` y `ep=ev=er=ew=0`.

## Archivos permitidos a modificar

```text
src/dron/lib_tray/include/
src/dron/lib_tray/src/
src/dron/lib_tray/test/
src/dron/lib_tray/CMakeLists.txt
src/dron/dron_individual/src/control_tray/gen_tray.cpp
src/dron/dron_individual/action/TrayAction.action
src/dron/dron_individual/config/
src/dron/dron_individual/CMakeLists.txt
codex/contexto/paquetes/lib_tray/
codex/contexto/paquetes/dron_individual/
```

Las rutas marcadas como propuestas deben confirmarse contra el árbol real antes de crearlas. Si Fase 5 ya contiene un componente equivalente, se amplía/reutiliza en lugar de duplicarlo.

## Archivos prohibidos

```text
src/dron/dron_individual/src/control_tray/control_calcular_fuerzas.cpp  # salvo incompatibilidad demostrada
src/dron/dron_individual/src/control_tray/aplicar_fuerzas_dron.cpp
src/servidor/orbslam3_server/
ORB_SLAM3/
```

Además, no tocar legacy o paquetes ajenos a la subfase como limpieza colateral.

## Funciones, clases, nodos o interfaces que hay que localizar

```text
`GenTrayPol3`
`GenTrayVelTrap`
`GenTrayElipse`
callbacks/goal handling de `gen_tray.cpp`
tests `test_gen_tray_pol3.cpp`, `test_gen_tray_veltrap.cpp`
fuente de condición inicial posterior a Fase 5
```

Los nombres nuevos que aparezcan en este documento son nombres de contrato/propuesta. Si el workspace real ya posee una abstracción equivalente, reutilizarla y documentar la correspondencia antes de implementar.

## Cambios requeridos

1. Diseñar una clase/estructura multi-tramo en `lib_tray` que posea lista de waypoints, tiempos por tramo y evaluación global `t -> referencia`.
2. Calcular tiempos/tramos con límites dinámicos ya existentes o parámetros claros; no introducir valores mágicos específicos de la casa.
3. Interpolar yaw con convención de ángulo corta/coherente y testear cruces ±π.
4. Garantizar continuidad de posición y, cuando el perfil elegido lo permita/contrato lo exija, velocidad y aceleración en las uniones.
5. Ampliar `gen_tray` para aceptar el goal multi-waypoint, crear la trayectoria y publicar feedback/resultado global manteniendo cancelación.
6. Mantener explícitamente los tipos legacy y sus tests; añadir tests unitarios de 2, 3 y N waypoints.
7. Añadir logs `TRAY-WAYPOINT-START`, `TRAY-WAYPOINT-SWITCH`, `TRAY-WAYPOINT-DONE`, evitando spam a frecuencia de control.

## Cambios prohibidos

- No resolver discontinuidades bajando ganancias del controlador.
- No concatenar goals externos independientes como sustituto de una trayectoria multi-waypoint si eso introduce paradas no acordadas.
- No eliminar tests/generadores anteriores.
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
./codex/herramientas/build_selected_packages.sh lib_tray dron_individual
```

Si la distribución física creada por Fase 2 requiere el helper por grupos, usar la herramienta vigente equivalente y registrar el comando exacto en historial. Añadir dependencias reales solo si el build demuestra que son necesarias.

## Pruebas Gazebo requeridas

Preparación común:

- Si la prueba necesita una secuencia reproducible, usar/crear un YAML de escenario dentro de `src/simulacion/simulacion_dron/config/scenarios/fase_6/` (ruta final a confirmar contra Fase 2), no en el grupo Dron/Servidor.
- No precalcular en el scenario runner la autonomía que precisamente se está validando; el runner solo prepara condiciones, inyecta eventos o espera resultados.
- Comando base para cualquier prueba Gazebo de esta subfase:

```bash
./codex/herramientas/run_simulation.sh   --prueba fase_6_6I   --launch "ros2 launch simulacion_dron multi_dron.launch.py"   --post-scenario-wait-sec 20
```

Si la prueba indicada es determinista/unitaria y no necesita Gazebo, no arrancarlo artificialmente; ejecutar el test del paquete y registrar el comando exacto en historial.

### Prueba 1 — Tests deterministas de `lib_tray`

Evaluar cada waypoint y puntos inmediatamente antes/después de un cambio de tramo. Verificar orden, continuidad y yaw, incluyendo un cruce de ±π.

### Prueba 2 — Vuelo multi-waypoint corto

En Gazebo, enviar a un dron una ruta pequeña de 3–4 waypoints con yaw distintos. El action debe terminar `success=true`, sin saltos visuales graves ni regresiones del control.

### Prueba 3 — Cancelación

Cancelar el goal multi-waypoint a mitad y confirmar que no continúa hacia los waypoints restantes y que el estado se propaga limpiamente.

## Patrones de reducción de logs

```text
TRAY-WAYPOINT|GOAL|RESULT|success|cancel|continu|yaw|ERROR|FATAL|Segmentation fault|Killed
```

El log completo se conserva como artefacto y se reduce antes de leerlo, según `AGENTS.md`. Si el reducido no contiene evidencia suficiente, regenerar un reducido con patrones más precisos; no usar el log completo como contexto directo.

## Criterio de éxito

La subfase se considera `CONSEGUIDA` solo si:

1. Los tests unitarios multi-waypoint pasan y los legacy no regresan.
2. El dron recorre waypoints en orden con yaw correcto.
3. No aparecen discontinuidades funcionalmente peligrosas.
4. Cancelación y resultado funcionan para la ruta completa.
5. La pose inicial proviene del estimador Fase 5.

Además, el build requerido debe devolver `0`, las pruebas obligatorias deben haberse ejecutado, los marcadores deben aparecer sin errores graves no explicados y la documentación/historial real debe quedar sincronizada.

## Criterio de fallo o parcial

- `PARCIAL` si posiciones funcionan pero yaw/continuidad no cumplen el contrato.
- `PARCIAL` si vuelo pasa pero se rompe un generador legacy.

- `NO CONSEGUIDA` si no compila, una prueba obligatoria no se ejecuta, falta evidencia crítica o el comportamiento contradice el objetivo.
- `BLOQUEADA` solo ante dependencia/información externa realmente irresoluble con cambios mínimos.

## Riesgos

- Discontinuidad en velocidad al cambiar de tramo.
- Wrap incorrecto de yaw.
- Duración global mal calculada que haga terminar el action antes/después.

## Documentación a actualizar

Al ejecutar realmente la subfase, actualizar solo la documentación que corresponda al código tocado, incluyendo:

```text
codex/pipeline/fase_6_tareas_trayectorias/historial/por_subfase/historial_6I.md
codex/pipeline/fase_6_tareas_trayectorias/historial/por_subfase/historial_6I_RESUMEN.md
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
