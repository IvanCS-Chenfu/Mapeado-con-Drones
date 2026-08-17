# Subfase 2D — Comprobar el funcionamiento completo tras la separación

## Estado

```text
SIN HACER
Dependencias: 2A, 2B y 2C suficientemente completadas
Prueba oficial: dos drones rodean completamente el edificio
Debug base: todos los flags en false
```

## Objetivo técnico

Demostrar que la nueva distribución física y la nueva arquitectura de
configuración conservan el funcionamiento del proyecto.

La comprobación no se limita a que el launch arranque. Debe validar:

- descubrimiento de paquetes;
- independencia de Dron y Servidor;
- integración de Simulación;
- includes, exports, librerías y recursos instalados;
- carga de YAML y tipos de parámetros;
- topics, services, actions, namespaces y frames;
- control y ejecución de trayectoria;
- wrapper ORB-SLAM3 y flujo del mapa cuando estén disponibles;
- servidor global;
- escenario largo de dos drones alrededor del edificio;
- ausencia de dependencias accidentales del árbol fuente;
- ausencia de errores graves no explicados.

## Contexto obligatorio a leer

```text
AGENTS.md
codex/contexto/00_CONTEXTO_COMPACTACION.md
codex/contexto/09_LOGS_Y_SUBLOGS.md
codex/pipeline/fase_2_separacion_paquetes/pipeline_fase_2.md
codex/pipeline/fase_2_separacion_paquetes/subfases/subfase_2B.md
codex/pipeline/fase_2_separacion_paquetes/subfases/subfase_2C.md
codex/contexto/04_TOPICS_SERVICES_ACTIONS.md
codex/contexto/paquetes/simulacion_dron/scenario_runner_node.md
codex/contexto/paquetes/simulacion_dron/launches.md
codex/contexto/paquetes/dron_individual/launches.md
codex/contexto/paquetes/orbslam3_server/launches.md
```

Leer el escenario final desde su nueva ruta instalada en Simulación. No usar la
copia histórica de `codex` como fuente runtime.

## Precondiciones

Antes de ejecutar la prueba larga:

1. `dron` ha compilado en aislamiento;
2. `servidor` ha compilado en aislamiento;
3. `simulacion` ha compilado como overlay;
4. las dos copias de `orbslam3_msgs` han sido comparadas y son idénticas;
5. los YAML parsean y están instalados;
6. todos los flags de `debug.yaml` están en `false`;
7. el escenario oficial existe en `simulacion_dron/config/scenarios/`;
8. no hay procesos Gazebo/ROS 2 residuales;
9. hay espacio suficiente en disco;
10. el checkpoint registra el comando que se va a ejecutar.

## Matriz de validación

### 1. Descubrimiento y prefijos

Comprobar por grupo:

```text
ros2 pkg prefix <paquete>
colcon list --base-paths <grupo>
AMENT_PREFIX_PATH
CMAKE_PREFIX_PATH
```

Validar que:

- Dron no resuelve paquetes de Servidor;
- Servidor no resuelve paquetes de Dron;
- Simulación resuelve ambos prefijos;
- `orbslam3_msgs` activo en integración corresponde a la precedencia acordada;
- no aparece una tercera copia de ningún paquete;
- los recursos se encuentran bajo `install/<grupo>` y no bajo `src/`.

### 2. Bibliotecas e includes

Comprobar:

- ejecutables enlazan librerías instaladas;
- headers públicos provienen de includes exportados;
- no existen includes con rutas relativas al antiguo árbol;
- `lib_tray` es consumida desde el prefijo Dron;
- `orbslam3_multi` es consumida desde el prefijo Servidor;
- el wrapper encuentra ORB-SLAM3 y sus símbolos requeridos;
- no aparecen `undefined reference` ni `undefined symbol`.

### 3. Configuración

Comprobar en runtime:

- `mass_total` e `inertia_total` llegan a los nodos previstos;
- no se carga la antigua copia de masa/inercia desde trayectoria;
- número de drones y posiciones proceden de Simulación;
- parámetros de servidor proceden de YAML de Servidor;
- flags debug proceden de Simulación y están desactivados;
- réplicas parciales tienen los valores acordados;
- ningún nodo intenta abrir un YAML de otro grupo.

### 4. Interfaces ROS 2

Inventariar y verificar:

- publishers/subscribers;
- services/clients;
- action servers/clients;
- tipos exactos;
- QoS relevante;
- namespaces por dron;
- frames y TF;
- parámetros declarados.

No exigir nombres nuevos si el comportamiento previo usa otros. Comparar con la
documentación vigente y actualizarla si el código real difiere.

### 5. Arranque aislado de Dron

Con entorno limpio de Dron:

- arrancar nodos base que no requieran sensores simulados;
- comprobar que la ausencia de Gazebo no provoca error de carga de paquete;
- comprobar que los nodos que sí requieren datos esperan/degradan de forma
  conocida en vez de depender de un paquete de Servidor;
- comprobar disponibilidad de `TrayAction` y librerías locales.

No se exige vuelo real fuera de Simulación.

### 6. Arranque aislado de Servidor

Con entorno limpio de Servidor:

- arrancar el servidor sin Dron ni Gazebo;
- comprobar que puede esperar deltas/services sin importar paquetes de control;
- comprobar que los YAML del servidor se cargan;
- comprobar que el proceso no necesita rutas a modelos o plugins.

### 7. Smoke de Simulación

Antes del escenario largo:

- arrancar Gazebo;
- crear un dron;
- crear dos drones;
- comprobar cámaras, GT permitido, motores y reloj;
- comprobar inclusión de launch de Dron y Servidor desde instalaciones;
- enviar un goal corto y verificar finalización;
- comprobar publicación del mapa si el frontend está disponible.

## Prueba oficial — Vuelta completa al edificio con dos drones

### Escenario

Usar el escenario típico:

```text
simulacion_dron/config/scenarios/prueba_tipica_rodeo_edificio_dos_fiduciales.yaml
```

Si el nombre físico final difiere, conservar la semántica: dos drones completan
la vuelta habitual alrededor del edificio. No cambiar la trayectoria para hacer
la prueba más fácil salvo acuerdo explícito.

### Configuración base

```text
número de drones: 2
mundo: edificio/casa habitual
comunicación: ROS 2 directa
pipeline_flow: desactivado
system_architecture: desactivado
navegadores: desactivados
RViz de debug: desactivado salvo que forme parte obligatoria de la evidencia
dumps/animaciones/logs verbose: desactivados
```

El objetivo es comprobar funcionalidad sin ayudas de debug.

### Secuencia

1. limpiar procesos residuales;
2. cargar ROS 2 base;
3. cargar instalación de Dron;
4. cargar instalación de Servidor con la precedencia acordada;
5. cargar instalación de Simulación;
6. registrar parámetros efectivos;
7. arrancar el launch oficial;
8. esperar a que Gazebo, drones, wrapper y servidor estén listos;
9. ejecutar el escenario completo;
10. esperar el resultado final de ambos drones;
11. permitir una ventana final para publicación/cleanup;
12. cerrar mediante la herramienta oficial;
13. reducir el log antes de leerlo;
14. analizar evidencia contra los criterios.

Comando orientativo:

```bash
./codex/herramientas/run_simulation.sh \
  --prueba fase_2_vuelta_edificio_2_drones \
  --launch "ros2 launch simulacion_dron multi_dron.launch.py"
```

Los argumentos concretos deben apuntar al escenario instalado y a la nueva
configuración.

### Resultados funcionales esperados

- los dos drones son creados en namespaces distintos;
- ambos reciben y ejecutan sus goals;
- ambos completan la vuelta;
- no se cruzan identificadores, topics o acciones;
- los motores/control reciben parámetros correctos;
- la simulación no intenta leer YAML fuera de su grupo;
- el wrapper y el servidor intercambian `orbslam3_msgs` compatibles;
- el servidor sigue recibiendo/publicando durante la prueba;
- el escenario termina con `success=true` para los dos drones;
- no hay crash, deadlock o bloqueo causado por la separación;
- el cleanup no deja procesos persistentes.

## Comprobaciones visuales posteriores

La validación de los visualizadores se completa en 2F, pero 2D puede realizar
una repetición acotada después de la prueba base:

- activar solo `debug_pipeline_flow_web` y su navegador si se desea;
- confirmar que la herramienta existente sigue funcionando;
- desactivar de nuevo el flag.

El nuevo diagrama arquitectónico todavía puede no existir antes de 2F.

## Pruebas negativas

Ejecutar comprobaciones intencionales:

1. intentar compilar Dron con prefijos externos eliminados;
2. intentar compilar Servidor con prefijos externos eliminados;
3. desactivar un recurso debug y confirmar que su proceso no arranca;
4. alterar temporalmente una ruta de escenario en una copia de prueba y
   comprobar que el error es explícito, no un fallback silencioso a `src/`;
5. comprobar que una divergencia simulada de `orbslam3_msgs` sería detectada
   por la guarda, sin dejar la divergencia en el repositorio.

No se inyectan cambios destructivos en la rama de trabajo sin rollback claro.

## Includes y rutas a revisar durante la prueba

Ante fallo, localizar:

```text
#include <lib_tray/...>
#include <orbslam3_multi/...>
#include <orbslam3_msgs/...>
get_package_share_directory(...)
FindPackageShare(...)
PathJoinSubstitution(...)
open(...yaml...)
```

No corregir un `#include` con una ruta al árbol fuente. Corregir exports e
instalación.

## Patrones de reducción de logs

### Build/arranque

```text
Starting >>>|Finished <<<|Failed <<<|package not found|Could not find|No such file|undefined reference|undefined symbol|ImportError|duplicate package|ERROR|FATAL
```

### Escenario

```text
SCENARIO-RUNNER|GOAL|RESULT|success|TrayAction|dron_1|dron_2|orb_map_delta|get_full_map|global_sparse_cloud|global_keyframes|Parameter|yaml|ERROR|FATAL|Segmentation fault|Killed|SIM-DONE
```

Añadir markers específicos de readiness y finalización que existan realmente.
No leer el log completo.

## Métricas mínimas

Registrar:

- paquetes compilados por grupo;
- tiempo de build por grupo;
- tests pasados/fallidos;
- número de drones creados;
- goals enviados y completados por dron;
- duración del escenario;
- procesos debug arrancados durante la prueba base: debe ser cero;
- errores graves;
- rutas de recursos resueltas;
- prefijo activo de `orbslam3_msgs`;
- resultado agregado.

No inventar umbrales de rendimiento nuevos en esta fase.

## Criterio de éxito

`CONSEGUIDA` solo si:

1. Dron y Servidor arrancan en sus entornos aislados;
2. Simulación integra ambos grupos;
3. no hay errores de includes, librerías o recursos;
4. los YAML correctos se cargan desde los prefijos instalados;
5. topics, services, actions, namespaces y frames son coherentes;
6. los dos drones completan la vuelta al edificio;
7. el escenario informa éxito;
8. no se usan rutas runtime a `src/codex` ni a otro grupo;
9. no aparecen errores graves no explicados;
10. los logs están reducidos y documentados.

## Criterio de parcial, fallo o bloqueo

`PARCIAL` si los builds y el smoke pasan, pero la prueba larga falla por una
regresión funcional preexistente demostrada y separable de la nueva estructura.
Debe quedar evidencia y no puede declararse cierre de Fase 2.

`NO CONSEGUIDA` si un grupo no arranca, falta una interfaz, un dron no completa
la trayectoria, se cargan YAML de otro grupo o el sistema depende del árbol
fuente.

`BLOQUEADA` solo si falta un componente externo imprescindible para ejecutar el
escenario y no existe una instalación compatible.

## Cambios permitidos durante diagnóstico

- corregir includes, manifests, exports y rutas;
- corregir launch/YAML para cumplir el contrato de 2C;
- corregir instalación de recursos;
- añadir markers mínimos de validación;
- ajustar herramientas de ejecución/reducción.

Si el fallo exige cambiar algoritmos o parámetros físicos/umbrales sin acuerdo,
suspender y preguntar.

## Documentación de cierre

Actualizar al ejecutar:

```text
codex/contexto/00_CONTEXTO_COMPACTACION.md
codex/contexto/01_ESTADO_ACTUAL_RESUMEN.md
codex/contexto/04_TOPICS_SERVICES_ACTIONS.md
codex/contexto/paquetes/**
codex/contexto/07_ULTIMA_SESION.md
codex/pipeline/fase_2_separacion_paquetes/historial/por_subfase/historial_2D.md
codex/pipeline/fase_2_separacion_paquetes/historial/por_subfase/historial_2D_RESUMEN.md
```

Cada ejecución real conserva una entrada propia, pase o falle.
