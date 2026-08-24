# Subfase 2F — Crear el diagrama arquitectónico estático y en vivo

## Especificación definitiva de `system_architecture`

- los nodos principales son paquetes agrupados en Dron, Servidor y Simulación;
- ejecutables, librerías, YAML y responsabilidades viven como metadata;
- las capas son `runtime`, `build/API`, `config/replica` y `deployment`;
- solo una arista runtime con evidencia directa puede iluminarse;
- un evento desconocido no se asigna a ninguna arista;
- la telemetría usa eventos propios ligeros y muestreados, no imágenes, nubes o mapas;
- `/global_mapping/flow_events` pertenece exclusivamente a `pipeline_flow`;
- web, navegador y telemetría tienen gating independiente y default `false`;
- el grafo estático funciona sin ROS 2 y el live recibe actividad mediante SSE.

La topología vigente incluye cámaras Simulación a `orbslam3`, GT provisional a
Dron y Servidor, control por `motor/*`, `AccionTrayectoria`, delta `OrbMap`,
request/response de `GetOrbMap`, backpressure, nube/keyframes y observabilidad.
No contiene el flujo futuro ficticio `orbslam3_to_dron`.

La distribución visual coloca Simulación y Servidor en la franja superior y
Dron en una franja inferior amplia. Las dependencias internas quedan debajo o
encima de su consumidor para reducir cruces, manteniendo zoom, pan y tooltips.

## Estado

```text
CONSEGUIDA
Dependencias: distribución, interfaces y launch estabilizados
system_architecture: estático y live validados de forma aislada y en prueba 200
pipeline_flow: independiente, lazy-gated y validado de forma aislada y en prueba 200
layout final: composición superior/inferior validada en dos viewports
```

## Objetivo técnico

Crear dentro de `simulacion_dron/web/` un segundo visualizador que represente la
arquitectura completa del proyecto:

- los tres grupos `Dron`, `Servidor` y `Simulación`;
- todos los paquetes contenidos en cada grupo;
- la copia de `orbslam3_msgs` presente en Dron y Servidor;
- topics, services, actions y flujos principales;
- dependencias de build/configuración relevantes;
- información transferida entre productores y consumidores;
- actividad real en vivo cuando el debug correspondiente esté habilitado.

El visualizador debe ser útil también sin ROS 2 activo: la topología estática no
depende de la telemetría.

## Separación obligatoria de herramientas

La estructura objetivo es:

```text
src/simulacion/simulacion_dron/web/
├── pipeline_flow/          # flujo interno ORB-SLAM3/mapa global existente
└── system_architecture/    # grupos, paquetes y comunicaciones
```

No reemplazar ni mezclar ambos visualizadores.

`pipeline_flow` mantiene su responsabilidad actual: detalle interno de ingesta,
bases de datos, worker, fusión, optimización y publicación.

`system_architecture` muestra el nivel de paquetes y grupos del sistema entero.

## Contexto obligatorio a leer

```text
AGENTS.md
codex/contexto/00_CONTEXTO_COMPACTACION.md
codex/contexto/03_ARQUITECTURA_ACTUAL.md
codex/contexto/04_TOPICS_SERVICES_ACTIONS.md
codex/contexto/05_MAPA_PAQUETES.md
codex/contexto/paquetes/simulacion_dron/pipeline_flow_visualizer.md
codex/contexto/paquetes/simulacion_dron/launches.md
codex/pipeline/fase_2_separacion_paquetes/pipeline_fase_2.md
codex/pipeline/fase_2_separacion_paquetes/subfases/subfase_2D.md
```

Antes de definir aristas, inventariar interfaces reales con documentación,
`ros2 node info`, `ros2 topic info`, `ros2 service type`, `ros2 action info` y
búsqueda estática. No inventar conexiones por intuición.

## Resultado visual objetivo

La vista debe contener contenedores claramente rotulados. Pueden representarse
como círculos, elipses, regiones o compound nodes, siempre que quede inequívoco
qué paquetes pertenecen a cada grupo.

Ejemplo conceptual:

```text
┌────────────────────── DRON ──────────────────────┐
│ dron_individual  lib_tray  ORB_SLAM3             │
│ orbslam3_ros2    orbslam3_msgs                   │
└──────────────────────────────────────────────────┘

┌──────────────────── SERVIDOR ────────────────────┐
│ orbslam3_server  orbslam3_multi  orbslam3_msgs   │
└──────────────────────────────────────────────────┘

┌─────────────────── SIMULACIÓN ───────────────────┐
│ simulacion_dron                                  │
└──────────────────────────────────────────────────┘
```

La representación final debe seguir el estilo web existente cuando resulte
práctico, sin sacrificar legibilidad.

## Información de los nodos

Cada nodo de paquete debe mostrar en tooltip o panel:

- nombre del paquete;
- grupo;
- ruta física;
- nombre ROS 2 si difiere del directorio;
- responsabilidad;
- ejecutables/librerías principales;
- YAML propietarios;
- dependencias dentro del grupo;
- dependencias permitidas hacia otros grupos;
- estado: activo, externo o integración;
- enlace a documentación local cuando sea viable.

Para las dos copias de `orbslam3_msgs`, mostrar:

```text
Servidor: canónica
Dron: réplica verificada
```

No representarlas como un único paquete compartido físicamente.

## Información de las aristas

Cada arista debe indicar cuando corresponda:

- dirección;
- productor;
- consumidor;
- nombre del topic/service/action;
- tipo de mensaje;
- namespace o patrón de namespace;
- QoS relevante;
- datos transportados;
- frecuencia aproximada solo si está documentada;
- si es runtime, build o configuración;
- si cruza grupos;
- si está activa en la ejecución actual.

No transportar payloads pesados al navegador. Los tooltips se basan en
metadatos y definiciones estáticas.

## Capas o modos

El usuario debe poder distinguir al menos:

### 1. Comunicaciones ROS 2

- topics;
- services;
- actions;
- TF si se incluye;
- flujos entre drones y servidor;
- conexiones de Simulación con sensores/actuadores.

### 2. Dependencias de build/configuración

- `dron_individual -> lib_tray`;
- wrapper -> ORB_SLAM3 y `orbslam3_msgs`;
- `orbslam3_server -> orbslam3_multi` y `orbslam3_msgs`;
- Simulación -> paquetes instalados de Dron/Servidor;
- launch que carga YAML de paquetes del mismo grupo;
- réplicas parciales entre grupos, representadas como relación de origen, no
  como carga directa de archivo.

Las capas pueden mostrarse con filtros, pestañas o estilos distintos.

## Definición declarativa

La topología y textos deben residir en un archivo declarativo independiente de
la UI. Ejemplos válidos:

```text
system_architecture/graph_definition.js
system_architecture/graph_definition.json
system_architecture/graph_definition.yaml
```

Debe contener:

- grupos;
- paquetes;
- conexiones;
- tooltips;
- categorías;
- IDs estables para telemetría.

La lógica de conexión, renderizado y animación debe estar separada, por ejemplo:

```text
index.html
styles.css
app.js
graph_definition.js
```

No generar manualmente la misma topología en varios archivos.

## Modo estático

Requisitos:

- abre localmente desde el bridge o servidor web;
- muestra todos los grupos y paquetes sin ROS 2 activo;
- permite zoom, pan, selección y tooltips;
- permite activar/desactivar capas;
- indica visualmente conexiones entre grupos;
- no requiere acceso a Internet;
- los assets quedan instalados con el paquete.

Si se usa Cytoscape.js u otra biblioteca ya incluida, conservarla localmente o
usar la estrategia existente. No introducir una dependencia remota obligatoria.

## Modo en vivo

### Fuente de actividad

La actividad debe provenir de eventos reales y ligeros. Se puede:

- reutilizar la infraestructura de bridge cuando sea apropiado;
- crear un topic de telemetría arquitectónica independiente;
- combinar eventos existentes con observación ROS 2 no invasiva.

La decisión debe priorizar bajo acoplamiento. El diagrama no puede gobernar el
sistema.

### Contenido de los eventos

Ejemplo conceptual:

```json
{
  "timestamp": 0.0,
  "source_package": "orbslam3_ros2",
  "target_package": "orbslam3_server",
  "edge_id": "orb_map_delta",
  "interface": "/dron_1/orbslam/orb_map_delta",
  "kind": "topic",
  "message_type": "orbslam3_msgs/msg/OrbMap",
  "count": 1,
  "status": "ok"
}
```

No enviar:

- nubes completas;
- descriptores;
- imágenes;
- mapas serializados completos;
- payloads de actions;
- datos sensibles o masivos innecesarios.

### Semántica visual

- iluminar la arista al transferirse datos;
- opcionalmente mostrar contadores y última actividad;
- no confundir “conexión declarada” con “mensaje observado”;
- soportar varios namespaces de dron;
- agrupar actividad repetitiva para no saturar la UI;
- descartar eventos antes de bloquear ROS 2.

## Debug y launch

Los flags pertenecen a `simulacion_dron/config/debug.yaml` y deben ser
independientes:

```text
debug_system_architecture_web: false
debug_open_system_architecture_browser: false
debug_architecture_telemetry: false
```

El launch debe respetar:

- si web es `false`, no arrancar el bridge/servidor;
- si navegador es `false`, no abrir el navegador;
- si telemetría es `false`, no instrumentar/publicar actividad específica;
- activar un flag no debe activar implícitamente los otros;
- al cerrar la subfase, los valores por defecto vuelven a `false`.

Los flags del visualizador `pipeline_flow` permanecen separados.

## Componentes probables

```text
src/simulacion/simulacion_dron/web/system_architecture/index.html
src/simulacion/simulacion_dron/web/system_architecture/styles.css
src/simulacion/simulacion_dron/web/system_architecture/app.js
src/simulacion/simulacion_dron/web/system_architecture/graph_definition.js
src/simulacion/simulacion_dron/src/visualizer/system_architecture_bridge.py
src/simulacion/simulacion_dron/launch/multi_dron.launch.py
src/simulacion/simulacion_dron/config/debug.yaml
src/simulacion/simulacion_dron/CMakeLists.txt
src/simulacion/simulacion_dron/package.xml
```

Puede reutilizar código común mínimo con `pipeline_flow`, pero no debe crear una
aplicación única que mezcle responsabilidades.

## Instrumentación en paquetes

Si la actividad no puede inferirse de forma fiable sin instrumentación, añadir
marcadores mínimos en los paquetes productores/consumidores.

Reglas:

- no bloquear callbacks;
- cola acotada;
- eventos por metadatos;
- sampling/coalescing para alta frecuencia;
- fallo silencioso controlado del canal de debug;
- sin cambios de lógica funcional;
- documentación en el paquete tocado.

Una alternativa válida es que el bridge observe el grafo ROS 2 y correlacione
interfaces con la definición estática, siempre que refleje actividad real y no
solo presencia de endpoints.

## Pasos de implementación

1. inventariar paquetes e interfaces reales;
2. definir IDs de grupos, nodos y aristas;
3. crear el grafo declarativo estático;
4. implementar layout y contenedores de grupo;
5. añadir tooltips y filtros;
6. instalar assets y probar sin ROS 2;
7. definir el contrato de eventos ligeros;
8. implementar bridge/telemetría;
9. conectar actividad a aristas;
10. añadir flags independientes al YAML/launch;
11. probar saturación, desconexión y cierre;
12. documentar la herramienta.

## Pruebas requeridas

### Prueba 1 — Estática sin ROS 2

- arrancar solo el servidor web;
- confirmar grupos y paquetes;
- comprobar tooltips;
- cambiar capas;
- verificar que no necesita Internet;
- comprobar que la copia canónica/réplica de mensajes se distingue.

### Prueba 2 — En vivo con smoke de dos drones

- activar web y telemetría;
- no abrir navegador automáticamente en una ejecución y abrirlo en otra;
- iniciar dos drones;
- comprobar actividad de sensores, control, wrapper y servidor;
- comprobar múltiples namespaces;
- cerrar navegador y confirmar que ROS 2 continúa.

### Prueba 3 — Vuelta al edificio

Repetir la prueba típica con:

```text
debug_system_architecture_web: true
debug_architecture_telemetry: true
```

Activar el navegador solo si se necesita observación humana. Confirmar que las
aristas se iluminan con actividad real y que el escenario mantiene éxito.

### Prueba 4 — Fallos y saturación

- detener el bridge;
- cerrar el navegador;
- saturar/coalescer eventos de forma controlada;
- desconectar/reconectar SSE o mecanismo elegido;
- confirmar que no se bloquean ingesta, control ni publicación.

## Patrones de reducción

```text
SYSTEM-ARCH|ARCHITECTURE|bridge|READY|SSE|client|event|dropped|queue|dron_1|dron_2|pipeline_flow|ERROR|FATAL|Segmentation fault|Killed|SIM-DONE
```

Añadir markers reales del bridge implementado.

## Criterio de éxito

`CONSEGUIDA` solo si:

1. existe `system_architecture` separado de `pipeline_flow`;
2. los tres grupos y todos los paquetes están representados;
3. cada paquete aparece dentro de su contenedor correcto;
4. aristas y tooltips se basan en interfaces reales;
5. la vista estática funciona sin ROS 2;
6. el modo en vivo ilumina actividad real;
7. telemetría, web y navegador tienen flags independientes;
8. todos los flags quedan `false` por defecto;
9. cerrar/saturar la herramienta no afecta al sistema;
10. la prueba de dos drones sigue terminando correctamente;
11. la documentación coincide con el grafo.

## Criterio de parcial, fallo o bloqueo

`PARCIAL` si la vista estática está completa pero falta actividad real de alguna
interfaz claramente identificada.

`NO CONSEGUIDA` si el diagrama mezcla herramientas, inventa conexiones, depende
de Internet, bloquea ROS 2 o necesita estar activo para que el sistema funcione.

`BLOQUEADA` solo si un paquete externo no expone información suficiente para
representar su interfaz; debe aparecer entonces como nodo de contrato pendiente,
no inventarse detalles.

## Cambios prohibidos

- No usar el diagrama para enviar comandos.
- No transportar payloads pesados.
- No activar debug por defecto.
- No sustituir RViz2.
- No borrar `pipeline_flow`.
- No inferir éxito funcional por una animación.
- No crear conexiones decorativas sin productor/consumidor real.
- No hacer que el launch espere a que el navegador confirme recepción.

## Documentación de cierre

Crear/actualizar al ejecutar:

```text
codex/contexto/paquetes/simulacion_dron/system_architecture_visualizer.md
codex/contexto/paquetes/simulacion_dron/00_summary.md
codex/contexto/paquetes/simulacion_dron/launches.md
codex/contexto/03_ARQUITECTURA_ACTUAL.md
codex/contexto/04_TOPICS_SERVICES_ACTIONS.md
codex/pipeline/fase_2_separacion_paquetes/historial/por_subfase/historial_2F.md
codex/pipeline/fase_2_separacion_paquetes/historial/por_subfase/historial_2F_RESUMEN.md
```
