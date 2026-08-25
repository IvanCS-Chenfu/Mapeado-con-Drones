# Visualizador `system_architecture`

## Estado

Implementado y validado en Fase 2.

## Componentes

```text
simulacion/simulacion_dron/src/visualizer/system_architecture_bridge.py
simulacion/simulacion_dron/src/visualizer/system_architecture_browser.py
simulacion/simulacion_dron/web/system_architecture/graph_definition.js
simulacion/simulacion_dron/web/system_architecture/graph_layout.js
simulacion/simulacion_dron/web/system_architecture/graph_metadata.js
simulacion/simulacion_dron/web/system_architecture/app.js
```

La topologia representa paquetes dentro de Dron, Servidor y Simulacion. Separa
capas `runtime`, `build`, `config` y `deployment`; solo runtime puede mostrar
actividad. `graph_metadata.js` conserva rutas, ejecutables, YAML propietarios,
dependencias y metadata ROS de interfaces fuera de la UI.

`graph_layout.js` conserva las posiciones de presentación separadas de la
topología. Simulación y Servidor ocupan la franja superior; Dron ocupa la franja
inferior con sus librerías debajo de los consumidores. `app.js` aplica esas
posiciones antes de crear el layout `preset`, y el contrato comprueba relaciones
espaciales y cobertura de paquetes sin inmovilizar coordenadas concretas.

## Telemetria

La evidencia directa live llega exclusivamente por
`/system_architecture/activity` como eventos JSON ligeros con `edge_id`,
`source`, `drone_id`, `interface`, `interface_kind` y timestamp. Productores o
consumidores reales emiten los eventos muestreados; el bridge no se suscribe a
imagenes, nubes ni mapas pesados y descarta eventos desconocidos.

Fase 4D incorpora `fiducial_config_server_to_wrapper`, una arista de servicio
Servidor->Dron que se activa cuando el wrapper solicita su configuracion.
4E+4F añaden `wrapper_to_server_fiducial_observations` para el topic reliable
de batches; wrapper y servidor emiten actividad directa con metadata ligera de
dron, epoch, KF y numero de tags.

```text
debug_system_architecture_web=false
  -> sin bridge, HTTP, SSE, navegador ni productores
web=true + telemetry=false
  -> grafo estatico
web=true + telemetry=true
  -> grafo estatico y actividad live
```

El bridge conserva una unica referencia de suscripcion propia, sin sobrescribir
las colecciones internas de `rclpy`; asi el shutdown no repite la destruccion
observada en la prueba 198.

La validacion aislada confirmo modo estatico, assets HTTP y un evento ROS real
en modo live. La prueba 200 confirmo ambos bridges, navegadores, RViz2 y cierre
limpio de `system_architecture_bridge` durante la vuelta oficial.

La prueba 211 arranco `system_architecture` en `mode=live` junto a
`pipeline_flow` y produjo 18 batches/18 matches reales en la arista nueva.

Tras 4H se elimina `sim_to_server_fiducial_gt` de definicion, metadata, bridge
y contrato. Permanecen `wrapper_to_server_fiducial_observations` y
`fiducial_config_server_to_wrapper`; `sim_to_dron_gt` sigue siendo la ruta de
control/Fase 5. El smoke 217 valida ambos bridges y navegadores en live.

El cierre visual recompiló `simulacion_dron`, pasó CTest 9/9 y la guarda 15/15.
Las capturas 1440x900 y 820x1000 verificaron contenedores completos, texto
legible y ausencia de solapes incoherentes.

## Referencias

```text
rg -n "RUNTIME_EDGES|_on_activity" \
  simulacion/simulacion_dron/src/visualizer/system_architecture_bridge.py
rg -n "SYSTEM_ARCHITECTURE_METADATA" \
  simulacion/simulacion_dron/web/system_architecture/
rg -n "SYSTEM_ARCHITECTURE_LAYOUT|layout.positions" \
  simulacion/simulacion_dron/web/system_architecture/
```
