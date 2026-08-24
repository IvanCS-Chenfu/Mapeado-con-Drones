# ADR 0010 - Observabilidad web opcional y coste especifico nulo al desactivarla

## Estado

Aceptada y aplicada durante Fase 2 (2026-08-24).

## Contexto

El proyecto dispone de dos visualizadores web diagnosticos independientes:

- `pipeline_flow`: flujo interno del pipeline sparse/global;
- `system_architecture`: grupos, paquetes, interfaces y despliegue completo.

Ninguno forma parte del camino funcional de control, SLAM, mapa o mision.

## Decision

Cuando el debug maestro de una herramienta es `false`, no arranca su bridge,
HTTP/SSE, navegador, publisher, subscription ni serializacion de eventos. Se
admite solo la comprobacion trivial de un booleano.

`debug_pipeline_flow_web=false` desactiva tambien la generacion de
`/global_mapping/flow_events` antes de construir strings. Para
`system_architecture` se aplica:

```text
web=false                         -> herramienta dormida
web=true, telemetry=false         -> grafo estatico
web=true, telemetry=true          -> grafo estatico y actividad live
open_browser                      -> solo apertura automatica
```

Los nodos principales del grafo son paquetes. Las aristas se clasifican como
`runtime`, `build`, `config` o `deployment`; solo `runtime` se ilumina. La
actividad requiere evidencia directa mediante un evento conocido en
`/system_architecture/activity`. Los eventos solo contienen metadata ligera:
`edge_id`, `drone_id`, timestamp, interfaz, contador y estado.

## Consecuencias

- ambos visualizadores permanecen independientes;
- debug apagado tiene coste especifico practicamente nulo;
- una conexion declarada no se confunde con trafico observado;
- cerrar o saturar la UI no gobierna el pipeline;
- las fases futuras deben actualizar topologia, metadata, productores y tests
  cuando cambien la arquitectura.

## Validacion

La prueba 199 demostro ausencia de procesos y telemetria con defaults `false`.
Las pruebas aisladas validaron `pipeline_flow`, el modo estatico y el modo live
de `system_architecture`. La prueba 200 valido ambos visualizadores durante la
vuelta oficial de dos drones.
