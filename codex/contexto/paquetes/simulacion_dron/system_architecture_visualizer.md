# system_architecture — Arquitectura declarativa y actividad live

## Estado

Herramienta existente desde Fase 2. Este documento fija la semántica objetivo acordada
para su corrección antes del cierre de la fase.

## Responsabilidad

Mostrar la arquitectura real vigente del proyecto a nivel de paquetes y grupos. No
sustituye `pipeline_flow` ni la GUI operativa futura.

## Nodos

Paquetes como nodos principales, agrupados en Dron, Servidor y Simulación. Tooltips o
paneles contienen ruta, nombre ROS, responsabilidad, ejecutables/librerías, YAML,
dependencias, estado y documentación.

## Aristas

Clasificación:
- `runtime`;
- `build/API`;
- `config/replica`;
- `deployment`.

Solo runtime puede iluminarse.

## Actividad

Una arista se activa solo con evidencia directa o un evento semántico explícito. No
inferir por similitud de nombre ni mapear eventos desconocidos. La telemetría debe ser
ligera y nunca transportar imágenes, PointCloud2, OrbMap completo ni payloads pesados.

## Topología actual que debe corregirse

- cámaras Simulación → `orbslam3`;
- GT Simulación → `dron_individual`, marcado provisional hasta Fase 5;
- GT → Servidor, marcado provisional para fiducial simulado hasta Fase 4;
- OrbMap delta wrapper → Servidor;
- GetOrbMap con dirección request/response explícita;
- comandos de motor Dron → Simulación;
- AccionTrayectoria Simulación/runner → Dron;
- backpressure/cloud/keyframes/observabilidad según productores reales;
- no representar ORB→control como flujo funcional si el control actual usa GT.

## Debug

```text
debug_system_architecture_web=false
  -> ninguna actividad específica

web=true + debug_architecture_telemetry=false
  -> servidor web y grafo estático, sin subscriptions runtime

web=true + telemetry=true
  -> modo live

debug_open_system_architecture_browser
  -> únicamente apertura automática
```

No depende de `pipeline_flow` ni de `/global_mapping/flow_events` como bus universal.

## Evolución

4E/4F/4H/4K y 5A/5B/5D/5E/5H/5I tienen obligaciones explícitas de actualizarlo. La
regla global del Pipeline Maestro cubre cualquier cambio posterior en Fases 6–9.
