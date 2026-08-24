# 00_summary - simulacion_dron

Paquete de launch, escenarios y observabilidad Gazebo. Integra servidor y, de
forma configurable, RViz2, `pipeline_flow` y `system_architecture`; tambien
ofrece replay sin Gazebo ni GT live.

El grafo `system_architecture` usa una topología, metadata y layout declarativos
separados. Su composición sitúa Simulación/Servidor arriba y Dron abajo para
facilitar la lectura de interfaces entre despliegues.

Desde 3T contiene en `config/global_map/` el perfil de parámetros controlables
por el despliegue simulado. Es una copia exacta del perfil del servidor durante
la etapa de simulación y un test contractual impide divergencias o parámetros
sin propietario.

Fase 2 separa configuracion propia de modelo/sensores en `physical_dron.yaml`
y `simulated_sensors.yaml`. `actuators_dron.yaml` es una replica parcial
declarada de Dron. Simulacion no abre YAML operacionales de otro grupo.

## Launches

```text
launch/multi_dron.launch.py -> Gazebo + N drones + servidor + debug opcional
launch/f3c_replay.launch.py -> replay raw 3C
launch/f3d_replay.launch.py -> replay 3D con anchor sintetico
launch/f3e_replay.launch.py -> replay raw + observaciones fiduciales
launch/f3f_replay.launch.py -> replay 3F + RViz2 + web + apertura de pestaña
launch/pipeline_flow_only.launch.py -> diagnostico web aislado
```

`multi_dron.launch.py` dispone de perfiles sin duplicar launches:

- `launch_gazebo_gui=false`: usa `gzserver` sin `gzclient`;
- `launch_mission_gui=false`: omite la GUI de mision;
- `fase3_debug.yaml`: RViz2, grafo, navegador y logs `[F3*]` independientes;
- `drone_start_stagger_sec=8.0`: arranque 0/8/16... s por defecto;
- `orb_vocabulary_path`: `ORBvoc.txt` completo por defecto; L5 solo por override.

El perfil visual completo se usa con dos drones. Para tres o mas drones y para
fases dense se usa headless y se habilitan solo las vistas necesarias.

## Observabilidad 3P

- RViz2 muestra `/global_sparse_cloud` con `RGB8` y
  `/global_keyframes` como frustums.
- El grafo web tiene 23 nodos y 39 aristas. Ademas del flujo fiducial incluye
  `CovisibilityDatabase`, `LoopDetector`, `LoopBoWIndex`,
  `SubcloudLoopVerifier`, `LoopDecision`, `LoopAnchorConstraintStore` y
  `FusedLandmarkManager` con salidas a covisibilidad, score y builder.
- La arista `SecondaryWorker --retry / LOW--> SecondaryTaskQueue` representa
  solo el nuevo intento real posterior a stale/rollback.
- El flujo secundario conserva iluminacion progresiva por `task_id` desde
  lifecycle `start` hasta `done`; las etapas ya no son pulsos independientes.
- En desktop, principal, poses/anchors y loop/fusion ocupan tres bandas con
  columnas ampliamente separadas; las rutas curvas evitan solapes en retornos
  y diagonales largas. El layout movil vertical permanece independiente.
- `pipeline_flow_browser.py` espera `/health=ready` y abre una sola pestaña
  desde el propio launch; no necesita un comando manual de Codex.
- Los launches limpian variables Snap/VS Code para RViz2 y evitan cargar
  bibliotecas GTK incompatibles.

## Validacion

- contrato web 9/9;
- live 98: intento funcional con bridge 11/18, conservado como no conseguido
  por bloqueos y swap agotada antes de las optimizaciones;
- replay 99: ejecución aislada sin Gazebo sobre 54 deltas;
- live visual 133: escenario completo, dos anchors y minimo disponible 612.3
  MiB sin PSI de memoria;
- prueba 137: tres drones en movimiento, seis goals, tres anchors, 141 KFs
  activos y minimo 878.8 MiB;
- prueba 138: estado normal de dos drones restaurado con Gazebo GUI, RViz2 y
  web, minimo 946.6 MiB y guarda inactiva.
- live 148: intento fallido preservado; hard constraint por carrera de control
  dejo mission gate activo hasta timeout;
- replay 150: reproduce las 1239 entradas y confirma la correccion sin hard;
- live 151: escenario fid2-fid1-fid2 completo y confirmado visualmente;
- replay 153: backlog 3M-3O drenado por completo;
- live 154: escenario secuencial A fiducial/B loop completo, 2 anchors con solo
  1 hard y guard inactivo. RViz2 y web arrancaron; su lectura visual humana
  queda pendiente del usuario.
- prueba 160: escenario tipico completo, bridge 3P listo, servidor y cola
  secundaria cierran limpios y guard de recursos inactiva; el usuario confirma
  RViz2 y grafo web correctos.
- prueba 161: mismo escenario completo, guard inactivo, contrato web 9/9 y
  cierre de servidor/colas limpio. La revision visual humana de esta ejecucion
  aun no se ha comunicado.
- cierre 3T: CTest 8/8; prueba 195 completa el escenario tipico con exit 0,
  guarda de recursos inactiva y perfiles YAML de simulacion cargados; el
  usuario confirma el resultado visual correcto.
- cierre 3S: prueba 196 `success=true`, cuatro goals correctos y servidor
  operativo; con los cuatro flags false no arrancan RViz2, bridge ni navegador
  y no aparece telemetria `[F3*]`.

La validacion automatica de topologia, lifecycle y configuracion 3T esta
conseguida.

Detalle: `launches.md`, `scenario_runner_node.md` y
`pipeline_flow_visualizer.md`.
