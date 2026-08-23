# Resultado final de Fase 3

## Estado

```text
Fase 3: CONSEGUIDA
Subfase 3T: CONSEGUIDA
Subfase 3Q: CONSEGUIDA; mejora futura documentada
Fase actual: Fase 2 - separacion servidor/dron/simulacion
```

La Fase 3 entrega un mapa sparse global multi-dron utilizable y una unica ruta
runtime activa. El fallo visual de 194 se conserva como aprendizaje; 195 no lo
reproduce y el usuario confirma RViz2 perfecto.

## Arquitectura entregada

```text
OrbMap -> PrimaryQueue -> PrimaryWorker -> SparseGlobalBackend -> ROS
fiducial MAX / database MEDIA / loop BAJA -> SecondaryWorker -> commit dirty
eventos no bloqueantes -> RViz2 y visualizador web
```

- `RawMapDatabase` conserva la entrada ORB-SLAM3 sin correcciones globales;
- `GlobalPoseStore` es la autoridad de anchors y poses globales;
- un unico worker principal compromete entrada y publica;
- un unico worker secundario serializa fiduciales, covisibilidad, loops,
  fusion y optimizacion;
- los commits geometricos son revisionados y atomicos;
- los landmarks fusionados conservan identidad transitiva;
- el score raw combina ORB, distancia, aislamiento e inliers;
- el score fused es `clamp(media(raw miembros) + 0.04*N, 0, 1)`;
- la nube completa publica `score` y `rgb` rojo-amarillo-verde;
- RViz2 y el grafo web observan el runtime sin gobernarlo.

## Configuracion y despliegue

La politica queda fijada por
`ADR_0009_configuracion_por_dominio_y_despliegue.md`. Los perfiles globales se
separan en:

```text
config/global_map/runtime.yaml
config/global_map/fiducials.yaml
config/global_map/optimization.yaml
config/global_map/loop_fusion.yaml
config/global_map/scoring.yaml
config/global_map/replay_debug.yaml
```

`orbslam3_server` contiene la copia standalone/real y `simulacion_dron` la
copia usada por Gazebo. Actualmente son iguales y un test comprueba valores,
claves, ownership unico, cobertura de parametros declarados y que el perfil de
debug sea opt-in. Los launches seleccionan la copia de su despliegue; los
defaults C++ quedan como fallback tipado.

La separacion fisica final en dominios dron, servidor y simulacion pertenece a
Fase 2. `body_T_camera` se identifica ya como calibracion intrinseca del dron,
aunque el servidor la consuma temporalmente.

## Interfaces activas

Entradas principales:

```text
/dron_X/orbslam/orb_map_delta
/dron_X/sensor/GT/pose        solo fiducial simulado live
/dron_X/orbslam/get_full_map  cliente de recuperacion
```

Salidas principales:

```text
/global_sparse_cloud
/global_keyframes
/global_mapping/backpressure_active
/global_mapping/flow_events
```

Launches de entrada:

```text
ros2 launch orbslam3_server global_orb_map_server.launch.py
ros2 launch simulacion_dron multi_dron.launch.py
```

Los launches replay `f3c`-`f3f` se conservan para regresion y cargan
`replay_debug.yaml` explicitamente.

`simulacion_dron/config/fase3_debug.yaml` controla RViz2, grafo web, navegador
y telemetria de terminal. Sus cuatro defaults quedan en `false`; la prueba 196
confirma que el mapa funciona sin iniciar esas herramientas ni emitir `[F3*]`.

## Validacion final 3T

- checkpoint previo: `1b96a7a`;
- build: 3/3 paquetes correctos;
- CTest: `orbslam3_multi` 9/9, `orbslam3_server` 10/10 y
  `simulacion_dron` 8/8;
- contrato estatico YAML/web: 15/15;
- manifests XML, formato y resolución de launches instalados correctos;
- prueba Gazebo 195: `success=true`, escenario y herramienta con exit 0;
- cierre 195: principal 741 y secundario 1262, ambos `pending=0`,
  `max_active=1`, `hard_failed=0`;
- 11 optimizaciones loop aceptadas y comprometidas;
- publicacion final: 23.978 puntos, 453 keyframes y campos de identidad,
  `score` y `rgb` presentes;
- score estable final observado: 59.599 tracked, 25.781 anchored, 431
  isolated, 217 near, 13.696 far y media 0.2763;
- recursos: servidor 249.5 MiB RSS maximo, grupo 1571.4 MiB, PSI full 0.18,
  guarda inactiva y minimo disponible 4406.2 MiB.

El usuario confirma que RViz2 se vio perfecto en 195. La prueba silenciosa 196
termina `success=true`, mantiene el servidor operativo, no crea procesos
RViz2/web y produce cero marcadores `[F3*]`; sus recursos son estables. El exit
255 de Gazebo aparece durante cleanup posterior a `SIM-DONE` y no afecta el
resultado.

## Limpieza y absorcion

Se retiraron de la version actual:

- `legacy/` y `legacy2/` de los paquetes de Fase 3;
- el paquete independiente obsoleto `ORB_SLAM3_MULTI/`, retirado en una
  correccion posterior de la limpieza ejecutada originalmente como 3X;
- snapshots y documentacion duplicada asociada;
- contratos legacy `12R-*`, `13`, `14` y `15`;
- contratos transversales absorbidos, cuyas capacidades ya estaban implantadas
  y aceptadas.

Los historiales, pruebas y conclusiones se conservan. El checkpoint permite
recuperar cualquier fuente retirada sin reescribir Git.

La auditoria no encontro una segunda autoridad, ruta de scheduling ni ruta de
publicacion activa. La duplicacion YAML servidor/simulacion es deliberada y
esta protegida por tests.

## Aprendizaje 3Q

La prueba 194 mostro una mala optimizacion del dron antihorario cerca del
fiducial 2. Dos loops asimetricos y ambiguos se aceptaron antes del segundo
fiducial hard y movieron 359/362 KFs; el fiducial posterior quedo dentro de
umbral y no corrigio el interior. 3T no modifica ese algoritmo.

Si reaparecen dobles paredes, loops incorrectos, ventanas excesivas o una
deformacion equivalente, la entrada correcta es 3Q y su historial. Queda
documentada, sin implementar, una politica de 2 apoyos independientes para
candidatos cercanos y hasta 8-10 para candidatos lejanos/ambiguos antes de una
unica optimizacion.

## Handoff a Fase 2

Fase 2 puede partir de una implementacion limpia y reproducible para separar
fisicamente drones, servidor y simulacion. Debe conservar:

- los invariantes de raw, poses, identidad y commits;
- el ADR 0009 y la procedencia de cada YAML;
- la independencia de los dos despliegues;
- los tests de sincronizacion mientras las copias deban coincidir;
- el aprendizaje y la mejora futura de 3Q, sin convertirlos en bloqueo de Fase 2.
