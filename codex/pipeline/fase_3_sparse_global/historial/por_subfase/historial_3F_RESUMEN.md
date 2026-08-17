# Historial 3F - resumen

Leer este archivo antes de `historial_3F.md` cuando haya que hablar o trabajar
sobre `3F`.

## Estado vigente

`CONSEGUIDA`. La paleta por epoch esta corregida, probada automaticamente y
confirmada visualmente por el usuario en live 151 con tres epochs por dron.

## Implementacion activa

- `LandmarkScoreManager` autoritativo con formula ORB e `is_bad => 0`;
- `GlobalMapBuilder` stateful con caches, slots, indices inversos y dirty IDs;
- asociacion KF estable y geometria por pose local/world del KF;
- MPs sin observador world valido se omiten; no existe fallback de submapa;
- cloud y frustums coherentes se publican juntos al final de `PrimaryTask`;
- `PointCloud2` incluye score, RGB e identidad; KFs usan una base por dron y un
  salto de hue de 137.507764 grados por epoch;
- grafo web 11 nodos/15 aristas con score, builder y RViz2.

## Evidencia vigente

- build final de tres paquetes exit 0;
- tests finales: 26/26 C++ y 8/8 web;
- replay 94: `NO CONSEGUIDA`, preservado; healthcheck Gazebo no aplicable y
  entorno Snap incompatible para RViz2;
- correcciones mecanicas: `--without-gazebo` y entorno RViz saneado;
- replay 95: 150 entradas, 2 anchors, 61 KFs, 5812 MPs, 8450 scores,
  49 skips pre-anchor, 101 publicaciones y cero errores;
- live 96: ambos drones completan fiducial 2 y x=-8; first anchors para los dos
  submapas, 28 skips pre-anchor y 156 publicaciones hasta 126 KFs/13191 MPs;
- frame/campos correctos y `fallback_submap=0` en replay/live.
- live 97: ejecucion operativa correcta y no analizada por peticion expresa;
  el usuario detecta el problema visual de enrutado que motiva el redisenado.
- live 145: 7 submapas. `(1,0)/(1,3)` usan tonos 205/206 y
  `(2,0)/(2,2)` 50/48; no cumplen el cambio de color perceptible esperado.
- build final exit 0 y `test_submap_color` 2/2: distancia RGB minima entre
  epochs consecutivos y entre drones;
- live 151: 6 submapas, epochs 0/1/2 de ambos drones; el usuario confirma que
  todo se vio perfecto.

## Cierre visual

- la repeticion 97 permitio revisar RViz2 y web con mas calma;
- el primer layout no supero el criterio web y no se oculta ese intento;
- la vista aislada final usa la composicion aceptada, conexiones legibles y
  retorno builder->server suave;
- el usuario ordena cerrar 3F como concluida.
- revision posterior: se reabre solo el criterio 14 de color; no invalida
  score, builder, nube, reproyeccion ni el layout aceptado en 97.
- correccion vigente: el criterio esta cubierto automaticamente, pero el cierre
  visual sigue abierto hasta la observacion de live 151.

## No repetir

- scans globales por delta, timer/publication worker o snapshots bajo mutex;
- borrar un replay fallido al repetirlo correctamente;
- ejecutar un replay sin Gazebo usando el healthcheck live;
- heredar rutas Snap en procesos RViz2;
- fallback `world_T_local * p_local_mp` sin KF world;
- recuperar el enrutado `taxi` ortogonal para builder->server.

## Detalle

`historial_3F.md` y `../../subfases/subfase_3F.md`.
