# Historial 3O - resumen

## Estado vigente

`CONSEGUIDA`: las correcciones post-loop estan implementadas y validadas
tecnicamente. El usuario confirmo tambien que RViz2 y el grafo web de la prueba
tipica 156 se veian correctamente. 3P y 3Q haran la validacion integral de las
evidencias de fusion y optimizacion que 3O deja preparadas.

## Estado actual

- Hasta tres regiones producen subnubes acotadas, matching ORB y RANSAC 3D-3D.
- Fusion compatible domina y se conserva para 3P; error alto anclado se reporta
  para 3Q. 3O no fusiona ni optimiza por loop.
- Dos queries independientes compatibles pueden anclar el lado no anclado;
  componentes relativos se resuelven en un batch atomico.
- El anchor loop es blando. Si se mueve el KF de apoyo del padre, todo el hijo
  se propaga rigidamente en el mismo commit.
- El primer fiducial directo del hijo reancla todo su submapa como hard, corta
  la dependencia loop y no usa ese KF blando como control fiducial.
- `LoopTask` se coalesce por huella semantica; una `validation_revision` exacta
  distinta protege dequeue/commit frente a geometria obsoleta.
- `same_submap_diagnostic` y `waiting_independent_support` tienen telemetria
  separada.

## Evidencia vigente

- build final correcto; CTests `orbslam3_multi` 8/8 y servidor 4/4;
- prueba 157: B/KF4 se ancla contra A/KF72, posterior al hard A/KF27; el commit
  fiducial mueve 213 KFs, de los cuales 78 son `control_propagated`;
- prueba 156: `(2,0)` se ancla por loop y su primer fiducial reancla 32 KFs como
  hard; tres optimizaciones fiduciales terminan `ACCEPT_FULL`;
- cierre 156: 497 principales, 1060 secundarias, 89 stale, `pending=0`,
  `hard_failed=0`, `max_active=1`, anchors=4, poses=486, active=439, hard=7;
- carga: live 154 2301/248 = 9.28 tareas/KF; prueba 157 2475/269 = 9.20;
  version intermedia 158 2477/300 = 8.26; version final 156 1060/486 = 2.18;
- recursos 156: PSS servidor 204.8 MiB, MemAvailable minimo 6134.8 MiB,
  memory PSI full 0 y guard inactivo.

## Evidencia negativa o limites

- La prueba 155 intento 1 anclo B contra A/KF11, anterior al primer fiducial,
  por lo que ese apoyo no pertenecia a la ventana y no podia probar
  propagacion. Se conserva como intento parcial, no como fallo del algoritmo.
- ORB creo siete submapas en 156 por perdidas de tracking; cuatro quedaron
  anclados y tres diferidos al cierre. La vista exacta queda pendiente del
  usuario.
- Los 769 loops de 156 incluyen 305 evidencias de optimizacion futura, 39
  candidatos de fusion, 19 constraints no ancladas, 78 diagnosticos
  same-submapa, 24 esperas de segunda evidencia, 207 rechazos y 82 stale.
- 3O no aplica fusion, optimizacion por loop ni aristas server; pertenecen a
  3P/3Q.
- El `exit 255` de Gazebo aparece despues de `SIM-DONE` durante cleanup y no es
  causa de fallo.

## No repetir

- No volver a usar el tramo suroeste del primer YAML 155 para probar
  propagacion: puede seleccionar un KF de apoyo anterior al primer hard.
- No incluir posiciones/descriptores exactos en la igualdad de scheduling;
  deben permanecer solo en `validation_revision`.
- No interpretar `processed` secundario como numero exclusivo de loops: incluye
  tambien `DatabaseUpdateTask` y fiduciales.

Detalle cronologico: `historial_3O.md`.
