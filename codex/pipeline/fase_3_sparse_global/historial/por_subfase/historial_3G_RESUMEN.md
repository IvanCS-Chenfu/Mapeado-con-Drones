# Historial 3G - resumen

Leer este archivo antes de `historial_3G.md` cuando haya que hablar o trabajar
sobre `3G`.

## Estado vigente

`CONSEGUIDA`. La prueba live 98 se conserva como intento `NO CONSEGUIDO` por
los bloqueos y la swap agotada, pero no es la ultima evidencia: las
optimizaciones se aplicaron y midieron una a una hasta completar live 133 con
dos drones y todas las vistas, carga real 137 con tres drones en movimiento y
restauracion visual 138. Build final, 37/37 tests C++ y 8/8 web son correctos.

## Acuerdo de reimplementacion

- `source=live|replay` y `kind=delta|full_snapshot` son dimensiones distintas;
- snapshot no-op termina tras raw y no se graba;
- snapshot material activa solo pose, score, asociaciones o geometria que hayan
  cambiado;
- el builder recibe IDs dirty, pero no se ejecuta dentro de `SnapshotInput`;
- ausentes activos se invalidan sin borrarse;
- el `.record` no contiene snapshots: guarda su efecto material como delta
  normalizado con el mismo `arrival_id`;
- scheduler 35/35 s, una solicitud en vuelo por dron y sin ticks acumulados bajo
  backpressure;
- prueba live con drop one-shot de un delta y replay delta-only del resultado;
- grafo conserva el layout 3F y añade tres aristas snapshot, sin request visual.

## Que se hizo

- `RawInsertResult` separa pose/asociacion/covisibilidad de KFs y
  geometria/score/asociacion/invalidation de MPs.
- `InsertFullSnapshot` invalida ausentes sin borrar y crea delta normalizado
  solo si el diff es material.
- `GlobalPoseStore` mantiene `raw_world_pose`, `correction_pose` y preserva la
  `world_pose` aceptada ante cambios raw.
- `SparseGlobalBackend` acumula dirty de snapshot sin ejecutar builder.
- El servidor implementa scheduler 35/35, una request en vuelo, backpressure,
  drop one-shot de prueba y replay delta-only.
- El grafo conserva 11 nodos y añade tres aristas hasta 18.
- La respuesta `GetOrbMap` se conserva mediante un `shared_ptr` aliasing: el
  servidor ya no copia el `OrbMap` completo al recibir cada snapshot.
- El delta normalizado parte de un shell vacio con metadatos exactos; no copia
  los vectores completos y los deltas normales no crean ese temporal.
- El record escribe incrementalmente y replay alimenta una ventana acotada;
  no se retiene un journal ni el record completo en RAM.
- Score y builder extraen lotes ligeros; el builder mantiene cache de
  proyecciones por KF e invalida solo lo dirty.
- El launch permite Gazebo/GUI de mision opcionales, arranque escalonado y
  vocabulario seleccionable. Multi-dron usa L5 compacto y stagger 8 s; el
  launch individual conserva el vocabulario completo por defecto.
- Las camaras multi-dron trabajan a 480x360, 20 Hz y 900 features, con
  calibracion consistente desde YAML hasta Xacro/wrapper.
- El monitor mide PSS y desglosa heap anonimo/fichero/compartido de ORB.

## Evidencia vigente

- build integrado de `orbslam3_multi`, `orbslam3_server` y `simulacion_dron`:
  exit 0; 29/29 tests C++ y 8/8 web.
- live 98: escenario `success=true`; delta omitido del dron 1 recuperado;
  todos los snapshots marcan `builder_executed=false publish=false`; dirty
  consumido por deltas posteriores; dos anchors y 103 poses.
- record live: 54 entradas, 2 submapas, 103 KFs, 10938 MPs y 12 observaciones;
  cada snapshot material genero un delta normalizado.
- replay 99: `success=true`; 54/54 entradas `kind=delta`, cero snapshots,
  `max_active=1`, mismos agregados raw/pose y vista final 6264 puntos/94 KFs.
- el usuario reporto bloqueos durante live; tras la prueba los 2 GiB de swap
  estaban ocupados, y varias tareas duraron 4-5 s;
- se localizaron copias completas redundantes del mensaje snapshot y dos
  requests simultaneas que pueden elevar el pico de memoria;
- las pruebas protegidas 100-103 validan las guardas, el scheduler global y
  `zero_copy_rx=true`; prueba 103 avanzo 102 s, pero termino con solo 485.8 MiB
  disponibles, servidor 191.7 MiB, ORB 1197.6 MiB y `code` externo 2098.8 MiB;
- prueba 104, tras normalizacion selectiva, termino protegida a 90 s con
  servidor 171.7 MiB y grupo 1654.2 MiB; el journal ya tenia 146 entradas y el
  record ocupaba 100 MiB, confirmando retencion lineal como cuello principal;
- prueba 105, tras comparar KF/MP por referencia, avanzo 100 s sin regresion de
  latencia normalizada, pero 162 entradas/130 MiB elevaron el servidor a
  198.1 MiB; el journal sigue dominando el crecimiento;
- prueba 106 elimina los arboles de IDs de los deltas: coste raw estacionario
  bajo de 8.113 a 6.462 ms/1000 MPs (muestras ORB no identicas); el smoke fue
  protegido a 62 s por menor margen inicial del sistema;
- prueba 107 conserva diff de asociaciones determinista usando dos indices
  hash; coste 7.154 ms/1000 MPs, sin regresion. Un intento Gazebo fallo temprano
  y el reintento valido termino protegido a 72 s;
- prueba 108 valida record incremental: 38/38 commits con journal residente
  cero, record 66.1 MB y servidor maximo 72.1 MiB; la guarda salto a 47 s por
  margen global previo del host, no por crecimiento del journal;
- prueba 109 reproduce 36/36 entradas del nuevo v2 y reconstruye 2 submapas,
  87 KFs, 9815 MPs, 2 anchors/87 poses y vista 6395/87; exit 0, PSI memoria 0;
- prueba 110 valida que cambios internos de score sin efecto visible no
  ensucian el builder: 9 de 243 inputs se filtraron. La guarda salto a 11 s
  por solo 480.3 MiB disponibles y swap ya llena, por lo que no constituye una
  medida sostenida de rendimiento;
- el diferimiento pre-anchor pasa 19/19 tests, incluido backfill de un MP no
  listado por KF. La prueba 111 fue protegida a 10 s antes del primer delta:
  servidor 35.8 MiB, ORB 964.9 MiB, minimo disponible 366.6 MiB y `code`
  externo 2280.7 MiB; confirma cierre seguro, no ahorro runtime;
- replay 112 procesa 54/54 y demuestra cero recalculo pre-anchor, backfill
  unico 32/3057 y 43/3176, y mismo estado final 103/10938 con vista 6264/94.
  Servidor 123.5 MiB y PSI 0; la cola aun alcanzo 23 entradas porque el replay
  carga y alimenta por adelantado;
- replay streaming compila y pasa 25/25 tests. Gazebo 113 quedo protegido a
  9 s antes del primer delta, con servidor 35.6 MiB pero solo 323.4 MiB libres
  y `code` externo 2286 MiB; valida cierre seguro, no el lector replay;
- replay 114 confirma el cambio con el mismo record que 112: cola maxima 8 en
  lugar de 23, mismo estado final y RSS servidor 82.0 en lugar de 123.5 MiB
  (-41.5 MiB, -33.6%), con PSI memoria 0;
- benchmark 2/4/8 con 10k/20k/40k MPs: RSS 22.1/33.7/56.4 MiB y coste de
  delta completo 353.7/341.5/351.9 ms por dron; escalado backend casi lineal.
  Gazebo 115 volvio a protegerse antes del primer delta: servidor 35.4 MiB,
  ORB 1003.7 MiB y solo 378.9 MiB disponibles;
- batch score pasa 22 tests y escala 3/3; a 8 drones insert+score consume solo
  66.4 ms frente a 1920.6 ms del builder. Gazebo 116 se protege a 10 s, antes
  de deltas, con servidor 35.7 MiB y ORB 974.8 MiB;
- `RawBuilderSnapshot` pasa 22 tests y reduce la mediana builder de 8 drones a
  1550.8 ms frente a 1920.6 ms (-19.3%); cinco repeticiones preservan conteos.
  Gazebo 117 se protege antes de deltas con servidor 35.9 MiB y ORB 1014.8 MiB;
- cache de proyeccion reduce la mediana builder 8 drones de 1550.8 a 594.5 ms
  (-61.7%) y el coste total a 86.5 ms por delta-dron (-75.4% frente al inicio).
  Replay 119 mantiene estado final y baja 5.91->2.22 s, con +1.4 MiB de RSS;
- pruebas 120/121 demostraron que headless por si solo no resolvia el problema;
  PSS 122/126 localizo aproximadamente 941 MiB de heap anonimo en dos ORB;
- 20 Hz redujo CPU de sistema un 22.1%; `MALLOC_ARENA_MAX=2` solo ahorro 0.5%;
- el vocabulario L5 redujo PSS ORB 969.6->212.1 MiB (-78.1%); la ruta L6 sigue
  disponible para validar relocalizacion/loops de maxima fidelidad;
- 480x360/900 features redujo otro 9.9% de PSS ORB y alrededor de 22% de CPU
  de grupo frente al perfil compacto 640x480;
- live visual 133 completo: 176 s, dos anchors, 90 KFs/9787 MPs, 174/174
  entradas, guarda inactiva, minimo disponible 612.3 MiB, PSI memoria 0 y PSS
  grupo 640.7 MiB;
- smoke 135 con tres drones headless: tres modelos/frontends/clientes, dron 3
  publicando deltas, guarda inactiva, minimo disponible 1124.0 MiB y PSS grupo
  837.4 MiB;
- restauracion visual 136 con dos drones: Gazebo GUI, RViz2 y web activos,
  minimo disponible 975.6 MiB, PSS grupo 541.1 MiB y PSI memoria 0;
- carga real 137 con tres drones: seis goals, tres anchors, 141 KFs activos,
  7981 puntos, 254 tareas con `max_active=1`, minimo disponible 878.8 MiB, PSI
  full 0.18 y guarda inactiva;
- restauracion definitiva 138: launch visual ordinario, dos modelos validos,
  minimo disponible 946.6 MiB, PSS grupo 539.1 MiB y PSI memoria 0;
- regresion 139: seis goals, dos anchors, 6343 puntos/120 KFs y recursos
  correctos, pero prueba visual `NO CONSEGUIDA`: Gazebo no mostro ventana,
  Chrome devolvio `ERR_CONNECTION_REFUSED` y RViz2 renderizo sin permitir mover
  la camara. El mapa no muestra regresion; queda abierta la infraestructura GUI;
- repeticion 140: ocho snapshots reales aunque el usuario no vio la arista por
  su pulso de 240 ms; snapshots con 36 KFs para dron 1 y 59 para dron 2 e
  invalidaciones que explican markers retirados en RViz2. La guarda externa de
  512 MiB cerro a los 146 s, exit 125 y minimo disponible 463.8 MiB;
- repeticion 141 tras cerrar numerosas pestanas de Chrome: la misma trayectoria
  termino en 233 s, seis goals, exit 0 y guarda inactiva. El host mantuvo como
  minimo 4826.3 MiB disponibles, PSI de memoria cero y swap estable; PSS del
  grupo medio/maximo 652.5/751.2 MiB. Confirma que el bloqueo de 140 procedia
  principalmente de la carga previa del host, no de un consumo de 13 GiB del
  pipeline. El usuario confirma posteriormente RViz2 y grafo web plenamente
  correctos;
- benchmark final 2/4/8: 84.3/85.5/89.3 ms por delta-dron y
  23.4/34.4/57.0 MiB para 10k/20k/40k MPs;
- build final 4/4, tests C++ 37/37 y web 8/8;
- las incidencias de cleanup posteriores a `SIM-DONE` son adicionales, no la
  causa de la saturacion observada.

## Aprendizajes

- Ignorar snapshots vacios/no inicializados; no tratarlos como evidencia util.
- No destruir poses globales aceptadas por cambios raw posteriores.
- Un snapshot recuperado no debe publicar por si mismo: su dirty espera al
  siguiente delta principal.
- No usar listas `updated` genéricas para despertar todas las bases; separar
  pose, asociación, geometría, score e invalidación.
- El replay delta-only puede publicar cada delta normalizado; la propiedad de
  diferimiento pertenece al live snapshot, mientras la igualdad exigida al
  replay es la del estado final.

## Operacion y pendiente

- Dos drones con Gazebo GUI, RViz2, web y GUI de mision forman el perfil de
  desarrollo visual.
- Tres o mas drones usan el perfil de escala: Gazebo headless y visualizadores
  selectivos. Las fases dense no deben mantener todas las GUIs por costumbre.
- No degradar mas resolucion, features o vocabulario sin medir calidad de
  tracking/relocalizacion/loop. El vocabulario completo sigue seleccionable.
- La incidencia de rendimiento y la incidencia visual quedan cerradas. La
  prueba 141 valida el perfil completo con margen de host y confirmacion visual
  del usuario. La siguiente subfase a preparar es 3H.

## Detalle

`historial_3G.md`.
