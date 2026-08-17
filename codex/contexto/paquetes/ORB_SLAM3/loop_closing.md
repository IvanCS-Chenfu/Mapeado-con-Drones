# `LoopClosing`

## Rol

`LoopClosing` realiza place recognition interno, estima la relación Sim3/SE3
entre regiones y, si confirma un loop, corrige el mapa de ORB-SLAM3.

Archivos:

```text
ORB_SLAM3/include/LoopClosing.h
ORB_SLAM3/src/LoopClosing.cc
```

## Detección

El hilo consume KeyFrames enviados por `LocalMapping` y ejecuta:

```cpp
NewDetectCommonRegions()
```

La detección usa BoW, consistencia temporal, matching y estimación Sim3. Si
confirma una región, establece `mbLoopDetected` y conserva:

- `mpCurrentKF`;
- `mpLoopMatchedKF`;
- `mg2oLoopScw`;
- matches y MapPoints de soporte.

La detección no necesita modificar todavía las poses históricas.

## Aplicación actual

Cuando el loop es válido, `Run()` llama directamente:

```cpp
CorrectLoop();
```

`CorrectLoop()`:

1. detiene `LocalMapping`;
2. cancela un Global BA anterior si existe;
3. cambia la pose del KF actual;
4. propaga poses corregidas a KFs covisibles;
5. corrige y fusiona MapPoints;
6. optimiza el essential graph;
7. añade aristas de loop internas;
8. puede lanzar Global Bundle Adjustment;
9. libera `LocalMapping`.

Por tanto, saltar solo el Global BA no basta: las poses ya se modifican antes,
durante la propagación Sim3 y la optimización del essential graph.

## Configuración existente

`System` reconoce:

```yaml
loopClosing: 0
```

Con ese valor, `NewDetectCommonRegions()` retorna sin hacer place recognition.
Se desactivan conjuntamente:

- detección interna;
- corrección de loop;
- merge;
- essential-graph optimization;
- Global BA disparado por loop.

El hilo sigue creado para mantener sin cambios su ciclo de vida, resets y
punteros desde `System`, `Tracking` y `LocalMapping`. La ruta inactiva se
resuelve antes de encolar:

```text
LocalMapping -> InsertKeyFrame()
  mbActiveLC=true  -> cola normal de LoopClosing
  mbActiveLC=false -> KeyFrameDatabase::add() y return
```

El KF inicial conserva la semántica anterior y no pasa por ninguna de las dos
rutas. Los demás KFs no se acumulan en la cola cuando el loop está desactivado
y sí quedan indexados por BoW. `KeyFrameDatabase::add()` protege su fichero
invertido con mutex, por lo que esta inserción desde `LocalMapping` mantiene la
sincronización interna.

## Modo `detect_only`

No existe actualmente como parámetro, pero es técnicamente viable.

La separación natural está en `LoopClosing::Run()`:

```text
NewDetectCommonRegions() -> loop confirmado -> CorrectLoop()
```

Un modo `detect_only` debería:

1. mantener `NewDetectCommonRegions()`;
2. publicar o exponer el par de KFs y la transformación estimada;
3. no llamar a `CorrectLoop()`;
4. no modificar KeyFrames ni MapPoints;
5. no añadir la arista interna como corrección aceptada;
6. no iniciar Global BA;
7. limpiar correctamente el estado de detección;
8. aplicar cooldown/deduplicación para no detectar el mismo loop en cada KF.

En el código vigente, el punto de corte exacto está después de validar
`bGoodLoop` y antes de:

```cpp
CorrectLoop();
```

La limpieza posterior ya libera `mpLoopLastCurrentKF` y `mpLoopMatchedKF`,
vacía los vectores temporales y restablece `mbLoopDetected`. Por tanto, un gate
equivalente a:

```text
place recognition activo
apply loop correction desactivado
```

puede conservar toda la detección y saltar únicamente `CorrectLoop()`. En ese
modo no deben incrementarse `mnNumCorrection` ni los contadores que signifiquen
una corrección realmente aplicada.

El gate debe cubrir también las llamadas `MergeLocal()` y `MergeLocal2()`.
Aunque no sean un loop dentro del mismo mapa, un merge interno corrige poses,
fusiona MapPoints y puede ejecutar optimizaciones globales sobre el Atlas. Para
la arquitectura del servidor, detectar una coincidencia entre mapas y aplicar
su transformación son autoridades distintas.

Parámetros conceptuales recomendados:

```yaml
loopClosing: 1
loopCorrection: 0
mapMergeCorrection: 0
```

Solo `loopClosing` existe actualmente. Los otros dos requieren una extensión
local pequeña en `System`/`LoopClosing`.

Si el servidor ya hace su propia detección BoW y verificación geométrica, el
modo más simple y barato es desactivar también la detección interna. Si se desea
usar ORB-SLAM3 como generador adicional de candidatos, `detect_only` permite
exportarlos sin concederle autoridad sobre las poses.

## Efecto sobre el resto de ORB-SLAM3

Saltar `CorrectLoop()` no desactiva:

- tracking ni `PoseOptimization` del frame actual;
- creación de KeyFrames y MapPoints;
- `LocalMapping`;
- Local Bundle Adjustment;
- BoW, KeyFrameDatabase ni covisibilidad local;
- publicación de `OrbMap` por el wrapper.

ORB-SLAM3 sigue refinando localmente su mapa. Lo único que se elimina es la
mutación histórica disparada por place recognition. Por ello este modo satisface
el objetivo de conservar el frontend local y ceder al servidor la autoridad
sobre cierres globales.

## Evidencia antes del cambio: `prueba_40`

ORB-SLAM3 registró:

```text
*Loop detected
Local Mapping STOP
Local Mapping RELEASE
```

El snapshot posterior del dron afectado actualizó `203` KFs y `12158`
MapPoints, con `49` poses raw cambiadas y `19` cambios grandes. Más tarde, el
grafo del servidor recibió discontinuidades temporales de aproximadamente
`24.96 m` y `18.05 m`.

Esta evidencia demuestra que el loop interno puede competir con una
optimización global previamente aceptada por el servidor.

## Revalidación con loop desactivado: `prueba_41`

El recorrido antihorario fiducial 2 -> 1 -> 2 termina en un único `map_epoch`
con:

```text
ORB-SLAM3-LOOP-CLOSING-CONFIG active=false policy=index_without_loop_queue
*Loop detected: 0
*Merge detected: 0
Local Mapping STOP/RELEASE por loop: 0
F1M-COVIS-SUMMARY confirmed_edges=5552 orbslam3_native=5552
```

Tracking, BoW y covisibilidad permanecen activos. ORB-SLAM3 todavía modifica
poses raw mediante optimización local: se observaron hasta `40` cambios raw en
un snapshot, sin loop, merge, reset ni pérdida de tracking. `GlobalPoseStore`
conservó las poses world `accepted`/`server_optimized` y reproyectó solo
`derived_tail` desde su anchor aceptado.

Por tanto, `loopClosing: 0` elimina la fuente global competitiva sin congelar el
frontend local ni privar al servidor de covisibilidad.
