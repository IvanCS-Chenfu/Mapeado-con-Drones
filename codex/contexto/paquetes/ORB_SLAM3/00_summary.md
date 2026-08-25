# 00_summary — ORB_SLAM3

Resumen: librería externa ORB-SLAM3 usada localmente en cada dron
(stereo/mono). No modificar sin justificación técnica y validación específica.

## Interfaces clave

- Entradas: imágenes de cámara stereo/mono.
- Salidas extraídas por el wrapper: `pose_local`, `OrbMap` incremental y
  snapshots completos.
- Datos exportados: `KeyFrame`, `MapPoint`, descriptores, BoW y covisibilidad.

Scripts / ejecutables: ninguno propio en este paquete; la biblioteca C++ es
consumida por `orbslam3_ros2`.

El wrapper requiere `System::GetAllKeyFrames()` y
`System::GetAllMapPoints()` para exportar `OrbMap`. Estos métodos son
passthroughs de solo lectura a `Atlas`.

Fase 4C añade `StereoTrackingReceipt`: cada `TrackStereo()` puede devolver por
valor el evento one-shot del KF creado en esa llamada y la imagen/calibracion
efectivas tras el preprocessing de `System`. `Tracking` limpia el evento en
cada frame y en resets; no se cambia la politica de creacion de KFs.

## Autoridad en el proyecto

ORB-SLAM3 debe actuar como frontend local:

```text
tracking + creación de KFs/MPs + BoW + covisibilidad local
```

El servidor debe ser la autoridad para:

```text
fiduciales + loops multi-dron + grafo global + optimización global
```

Un cierre de bucle interno de ORB-SLAM3 puede modificar poses históricas y
MapPoints después de que el servidor los haya aceptado. La ruta y sus posibles
modos de desactivación están documentados en `loop_closing.md`.

## Estado runtime

Estado 2026-07-27: `ORB_SLAM3/` se mantiene sobre upstream
`4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4` con cambios locales justificados:

- C++14 en `CMakeLists.txt` por compatibilidad con Pangolin/sigslot;
- `System::GetAllKeyFrames()` y `System::GetAllMapPoints()`;
- guarda numérica en `Sim3Solver` antes de `Sophus::SO3f::exp`.
- modo `loopClosing: 0` robusto: los KFs se indexan directamente en
  `KeyFrameDatabase` y no se acumulan en la cola de `LoopClosing`.
- evento `KeyFrameCreationEvent` y recibo estereo exacto para Fase 4C.

`install/orbslam3/lib/libORB_SLAM3.so` es un enlace a
`src/ORB_SLAM3/lib/libORB_SLAM3.so`.

Validación acumulada:

- `cmake --build ORB_SLAM3/build -j4`: código 0;
- ambos símbolos `System::GetAll*` aparecen en `nm -D`;
- el ejecutable stereo carga la biblioteca sin `undefined symbol`;
- la guarda de `Sim3Solver` evita `SO3::exp failed` y `exit code -6`.
- `prueba_41` arranca ambos frontends con
  `active=false policy=index_without_loop_queue`;
- no aparecen loops, merges ni paradas de `LocalMapping` por loop;
- la covisibilidad nativa sigue llegando al servidor
  (`orbslam3_native=5552` al final).

No compilar ORB-SLAM3 con paralelismo alto sin control: un intento con `-j16`
terminó con `cc1plus` matado por el sistema.

## Configuración

Calibraciones activas:

```text
simulacion_dron/config/orbslam/
dron_individual/config/orbslam/
```

La biblioteca reconoce la clave OpenCV `loopClosing`. Si vale `0`, desactiva
place recognition, loop closing y merge internos. El hilo `LoopClosing` sigue
existiendo para conservar el ciclo de vida y las interfaces internas, pero su
cola queda vacía: `InsertKeyFrame()` indexa cada KF no inicial directamente en
`KeyFrameDatabase`. Se conserva así la relocalización interna sin conceder a
ORB-SLAM3 autoridad para corregir poses históricas.

No existe todavía un parámetro que conserve place recognition y desactive solo
la aplicación de la corrección. Ese modo `detect_only` es viable separando
`NewDetectCommonRegions()` de `CorrectLoop()` y bloqueando también
`MergeLocal()`/`MergeLocal2()`. El detalle y el punto de corte están en
`loop_closing.md`.

## Detalle

- `system.md`: construcción de subsistemas, configuración y modos.
- `tracking.md`: tracking y optimización de pose del frame actual.
- `local_mapping.md`: creación local de mapa y Local BA.
- `loop_closing.md`: detección, corrección de loop, merge y Global BA.
- `optimizer.md`: operaciones de optimización y qué estado modifican.
- `sim3_solver_guard.md`: guarda numérica local de `Sim3Solver`.
