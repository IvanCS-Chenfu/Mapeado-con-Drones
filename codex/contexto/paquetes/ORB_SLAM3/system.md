# `System`

## Rol

`ORB_SLAM3::System` construye y conecta los componentes principales de una
instancia ORB-SLAM3:

```text
Tracking
LocalMapping
LoopClosing
Atlas
KeyFrameDatabase
ORBVocabulary
```

Archivos:

```text
ORB_SLAM3/include/System.h
ORB_SLAM3/src/System.cc
```

## Inicialización

El constructor carga calibración, vocabulario y Atlas; crea los hilos de
`LocalMapping` y `LoopClosing`; y conecta los punteros entre `Tracking`,
`LocalMapping` y `LoopClosing`.

El hilo `LoopClosing` se crea incluso cuando su actividad se deshabilita por
configuración.

## Configuración de loop closing

`System` lee:

```yaml
loopClosing: 0  # desactivado
loopClosing: 1  # activado; valor por defecto
```

La clave se transforma en `activeLC` y se entrega al constructor de
`LoopClosing`.

El arranque informa de la política efectiva con:

```text
[ORB-SLAM3-LOOP-CLOSING-CONFIG] active=<true|false> policy=<detect_and_correct|index_without_loop_queue>
```

Con `loopClosing: 0` se conserva el resto de ORB-SLAM3:

- tracking;
- inserción de KeyFrames;
- creación y mantenimiento local de MapPoints;
- Local Bundle Adjustment;
- indexación BoW en `KeyFrameDatabase`;
- exportación de mapa por el wrapper.

No equivale a `ActivateLocalizationMode()`.

## Localization mode

`ActivateLocalizationMode()` detiene `LocalMapping` e informa a `Tracking` de
que debe operar en modo solo tracking. Este modo no es adecuado para el pipeline
normal del servidor porque deja de crear el mapa local que alimenta `OrbMap`.

`DeactivateLocalizationMode()` reactiva `LocalMapping`.

## Extensiones locales

El wrapper ROS necesita consultar el Atlas completo. La copia del proyecto
expone:

```cpp
System::GetAllKeyFrames()
System::GetAllMapPoints()
```

Son consultas; no aplican correcciones.

Fase 4C añade el recibo opcional exacto de tracking:

```cpp
System::KeyFrameCreationEvent
System::EffectiveCameraModel
System::StereoTrackingReceipt
System::TrackStereo(..., StereoTrackingReceipt* receipt)
System::UsesInternalStereoRectification()
System::UsesInternalStereoResize()
```

Tras `GrabImageStereo()`, `TrackStereo()` consume una sola vez el evento de
`Tracking`. Solo cuando esa llamada crea KF, el recibo contiene ID de KF/frame,
timestamp, copia de `imLeftToFeed`, `K`, distorsion y dimensiones efectivas.
El wrapper ya no sondea estado persistente ni asocia por proximidad temporal.
Las dos consultas de preprocessing permiten rechazar doble rectificacion.

Fase 5B amplía aditivamente el mismo recibo con `tracking_state`, reference KF
real e ID, y `Tcr = C_T_Kref`. Los campos se capturan inmediatamente después
del `TrackStereo` que produjo la pose, usando
`Tracking::mCurrentFrame.mpReferenceKF`; por tanto no mezclan consultas de
frames distintos. No se modifica mapa, tracking ni optimización del core.

Con `StereoTrackingReceipt::collect_visual_evidence=true`, 5H añade al mismo
recibo `frame_id`, features, asociaciones, outliers, right-x/depth y los
contadores de candidatos/inliers de `Tracking`. La copia se hace tras
`GrabImageStereo()` y solo para telemetría; con el flag falso no se copian esos
vectores. `Tcr` sigue incluido para reconstruir la pose ORB cruda del frame.

Referencias:

```text
ORB_SLAM3/include/System.h -> StereoTrackingReceipt, TrackStereo
ORB_SLAM3/src/System.cc -> System::TrackStereo
rg "StereoTrackingReceipt|collect_visual_evidence|GetMatchesCandidates|Tcr"
aproximadamente lineas 100-150 y 249-355
```

## Restricción de autoridad

En la arquitectura multi-dron, `System` no debe convertir una detección local
de loop en una corrección global irreversible si el servidor es quien fusiona
fiduciales, loops y submapas.

Un modo futuro `server_frontend_only` debería expresar esta política
explícitamente, sin reutilizar `LocalizationMode`.
