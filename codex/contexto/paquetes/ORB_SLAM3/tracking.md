# `Tracking`

## Rol

`Tracking` estima la pose de la cámara en cada frame, decide cuándo insertar un
KeyFrame y mantiene el enlace con el mapa local.

Archivos:

```text
ORB_SLAM3/include/Tracking.h
ORB_SLAM3/src/Tracking.cc
```

## Operaciones relevantes

Durante tracking normal:

- extrae y asocia features;
- usa el modelo de movimiento y el mapa local;
- optimiza la pose del frame actual;
- decide la creación de nuevos KeyFrames;
- entrega KeyFrames a `LocalMapping`.

Las llamadas a:

```cpp
Optimizer::PoseOptimization(&mCurrentFrame)
```

ajustan la pose del frame en curso. Esta optimización es parte esencial del
tracking y no debe confundirse con una optimización histórica del grafo.

## Inicialización

La inicialización monocular incluye un Global Bundle Adjustment inicial para
crear escala y geometría coherentes. La ruta stereo del proyecto dispone de
profundidad y no depende de la misma inicialización monocular.

## Qué debe mantenerse

Aunque ORB-SLAM3 se use como frontend del servidor, deben mantenerse:

- optimización de pose del frame actual;
- relocalización local cuando sea necesaria;
- asociación con MapPoints;
- decisión e inserción de KeyFrames.

Eliminar estas operaciones no solo ahorra optimización global: degrada o rompe
el tracking.

## Evento one-shot de KeyFrame de 4C

En stereo los dos puntos de creación registran el mismo evento escalar:

- `Tracking::StereoInitialization()` crea el KF inicial;
- `Tracking::CreateNewKeyFrame()` crea los KFs durante tracking normal.

`GrabImageStereo()` limpia el evento antes de cada frame; ambas rutas llaman a
`RecordKeyFrameCreation()` y `System` usa
`ConsumeLastCreatedKeyFrameEvent()` una sola vez. `Reset()` y
`ResetActiveMap()` tambien limpian el estado. El getter `GetLastKeyFrame()` se
mantiene solo para dependencias internas de `LoopClosing`, no para el wrapper.
`GetEffectiveCameraCalibration()` copia `mK` y `mDistCoef` por valor.

Referencias:

```text
ORB_SLAM3/include/Tracking.h -> KeyFrameCreationEvent,
  ConsumeLastCreatedKeyFrameEvent
ORB_SLAM3/src/Tracking.cc -> Tracking::StereoInitialization,
  Tracking::CreateNewKeyFrame, Tracking::RecordKeyFrameCreation
rg "RecordKeyFrameCreation|ConsumeLastCreatedKeyFrameEvent|ClearLastCreated"
aproximadamente lineas 520-570, 2335-2430 y 3216-3345
```

## Qué no controla

`Tracking` no es la ruta principal que reescribe todos los KeyFrames tras un
loop. Esa mutación pertenece a `LoopClosing::CorrectLoop()` y a las funciones
de `Optimizer` que esta invoca.
