# Subfase 5B — Frame continuo O, estado ORB y semántica de navegación

## Estado

```text
CONSEGUIDA el 2026-08-26; validación integrada final en prueba 225
```

## Objetivo

Construir la base local coherente de Fase 5:

1. exponer tracking, `map_epoch`, reference KF real y `Tcr` por frame;
2. medir/definir el frame continuo `O` y publicar `O_T_B`;
3. preparar la pose local para trayectoria/control;
4. fijar aceptación y congelación de goals;
5. observar cambios de KF, Local BA y pérdida de tracking.

5B no implementa backend global completo, velocidad ni `GT_FALLBACK`.

## Contexto y ámbitos

Leer 5A y docs vigentes de `orbslam3_ros2`, `orbslam3_msgs`,
`dron_individual` y `simulacion_dron` antes de código.

Ámbitos probables:

```text
dron/orbslam3_ros2/**
dron/orbslam3_msgs/**                # interfaz mínima, si es necesaria
dron/dron_individual/src/control_tray/gen_tray.cpp
dron/dron_individual/action/**
dron/dron_individual/config/**
dron/dron_individual/launch/**
simulacion/simulacion_dron/**        # instrumentación y pruebas
```

No tocar `servidor/orbslam3_multi`, `servidor/orbslam3_server`, fallback ni
control de fuerzas salvo instrumentación expresamente acordada. Se permite el
cambio mínimo y aditivo ya acordado en `dron/ORB_SLAM3` para ampliar
`StereoTrackingReceipt` con el estado coherente del mismo `TrackStereo`, la
reference KF real y `Tcr`; no se permiten otros cambios funcionales del core.

## Contrato requerido

Cada muestra lógica del mismo `TrackStereo` contiene:

```text
timestamp, map_epoch, tracking_state
reference_keyframe_valid, reference_keyframe_id
Tcr, pose_local
```

Reglas de tracking:

```text
OK             -> local válida
RECENTLY_LOST  -> ORB deja de ser fuente normal
LOST           -> local inválida
OK_KLT         -> no declarar válida sin prueba explícita
```

No declarar `O == L` antes de medir. Si `pose_local` salta por Local BA o por
cambio de KF, construir continuidad mediante `O_T_Kref`/`Tcr` o una relación
equivalente `O_T_L`.

Goals:

```text
relativo + local válida      -> aceptar y ejecutar en O
absoluto + W_T_O inválida    -> rechazar
```

No cambiar flags ni reinterpretar valores world como relativos. Preparar la
capa de trayectoria para congelar frame, `map_epoch` y objetivo al aceptar la
acción. La interfaz ROS será coherente y ampliable para estado local/global
futuro; en 5B los campos globales y de velocidad quedarán explícitamente
inválidos.

La continuidad exigida en 5B es intra-epoch. No se fingirá continuidad entre
epochs ni durante una pérdida; esa recuperación corresponde a 5G mediante el
`GT_FALLBACK` temporal acordado.

## Prohibiciones

- nearest-KF o último KF creado como referencia;
- round-trip al Servidor por frame;
- smoothing, velocidad final o fallback;
- GT para decidir tracking;
- timeout como sustituto de estado ORB real.

## Pruebas acordadas

1. tests deterministas del recibo coherente, transformaciones, tracking,
   continuidad sintética y política de goals;
2. builds aislados de los paquetes afectados;
3. simulación integrada multi-dron con recorrido relativo, cambios de
   reference KF/Local BA y saltos de `O_T_B` medidos;
4. goal absoluto sin global rechazado y goal relativo posterior aceptado;
5. llegada al fiducial, anclaje confirmado y giro relativo de yaw de 180
   grados respecto a la pose normal de la trayectoria; la cámara queda mirando
   hacia la zona sin texturas para provocar la pérdida completa;
6. detección inequívoca e inmediata de `RECENTLY_LOST` y, si progresa, `LOST`,
   usando el estado ORB real del mismo frame y no la ausencia de publicación.

La maniobra de 180 grados es el mecanismo acordado para provocar la pérdida;
no se añadirá blackout visual artificial. GT solo podrá conservar el control
legacy de esta subfase y actuar como métrica externa: nunca construirá `O` ni
decidirá el estado de tracking.

Patrones iniciales:

```text
F5B|TRACKING|RECENTLY_LOST|LOST|REFERENCE_KF|MAP_EPOCH|O_CONTINUITY|GOAL_REJECT|GOAL_ACCEPT|ERROR|FATAL
```

## Criterio de éxito

Muestra ORB coherente, epoch/ref-KF/`Tcr` inequívocos, continuidad `O_T_B`
demostrada, cambio de ref-KF sin salto inaceptable, absoluto sin global
rechazado, relativo funcional y pérdida detectada por estado real tras la
secuencia fiducial-anclaje-giro de 180 grados.

`PARCIAL` si el estado funciona pero falta continuidad. `BLOQUEADA` si la API
exige un cambio no autorizado del core ORB.
