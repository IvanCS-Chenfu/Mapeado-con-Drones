# Subfase 5B — Frame continuo O, estado ORB y semántica de navegación

## Estado

```text
sin hacer; pendiente de preparación conversada
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
control de fuerzas salvo instrumentación expresamente acordada. Si hace falta
modificar `dron/ORB_SLAM3`, suspender y pedir autorización específica.

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
capa de trayectoria para congelar frame y objetivo al aceptar la acción.

## Prohibiciones

- nearest-KF o último KF creado como referencia;
- round-trip al Servidor por frame;
- smoothing, velocidad final o fallback;
- GT para decidir tracking;
- timeout como sustituto de estado ORB real.

## Pruebas acordables

1. recorrido corto con ORB sano y frecuencia/estado medidos;
2. recorrido largo con cambios de KF/Local BA y saltos de `O_T_B` medidos;
3. absoluto sin global rechazado y relativo posterior aceptado;
4. detección inequívoca e inmediata de `RECENTLY_LOST`.

Patrones iniciales:

```text
F5B|TRACKING|RECENTLY_LOST|LOST|REFERENCE_KF|MAP_EPOCH|O_CONTINUITY|GOAL_REJECT|GOAL_ACCEPT|ERROR|FATAL
```

## Criterio de éxito

Muestra ORB coherente, epoch/ref-KF/`Tcr` inequívocos, continuidad `O_T_B`
demostrada, cambio de ref-KF sin salto inaceptable, absoluto sin global
rechazado, relativo funcional y pérdida detectada por estado real.

`PARCIAL` si el estado funciona pero falta continuidad. `BLOQUEADA` si la API
exige un cambio no autorizado del core ORB.
