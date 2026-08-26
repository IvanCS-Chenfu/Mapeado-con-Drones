# Pipeline Fase 5 — Poses de los drones sin Ground Truth — RESUMEN

## Estado

```text
5A: CONSEGUIDA documentalmente el 2026-08-25
5B: siguiente subfase funcional, pendiente de preparación
5C-5H: sin hacer
5I: absorbida en 5H
Fase 5 funcional: sin hacer
```

## Objetivo

Separar de forma estricta:

```text
control continuo: O_T_B
global corregible: W_T_B
```

`RawMapDatabase` conserva ORB crudo, `GlobalPoseStore` conserva la autoridad
world y ORB-SLAM3 no recibe correcciones globales reinyectadas en su mapa.

## Contrato principal

La pose por frame usa el reference KF real y su relación:

```text
Tcr = C_T_Kref
O_T_B = O_T_Kref * inverse(Tcr) * C_T_B
W_T_B = W_T_Kref * inverse(Tcr) * C_T_B
```

No se usa el último KF creado, nearest-KF ni round-trip al Servidor por frame.

El Servidor resuelve `(drone_id, map_epoch, keyframe_id) -> W_T_KF + revision`.
La consulta inicial es asíncrona y las revisiones posteriores llegan por push.

## Goals

```text
relativo:
    aceptar con local válida y ejecutar en O

absoluto + W_T_O válida:
    convertir world -> O al aceptar y congelar

absoluto sin W_T_O:
    RECHAZAR
```

Las correcciones globales posteriores no modifican el goal activo.

## Pérdida

```text
OK             -> ORB/O
RECENTLY_LOST  -> GT_FALLBACK temporal
```

El fallback permite terminar la misión, mantiene `O` continuo y se identifica
mediante `pose_source`. GT no entra en mapa, anchors, optimización ni pose
global final. Se mantiene durante Fase 5 y Fase 6 debe eliminarlo al aportar
recovery real.

## Decisiones de 5A

- smoothing no obligatorio;
- estimador en `orbslam3` del Dron;
- frontera obligatoria Servidor↔`orbslam3`;
- velocidad derivada de `O_T_B`;
- 5I absorbida en 5H;
- antes de 5C/5D se comprueba el cierre final de 3Q;
- antes de 5G se reconcilia formalmente el ADR de GT.

## Secuencia

```text
5A -> 5B -> 5C -> 5D -> 5E -> 5F -> 5G -> 5H
```

Siguiente paso: preparar conversadamente 5B. Ningún cambio funcional de Fase 5
está autorizado por el cierre documental de 5A.
