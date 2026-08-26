# Subfase 5H — Integración final de control y regresión multi-dron

## Estado

```text
sin hacer; absorbe la antigua subfase 5I
```

## Objetivo

Conectar el estado validado de Fase 5 a:

```text
gen_tray
control_calcular_fuerzas
```

y ejecutar la regresión final multi-dron. Al finalizar, GT no es una entrada
directa del funcionamiento normal. Solo puede aparecer dentro de la interfaz
común de estado cuando `pose_source=GT_FALLBACK` según 5G.

## Puertas previas

- 5B: `O` y muestra ORB coherentes;
- 5C/5D: `W_T_KF` versionada y transporte asíncrono;
- 5E: pose local/global;
- 5F: calidad de pose aceptada por el usuario;
- 5G: velocidad/fallback aceptados y excepción GT documentada.

## Arquitectura

```text
ORB-SLAM3 OK --------------------> estado O
ORB-SLAM3 RECENTLY_LOST -> GT_FALLBACK -> estado O
                                           |      |
                                           v      v
                                       gen_tray  control
```

El estado incluye pose/velocidad en `O`, timestamp, validez, `pose_source` y
estado local/global. `gen_tray` y control no duplican ramas ORB/GT.

## Semántica de goals

Relativo: generar en `O`, conservando la semántica vigente de
`TrayAction`/`lib_tray`.

Absoluto: exigir `W_T_O` válida y convertir al aceptar:

```text
O_T_goal = inverse(W_T_O_latest) * W_T_goal
```

Después queda congelado en `O`. Una revisión global no cambia el setpoint. Sin
global se rechaza; una relativa posterior sí puede aceptarse con local válida.

Una nueva orden sustituye/cancela la anterior según el action server. Auditar
la cancelación para que no deje velocidad, aceleración o jerk residuales. No
introducir replanning/recovery de Fase 6.

## Frecuencia

No cambiar 50 Hz->20 Hz por intuición. Medir edad de estado y latencia. El
control puede consumir la última muestra válida dentro de límites acordados.

## Ámbitos

```text
dron/dron_individual/src/control_tray/gen_tray.cpp
dron/dron_individual/src/control_tray/control_calcular_fuerzas.cpp
dron/dron_individual/config/**
dron/dron_individual/launch/**
simulacion/simulacion_dron/**
instrumentación ya validada
```

No cambiar ganancias para ocultar mala estimación, ni algoritmos `lib_tray` sin
un bug demostrado y nueva autorización. Si falla 5E/5G, reabrir su subfase.

## Pruebas de integración

1. relativa con `pose_source=ORB`;
2. absoluta con `GLOBAL_VALID`, revisión global durante el goal y setpoint O
   inmutable;
3. absoluto sin global rechazado y relativa posterior aceptada;
4. pérdida durante goal: `ORB -> GT_FALLBACK -> ORB` sin salto;
5. dos drones completan la vuelta representativa al edificio.

La vuelta debe cubrir anchor temprano, varios ref-KF, correcciones globales y
goals relativos/absolutos. Los eventos que no ocurran naturalmente se validan
en pruebas específicas separadas.

Métricas por dron:

```text
tiempo total/ORB/GT_FALLBACK y ratio ORB
pérdidas, map_epochs y cambios reference KF
revisiones globales, error pose/velocidad
latencia/jitter y resultados de goals
```

Patrones iniciales:

```text
F5H|GOAL|ACCEPT|REJECT|RESULT|POSE_SOURCE|ORB|GT_FALLBACK|REFERENCE_KF|REVISION|GLOBAL_VALID|LOCAL_ONLY|CONTROL|SUCCESS|ERROR|FATAL|Segmentation fault|Killed
```

## Criterio de éxito

Builds correctos; pose/velocidad aceptadas; ausencia de GT directo en
`gen_tray`/control normal; semántica de goals correcta; revisiones sin salto
local; transporte de KFs funcional; dos drones completan la vuelta; fallback
perfectamente identificado y documentación/historial coherentes.

`PARCIAL` si la vuelta termina pero GT aparece fuera del fallback o alguna
transición no es continua. `NO CONSEGUIDA` si hay mezcla de frames,
inestabilidad, absolutos aceptados sin global o fallo propio de Fase 5.

## Deuda obligatoria de Fase 6

```text
eliminar GT_FALLBACK
implementar recovery real
implementar búsqueda/reanclaje sin GT
```
