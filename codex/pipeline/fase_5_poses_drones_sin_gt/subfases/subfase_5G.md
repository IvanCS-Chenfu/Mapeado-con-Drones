# Subfase 5G — Velocidad estimada y GT_FALLBACK temporal

## Estado

```text
preparada; requiere reconciliar ADR 0002 antes de implementar
```

## Objetivo

Completar el estado común de control con:

1. velocidad lineal/angular derivada de la pose continua `O`;
2. aceleración opcional solo para debug;
3. `GT_FALLBACK` temporal para continuar una misión absoluta cuando la
   estimación todavía no permite gobernarla o ORB pierde tracking.

La recuperación real sin GT pertenece a Fase 6.

## Puerta documental obligatoria

Antes de implementar:

- reconciliar ADR 0002 con la excepción exclusiva de Simulación/Fase 5;
- añadir un parámetro explícito de activación, desactivado por defecto;
- hacer visibles `pose_source=GT_FALLBACK`, causa y métricas;
- registrar su retirada obligatoria en Fase 6.

## Velocidad

Fuente:

```text
O_T_B(t0), O_T_B(t1)
```

Usar timestamps coherentes y rechazar `dt <= 0`, repetidos, muestras no finitas
y gaps excesivos. Obtener velocidad angular del incremento de orientación, sin
discontinuidades Euler. No derivar movimiento de `W_T_B`.

Aplicar un filtro causal de baja latencia solo si las métricas lo justifican.
La aceleración debug queda desactivada por defecto.

## Entrada y permanencia en fallback

Las causas se distinguen explícitamente:

```text
STARTUP_UNANCHORED_ABSOLUTE
TRACKING_LOST
NEW_EPOCH_UNANCHORED
```

`STARTUP_UNANCHORED_ABSOLUTE` no significa que ORB esté perdido: existe pose
local, pero todavía no hay `W_T_O` autoritativa para convertir el goal world.
Un goal relativo con O local válida no necesita anchor ni debe activar GT.

La entrada por tracking ocurre desde `RECENTLY_LOST`; `LOST` mantiene el modo.
Si un absoluto comienza sin anchor, el fallback permite iniciar directamente
la trayectoria world y acercarse al primer fiducial, sin bootstrap adicional.

Captura y propagación:

```text
O_T_G = O_T_B_last * inverse(G_T_B_last)
O_T_B = O_T_G * G_T_B_GT
pose_source = GT_FALLBACK
```

La velocidad GT, si es necesaria, se transforma a `O` y queda confinada al
fallback. La trayectoria activa y los siguientes tramos continúan. `gen_tray`
y control consumen una interfaz común, no topics GT directos en operación
normal.

La fuente queda congelada durante cada goal. Si el goal comienza en
`GT_FALLBACK`, un anchor nuevo no provoca `GT -> ORB` hasta que termine esa
trayectoria. ORB puede cualificarse en segundo plano y tomar el control en la
frontera anterior al siguiente goal. Si un goal comienza en ORB y tracking deja
de ser valido, el fallback entra inmediatamente y permanece bloqueado hasta el
final del goal; no se intenta volver a ORB a mitad de esa trayectoria.

## Salida y nuevo epoch

Salir del fallback, ademas de estar entre goals, exige simultaneamente:

```text
tracking_state == OK
map_epoch actual anclado por fiducial o loop
```

Un anchor sin tracking o tracking recuperado sin anchor no bastan. Si el
submapa nunca se ancla, GT continúa hasta terminar la misión.

No se exige transición sin salto en Fase 5. El cambio de fuente y cualquier
salto se registran en la frontera; smoothing y recovery quedan para Fase 6.

Con epoch nuevo, GT mantiene provisionalmente `O`. La global permanece inválida
hasta anchor/loop y el fallback sigue activo aunque ORB vuelva antes.

## Aislamiento de GT

Prohibido usar GT durante operación estimada válida o para `W_T_KF`, mapa, KFs,
anchors, loops, optimización, selección de KF o pose global autoritativa. La
única excepción con tracking `OK` es un absoluto sin anchor, identificado como
`STARTUP_UNANCHORED_ABSOLUTE` o `NEW_EPOCH_UNANCHORED`.

El fallback no puede quedar oculto ni confundirse con recovery final. Puede
mantener en memoria la última muestra GT para entrar de forma reproducible,
pero esa muestra no afecta salidas mientras el fallback esté inactivo.

La diferencia `GT - pose estimada` es exclusivamente una metrica externa. La
deriva puede hacerla grande sin que ORB haya perdido tracking, por lo que no
activa fallback, no invalida ORB y no participa en la cualificacion de fuente.

## Métricas y pruebas

Velocidad: RMSE, MAE, p95, máximo y delay.

Fallback: causa, entradas, tiempo total/máximo, ratio ORB, mismo/nuevo epoch y
salto al entrar/salir.

Pruebas deterministas:

1. velocidad normal y corrección global sin pico falso;
2. arranque absoluto sin anchor mediante fallback;
3. `ORB -> GT_FALLBACK` durante un goal largo;
4. salida solo con ORB `OK` mas anchor y en frontera de goal;
5. anchor durante goal GT no cambia la fuente hasta finalizar;
6. perdida durante goal ORB entra en GT y no vuelve a ORB hasta finalizar;
7. epoch nuevo sin anchor y fallback hasta fiducial/loop.

Patrones iniciales:

```text
F5G|VEL_EST|ANG_VEL|STARTUP_UNANCHORED_ABSOLUTE|TRACKING_LOST|NEW_EPOCH_UNANCHORED|GT_FALLBACK|POSE_SOURCE|ANCHOR|ERROR|FATAL
```

## Criterio de éxito

Velocidad aceptada, correcciones globales aisladas, fallback inmediato y
visible, misión continua, salida condicionada por ORB+anchor, epoch nuevo
provisional correcto y ADR/documentación reconciliadas. El salto de transición
se mide, pero no condiciona el éxito de Fase 5.

`NO CONSEGUIDA` si GT se usa fuera del fallback o contamina global/mapa.
