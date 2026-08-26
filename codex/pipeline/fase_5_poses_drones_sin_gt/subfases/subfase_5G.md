# Subfase 5G — Velocidad estimada y GT_FALLBACK temporal

## Estado

```text
sin hacer; requiere 5F aceptada y reconciliar ADR 0002
```

## Objetivo

Completar el estado de control con:

1. velocidad lineal/angular derivada de la pose continua `O`;
2. aceleración opcional solo para debug;
3. `GT_FALLBACK` temporal para continuar la misión al entrar ORB en
   `RECENTLY_LOST`.

La recuperación real sin GT pertenece a Fase 6.

## Puerta documental obligatoria

Antes de implementar:

- reconciliar formalmente el ADR vigente de GT con la decisión del usuario;
- declarar la excepción exclusiva de Simulación/Fase 5;
- añadir un parámetro explícito de activación;
- hacer visible `pose_source=GT_FALLBACK` y sus métricas;
- registrar la retirada obligatoria en Fase 6.

## Velocidad

Fuente:

```text
O_T_B(t0), O_T_B(t1)
```

Usar timestamps coherentes y rechazar `dt <= 0`, repetidos, muestras no finitas
y gaps excesivos. La velocidad angular se obtiene del incremento de orientación
sin discontinuidades Euler. No derivar movimiento de `W_T_B`.

Aplicar un filtro causal de baja latencia solo si las métricas lo justifican.
La aceleración debug queda desactivada por defecto.

## Entrada y permanencia en fallback

Trigger exacto:

```text
tracking_state == RECENTLY_LOST
```

Capturar coherentemente:

```text
O_T_G = O_T_B_last * inverse(G_T_B_last)
```

Durante fallback:

```text
O_T_B = O_T_G * G_T_B_GT
pose_source = GT_FALLBACK
```

La velocidad GT, si es necesaria durante fallback, se transforma al frame `O`
y queda confinada a este modo. La trayectoria activa y los siguientes tramos
de la misión continúan; `gen_tray` y control consumen una interfaz común.

## Salida y nuevo epoch

Al recuperar ORB en el mismo epoch, realinear ORB con el `O` actual antes de
cambiar `pose_source`, sin salto ni cancelación del goal.

Con epoch nuevo, GT mantiene provisionalmente `O` mediante la relación entre
el nuevo mapa y el body. La global permanece inválida hasta anchor/loop.

## Aislamiento de GT

Prohibido usar GT durante tracking `OK` o para `W_T_KF`, mapa, KFs, anchors,
loops, optimización, selección de KF o pose global autoritativa. El fallback no
puede quedar oculto ni confundirse con recovery final.

## Métricas y pruebas acordables

Velocidad: RMSE, MAE, p95, máximo y delay.

Fallback: entradas, tiempo total/máximo, ratio ORB, mismo/nuevo epoch y salto
en `O` al entrar/salir.

Pruebas:

1. velocidad normal y corrección global sin pico falso;
2. `ORB -> GT_FALLBACK` durante un goal largo;
3. retorno al mismo epoch sin salto;
4. epoch nuevo con `O` continuo, global inválida y anchor posterior.

Patrones iniciales:

```text
F5G|VEL_EST|ANG_VEL|ACC_DEBUG|RECENTLY_LOST|GT_FALLBACK|POSE_SOURCE|SAME_EPOCH|NEW_EPOCH|REALIGN|ANCHOR|ERROR|FATAL
```

## Criterio de éxito

Velocidad aceptada, correcciones globales aisladas, fallback inmediato y
visible, misión continua, entrada/salida sin salto, epoch nuevo provisional
correcto y ADR/documentación reconciliadas.

`NO CONSEGUIDA` si GT se usa fuera del fallback o contamina global/mapa.
