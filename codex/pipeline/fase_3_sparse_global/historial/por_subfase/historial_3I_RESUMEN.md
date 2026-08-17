# Historial 3I - resumen

## Estado vigente

`CONSEGUIDA`: grafo temporal reimplementado, validado automaticamente y
observado en el flujo de live 145. El parpadeo pertenece a la semantica de
tarea del visualizador 3H, no a la construccion del grafo.

## Estado actual

- Ventana mono-submapa desde `last_accepted_control_kf` hasta target.
- Seleccion aproximada del 30 % por camino 3D/densidad/esquinas SE(3), extremos
  obligatorios y vecindades protegidas 20 %.
- Aristas temporales SE(3) y plan de propagacion para no controles.
- Intermedios inactivos se omiten sin reactivarlos; un hard control inactivo se
  conserva como frontera fija y el target debe estar activo.
- Sin covisibilidad ni maximo absoluto de ventana.

## Evidencia

- Test de grafo 4/4, incluido control hard inactivo.
- Replay 144: 10 grafos correctos.
- Live/replay v3: 30 grafos comprometidos, ventana maxima 128 y error target
  final cero.
- 142/143 preservan fallos de orden/inactivos ya corregidos.

Detalle: `historial_3I.md`.
