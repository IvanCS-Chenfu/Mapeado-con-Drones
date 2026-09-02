# Subfase 6K - Replanning incremental y handover continuo

## Estado

```text
sin hacer
```

## Dependencia

6D, 6G, 6I y 6J.

## Objetivo tecnico

Reparar planes en preparacion/ejecucion conservando segmentos validos y evitando
paradas salvo seguridad o ausencia de siguiente plan.

## Comportamiento esperado

Causas HARD: occupancy, clearance, conflicto o geometria que invalida. Causas
SOFT: coverage, distancia de observacion, orientacion o ruta mejor.

Ante `S0'` distinto, conservar W1... y regenerar solo hasta el primer estado
antiguo identico. Si no conecta, insertar Wa/Wb y volver a leer el estado actual
antes del enlace final. Ante obstaculo medio, D* repara el tramo y conserva
prefix/suffix cuyos corredores y estados siguen validos.

`PlanningQueue`: P0 emergencia/dron parado, P1 plan por agotarse, P2 normal;
FIFO dentro de prioridad, anti-starvation y una solicitud coalesced por dron
con ultima revision/reasons.

Un cambio de mapa no interrumpe D* desde otro thread. Tras calcular se comparan
revisiones; si afecta al corredor, repair antes de reservar. El reemplazo de
reserva es atomico y el handover conserva posicion, velocidad, aceleracion,
yaw/pitch y derivadas necesarias.

## Cambios requeridos

1. Implementar cola priorizada/coalesced y motivos HARD/SOFT.
2. Indexar cambios por corredores de segmento.
3. Reparar prefijo, tramo medio y sufijo reutilizable.
4. Insertar waypoints de enlace dinamicamente viables.
5. Regenerar/validar solo partes nuevas con `lib_tray`.
6. Sustituir reserva y enviar `plan_revision` sin salto de referencia.
7. Adaptar longitud/velocidad del plan a incertidumbre.
8. Medir segments reused, repair time, coalescing y handover.

## Limites

No replanificar por conteo bruto de MPs, no recalcular todo por defecto y no
parar ante cada cambio SOFT. No permitir que otro thread mute D* activo.

## Pruebas

Start mismatch, enlace imposible/directo, obstaculo medio, cambio irrelevante,
conservacion de suffix, requests coalesced, anti-starvation, handover continuo y
fallback a hover. Validacion visual con GUI+Gazebo+grafo.

## Criterio de exito

Cambios locales producen reparaciones locales reproducibles; no hay huecos de
reserva ni discontinuidades y las paradas quedan justificadas.
