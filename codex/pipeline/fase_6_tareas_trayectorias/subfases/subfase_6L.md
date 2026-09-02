# Subfase 6L - Proteccion visual, STOP y VISUAL_RETREAT

## Estado

```text
sin hacer
```

## Dependencia

6F, 6I-6K y auditoria del recovery/soporte visual final de Fase 5.

## Objetivo tecnico

Crear fast paths locales que eviten colision inmediata y prevengan
`TRACKING_LOST`, sin duplicar el recovery de F5.

## TRACKING_RISK

`VisualRiskEstimator` no usa solo numero de features. Considera distribucion
espacial, movimiento/yaw/pitch previstos, flujo disponible y tendencia para
estimar soporte visible futuro. Umbrales e historial quedan `A MEDIR`.

Ante riesgo: cancelar plan normal, ejecutar `VISUAL_RETREAT` por la trayectoria
realmente recorrida hasta el ultimo estado visual estable, informar contexto y
penalizar zona/direccion/orientacion para no repetir el intento.

## STOP

Depth local puede ordenar STOP sin permiso del servidor. `dron_individual`
genera frenado dinamico desde estado actual, sin salto instantaneo a velocidad
cero, y termina en hover. Despues `task_manager` reporta, servidor mantiene
HOLD, integra obstaculo y replanifica.

Si depth se vuelve peligroso durante retreat: `STOP > VISUAL_RETREAT`. Si la
prevencion falla y llega LOST, se usa recovery F5; despues puede intervenir
ANCHOR_SUBMAP en 6N.

## Cambios requeridos

1. Auditar señales reales F5 y definir estimator predictivo en `task_manager_lib`.
2. Mantener historial corto de estados recorridos y calidad visual.
3. Implementar decision local/cancelacion y reportes `mission_msgs`.
4. Extender `dron_individual/lib_tray` con STOP y retreat dinamicos.
5. Priorizar safety y aislarlo de red, voxelizacion y grafo.
6. Registrar contexto del intento y evitar ciclos riesgo-retreat-retry.
7. Actualizar telemetria con counts/distancia/tiempo, sin samples masivos.

## Limites

No interpretar MPs en el controlador, no invertir velocidad sin seguir corredor,
no esperar servidor para STOP y no crear segundo mecanismo LOST.

## Pruebas

Obstaculo depth inesperado y frenado; risk antes de LOST; features laterales;
retreat sobre recorrido real; transicion suave; riesgo depth durante retreat;
ruta/orientacion alternativa; LOST/recovery F5. Medir braking y thresholds con
GUI+Gazebo+grafo.

## Criterio de exito

STOP siempre domina y alcanza hover seguro; TRACKING_RISK actua preventivamente,
retreat no abandona corredor y el sistema no repite indefinidamente el intento.
