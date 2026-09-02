# Subfase 6N - GO_TO, ANCHOR_SUBMAP y fiduciales oportunistas

## Estado

```text
sin hacer
```

## Dependencia

6E-6M y contratos finales de Fases 4/5.

## Objetivo tecnico

Integrar comportamientos especiales usando exactamente el mismo pipeline de
mapa, D*, trayectoria y reservas, sin canales directos alternativos.

## GO_TO

Objetivo `(x,y,z,yaw)` en W, con prioridad alta entre pendientes. No preempta
una tarea `RUNNING`; si no hay dron libre espera. Se valida contra
`hard_flight_volume`, mapa, dinamica, reservas, depth y riesgo visual. La GUI F7
nunca llama `TrayAction` directamente.

## ANCHOR_SUBMAP

Cuando tracking local sigue valido pero falta anclaje global:

```text
MAP_SECTION RUNNING -> PAUSED -> ANCHOR_SUBMAP -> RUNNING
```

La tarea/branch ownership se conserva. LOST real usa primero recovery F5. La
primera version reconoce garantia global debilitada durante movimiento relativo:
no introduce automaticamente global lock ni detiene todos los drones; documenta
la limitacion y puede usar region de incertidumbre en una mejora futura.

## Fiducial oportunista

Una deteccion F4 de geometria pobre puede crear subobjetivo temporal si el
beneficio compensa coste/riesgo. Se aproxima mediante el pipeline normal,
intenta mejorar observacion y retoma la misma `MAP_SECTION/task_id`. No se
persigue si ruta, reservas, tracking, coste o evidencia previa lo desaconsejan.

## Cambios requeridos

1. API de encolado GO_TO y estado visible para GUI/progreso.
2. Validacion/planificacion/reserva normal, sin bypass.
3. Lifecycle PAUSED/ANCHOR/reanudacion con ownership intacto.
4. Auditar y reutilizar recovery, anchors e hints F4/F5 existentes.
5. Modelar subobjetivo fiducial y retorno al target previo.
6. Exponer eventos correlacionados al grafo y GUI.

## Limites

No preemption arbitraria, no control directo, no nuevo recovery paralelo y no
perseguir fiduciales sin utilidad/seguridad demostrable.

## Pruebas

GO_TO con dron libre/todos ocupados/fuera de volumen; no preemption; perdida de
ancla con tracking; LOST; pausa/ownership; fiducial util/no util y retorno a la
tarea. Todo con GUI+Gazebo+grafo, sin RViz ni GT funcional.

## Criterio de exito

Los tres comportamientos reutilizan el pipeline normal, mantienen lifecycle y
ownership correctos y no crean atajos inseguros.
