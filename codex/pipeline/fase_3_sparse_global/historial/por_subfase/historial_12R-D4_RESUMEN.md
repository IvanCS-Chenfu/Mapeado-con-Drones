# Historial 12R-D4 - resumen

Leer este archivo antes de `historial_12R-D4.md` si se consulta material legacy.

## Estado vigente

`12R-D4` es legacy y no forma parte de la planificacion activa. Su cierre fue
`PARCIAL/NO CONSEGUIDA` segun intento; no debe usarse como ruta actual salvo
peticion explicita.

## Que se hizo

- Se intento desbloquear apply de optimizacion local legacy.
- Se amplio diagnostico de loops/subcloud/unified y patrones de reduccion.
- Se anadio un puente auxiliar desde eventos unified fuertes hacia
  `LOCAL_LOOP_OPT_TASK`.

## Evidencia

- Builds legacy seleccionados terminaron con `BUILD-EXIT-CODE 0` tras limpiar
  artefactos generados.
- Hubo simulaciones mecanicamente correctas con `SIM-DONE success=true`.
- La ruta moderna llego a crear eventos/tareas en algun caso, pero la prueba con
  deriva no demostro solver/apply util con `moved>0`.

## Aprendizajes

- No reactivar `LOOP-VIEWMAP-ERROR` como criterio principal.
- La decision debe basarse en evidencia geometrica suficiente de subnube.
- Esta subfase fue sustituida por la Fase 3 nueva `3A-3X`.

## Detalle

`historial_12R-D4.md`.
