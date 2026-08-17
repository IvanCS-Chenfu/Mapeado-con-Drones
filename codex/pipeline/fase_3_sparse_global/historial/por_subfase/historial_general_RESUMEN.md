# Historial general - resumen

Leer este archivo antes de `historial_general.md` cuando haya que hablar de
decisiones documentales transversales de Fase 3.

## Estado vigente

Tema general documental. No es una subfase funcional. Registra cambios de
planificacion, handoffs y reasignaciones conceptuales.

## Que se hizo

- Se marco la planificacion activa como `3A-3X`; `12R-*`, `13`, `14` y `15`
  quedaron legacy.
- Se fijo que `codex/contexto/paquetes/` describe codigo vigente y el pipeline
  describe plan/contratos.
- Se limpiaron duplicados activos de `orbslam3_multi` tras `3C`.
- Se reabrieron/corrigieron estados cuando aparecieron problemas de anclajes.
- Se separo propiedad conceptual: `3I` grafo, `3J` dry-run/HTML, `3K` apply,
  `3L` validacion.
- Se reindexo `3M-3X`.
- El 2026-08-05 se cerro el acuerdo transversal de flujo principal, worker
  secundario priorizado y visualizador JavaScript.
- Ese mismo dia se implemento el acuerdo, se compilaron los paquetes y se
  ejecuto la regresion larga hasta `prueba_76`. El diagnostico posterior del
  2026-08-09 reabre como `PARCIAL` la independencia temporal y `3U`.
- El trabajo se distribuyo entre `3C-3X`: `3P` conserva solo fusion y `3R`
  queda absorbida por `3D/3K/3Q`.
- El 2026-08-09, tras revisar los fallos de `prueba_75/76`, se marcaron
  `REHACER` los contratos propietarios `3B-3U` y `3W`. Cada uno distingue
  capacidad histórica reutilizable, implementación incorrecta y contrato
  sustituto; no se modificó código ni se borró evidencia anterior.

## Aprendizajes

- No avanzar subfases si una invariante anterior queda rota.
- Los archivos de subfase deben ser contratos ejecutables, no historiales.
- Los nombres legacy `f1l_*` pueden apuntar a funciones que hoy pertenecen a
  `3J`; no usar el prefijo como unica fuente de propiedad conceptual.
- Una prueba historica valida la implementacion que ejecuto, no contratos
  posteriores.
- Las decisiones transversales se registran aqui, no en el historial de `3P`.

## Punto de reentrada

Contratos `REHACER` preparados documentalmente; implementación funcional aún no
iniciada ni autorizada. Empezar por `3B` y avanzar por propietarios, conservando
los núcleos validados que cada contrato identifica. No ejecutar `3V-3X` antes
de completar la cadena.

## Detalle

`historial_general.md`.
