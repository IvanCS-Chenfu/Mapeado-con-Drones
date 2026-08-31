# Subfase 5J - Limpieza, consolidacion y preparacion para Fase 6

## Estado

`CONSEGUIDA` el 2026-09-01.

## Objetivo

Dejar el producto de Fase 5 sin overrides de baterias cerradas, conservar la
instrumentacion util y documentar la frontera con Fase 6.

## Clasificacion

- PRODUCTIVO: estimador dynamic, O/W, source lock, `GT_FALLBACK`, buffers,
  gravedad O, `MIDPOINT_DYNAMIC` y handoff.
- TEST UTIL: tests de contratos causales, frames, predictores y regresiones.
- DEBUG UTIL: estado de control y evidencia visual del mismo frame.
- OBSOLETO: nodo GT timing, forcing GT/ORB, shadow manual y overrides parciales.

## Debug

`debug_fase_5=false` es puerta maestra. Los subflags
`debug_orb_control_state` y `debug_orb_visual_evidence` solo son efectivos si
el master esta activo. Warnings funcionales, errores, NaN e invariantes rotas
no se ocultan.

## Contrato orientativo para Fase 6

Fase 6 podra clasificar evidencia como `EVIDENCE_GOOD`,
`EVIDENCE_DEGRADING` o `EVIDENCE_POOR` combinando tendencia y persistencia de:
inliers, ratio de inliers, cobertura espacial, profundidad/disparidad estereo,
tracking, reference KF y coherencia temporal. No debe reaccionar a un frame
aislado ni esperar necesariamente a `LOST`.

Esta subfase no implementa score, thresholds ni decisiones de planificacion.
Los umbrales deben derivarse de baselines empiricos y la politica pertenece a
Fase 6.

## Validacion obligatoria

- builds `orbslam3`, `dron_individual`, `simulacion_dron`;
- tests unitarios y `git diff --check`;
- prueba debug apagado y encendido;
- dos regresiones favorables equivalentes a 353-355.

## Criterio de cierre

La limpieza no cambia el comportamiento productivo ORB, el debug master
funciona en ambos estados y las regresiones completan sin fallback ni perdida.

Cumplido por 357 y 358. La 356 se conserva como intento invalido para ORB por
no obtener anchor, aunque valido el silencio debug OFF.
