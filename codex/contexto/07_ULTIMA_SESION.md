# Ultima sesion

Fecha: 2026-08-31

Se añadió telemetría visual ORB opcional y del mismo frame: candidatos,
inliers, depth/disparidad, cobertura 4x3, reference KF y `Tcr`. Está apagada
por defecto y no cambia tracking, predictor, mux ni control. Core y wrapper
compilan; CTest pasa 3/3 y el analizador offline 2/2.

350R establece el baseline. En 351, GT gobierna la ruta de dos fachadas y ORB
queda shadow: la evidencia de drone2 cae varios segundos antes de 62 frames no
`OK`, demostrando degradación visual causal. 352 confirma buena evidencia en
una ruta lenta `+2 m X`, `+2 m Y` junto a pared.

353, 354 y 355 completan consecutivamente esa ruta bajo autoridad ORB, sin
fallback posterior ni tracking no `OK`. Conclusión: 5H y la Fase 5 funcional
quedan `CONSEGUIDAS` cuando ORB dispone de evidencia adecuada. La vuelta larga
no queda validada ORB-only; evitar zonas pobres, fragmentar tareas y retirar
`GT_FALLBACK` corresponde a Fase 6. No hay simulaciones activas.
