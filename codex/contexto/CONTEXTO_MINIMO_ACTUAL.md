# Contexto minimo actual

Precondicion: leer fisicamente `00_CONTEXTO_COMPACTACION.md` antes de este
archivo y reconciliarlo con la peticion mas reciente.

## Estado

```text
Fase: 3 - mapa sparse global multi-dron
3B-3P: CONSEGUIDAS
3Q: A REVISAR; aceptada para continuar
3S: CONSEGUIDA; recalibracion tecnica y cierre RViz2 confirmados
3T: CONSEGUIDA; arquitectura auditada y rendimiento aceptado
3U: CONSEGUIDA; grafo web y transporte live aceptados
3V: CONSEGUIDA; regresion integral acumulada aceptada
3W: CONSEGUIDA; rendimiento y robustez aceptados
```

## Runtime

```text
wrapper -> PrimaryQueue -> PrimaryWorker -> SparseGlobalBackend -> builder/ROS
fiducial MAX / database MEDIA / loop BAJA -> SecondaryWorker
LandmarkScoreManager: raw ORB*distancia*aislamiento+inliers -> fused media+0.04*N
```

- raw y BoW originales permanecen inmutables;
- score geometrico se actualiza por IDs y voxels afectados;
- cambios raw/pose se propagan solo a fused tracks miembros;
- visibilidad sparse no resta score; oclusion queda para Fase 8;
- builder no filtra y RViz2 usa rojo-amarillo-verde.

## Evidencia vigente

- build 3/3; tests multi 9/9, servidor 4/4 y web 1/1;
- 192 `success=true` pero backlog primario 45, corregido sin cambiar politica;
- 193 `success=true`, principal/secundario `pending=0`, `hard_failed=0`;
- 60.524 tracked, 24.969 anchored, 529 isolated, 1 near y 24.195 far;
- score min/media/max `0/0.1502/1`, incluyendo 30.836 bad a cero;
- 77 commits fused muestreados y cero penalizaciones sparse;
- 23.531 puntos publicados con score/rgb; recursos estables.
- 194 recalibra banda 1-5 m: 24.977 anchored, 99 near, 11.433 far y media
  0.2596; colas cero, 23.564 puntos score/rgb y recursos estables.

## Lectura siguiente

```text
codex/contexto/01_ESTADO_ACTUAL_RESUMEN.md
codex/pipeline/fase_3_sparse_global/subfases/subfase_3X.md
codex/pipeline/fase_3_sparse_global/historial/por_subfase/historial_3V_RESUMEN.md
codex/pipeline/fase_3_sparse_global/historial/por_subfase/historial_3W_RESUMEN.md
```

Revision visual: las revisitas elevan correctamente el score, pero la estructura
valida queda demasiado baja y los puntos a menos de 1 m demasiado altos. Con
baseline aproximado `0.06 m`, los umbrales actuales son `0.20 m` y `2.4 m`.
El usuario confirma que los scores visuales de 194 han salido perfectos y
concluye 3S. Diagnostico separado: dos loops 3Q asimetricos y ambiguos movieron
359/362 KFs del dron 2 antes del segundo fiducial hard; el dominante corrigio
3.950 m/0.454 rad. El fiducial posterior quedo dentro de umbral y no corrigio
el interior. No se aplicaron cambios. Los logs completos nunca se leen
directamente.

La auditoria posterior confirma que 3T ya estaba implantada por 3C-3S y que 3U
ya habia retirado la cola 110 ms y el replay SSE desde cero. El contrato web
pasa 9/9; el usuario considera muy buenos tanto el rendimiento como el grafo y
concluye ambas subfases. No hubo cambios funcionales ni simulacion nueva.

El usuario acepta tambien el conjunto de pruebas 187/188/191/194 como regresion
integral suficiente para 3V y considera buenos rendimiento y robustez para 3W.
Ambas quedan concluidas sin codigo ni simulacion adicional; se conservan los
picos residuales y la ausencia de una nueva prueba A/B/stress como evidencia
explicita. El siguiente bloque pendiente es 3X.
