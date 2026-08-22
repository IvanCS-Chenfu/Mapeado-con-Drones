# Estado actual - resumen

## Situacion

```text
Fase 3: ACTUAL - REIMPLEMENTACION EN CURSO
3B-3P: CONSEGUIDAS
3Q: A REVISAR; RESULTADO ACEPTADO PARA CONTINUAR
3S: CONSEGUIDA; RECALIBRACION TECNICA Y CIERRE RVIZ2 CONFIRMADOS
3T: CONSEGUIDA; ARQUITECTURA AUDITADA Y RENDIMIENTO ACEPTADO
3U: CONSEGUIDA; GRAFO WEB Y TRANSPORTE LIVE ACEPTADOS
3V: CONSEGUIDA; REGRESION ACUMULADA ACEPTADA
3W: CONSEGUIDA; RENDIMIENTO Y ROBUSTEZ ACEPTADOS
```

## Runtime activo

```text
wrapper -> PrimaryQueue -> PrimaryWorker -> SparseGlobalBackend -> ROS
fiducial MAX / database MEDIA / loop BAJA -> SecondaryWorker
raw score = ORB * distancia * aislamiento + inliers
fused score = media(raw miembros) + 0.04 * N
```

- `LandmarkScoreManager` es autoridad unica de score raw/fused;
- raw no anclado conserva factores neutros;
- distancia y aislamiento son configurables, acotados y recuperables;
- el indice voxel y la propagacion a tracks son incrementales;
- visibilidad sparse solo diagnostica; oclusion numerica queda para Fase 8;
- builder publica todos los puntos y servidor colorea rojo-amarillo-verde.

## Evidencia 3S

- build final 3/3; `orbslam3_multi` 9/9, servidor 4/4 y web 1/1;
- prueba 192 `success=true`, pero cola primaria final 45 por reindexado de
  geometria identica; se conserva como intento parcial;
- prueba 193 `success=true`, 722 principales y 1288 secundarias, ambas colas
  `pending=0`, `hard_failed=0`;
- cierre: 60.524 scores, 24.969 anclados, 529 aislados, 1 near, 24.195 far,
  min/media/max `0/0.1502/1`; 30.836 bad a cero;
- 166 intentos fused reducidos, 77 commits, 12.672 positivos, 6.319
  diagnosticos y cero negativos sparse;
- publicaciones finales: 23.531 puntos con `score_field=true rgb_field=true`;
- recursos estables: servidor RSS 269.5 MiB, grupo 1639.2 MiB, PSI memoria 0
  y guarda inactiva.
- prueba 194 `success=true`: principal/secundario pending 0, hard_failed 0;
  24.977 anclados, 99 near, 11.433 far y media 0.2596;
- frente a 193, far baja 24.195->11.433 y near sube 1->99 con anclados casi
  identicos; publicacion final 23.564 puntos score/rgb;
- 53 commits fused, cero negativos sparse; servidor RSS 248.0 MiB, grupo
  1571.3 MiB, PSI memoria 0 y guarda inactiva.

## Cierre y pendiente

RViz2 confirma que las revisitas elevan correctamente el score, pero tambien
que la estructura valida queda demasiado baja y los puntos a menos de 1 m
demasiado altos. El baseline actual es aproximadamente `0.06 m`: el limite
lejano efectivo `40*baseline` queda en `2.4 m` y el cercano en `0.20 m`.
La banda neutra 1-5 m esta validada tecnica y visualmente; el usuario confirma
scores perfectos y concluye 3S. La mala optimizacion final del dron antihorario
se atribuye a dos loops 3Q asimetricos/ambiguos antes del segundo fiducial hard:
el principal corrige 3.950 m y mueve 359 KFs. El fiducial posterior queda dentro
de umbral y no reoptimiza el interior. No se modifica codigo.

3T y 3U quedan cerradas sin cambios funcionales adicionales. La auditoria
confirma dos workers persistentes, ownership separado, commits revisionados y
publicacion principal; el visualizador ya usa SSE live, `Last-Event-ID`,
`state_reset`, drenaje por frame y lifecycle por `flow_id`. El contrato web pasa
9/9 y el usuario acepta rendimiento y grafo.

El usuario acepta 187/188/191/194 como regresion integral suficiente y da 3V
por concluida. Tambien considera buenos el rendimiento y la robustez actuales y
cierra 3W sin mas tuning. No hubo ejecucion A/B, stress o simulacion nueva; los
picos ya documentados permanecen aceptados. El siguiente bloque pendiente es
3X.

## Referencias

```text
codex/pipeline/fase_3_sparse_global/subfases/subfase_3X.md
codex/pipeline/fase_3_sparse_global/historial/por_subfase/historial_3V_RESUMEN.md
codex/pipeline/fase_3_sparse_global/historial/por_subfase/historial_3W_RESUMEN.md
```
