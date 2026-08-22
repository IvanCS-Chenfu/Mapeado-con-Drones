# Subfase 3P - Fusion y evidencia de landmarks

```text
CONSEGUIDA; PRUEBA 161 Y CIERRE VISUAL CONFIRMADOS
```

Completa la rama de error bajo de la misma `LoopTask`: `LoopPipeline` reutiliza
subnubes/RANSAC de 3O, fusiona tracks transitivos, aplica score por inliers y
visibilidad sparse simetrica, compromete bases derivadas y deja dirty sets para
el siguiente `PrimaryInput`. No crea otra tarea, publica, modifica raw/poses ni
filtra puntos por score. La baseline anterior fue referencia algorítmica, no
una integración a copiar. La rama de error alto queda para 3Q; 3P no crea almacenamiento
permanente anticipado de sus inliers.

El acuerdo 3Q mantiene la evidencia RANSAC en memoria dentro de la `LoopTask`
que activa la optimizacion. Tras `ACCEPT`, reutiliza esos pares y entra
directamente en esta fusion. Si la fusion se omite por sus guardas, las poses
validas se conservan; stale/rollback conserva la politica de BAJA fresca.

Tras la prueba 160 se elimina todo presupuesto temporal de visibilidad: el
trabajo sigue acotado por regiones/subnubes. Un intento de fusion stale o con
rollback termina y crea una `LoopTask` BAJA fresca para el mismo KF, con
revisiones actuales, coalescencia y sin limite fijo de reintentos. El objetivo
diagnostico de commit de 5 ms desaparece y no se sustituye por warning.

- `subfase_3P_especificacion.md`: ownership, evidencia, tracks e invariantes.
- `subfase_3P_implementacion.md`: algoritmo, score, commit e incrementalidad.
- `subfase_3P_testing.md`: build, pruebas, simulacion y metricas.
- `subfase_3P_criterios.md`: exito, parcial, fallo y bloqueos.

El grafo visual sigue `LoopPipeline -> FusedLandmarkManager ->
CovisibilityDatabase/LandmarkScoreManager -> GlobalMapBuilder dirty` bajo un
unico lifecycle, conforme a `../CONTRATO_VISUAL_INCREMENTAL.md`.
