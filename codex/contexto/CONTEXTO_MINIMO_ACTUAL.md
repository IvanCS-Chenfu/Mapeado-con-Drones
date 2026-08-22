# Contexto minimo actual

Precondicion: leer fisicamente `00_CONTEXTO_COMPACTACION.md` antes de este
archivo y reconciliarlo con la peticion mas reciente.

## Estado

```text
Fase actual: Fase 2 - separacion servidor/dron/simulacion
Fase 3: CONSEGUIDA
Subfases 3B-3T: CONSEGUIDAS
```

La numeracion final posterior a 3Q es:

- `3R`: scoring raw/fused incremental;
- `3S`: perfil YAML de observabilidad y debug;
- `3T`: limpieza, configuracion y handoff.

Las auditorias provisionales de arquitectura, visualizador, regresion y
rendimiento quedaron absorbidas por capacidades ya implantadas. Sus historiales
se conservan en `historial/absorbidas/`, pero ya no son subfases activas.

## Runtime entregado

```text
wrapper -> PrimaryQueue -> PrimaryWorker -> SparseGlobalBackend -> ROS
fiducial MAX / database MEDIA / loop BAJA -> SecondaryWorker
raw score = ORB * distancia * aislamiento + inliers
fused score = media(raw miembros) + 0.04 * N
```

- raw ORB-SLAM3 y BoW permanecen inmutables;
- `GlobalPoseStore` posee anchors y poses globales/optimizadas;
- `LandmarkScoreManager` posee score raw/fused y el builder publica fused
  tracks con score/rgb;
- visibilidad sparse no penaliza; la correccion por oclusion se aplaza a Fase 8;
- el YAML `simulacion_dron/config/fase3_debug.yaml` gobierna RViz2, grafo web,
  navegador y logs terminales de Fase 3.

## Evidencia de cierre

- prueba 195: simulacion completa correcta, colas 0/0, 11 commits loop y
  23.978 puntos con score/rgb; revision humana de RViz2: resultado correcto;
- prueba 196: escenario corto `success=true`, servidor operativo, cuatro goals
  correctos, cero marcadores `[F3*]` y ningun proceso RViz2/web/navegador con
  los cuatro flags de debug a false;
- build final 3/3 y CTest final: `orbslam3_multi` 9/9,
  `orbslam3_server` 10/10 y `simulacion_dron` 8/8.

La deformacion observada en la prueba 194 permanece documentada en 3Q, pero no
se reprodujo en la revision visual de 195 y no bloquea el cierre. Como mejora
futura, 3Q propone acumular dos apoyos independientes para candidatos cercanos
y hasta 8-10 para candidatos lejanos o ambiguos antes de una unica
optimizacion. No se modifico el algoritmo con esa hipotesis.

## Lectura siguiente

```text
codex/pipeline/fase_2_separacion_paquetes/
codex/pipeline/fase_3_sparse_global/RESULTADO_FINAL_FASE_3.md
codex/pipeline/fase_3_sparse_global/historial/INDEX.md
```
