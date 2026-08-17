# ORB_SLAM3 — Guarda numérica en Sim3Solver

## Incidente

Durante la validación live de `3N`, el nodo `orbslam3/stereo` de un dron moría
con:

```text
SO3::exp failed! omega: -nan -nan -nan
stereo-8 exit code -6
```

El fallo no venía de `orbslam3_multi` ni del detector BoW: se producía dentro de
`ORB_SLAM3`, antes de llamar a `Sophus::SO3f::exp`.

## Decisión

No se modificó Sophus. Se añadió una guarda mínima en
`ORB_SLAM3/src/Sim3Solver.cc` para evitar que una hipótesis Sim3 degenerada o no
finita llegue a `Sophus::SO3f::exp`.

Cambios relevantes:

- `Sim3Solver::ComputeSim3(...)` pasa de `void` a `bool`.
- Los bucles RANSAC descartan la muestra si `ComputeSim3(...)` devuelve `false`.
- Se comprueban entradas, centroides, matriz `M`, matriz `N`, autovalores,
  autovectores, rotación, escala, traslación y transformaciones finales con
  `allFinite()` / `std::isfinite()`.
- Si la parte imaginaria del cuaternión tiene norma casi cero, se usa vector de
  rotación cero. Esto preserva la rotación identidad válida y evita dividir por
  `vec.norm() == 0`.
- Si la escala libre tiene denominador no finito o casi cero, la hipótesis se
  rechaza.

## Riesgos

- ORB-SLAM3 puede descartar alguna hipótesis de relocalización o loop interno
  que antes intentaba evaluar aunque fuese numéricamente degenerada.
- En escenas pobres o repetitivas, podría tardar más en relocalizar o crear un
  mapa nuevo si demasiadas muestras degeneran.
- Si reaparece `SO3::exp failed`, buscar otro origen de NaN antes de Sophus; no
  asumir que esta guarda cubre todos los caminos.

## Validación

Fecha: 2026-07-21.

```text
cmake --build ORB_SLAM3/build -j4: código 0
build_selected_packages.sh orbslam3_multi orbslam3_server simulacion_dron: BUILD-EXIT-CODE 0
prueba_1 live: SCENARIO-RUNNER-DONE success=true, SIM-DONE success=true, SIM-EXIT-CODE 0
SO3::exp failed: 0 apariciones
exit code -6: 0 apariciones
process has died: 0 apariciones
undefined symbol: 0 apariciones
```

Evidencia funcional de la misma prueba:

```text
F1M-COVIS-SUMMARY final: confirmed_edges=6820 orbslam3_native=6820
F1N-LOOP-CANDIDATE: 1947
same_drone=false: 1056
same_drone=true: 891
F1N-BOW-SKIP-CONFIRMED-COVIS: 188
```

## Nota operativa

No compilar `ORB_SLAM3` con `build_selected_packages.sh ORB_SLAM3` sin limitar
paralelismo: colcon usó `-j16` y el sistema mató `cc1plus` por memoria. Para la
librería, usar:

```bash
cmake --build ORB_SLAM3/build -j4
```
