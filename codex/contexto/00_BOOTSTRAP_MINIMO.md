# 00 — Bootstrap mínimo para nuevo chat

Este archivo queda como respaldo corto. Antes de leerlo, abrir físicamente:

```text
codex/contexto/00_CONTEXTO_COMPACTACION.md
```

Tras una compactación hay que repetir esa lectura antes de continuar. El resumen
automático del chat no la sustituye.

## Lectura recomendada

1. `codex/contexto/00_CONTEXTO_COMPACTACION.md`
2. `codex/contexto/CONTEXTO_MINIMO_ACTUAL.md`
3. `codex/contexto/08_POLITICA_TOKENS_DOCUMENTACION.md`
4. `codex/pipeline/fase_3_sparse_global/pipeline_fase_3_RESUMEN.md`
5. Subfase concreta si se va a implementar.
6. `historial/INDEX.md` y `historial_<ID>_RESUMEN.md` de la subfase necesaria.
7. Docs del paquete/componente que se vaya a tocar.

## Estado operativo

- Fase activa: Fase 3 — mapa sparse global multi-dron.
- Fase 1 — control del dron está documentada como `realizado`.
- Fase 2 — separación de paquetes queda pendiente hasta cerrar Fase 3.
- `3B-3P` y `3S-3W` estan `CONSEGUIDAS`.
- `3Q` queda `A REVISAR`, aceptada para continuar y con punto de reentrada
  obligatorio en su historial; `3X` sigue pendiente.
- Objetivo global del proyecto: nube densa global sin usar ground truth para mapa
  final ni pose final.

## Invariantes que no se negocian

- `submapa = (drone_id, map_epoch)`.
- `RawMapDatabase` conserva raw ORB-SLAM3 y no se modifica por optimización.
- `GlobalPoseStore` conserva poses globales, anchors, optimizaciones y rollback.
- Fiduciales son observaciones absolutas, no loops.
- Ground truth solo para fiducial simulado, debug o métricas externas.
- Wrapper y mensajes son estables.
- `ORB_SLAM3` no se toca salvo permiso explícito; si reaparecen errores de la
  librería, leer primero
  `codex/contexto/paquetes/ORB_SLAM3/sim3_solver_guard.md`.
- Una primera orden de ejecutar una subfase nunca autoriza código, configuración,
  build ni simulación: primero debatir y comprobar comprensión mutua.
- No repetir preguntas si `00_CONTEXTO_COMPACTACION.md` conserva un acuerdo
  previo completo, confirmado y sin dudas.
- La autorización se limita al acuerdo; una duda funcional o cambio de
  alcance/prueba obliga a suspender, parar y pedir nueva confirmación.
- En tareas largas, reemplazar `00_CONTEXTO_COMPACTACION.md` tras plan, cambios,
  build, prueba y diagnóstico, y cerrarlo al terminar.

## Ruta actual de la arquitectura

```text
orbslam3_ros2
  -> orbslam3_server
  -> orbslam3_multi
      RawMapDatabase
      GlobalPoseStore
      FiducialAnchorManager
      CovisibilityDatabase
      LoopDetector
      SubcloudLoopVerifier
      PoseGraphBuilder
      OptimizationManager
      GlobalMapBuilder
```

`orbslam3_server` debe ser adaptador ROS 2; `orbslam3_multi` debe concentrar la
lógica algorítmica.

## Herramientas

```bash
./codex/herramientas/build_selected_packages.sh <paquetes>
./codex/herramientas/reduce_build_log.sh
./codex/herramientas/run_simulation.sh --prueba X --launch "ros2 launch simulacion_dron multi_dron.launch.py"
./codex/herramientas/reduce_simulation_log.sh --prueba X --patterns "<patrones>"
```

Los logs completos nunca se leen directamente. Reducir primero y leer solo el
reducido; si falta evidencia, regenerarlo con otros patrones o crear sublogs.

## Dónde está el detalle

| Necesidad | Archivo |
|---|---|
| Memoria viva | `codex/contexto/00_CONTEXTO_COMPACTACION.md` |
| Estado completo | `codex/contexto/01_ESTADO_ACTUAL.md` |
| Estado corto | `codex/contexto/01_ESTADO_ACTUAL_RESUMEN.md` |
| Contexto mínimo | `codex/contexto/CONTEXTO_MINIMO_ACTUAL.md` |
| Reglas técnicas | `codex/contexto/02_REGLAS_TECNICAS.md` |
| Mapa de código | `codex/contexto/06_MAPA_CODIGO.md` |
| Historial | `codex/pipeline/fase_3_sparse_global/historial/INDEX.md` y `historial_<ID>_RESUMEN.md` |
| Política antitokens | `codex/contexto/08_POLITICA_TOKENS_DOCUMENTACION.md` |
