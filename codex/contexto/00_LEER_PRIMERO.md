# 00 — Leer primero

Este archivo queda como índice corto. Para arrancar un chat nuevo con bajo coste
de tokens, leer en este orden:

```text
codex/contexto/00_CONTEXTO_COMPACTACION.md
codex/contexto/CONTEXTO_MINIMO_ACTUAL.md
codex/contexto/00_BOOTSTRAP_MINIMO.md
codex/contexto/01_ESTADO_ACTUAL_RESUMEN.md
codex/contexto/08_POLITICA_TOKENS_DOCUMENTACION.md
```

La primera lectura debe hacerse físicamente mediante una herramienta. Tras una
compactación hay que repetirla antes de continuar; el resumen automático del
chat no sustituye `00_CONTEXTO_COMPACTACION.md`.

La documentación extensa sigue existiendo, pero no debe abrirse por defecto.

## Fuente de verdad operativa

| Necesidad | Leer |
|---|---|
| Memoria viva y reanudación | `00_CONTEXTO_COMPACTACION.md` |
| Estado corto | `01_ESTADO_ACTUAL_RESUMEN.md` |
| Contexto mínimo | `CONTEXTO_MINIMO_ACTUAL.md` |
| Estado completo | `01_ESTADO_ACTUAL.md` |
| Reglas técnicas | `02_REGLAS_TECNICAS.md` |
| Arquitectura | `03_ARQUITECTURA_ACTUAL.md` |
| Topics/services/actions | `04_TOPICS_SERVICES_ACTIONS.md` |
| Paquetes | `05_MAPA_PAQUETES.md` y docs del paquete afectado |
| Código | `06_MAPA_CODIGO.md` |
| Última sesión | `07_ULTIMA_SESION.md` |
| Política antitokens | `08_POLITICA_TOKENS_DOCUMENTACION.md` |
| Logs/sublogs | `09_LOGS_Y_SUBLOGS.md` |
| Historial | `codex/pipeline/fase_3_sparse_global/historial/INDEX.md` y luego `historial_<ID>_RESUMEN.md` |

## Fase activa

```text
Fase 3 — Mapa sparse global multi-dron
3B — CONSEGUIDA: runtime vacío y grafo base confirmados en prueba 78.
3C — CONSEGUIDA; 3D — PARCIAL pendiente de confirmación visual; 3E-3U/3W — REHACER; 3V-3X pendientes.
Fase 2 — separación de paquetes queda pendiente para después del cierre de Fase 3.
Punto de reentrada — incorporar la observación visual del usuario sobre prueba
88 y decidir el cierre definitivo de 3D.
```

## Reglas rápidas

- No empezar desde cero.
- No usar subfases legacy `12R-*`, `13`, `14` o `15` como plan activo.
- No modificar `ORB_SLAM3` salvo permiso explícito.
- No rediseñar `orbslam3_msgs` ni `orbslam3_ros2` salvo necesidad fuerte.
- No usar ground truth para mapa final, loops, fusión, score, pose final ni nube
  densa.
- El GT solo puede usarse para fiducial simulado, debug o métricas externas.
- Identificar submapas como `(drone_id, map_epoch)`.
- Leer resúmenes e índices antes de abrir MDs grandes.
- En tareas largas, actualizar `00_CONTEXTO_COMPACTACION.md` tras plan, cambios,
  build, prueba y diagnóstico; no aplazarlo al cierre.
- Una primera orden de ejecutar una subfase nunca autoriza actuar: preparar,
  explicar y comprobar comprensión mutua antes de modificar o probar.
- No repetir preguntas si existe un acuerdo previo completo, confirmado y sin
  dudas en `00_CONTEXTO_COMPACTACION.md`.
- La autorización solo cubre ese acuerdo; cualquier duda funcional o cambio de
  alcance/prueba la suspende hasta una nueva confirmación del usuario.
- **Subfases grandes** (`3O`, `3P`, `3Q`, `3S`) se dividen en subarchivos temáticos cuando aplica:
  leer primero `subfase_<ID>.md` (índice breve), luego abrir el subarchivo temático
  necesario (`especificacion`, `implementacion`, `testing` o `criterios`).

## Historial

Para trabajar en una subfase, usar:

```text
codex/pipeline/fase_3_sparse_global/historial/por_subfase/historial_<ID>_RESUMEN.md
```

Abrir `historial_<ID>.md` solo si falta evidencia concreta. Si se modifica una
subfase, añadir la entrada al historial largo y actualizar tambien
`historial_<ID>_RESUMEN.md` e `historial/INDEX.md` si cambia el estado.

## Después de validar una subfase

Actualizar solo lo necesario:

- `00_CONTEXTO_COMPACTACION.md` durante cada hito y al cierre;
- `01_ESTADO_ACTUAL_RESUMEN.md` si cambia el punto de entrada;
- `01_ESTADO_ACTUAL.md` si cambia el estado completo;
- docs del paquete afectado;
- historial por subfase;
- resumen `historial_<ID>_RESUMEN.md`;
- `07_ULTIMA_SESION.md`;
- `pipeline_fase_3.md` o subfase solo si cambia el plan.

`07_ULTIMA_SESION.md` se reemplaza en cada cierre. No acumular sesiones antiguas
ahí: para conservar detalle anterior está el historial por subfase.

No duplicar la misma evidencia larga en varios archivos.
