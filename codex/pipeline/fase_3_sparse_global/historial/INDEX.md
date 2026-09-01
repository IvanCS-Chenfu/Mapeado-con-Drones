# Índice de historial — Fase 3

Leer este índice antes de abrir historiales. Para una subfase o tema, abrir
primero `por_subfase/historial_<ID>_RESUMEN.md`. Abrir el historial largo o sus
fragmentos solo si falta evidencia concreta.

## Estado rápido

| Subfase/tema | Archivo | Estado útil |
|---|---|---|
| Legacy `12R-D4` | `por_subfase/historial_12R-D4_RESUMEN.md` | Referencia histórica, no planificación activa. |
| General/planificación | `por_subfase/historial_general_RESUMEN.md` | Acuerdo transversal del 2026-08-05 y diagnostico del 2026-08-09: independencia temporal y visualizador reabiertos. |
| Infraestructura | `por_subfase/historial_infraestructura_RESUMEN.md` | Reglas Codex; incluye compactación, aprobación, conclusiones vivas y prohibición de leer logs completos. |
| Pruebas típicas | `por_subfase/historial_pruebas_tipicas_RESUMEN.md` | Rodeo largo; 75/76 documentan backlog, loops pre-anchor, latencia RViz2 y ausencia variable de `fid=1`. |
| `3A` | `por_subfase/historial_3A_RESUMEN.md` | Baseline conseguida. |
| `3B` | `por_subfase/historial_3B_RESUMEN.md` | `CONSEGUIDA`: runtime/grafo base y apertura integrada desde launch validados. |
| `3C` | `por_subfase/historial_3C_RESUMEN.md` | `CONSEGUIDA`: pruebas 85/86 validan raw, FIFO/worker, replay, backpressure 8/2, web 6/5 y RViz2 sin publishers. |
| `3D` | `por_subfase/historial_3D_RESUMEN.md` | `CONSEGUIDA`: backend/poses y grafo 7/6 validados técnica y visualmente. |
| `3E` | `por_subfase/historial_3E_RESUMEN.md` | `CONSEGUIDA`: 2 anchors/61 poses/2 hard y replay equivalentes; usuario acepta el pulso breve sin pedir cambios. |
| `3F` | `por_subfase/historial_3F_RESUMEN.md` | `CONSEGUIDA`: color por epoch confirmado visualmente en live 151. |
| `3G` | `por_subfase/historial_3G_RESUMEN.md` | `CONSEGUIDA`: prueba 141 completa con 4.83 GiB minimos disponibles, PSI 0 y RViz2/grafo web confirmados por el usuario. |
| `3H` | `por_subfase/historial_3H_RESUMEN.md` | `CONSEGUIDA`: MAX/STALE/lifecycle confirmados en live 151. |
| `3I` | `por_subfase/historial_3I_RESUMEN.md` | `CONSEGUIDA`: grafo 30/20 mono-submapa validado; el parpadeo no nace aqui. |
| `3J` | `por_subfase/historial_3J_RESUMEN.md` | `CONSEGUIDA`: solver SE(3) lleva target a cero; fallo posterior pertenece a 3K. |
| `3K` | `por_subfase/historial_3K_RESUMEN.md` | `CONSEGUIDA`: continuidad futura tecnica y visual validada en live 151. |
| `3L` | `por_subfase/historial_3L_RESUMEN.md` | `CONSEGUIDA`: live 151 cerrada tras conservar fallo 148. |
| `3M` | `por_subfase/historial_3M_RESUMEN.md` | `CONSEGUIDA`: patch MEDIA canonico y encadenado causal a loops. |
| `3N` | `por_subfase/historial_3N_RESUMEN.md` | `CONSEGUIDA`: indice BoW, regiones y ledger causal con cola drenada. |
| `3O` | `por_subfase/historial_3O_RESUMEN.md` | `CONSEGUIDA`: reanchor, propagacion, carga y validacion RViz2/web completados. |
| `3P` | `por_subfase/historial_3P_RESUMEN.md` | `CONSEGUIDA`: prueba 161 y cierre del usuario; queda pulido de layout web. |
| `3Q` | `por_subfase/historial_3Q_RESUMEN.md` | `A REVISAR`: 373 no ejecuta optimizaciones de cierre esperadas desde marker 368; aplazado por el usuario. |
| `3R` | `por_subfase/historial_3R_RESUMEN.md` | `CONSEGUIDA`: scoring 1-5 m y scores visuales confirmados; antes denominada 3S. |
| `3S` | `por_subfase/historial_3S_RESUMEN.md` | `CONSEGUIDA`: perfil silencioso validado por prueba 196, sin RViz2/web ni marcadores F3. |
| `3T` | `por_subfase/historial_3T_RESUMEN.md` | `CONSEGUIDA`: limpieza, renumeracion y handoff final; antes denominada 3X. |

Los historiales de auditorias transversales absorbidas se conservan en
`historial/absorbidas/`; no son subfases activas de la numeracion final.

## Cómo añadir historial nuevo

1. Añadir la entrada al final de `por_subfase/historial_<ID>.md`.
2. Actualizar `por_subfase/historial_<ID>_RESUMEN.md` con el estado operativo,
   evidencia clave, aprendizaje o pendiente nuevo.
3. Actualizar esta tabla si cambia estado o hay un aprendizaje clave.
4. No copiar la misma evidencia larga en `01_ESTADO_ACTUAL.md`,
   `07_ULTIMA_SESION.md` y docs de paquete.
5. No recrear un historial monolítico; el detalle vive en `por_subfase/`.

Formato mínimo de entrada:

```text
## YYYY-MM-DD HH:MM — Subfase <ID> — título corto

- objetivo intentado:
- archivos modificados:
- paquetes compilados:
- resultado de build:
- pruebas Gazebo/replay:
- patrones de reducción:
- evidencia positiva:
- evidencia negativa o ausente:
- conclusión:
- siguiente paso recomendado:
```
