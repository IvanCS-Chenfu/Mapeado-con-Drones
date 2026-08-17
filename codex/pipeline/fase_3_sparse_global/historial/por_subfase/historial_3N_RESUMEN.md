# Historial 3N - resumen

## Estado vigente

`CONSEGUIDA` el 2026-08-15. Cada KF materialmente elegible, anclado o no,
entra en una `LoopTask` BAJA causal.

## Estado actual

- `LoopPipeline` mantiene un indice BoW derivado y agrupa hasta tres regiones.
- La cola deduplica solo revisiones causales exactas y conserva como maximo un
  rerun nuevo durante una tarea activa.
- La cache negativa se invalida al cambiar apariencia, geometria o anchor.
- Query/candidate es simetrico y la tarea abarca BoW, geometria y decision.

## Evidencia vigente

- `test_loop_pipeline` y `test_secondary_queue` pasan dentro de 53/53 C++;
- replay 152 detecto geometria intra-submapa redundante y backlog;
- tras acotar subnubes a 320 puntos, RANSAC a 80 y usar covisibilidad fuerte,
  replay 153 termina `pending=0`, con loops finales de ~0.16-0.18 s;
- live 154 conserva queries no ancladas B/KF5 y B/KF7.

## Mejora abierta

Live 154 ejecuto 2301 `LoopTask` para 248 KFs. No hubo backlog final, pero la
relacion muestra reevaluacion causal excesiva y mantiene el subsistema/web
continuamente ocupado. Antes de 3P conviene distinguir cambios de apariencia o
geometria realmente relevantes de simples revisiones de soporte/covisibilidad.

## No repetir

No excluir KFs por carecer de pose world, no guardar BoW en covisibilidad y no
volver al limite silencioso de un unico candidato global.

Detalle: `historial_3N.md`.
