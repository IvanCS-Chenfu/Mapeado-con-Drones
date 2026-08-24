# 00 — Leer primero

Para arrancar un chat nuevo o reanudar tras compactación:

```text
1. 00_CONTEXTO_COMPACTACION.md
2. CONTEXTO_MINIMO_ACTUAL.md
3. 01_ESTADO_ACTUAL_RESUMEN.md
4. 08_POLITICA_TOKENS_DOCUMENTACION.md
5. pipeline/fase_2_separacion_paquetes/pipeline_fase_2_RESUMEN.md
```

La fase activa es **Fase 2**. Fase 3 está conseguida y se consulta solo como historial/arquitectura previa cuando una duda concreta lo requiera.

## Reglas rápidas

- No abrir logs completos; reducir o crear sublogs.
- No usar GT para mapa/pose final; el consumo GT actual de control es una excepción legacy temporal hasta 5H.
- No refactorizar el fiducial actual durante el cierre de Fase 2; Fase 4 lo sustituye.
- No crear comunicación directa Servidor↔`dron_individual` en Fase 5; la frontera cross-group termina en `orbslam3`.
- Dron es caja negra; no cargar YAML de otro grupo.
- `system_architecture` debe seguir la arquitectura real y actualizarse en la misma subfase que cambie interfaces/paquetes/deployment.
- Todo debug web apagado debe dejar también dormida su instrumentación productora.
- Una primera orden de ejecutar una subfase inicia preparación, no ejecución automática.

## Historial y documentación

Para Fase 2 usar `codex/pipeline/fase_2_separacion_paquetes/`. Los historiales se crean/actualizan únicamente con ejecuciones reales; no inventar evidencia.
