# Contexto mínimo actual

Precondición: leer físicamente `00_CONTEXTO_COMPACTACION.md`.

## Estado

```text
Fase actual: Fase 2 — separación servidor/dron/simulación, en cierre
Fase 3: CONSEGUIDA
Correcciones finales Fase 2: acordadas y documentadas
Implementación de esas correcciones: pendiente de autorización
Prueba 198: PASADA sobre snapshot previo a las correcciones
```

## Arquitectura relevante hoy

- Simulación alimenta cámaras estéreo al wrapper `orbslam3`.
- El wrapper publica mapa ORB hacia Servidor.
- `dron_individual` todavía consume GT de pose/velocidad para trayectoria/control: deuda provisional hasta Fase 5.
- Servidor todavía usa GT para fiducial simulado: deuda provisional hasta Fase 4.
- Dron manda comandos de motor a Simulación.
- `pipeline_flow` observa el pipeline interno sparse/global.
- `system_architecture` representa paquetes y relaciones del sistema y debe corregirse para usar evidencia directa y telemetría propia.

## Reglas de cierre

- Dron caja negra; no YAML cross-group.
- Ownership, authority y deployment profile son conceptos distintos.
- `global_map` es réplica completa deliberada Servidor↔Simulación.
- Standalone Dron/Servidor `use_sim_time=false`; Simulación `true`.
- ORBvoc completo con bootstrap/preflight; L5 no sustituye silenciosamente al normal.
- Debugs web apagados = también productores dormidos.
- Logs completos nunca se leen directamente.

## Lectura siguiente

```text
codex/pipeline/fase_2_separacion_paquetes/pipeline_fase_2_RESUMEN.md
codex/pipeline/fase_2_separacion_paquetes/subfases/
codex/contexto/decisiones/ADR_0009_configuracion_por_dominio_y_despliegue.md
codex/contexto/decisiones/ADR_0010_observabilidad_web_debug_coste_cero.md
```
