# Mapeado-MultiDrone-Orbslam

Proyecto ROS 2 + Gazebo + ORB-SLAM3 para mapeado multi-dron. El objetivo final es obtener una nube densa global sin usar Ground Truth como fuente funcional de pose final ni de mapa final.

## Estado

- Fase 1: realizada.
- Fase 3 sparse global: conseguida.
- Fase 2 separación Dron/Servidor/Simulación: fase activa, en cierre técnico.
- Fases 4 y 5: planificadas, todavía sin ejecutar.

La prueba 198 de dos drones alrededor del edificio fue ejecutada y validada funcional y visualmente por el usuario sobre el snapshot anterior a las correcciones finales de Fase 2. Tras aplicar esas correcciones se repetirá una regresión equivalente.

## Estructura de despliegue

```text
dron/       software embarcado por dron
servidor/   backend sparse/global y servidor ROS 2
simulacion/ Gazebo, escenarios, integración y observabilidad
codex/      documentación, herramientas y evidencias de trabajo
```

`orbslam3_msgs` existe de forma deliberada en Dron y Servidor; la copia de Servidor es canónica y la igualdad se protege automáticamente.

## Decisiones principales

- Dron es caja negra de despliegue.
- Configuración: distinguir ownership, autoridad y perfil de despliegue.
- No se cargan YAML directamente entre grupos.
- `global_map` Servidor↔Simulación es una réplica completa de deployment profile deliberada y guardada.
- `use_sim_time=false` en Dron/Servidor standalone; Simulación aplica `true`.
- GT de control es deuda legacy temporal hasta Fase 5; fiducial GT temporal hasta Fase 4.
- `pipeline_flow` y `system_architecture` son debug opcional y deben tener coste específico prácticamente nulo cuando están apagados.
- Los logs completos se conservan, pero Codex solo analiza reducidos/sublogs.

La fuente de verdad operativa para Codex comienza en `AGENTS.md` y `codex/contexto/00_CONTEXTO_COMPACTACION.md`.
