# AGENTS.md — Arranque compacto para Codex

## Estado de trabajo

- Fase activa: **Fase 2 — separación Dron/Servidor/Simulación**, actualmente en cierre técnico.
- Fase 3: **CONSEGUIDA**.
- Fases 4 y 5: **sin ejecutar**; solo se han refinado sus contratos documentales.
- La actualización documental no autoriza por sí sola cambios funcionales. Antes de implementar, releer este archivo, `codex/contexto/00_CONTEXTO_COMPACTACION.md` y el contrato de Fase 2.

## Puerta de continuidad obligatoria

Antes de actuar:

1. leer físicamente `codex/contexto/00_CONTEXTO_COMPACTACION.md`;
2. leer `codex/contexto/CONTEXTO_MINIMO_ACTUAL.md`;
3. leer `codex/pipeline/fase_2_separacion_paquetes/pipeline_fase_2_RESUMEN.md`;
4. abrir solo la subfase y documentación de paquetes necesaria;
5. reconciliar siempre documentación con el código real antes de modificar.

No considerar el resumen automático del chat como sustituto de esas lecturas.

## Idioma y estilo

- Responder y documentar en español.
- Mantener en inglés nombres de paquetes, archivos, símbolos, topics, services, actions, parámetros, logs y comandos.
- Leer resúmenes/índices antes de documentos largos.
- Localizar símbolos con `rg` y abrir rangos mínimos de código.

## Autorización y preparación

Una primera orden del tipo `haz la subfase X` no autoriza automáticamente modificaciones. Antes de código/YAML/launch/build/simulación:

1. explicar objetivo, alcance, archivos probables, riesgos, pruebas y criterios;
2. resolver dudas funcionales;
3. confirmar que el acuerdo sigue vigente;
4. recibir autorización posterior cuando corresponda.

Si durante la ejecución aparece una decisión funcional no acordada, suspender esa autorización y preguntar.

## Logs: regla absoluta

Los logs completos de build y simulación pueden generarse y conservarse en `codex/archivos_auxiliares/`, pero Codex **nunca los lee directamente**.

```text
log completo -> reductor -> reducido/sublog -> agente
```

Si falta evidencia, generar otro reducido/sublog con patrones más específicos. Nunca abrir el completo como fallback.

## Layout de artefactos

Los artefactos ROS/colcon viven fuera de `src/`:

```text
build/dron       install/dron       log/dron
build/servidor   install/servidor   log/servidor
build/simulacion install/simulacion log/simulacion
```

`codex/archivos_auxiliares/` es una excepción diagnóstica: puede contener logs, reducidos, replays, trayectorias, métricas y evidencias, pero ningún paquete funcional puede depender de esa ruta para funcionar.

## Invariantes técnicas actuales

- `submapa = (drone_id, map_epoch)`.
- `RawMapDatabase` conserva raw ORB-SLAM3 y no se modifica por optimización.
- `GlobalPoseStore` conserva estado global, anchors, optimizaciones y rollback.
- Fiduciales son observaciones absolutas, no loops.
- GT no forma parte de la arquitectura final. Existe una excepción legacy temporal: `gen_tray` y `control_calcular_fuerzas` consumen `sensor/GT/pose` y `sensor/GT/vel`; se retirará en Fase 5/5H y no autoriza nuevos consumidores.
- El fiducial simulado basado en GT se conserva provisionalmente hasta Fase 4.
- Dron se trata como caja negra de despliegue.
- Distinguir `semantic ownership`, `authority/control` y `deployment source/profile`.
- Carga directa de YAML entre grupos: prohibida.
- Réplicas parciales declaradas: permitidas con claves exactas.
- Réplicas completas declaradas: solo como perfil de despliegue justificado y guardado. `global_map` Servidor↔Simulación es la excepción vigente.
- Dron/Servidor standalone usan `use_sim_time=false`; Simulación hace override explícito `true`.

## Observabilidad web

`pipeline_flow` y `system_architecture` son herramientas de debug independientes.

Con el debug maestro de una herramienta en `false` debe quedar dormida: sin bridge, navegador, HTTP/SSE, observers, publishers específicos, construcción/serialización de eventos ni inspección de tráfico.

`system_architecture` usa paquetes como nodos principales, separa capas `runtime`, `build/API`, `config/replica` y `deployment`, y solo ilumina aristas runtime con evidencia directa. Eventos desconocidos no se asignan por aproximación.

Toda subfase futura que cambie paquetes, interfaces, relaciones cross-group, réplicas o deployment debe actualizar `system_architecture`, metadata, guardas/tests y telemetría live si aplica.

## Build y simulación

Usar las herramientas de `codex/herramientas/`. El entorno de simulación se prepara sin perfiles personales: ROS base -> `install/dron` -> `install/servidor` -> `install/simulacion`; la corrección acordada usa `bash -c`, no `bash -lc`.

## Índice compacto

```text
contexto_vivo: codex/contexto/00_CONTEXTO_COMPACTACION.md
contexto_minimo: codex/contexto/CONTEXTO_MINIMO_ACTUAL.md
estado_corto: codex/contexto/01_ESTADO_ACTUAL_RESUMEN.md
reglas: codex/contexto/02_REGLAS_TECNICAS.md
arquitectura: codex/contexto/03_ARQUITECTURA_ACTUAL.md
interfaces: codex/contexto/04_TOPICS_SERVICES_ACTIONS.md
paquetes: codex/contexto/05_MAPA_PAQUETES.md
herramientas: codex/herramientas/USO_HERRAMIENTAS.md
pipeline_actual: codex/pipeline/fase_2_separacion_paquetes/pipeline_fase_2_RESUMEN.md
subfases_actuales: codex/pipeline/fase_2_separacion_paquetes/subfases/
```
