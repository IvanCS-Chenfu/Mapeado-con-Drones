# ADR 0010 — Observabilidad web opcional y coste específico nulo cuando está desactivada

## Estado

Aceptada durante el cierre documental de Fase 2 (2026-08-24).

## Contexto

El proyecto dispone de dos visualizadores web diagnósticos:

- `pipeline_flow`: detalle interno del pipeline sparse/global de Fase 3;
- `system_architecture`: paquetes, grupos, interfaces y relaciones del sistema completo.

Su objetivo es ayudar a depurar. No forman parte del camino funcional de control, SLAM,
mapa ni misión. Una implementación que únicamente oculte el navegador pero siga
mirando tráfico, construyendo JSON o publicando telemetría consume recursos sin aportar
valor cuando el debug está apagado.

La auditoría de Fase 2 detectó además que `pipeline_flow_bridge` se apaga correctamente
cuando su web está desactivada, pero `GlobalMapServer` continúa produciendo
`/global_mapping/flow_events`. Esa deuda debe corregirse.

## Decisión

### 1. Independencia

`pipeline_flow` y `system_architecture` son herramientas independientes. Activar una no
activa ni requiere la instrumentación de la otra.

### 2. Debug apagado

Cuando el debug maestro de una herramienta está en `false`, no se realiza trabajo
específico de esa herramienta:

```text
sin bridge
sin servidor HTTP/SSE
sin navegador
sin subscriptions/observers de debug
sin publishers de debug
sin construcción/serialización de eventos
sin inspección de tráfico
```

Se admite únicamente el coste trivial de comprobar un booleano cuando sea inevitable.

### 3. Pipeline flow

`debug_pipeline_flow_web=false` debe desactivar también la generación de
`/global_mapping/flow_events`, no solo el bridge. Los productores deben cortar el
trabajo antes de construir strings/JSON caros.

### 4. System architecture

Matriz:

```text
web=false                         -> completamente dormido
web=true, telemetry=false         -> grafo estático
web=true, telemetry=true          -> grafo estático + live
open_browser                      -> solo apertura automática
```

`telemetry=true` no puede saltarse un `web=false` maestro.

### 5. Semántica del grafo

Los nodos principales son paquetes. Las aristas se clasifican en:

- runtime;
- build/API;
- config/replica;
- deployment.

Solo runtime se ilumina. La actividad requiere evidencia directa o un evento semántico
explícito. Un evento desconocido no se asigna por aproximación.

### 6. Telemetría ligera

No duplicar imágenes, nubes, mapas ni payloads pesados. Los eventos contienen metadata
mínima como `edge_id`, `drone_id`, timestamp, interfaz, contador/estado. Se permite
sampling/coalescing.

### 7. Evolución futura

Toda subfase que cambie arquitectura debe actualizar topología, metadata, actividad y
tests/guardas de `system_architecture` en la misma subfase.

## Consecuencias

- los debugs apagados reducen realmente consumo;
- las herramientas no gobiernan el pipeline;
- no se confunde conexión declarada con tráfico observado;
- el grafo permanece útil a medida que Fases 4–9 cambien interfaces y paquetes.

## Pruebas mínimas

1. todos los debugs false: ausencia de procesos y de producción específica de eventos;
2. cada visualizador activado por separado;
3. `system_architecture` estático sin telemetría;
4. live con varios namespaces;
5. cerrar bridge/navegador sin afectar al sistema.
