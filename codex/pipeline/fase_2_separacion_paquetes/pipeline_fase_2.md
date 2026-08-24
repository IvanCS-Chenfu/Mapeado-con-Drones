# Pipeline Fase 2 — Separación de paquetes, configuración y despliegue

Resumen de entrada: `pipeline_fase_2_RESUMEN.md`.

## Estado

La Fase 2 está **en cierre técnico**. La separación y builds ya fueron probados; quedan las correcciones finales documentadas y una regresión post-cambios. Este documento no autoriza por sí solo la implementación.

## Arquitectura de grupos

```text
Dron       -> software embarcado; no depende de Servidor/Simulación
Servidor   -> backend global; no depende de Dron/Simulación
Simulación -> integra explícitamente ambos grupos
```

`orbslam3_msgs` existe en Dron y Servidor. Servidor es canónico y la réplica Dron debe permanecer contractualmente idéntica.

## Build

Usar bases de workspace:

```text
build/{dron,servidor,simulacion}
install/{dron,servidor,simulacion}
log/{dron,servidor,simulacion}
```

Preferir un paquete por invocación y orden topológico. Dron y Servidor deben compilarse en entornos que no hereden el prefijo del otro. Simulación se construye como overlay de ambos.

## Configuración

### Modelo

Distinguir:

1. ownership semántico;
2. autoridad/control;
3. deployment source/profile.

Dron es caja negra. Ningún launch/nodo abre directamente un YAML de otro grupo.

### Réplicas

- accidental/semánticamente duplicada: prohibida;
- parcial declarada: contiene solo claves consumidas por el grupo receptor y documenta origen/consumidor/igualdad;
- completa declarada: solo como deployment profile justificado y guardado.

`global_map` Servidor↔Simulación es una réplica completa deliberada. No convertir esta excepción en una regla general de duplicación.

Ejemplos de réplicas parciales de Dron en Simulación cuando realmente se consuman: `physical_dron.yaml`, `camera_dron.yaml`, `actuators_dron.yaml`. `actuadores.conversor.fuerza2torque` es un caso de réplica parcial declarada.

### Reloj

- Dron standalone: `use_sim_time=false`.
- Servidor standalone: `use_sim_time=false`.
- Simulación: `use_sim_time=true` explícito.

Identidad y valores por ejecución tienen una única autoridad visible en launch.

### Deudas concretas de cierre

- retirar `usar_veltrap` tras `rg` global; `TrayAction.tipo_trayectoria` manda;
- Xacro: masa del cuerpo desde configuración, no literal `1.0`;
- ORBvoc completo con bootstrap/preflight reproducible;
- mantener `body_T_camera` como intrínseco Dron y documentar transporte futuro;
- no cambiar fiducial funcional en Fase 2;
- no retirar GT funcional de control en Fase 2.

## Artefactos y resultados

Los paquetes funcionales no dependen de `src/codex` ni escriben artefactos ROS dentro de `src/`. `codex/archivos_auxiliares/` sí puede guardar evidencia diagnóstica de Codex (incluido log completo), siempre sin convertirse en dependencia runtime.

## Observabilidad

### pipeline_flow

Herramienta interna del pipeline sparse. `debug_pipeline_flow_web=false` debe apagar bridge/web/navegador **y también** producción de `/global_mapping/flow_events`, construcción de JSON y cualquier trabajo específico.

### system_architecture

Herramienta separada. Nodos principales = paquetes. Capas:

```text
runtime
build/API
config/replica
deployment
```

Solo runtime puede iluminarse y únicamente con evidencia directa/evento semántico explícito. Evento desconocido no se asigna. No suscribirse a Image/PointCloud2/OrbMap pesado para detectar tráfico; usar telemetría ligera propia.

Matriz:

```text
web=false                         -> completamente dormido
web=true, telemetry=false         -> grafo estático
web=true, telemetry=true          -> grafo + live
open_browser                      -> solo apertura automática
```

### Topología actual a representar

- cámaras Simulación -> `orbslam3`;
- GT Simulación -> `dron_individual` provisional hasta Fase 5;
- GT -> Servidor provisional para fiducial hasta Fase 4;
- `OrbMap` delta wrapper -> Servidor;
- `GetOrbMap` con request Servidor->wrapper y response wrapper->Servidor;
- motores Dron -> Simulación;
- `AccionTrayectoria` runner/GUI -> `dron_individual`;
- backpressure/cloud/keyframes y demás interfaces reales según productores/consumidores;
- no mostrar ORB->control como activo mientras control use GT.

## Logs y entorno

Codex nunca lee logs completos. Reducir o crear sublogs. La ejecución prepara ROS base + overlays Dron/Servidor/Simulación y no debe reinyectar perfiles personales; usar `bash -c` tras la corrección.

## Subfases

- 2A: grupos y movimientos.
- 2B: builds aislados.
- 2C: configuración/ownership/launch/recursos.
- 2D: validación funcional y prueba larga.
- 2E: documentación/contexto.
- 2F: `system_architecture` estático/live.
- 2G: guardas, regresión y cierre.

## Puerta de cierre

1. guardas estáticas;
2. builds aislados;
3. tests;
4. smoke con todos los debug false, comprobando también productores;
5. `pipeline_flow` y `system_architecture` activados por separado;
6. regresión equivalente a prueba 198;
7. guardas finales;
8. documentación/historial actualizado.

Fase 2 solo se cierra cuando la implementación real, los MD y `system_architecture` coinciden.
