# 02 — Reglas técnicas permanentes

## Identidad y autoridad de datos

- `submapa = (drone_id, map_epoch)`.
- `RawMapDatabase` conserva datos ORB-SLAM3 crudos y no se modifica por optimización/fusión global.
- `GlobalPoseStore` conserva anchors, poses world, poses optimizadas/propagadas y rollback.
- Fiduciales son observaciones absolutas, no loops.

## Ground Truth

Arquitectura objetivo: GT solo para fiducial simulado temporal, debug y métricas externas; nunca para mapa/pose final, loops, fusión, score o nube densa.

Excepción legacy actual y acotada: `gen_tray` y `control_calcular_fuerzas` consumen `sensor/GT/pose` y `sensor/GT/vel`. No se autoriza ampliar ese uso. Se retira funcionalmente en Fase 5/5H. El fiducial GT se sustituye en Fase 4.

## Grupos y configuración

Distinguir siempre:

1. **semantic ownership**: grupo propietario conceptual;
2. **authority/control**: quién decide el valor;
3. **deployment source/profile**: copia local que carga el despliegue.

Dron es caja negra. Un YAML solo configura al nodo que lo carga; no distribuye configuración remotamente.

### Carga entre grupos

No cargar directamente YAML de otro grupo. Cada deployment usa recursos instalados en su propio grupo.

### Réplicas

- duplicado accidental o semántico: prohibido;
- réplica parcial declarada: permitida con claves exactas, origen, consumidores y regla de igualdad;
- réplica completa declarada: permitida únicamente como deployment profile justificado y con guarda.

Caso vigente: `orbslam3_server/config/global_map/` y `simulacion_dron/config/global_map/` son dos copias completas deliberadas del mismo perfil validado.

### Contratos futuros

- valor controlado por Servidor y consumido por Dron: `Dron startup client -> servicio de configuración Servidor -> valor local Dron`;
- intrínseco Dron requerido por Servidor (`body_T_camera`): TF/contrato de calibración Dron→Servidor.

No implementar esos transportes en Fase 2 si no son necesarios para el comportamiento actual.

## Tiempo

- Dron standalone/real: `use_sim_time=false` por defecto.
- Servidor standalone/real: `use_sim_time=false` por defecto.
- Simulación: override explícito `use_sim_time=true` a nodos que dependan de `/clock`.

Identidad y valores por ejecución deben tener una única autoridad visible en launch.

## Observabilidad

Los grafos son diagnóstico, nunca dependencia funcional.

Debug maestro false implica herramienta dormida: sin bridge, HTTP/SSE, navegador, observers, publishers específicos, construcción/serialización de eventos ni inspección de tráfico.

`pipeline_flow` y `system_architecture` son independientes. `system_architecture` no usa `/global_mapping/flow_events` como bus universal.

`system_architecture`:

- nodos principales = paquetes;
- capas = runtime, build/API, config/replica, deployment;
- solo runtime puede iluminarse;
- actividad = evidencia directa/evento semántico explícito;
- evento desconocido = ninguna arista;
- payload de telemetría = metadata ligera, nunca Image/PointCloud2/OrbMap completo.

Toda subfase futura que cambie arquitectura actualiza el grafo y sus guardas/tests en la misma subfase.

## Artefactos y logs

ROS/colcon:

```text
build/<grupo>
install/<grupo>
log/<grupo>
```

`codex/archivos_auxiliares` puede contener evidencias diagnósticas, pero ningún paquete funcional depende de él.

Logs completos: solo entrada de reductores. Codex analiza únicamente reducidos/sublogs y, si falta contexto, genera otro reducido.
