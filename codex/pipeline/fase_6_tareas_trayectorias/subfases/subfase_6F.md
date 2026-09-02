# Subfase 6F - Base autonoma de task_manager

## Estado

```text
sin hacer
```

## Dependencia

6C, 6E y contratos vigentes de `NavigationState`, depth, fiduciales y
`TrayAction` tras Fases 1J/4/5.

## Objetivo tecnico

Crear `task_manager` y `task_manager_lib` por dron para conectar mision con
ejecucion, manteniendo `dron_individual` libre de estrategia global.

## Responsabilidades

`task_manager` gestiona ROS, registro, planes, feedback y cliente de
`TrayAction`. `task_manager_lib` contiene state machine local, validaciones,
safety/riesgo y subobjetivos. `dron_individual` genera/ejecuta referencias y
maniobras fisicas; no decide mision, coverage ni ruta global.

Estado minimo: task/plan/trajectory IDs y revisions, pose/velocidad, epoch,
alineamiento, tracking/risk, depth, fiduciales, anchor y progreso.

Flujo normal: registrar -> recibir plan validado -> comprobar version/contexto
-> convertir W/O cuando exista 6I -> ejecutar con `dron_individual` -> reportar.
Flujo excepcional: depth emergency, tracking risk/lost, mismatch, imposibilidad
local, fiducial de interes y progreso/fin.

El fast path de safety nunca espera voxelizacion, red ni servidor.

## Cambios requeridos

1. Crear ambos paquetes y lifecycle local testeable.
2. Implementar handshake 6C y recepcion/rechazo basico de tarea/plan.
3. Conectar estado real de F5 sin GT funcional.
4. Preparar cliente de ejecucion conservando `task_id != trajectory_id`.
5. Separar fast safety path de communications/mission path.
6. Publicar estado/eventos para servidor, GUI y grafo sin alta frecuencia inutil.

## Limites

No recalcular waypoints globales, no implementar aun STOP/VISUAL_RETREAT
completos ni multi-waypoint, y no controlar Gazebo directamente.

## Pruebas

- State machine, registro, asignacion, ack/reject y excepciones sinteticas.
- Caida/reinicio de servidor sin bloquear safety/control.
- Plan simple extremo a extremo solo si el contrato vigente ya lo permite;
  cualquier ampliacion fisica queda para 6I/6L.
- Integracion con GUI+Gazebo+grafo sin RViz y sin GT como autoridad.

## Criterio de exito

El agente se registra, conserva lifecycle coherente, recibe/rechaza ordenes de
forma segura y mantiene separadas estrategia, tactica y control fisico.
