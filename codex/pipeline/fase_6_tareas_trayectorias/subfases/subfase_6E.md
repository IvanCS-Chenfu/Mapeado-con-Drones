# Subfase 6E - Gestor y asignador global de tareas

## Estado

```text
sin hacer
```

## Dependencia

6B, 6C y estado/pose global publica posterior a Fase 5. Puede empezar con mapa
pobre; mejora su coste cuando 6D/6G esten disponibles.

## Objetivo tecnico

Crear en `task_lib` el nucleo autoritativo de mision y en `task_server` el
`TaskWorker`: tareas regionales, lifecycle, drones, prioridades y asignacion.

## Comportamiento esperado

Al iniciar se crean cuatro `MAP_SECTION` por nivel. La unidad asignada es una
responsabilidad regional; no contiene A-B-C ni sentido de entrada fijo.

El coste puede usar ruta/distancia conocida, cambio de altura, carga,
continuidad regional, anclaje, accesibilidad y calidad de acceso. Con mapa pobre
se permite aproximacion simple determinista; tras 6G se prefiere coste de ruta
navegable. Los niveles no forman barreras.

Estados y ownership tienen una unica autoridad. `PAUSED` conserva tarea y ramas.
Un intento fallido no completa ni cierra automaticamente una region; se evita
repetir indefinidamente el mismo intento y se permite reasignacion.

`GO_TO` puede existir como pendiente de alta prioridad, pero no preempta una
tarea `RUNNING`; su ejecucion completa pertenece a 6N.

## Cambios requeridos

1. Implementar registro por `task_id`, transiciones atomicas y validacion de dron.
2. Crear todas las tareas base antes de asignar y conservar resultados.
3. Implementar asignador determinista regional, sin puntos A/C.
4. Gestionar idle/busy, prioridades, fallo, pausa, reanudacion y reasignacion.
5. Separar tarea regional del target interno que elegira `PlanningWorker`.
6. Definir completion de mision sin convertir FAILED en COMPLETED.
7. Exponer snapshot transient-local para GUI 7I y eventos agregados del grafo.

## Limites

No ejecutar planes, no implementar D* ni frontiers y no usar GT. No declarar
inaccesible una region por un unico intento.

## Pruebas

- Un/multiples drones y tareas; mas drones que tareas y viceversa.
- Transiciones duplicadas/reordenadas, dron no registrado/no anclado.
- Asignacion determinista entre niveles y continuidad sin FIFO rigido.
- PAUSED conserva ownership; fallo permite alternativa sin retry infinito.
- GO_TO pendiente no preempta RUNNING.
- Integracion GUI+Gazebo+grafo con lifecycle sintetico, sin movimiento fisico.

## Criterio de exito

No hay dobles asignaciones ni estados imposibles; el coste es testeable y
regional; mission/task snapshots permiten a GUI mostrar estado real.
