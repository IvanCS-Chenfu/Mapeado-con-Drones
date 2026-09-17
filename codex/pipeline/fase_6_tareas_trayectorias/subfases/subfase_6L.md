# Subfase 6L - Integracion, observabilidad y cierre de la migracion

## Estado

`PENDIENTE DE MIGRACION`.

## Objetivo

Validar la arquitectura desacoplada completa sin ocultar estados importantes
ni reintroducir esperas entre servidor y dron.

## Contrato funcional

Los logs de fase deben identificar `drone_id`, `task_id`, `workflow_id`,
`command_id`, `map_epoch`, cola de origen/destino, revision de mapa y motivo de
cualquier STOP. La GUI muestra el subROI, coverage, tramo de fachada asociado
a la tarea, reservas y solo la trayectoria que se esta ejecutando. No muestra
rayos depth ni telemetria masiva.

Cada simulacion parte de ROS/Gazebo/GUI limpios y se valida antes del launch
que no hay nodos de una prueba anterior. Los resultados se registran por
prueba, incluso si fallan, sin reescribir conclusiones anteriores.

## Pruebas requeridas

- Unitarias: FIFO, deduplicacion, IDs tardios, prioridad STOP, transiciones de
  workflow, base de evidencia, delete/reproyeccion de fuente y prefijo FREE.
- ROS: aceptacion inmediata de ambos servicios, ausencia de trabajo pesado en
  callbacks y una orden normal activa por dron.
- Gazebo D1: fiducial 2, asignacion, mirada/captura, aplicacion de evidence,
  planificacion, movimiento, captura de fachada y nueva eleccion.
- Gazebo D1+D2: dos subROIs, workers FIFO, corredores reservados concurrentes,
  conflicto de reserva, STOP y continuaciones independientes por drone.
- Fiducial oportunista: visto/no visto, aproximacion segura y salida sin
  progreso.

## Criterios de exito

Ningun worker espera la finalizacion de una accion remota. Un resultado del
dron solo encola la siguiente fase y `VoxelMapBuilder` libera continuaciones
despues de materializar. Los drones pueden progresar en paralelo y el mapa se
actualiza por deltas sin reconstrucciones completas.

## Exclusiones y trabajo posterior

La calibracion fina de scores, la topologia completa de ramas y la nube densa
global pertenecen a fases posteriores. La migracion no corrige de paso la
optimizacion por fiducial; se valida estabilidad inicial y se documentan sus
fallos por separado.
