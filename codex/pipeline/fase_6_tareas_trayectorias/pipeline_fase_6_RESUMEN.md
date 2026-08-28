# Resumen — Fase 6: Tareas y Trayectorias

## Estado

```text
sin hacer
Preparación documental: cerrada
Autorización de ejecución: pendiente
Historial: vacío; no existen ejecuciones reales en este ZIP
```

## Objetivo

A partir de `tarea_principal.yaml`, el servidor debe generar y repartir una misión de mapeo sparse de todo lo accesible dentro de un ROI. Cada dron ejecuta la cobertura adaptándose a geometría desconocida, evitando obstáculos y manteniendo tracking; el servidor coordina exclusivamente Dron-Dron mediante reservas espaciales de trayectorias cortas.

## Decisiones cerradas

- `mapping_roi`: objetivo de reconstrucción, no zona de vuelo.
- `flight_bounds`: límite duro independiente para no alejarse demasiado.
- Ambos cuboides son axis-aligned en `world` (`min/max`).
- `mapping_hysteresis`: permite seguir ligeramente geometría fuera del ROI sin convertirla en objetivo obligatorio.
- La altura se divide por `level_height`; el resto se suma a la última rebanada.
- El dron trabaja cerca del centro de la banda pero puede variar `z` por seguridad/percepción.
- No se exige terminar un nivel antes de empezar otro; drones sobrantes suben.
- `tasks_per_level` se mantiene; baseline: 4 esquinas o 8 puntos (esquinas + centros de lado).
- Para B: tarea nominal A-B-C, con sentido elegido por cercanía/coste a A/C, no por distancia a B.
- Las tareas vecinas se solapan deliberadamente: B=A-B-C y C=B-C-D comparten B-C.
- `MAP_SECTION` es cobertura, no ruta rígida; debe funcionar igual en fachada, L, interior, pasillos y laberinto.
- Todo lo accesible del ROI debe poder descubrirse. Regiones físicamente inaccesibles no bloquean eternamente.
- Una tarea larga usa muchas trayectorias cortas. `task_id != trajectory_id`.
- Cada waypoint contiene `(x,y,z,yaw)`; `lib_tray`/`TrayAction` se amplían sin borrar modos legacy.
- Depth/estéreo local es autoridad de obstáculos; MapPoints ayudan a tracking/vista, no bastan para colisiones.
- El z-buffer sparse simetrico de Fase 3P es solo una comprobacion temporal de
  visibilidad entre subnubes de loop. Puede servir como prototipo de
  proyeccion/oclusion/telemetria al preparar 6K, pero nunca como depth ni como
  autoridad para autorizar una trayectoria.
- Wrapper puede ampliarse para entregar tracking/soporte visual actual; no tocar ORB_SLAM3 sin necesidad demostrada y nuevo acuerdo.
- Mismo algoritmo interior/exterior. Yaw se selecciona por calidad visual; los giros de inspección se abortan si cae soporte ORB.
- Replanning receding-horizon: si aparece geometría, el dron para, cancela/libera y propone otra trayectoria sin cancelar la tarea.
- El servidor procesa propuestas de trayectoria en una cola serializada.
- Reservas Dron-Dron son puramente espaciales: si corredores se cruzan, la segunda se rechaza aunque los tiempos difieran.
- Cada corredor incluye tamaño del dron + margen de seguridad/error. Dron parado también ocupa volumen.
- El servidor puede sugerir `A-C-B`, pero C es solo un hint frente a otros drones. El dron lo valida contra paredes/tracking y vuelve a presentar la ruta completa.
- `ANCHOR_SUBMAP`: búsqueda local conservadora de fiducial/loop sin perder tracking ni usar GT.
- `GO_TO(x,y,z,yaw)`: máxima prioridad pendiente, **no** interrumpe una tarea `RUNNING`; usa el mismo planner/reservas.
- Fase 6 no implementa nube densa global. Fase 8 reutilizará/mejorará las fuentes depth y la planificación perceptiva.
- Deuda temporal heredada de Fase 5: retirar `GT_FALLBACK`, el source lock entre
  goals, el handshake de frontera, el hold y el handoff angular SO(3) de
  `control_calcular_fuerzas`. Las
  tareas/recovery de Fase 6 deben proporcionar continuidad real sin dejar ese
  codigo residual. Conservar como invariante que toda trayectoria comienza con
  pose y velocidad de una misma muestra y con `ep=ev=er=ew=0`.

## Secuencia

```text
6A  tarea_principal.yaml
6B  ROI/rebanadas/puntos
6C  generación MAP_SECTION
6D  contratos lifecycle tareas
6E  Task/Mission Manager
6F  asignador por entradas/cercanía
6G  TaskExecutor/control_trayectorias
6H  contrato waypoints
6I  lib_tray + TrayAction multi-waypoint
6J  trayectorias cortas/lifecycle
6K  depth local obstáculos
6L  tracking/soporte visual ORB
6M  yaw/next-view perceptivo
6N  cobertura/frontiers adaptativa
6O  replanning incremental
6P  cola y reservas servidor
6Q  conflicto espacial Dron-Dron
6R  hints de desvío A-C-B
6S  ANCHOR_SUBMAP + GO_TO
6T  integración final N drones
```

## Prueba final

Dar el ROI de la casa y N drones en `tarea_principal.yaml`. Sin rutas de mapeo manuales, el sistema debe completar todas las tareas y dejar en RViz2 un sparse global de todo lo accesible dentro del ROI, con solape entre submapas, sin colisiones y sin GT funcional.

Leer `pipeline_fase_6.md` y el MD de la subfase concreta antes de ejecutar. Tras Fase 5, localizar siempre los topics/frames/interfaces reales del workspace y reutilizarlos en vez de inventar duplicados.

Al preparar 6K, releer tambien `fase_3_sparse_global/subfases/subfase_3P_especificacion.md`
y su implementacion real vigente. Evaluar reutilizacion de interfaces solo si
no acopla la seguridad local al worker secundario ni a MapPoints sparse.
