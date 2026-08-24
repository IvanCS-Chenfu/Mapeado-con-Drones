# Subfase 5H — Sustitución de Ground Truth en generación y control de trayectorias

<!-- ACUERDOS_CIERRE_F2_2026_08_24_START -->
## Retirada funcional de GT reflejada en system_architecture

> **Vigencia:** acuerdo cerrado el 2026-08-24. Este bloque prevalece sobre cualquier
> frase anterior incompatible del mismo documento. No borra ni reescribe evidencia
> histórica; distingue siempre entre estado actual, deuda conocida y arquitectura objetivo.

Al sustituir finalmente `sensor/GT/pose` y `sensor/GT/vel` en `gen_tray` y
`control_calcular_fuerzas`, retirar esas aristas como dependencia funcional. GT puede
figurar únicamente como métrica externa de Simulación. Validar que control usa el estado
estimado real y que el visualizador no maquilla datos incorrectos.
<!-- ACUERDOS_CIERRE_F2_2026_08_24_END -->

## Estado

```text
sin hacer
```

## Objetivo técnico

Eliminar `sensor/GT/pose` y `sensor/GT/vel` del camino funcional de `gen_tray` y `control_calcular_fuerzas`, sustituyéndolos por la pose y velocidad estimadas validadas en 5F/5G.

Esta subfase solo puede ejecutarse después de que:

- 5F haya producido gráficas/métricas de pose y el usuario haya decidido la política raw/smoothed;
- 5G haya producido gráficas/métricas de velocidad y el usuario considere el error suficientemente bajo;
- el estado `GLOBAL_VALID / LOCAL_ONLY / LOCALIZATION_LOST` y la semántica de 5B estén funcionando.

No se debe interpretar la orden de ejecutar 5H como permiso para saltarse esa puerta. Si el usuario todavía no ha aceptado la calidad de pose/velocidad, 5H queda bloqueada.

El resultado final debe ser que los paquetes del Dron puedan compilarse/ejecutarse sin necesitar topics GT y que el mismo código de control funcione en simulación y, posteriormente, hardware real.

## Contexto obligatorio a leer

```text
AGENTS.md
codex/contexto/00_CONTEXTO_COMPACTACION.md
codex/contexto/CONTEXTO_MINIMO_ACTUAL.md
codex/contexto/08_POLITICA_TOKENS_DOCUMENTACION.md
codex/contexto/01_ESTADO_ACTUAL_RESUMEN.md
codex/contexto/01_ESTADO_ACTUAL.md
codex/pipeline/PIPELINE_MAESTRO.md
codex/pipeline/fase_5_poses_drones_sin_gt/pipeline_fase_5_RESUMEN.md
codex/pipeline/fase_5_poses_drones_sin_gt/pipeline_fase_5.md
codex/pipeline/fase_5_poses_drones_sin_gt/subfases/subfase_5B.md
codex/pipeline/fase_5_poses_drones_sin_gt/subfases/subfase_5E.md
codex/pipeline/fase_5_poses_drones_sin_gt/subfases/subfase_5F.md
codex/pipeline/fase_5_poses_drones_sin_gt/subfases/subfase_5G.md
```

Leer historiales/resúmenes reales de 5F y 5G y verificar que contienen la decisión explícita del usuario para pose y que la calidad de velocidad ha sido aceptada.

Documentación:

```text
codex/contexto/paquetes/dron_individual/control.md
codex/contexto/paquetes/dron_individual/trayectorias.md
codex/contexto/paquetes/dron_individual/launches.md
codex/contexto/paquetes/lib_tray/00_summary.md
codex/contexto/paquetes/lib_tray/generacion_trayectorias.md
codex/contexto/paquetes/simulacion_dron/launches.md
codex/contexto/paquetes/simulacion_dron/scenario_runner_node.md
```

## Diagnóstico de partida

En el baseline entregado, los dos puntos funcionales de GT son claros:

```text
gen_tray
  <- sensor/GT/pose
  <- sensor/GT/vel

control_calcular_fuerzas
  <- sensor/GT/pose
  <- sensor/GT/vel
```

`gen_tray` usa esas muestras para obtener condición inicial de posición/yaw y velocidad antes de construir Pol3/VelTrap/elipse.

`control_calcular_fuerzas` usa:

```text
pose.position -> x
pose.orientation -> R_act
vel.linear -> x_dot
vel.angular -> w_world -> w_b
```

No necesita aceleración actual medida.

5B ya debe haber introducido la semántica de goals local/global y congelación de frame. 5E/5G ya deben disponer de interfaces estables de pose/velocidad estimadas y estado de localización.

El objetivo de 5H no es rediseñar el controlador dinámico: es cambiar su fuente de estado preservando la matemática existente, salvo correcciones estrictamente necesarias por frames/unidades.

## Archivos permitidos a modificar

Paths finales fijados por 5A. Baseline:

```text
src/dron_individual/src/control_tray/gen_tray.cpp
src/dron_individual/src/control_tray/control_calcular_fuerzas.cpp
src/dron_individual/launch/generar_dron.launch.py
src/dron_individual/config/tray_dron.yaml
src/dron_individual/package.xml
src/dron_individual/CMakeLists.txt

src/dron_individual/action/TrayAction.action      # solo si el contrato de 5B ya requiere un campo adicional y no se hizo antes

src/simulacion_dron/launch/*
src/simulacion_dron/src/control_tray/scenario_runner_node.cpp
src/simulacion_dron/config/*
codex/archivos_auxiliares/trayectorias/*

codex/contexto/paquetes/dron_individual/
codex/contexto/paquetes/simulacion_dron/
codex/pipeline/fase_5_poses_drones_sin_gt/
```

El estimador de 5E/5G solo puede tocarse para una corrección mecánica que mantenga el contrato ya validado. Si aparece un problema funcional de estimación, reabrir 5E/5F/5G y suspender 5H.

## Archivos prohibidos

```text
ORB_SLAM3/**
src/orbslam3_multi/**
src/orbslam3_server/**
orbslam3_msgs/**                  # salvo compatibilidad ya acordada
src/lib_tray/**                   # no cambiar algoritmos de trayectoria salvo bug preexistente demostrado y autorización
build/**
install/**
log/**
```

No ajustar ganancias como sustituto de una mala pose/velocidad sin una prueba separada y nueva autorización.

## Funciones, clases o nodos que hay que localizar

Baseline:

```text
Clase_Servicio_Accion::pose_actual_callback
Clase_Servicio_Accion::vel_actual_callback
Clase_Servicio_Accion::execute
Clase_Servicio_Accion::handle_goal
Clase_Servicio_Accion::handle_cancel

Clase_Publisher::callback_pose
Clase_Publisher::callback_vel
Clase_Publisher::callback_feedback
Clase_Publisher::enviar_fuerzas

<nodo estimador 5E/5G>
<pose state/status topic>
<estimated pose topic>
<estimated velocity topic>
```

Topics baseline a eliminar como dependencias funcionales del Dron:

```text
sensor/GT/pose
sensor/GT/vel
```

GT puede seguir existiendo en Simulación y ser consumido por nodos de gráficas/métricas externos.

## Cambios requeridos

1. Sustituir en `gen_tray` la suscripción funcional de pose GT por la pose estimada seleccionada según el frame congelado del goal:

```text
LOCAL_ONLY / goal local -> pose local estimada
GLOBAL_VALID / goal world -> pose global elegida en 5F
```

2. Sustituir en `gen_tray` `sensor/GT/vel` por la velocidad estimada de 5G, expresada en el mismo frame que la pose/referencia de la trayectoria.

3. Mantener la regla de 5B: si no hay pose global y un goal llega con `absoluto_*=true`, conservar valores y tratar flags como `false`.

4. Al aceptar un goal, verificar que pose y velocidad necesarias están válidas y sincronizadas suficientemente. No aceptar una condición inicial stale como actual.

5. Mantener la semántica de goal ya acordada. No modificar Pol3/VelTrap/elipse salvo incompatibilidad real de frame detectada en prueba.

6. Sustituir en `control_calcular_fuerzas` la pose GT por la pose estimada que corresponda al frame de la trayectoria activa.

7. Sustituir la velocidad GT por `TwistStamped` estimado de 5G.

8. Garantizar que el par pose/velocidad usado por control está en frames coherentes. Si `control_calcular_fuerzas` asume world para `x`, `x_dot` y `w_world`, introducir el adaptador mínimo necesario para que:
   - un goal global use world;
   - un goal local use un frame local consistente sin cambiar la matemática relativa;
   - no se mezclen pose local y velocidad world o viceversa.

9. Mantener `R_act` derivada de la pose estimada y la transformación de velocidad angular a body con la convención validada en 5G.

10. No añadir aceleración estimada como entrada funcional salvo decisión nueva. `x_ddot_des`, yaw acceleration deseada y jerk siguen viniendo de la trayectoria.

11. Cuando una trayectoria está en curso, el control debe conservar la fuente/frame congelados definidos en 5B. Ejemplo:

```text
goal local activo
aparece global
-> control sigue con pose/vel local hasta terminar
```

12. Si se pierde global durante un goal world:
   - 5B cancela/interrumpe el goal;
   - no intentar seguir controlando contra el target world con pose local;
   - la siguiente maniobra local usa estado local válido.

13. Si se pierde ORB:
   - no continuar el controlador normal con la última pose/velocidad;
   - entrar en recovery de 5G;
   - el control normal solo vuelve cuando el estado observado es válido.

14. Eliminar de launch/config del Dron cualquier requirement que haga fallar el arranque por ausencia de `sensor/GT/*`.

15. Mantener GT en Simulación solo en nodos externos de evaluación. La presencia del plugin GT no debe ser una dependencia del launch Dron.

16. Añadir markers equivalentes a:

```text
[F5H-STATE-SOURCE] pose=LOCAL|GLOBAL velocity=LOCAL|GLOBAL frame=...
[F5H-GOAL-START] execution_frame=... localization_state=...
[F5H-GT-FREE] gen_tray_gt_subscribers=0 control_gt_subscribers=0
[F5H-GOAL-STOP] reason=completed|cancelled_global_loss|tracking_lost|...
```

17. Añadir un test estático/ROS que compruebe que `gen_tray` y `control_calcular_fuerzas` no tienen suscriptores a `sensor/GT/pose` ni `sensor/GT/vel`.

18. Compilar el grupo Dron sin Gazebo/Simulación según Fase 2. Esta es una puerta esencial: si necesita `simulacion_dron` para resolver tipos/topics GT, 5H no está conseguida.

19. Ejecutar pruebas progresivas antes de la larga:
   - hover/estabilización;
   - +1 m Z relativo sin anchor;
   - yaw relativo;
   - traslación local;
   - anchor y goal world corto;
   - pérdida global → local;
   - pérdida ORB → recovery.

20. Comparar externamente contra GT para confirmar que el cambio de fuente no produce divergencia, pero no reintroducir GT si una prueba falla; volver a 5F/5G si la estimación es la causa.

## Cambios prohibidos

- No dejar una suscripción GT “de fallback” dentro del Dron.
- No usar GT si el estimador deja de publicar.
- No hacer `if estimated_invalid -> use sensor/GT`.
- No cambiar a pose global a mitad de un goal local.
- No continuar un goal world después de perder global.
- No mezclar pose local con velocidad global.
- No añadir aceleración actual al controlador solo porque 5G la calcula en debug.
- No reajustar Kp/Kv/Kr/Kw para compensar un error de estimación sin una subfase/decisión separada.
- No mover lógica de Fase 6 de tareas/planificación dentro de `gen_tray`.
- No declarar éxito únicamente porque “el dron se mueve”; verificar ausencia de dependencias GT.

## Paquetes a compilar

Primero build aislado del grupo Dron según Fase 2. Baseline por paquetes:

```bash
./codex/herramientas/build_selected_packages.sh lib_tray dron_individual
```

Si el estimador está en otro paquete Dron, incluirlo.

Después compilar la integración de Simulación:

```bash
./codex/herramientas/build_selected_packages.sh simulacion_dron
```

No usar un build conjunto que oculte una dependencia accidental del Dron hacia Simulación.

## Pruebas Gazebo requeridas

### Prueba 1 — Arranque Dron sin topics GT funcionales

Con Simulación publicando o no GT, comprobar con `ros2 node info`/introspección que:

```text
gen_tray                  no subscribe sensor/GT/pose|vel
control_calcular_fuerzas  no subscribe sensor/GT/pose|vel
```

Los nodos de gráfica sí pueden subscribirse.

### Prueba 2 — Movimiento local sin anchor

1. arrancar sin anchor;
2. ORB tracking válido;
3. enviar `+1 m Z` relativo;
4. enviar yaw/giro de búsqueda relativo;
5. comprobar que el control usa pose/vel local estimadas;
6. repetir con flags absolutos activados y comprobar semántica relativa de 5B.

### Prueba 3 — Anchor y goal absoluto corto

1. adquirir fiducial/anchor;
2. terminar cualquier goal local activo antes de cambiar de frame;
3. enviar un goal world corto;
4. comprobar pose/vel global estimadas como fuente del control;
5. GT solo registra error externo.

### Prueba 4 — Anchor aparece a mitad de goal local

Repetir la regla crítica:

```text
goal local activo
-> aparece global
-> control permanece local hasta RESULT
-> siguiente goal puede ser world
```

### Prueba 5 — Pérdida global durante goal world

1. goal world activo;
2. perder relación global con ORB válido;
3. comprobar cancelación/interrupción;
4. comprobar transición a control local para nueva maniobra;
5. no continuar target world.

### Prueba 6 — Pérdida total y recovery

1. con control ya GT-free, provocar ORB `LOST`;
2. comprobar que no hay fallback a GT;
3. activar la estrategia de recovery elegida/provisional de 5G;
4. si ORB recupera, volver a local/global según corresponda;
5. si no recupera, detener recovery por límites.

### Prueba 7 — Regresión de trayectorias básicas

Ejecutar al menos:

```text
hover
traslación X/Y/Z
yaw
elipse o trayectoria representativa de Fase 1
```

con estimación, no GT funcional.

### Prueba 8 — Comparación externa contra GT

En paralelo a las pruebas anteriores, registrar:

```text
error pose
error velocidad
tracking state
control result
```

sin que el nodo de control tenga acceso a GT.

## Patrones de reducción de logs

```text
F5H-STATE-SOURCE|F5H-GOAL-START|F5H-GT-FREE|F5H-GOAL-STOP|F5B-STATE|F5G-RECOVERY|GOAL|RESULT|success|sensor/GT|ERROR|FATAL|Segmentation fault|Killed
```

Para demostrar ausencia de GT, combinar logs con introspección ROS y búsqueda estática del código modificado.

## Criterio de éxito

La subfase se considera `CONSEGUIDA` solo si:

1. el usuario ha aceptado previamente la calidad de pose/velocidad de 5F/5G;
2. `gen_tray` no depende funcionalmente de `sensor/GT/pose` ni `sensor/GT/vel`;
3. `control_calcular_fuerzas` no depende funcionalmente de esos topics;
4. el grupo Dron compila sin Gazebo/Simulación/GT según Fase 2;
5. movimiento relativo sin anchor funciona con pose/vel estimadas;
6. goal absoluto funciona tras anchor;
7. una trayectoria local no cambia de frame al aparecer global;
8. un goal world se cancela al perder global y puede continuarse después con una nueva maniobra local;
9. ORB `LOST` no provoca fallback a GT y activa recovery de 5G;
10. trayectorias básicas de Fase 1 siguen funcionando con estado estimado;
11. GT queda limitado a métricas externas;
12. build y pruebas pasan sin errores graves no explicados;
13. historial y documentación se actualizan.

## Criterio de fallo o parcial

- `NO CONSEGUIDA`: queda cualquier fallback funcional a GT, el Dron no compila aislado, se mezclan frames o las trayectorias básicas divergen por la nueva fuente.
- `PARCIAL`: GT ya está eliminado pero una trayectoria/regla de pérdida aún falla o la estimación aceptada en 5F/5G no resulta suficiente bajo control cerrado.
- `BLOQUEADA`: 5F/5G no tienen aceptación del usuario o el estimador no ofrece pose/velocidad con el contrato necesario.

Si el control cerrado revela un error de estimación no visible en las gráficas, no “arreglar” 5H con GT: reabrir 5F/5G y documentar la nueva evidencia.

## Documentación a actualizar

```text
codex/contexto/00_CONTEXTO_COMPACTACION.md
codex/contexto/paquetes/dron_individual/control.md
codex/contexto/paquetes/dron_individual/trayectorias.md
codex/contexto/paquetes/dron_individual/launches.md
codex/contexto/paquetes/simulacion_dron/
codex/pipeline/fase_5_poses_drones_sin_gt/pipeline_fase_5_RESUMEN.md
codex/pipeline/fase_5_poses_drones_sin_gt/historial/INDEX.md
codex/pipeline/fase_5_poses_drones_sin_gt/historial/por_subfase/historial_5H.md
codex/pipeline/fase_5_poses_drones_sin_gt/historial/por_subfase/historial_5H_RESUMEN.md
```
