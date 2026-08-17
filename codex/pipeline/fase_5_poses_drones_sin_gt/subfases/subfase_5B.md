# Subfase 5B — Pose local, navegación sin anchor y gestión de estados de localización

## Estado

```text
sin hacer
```

## Objetivo técnico

Convertir la pose local de ORB-SLAM3 en una fuente de navegación utilizable incluso cuando el submapa todavía no está anclado a `world`, definir la máquina de estados de localización y fijar el comportamiento de las trayectorias cuando aparece o desaparece la pose global.

Esta subfase debe demostrar que “sin anchor” no significa “sin poder moverse”. Mientras ORB-SLAM3 mantenga tracking, el dron debe poder ejecutar movimientos relativos en su `map_local` para buscar un fiducial o volver a una zona útil.

Reglas funcionales obligatorias:

```text
GLOBAL_VALID
  local pose válida + global pose válida
  -> relativos + absolutos permitidos

LOCAL_ONLY
  local pose válida + global pose no disponible
  -> relativos permitidos
  -> cualquier absoluto_* se trata como false manteniendo el valor del goal

LOCALIZATION_LOST
  local pose no válida
  -> navegación normal cancelada
  -> no hay pose actual válida
  -> se prepara/activa la recuperación ciega definida completamente en 5G
```

La interpretación del frame de una trayectoria queda congelada al aceptarla:

- si empieza en `LOCAL_ONLY`, sigue usando pose local hasta terminar aunque aparezca un anchor;
- si empieza como absoluta en `GLOBAL_VALID` y se pierde pose global, se interrumpe y no se reinterpreta silenciosamente el destino world restante;
- al recuperar global durante una trayectoria local, solo el siguiente goal puede ser absoluto.

Además debe crearse una línea base antes de modificar el comportamiento: provocar pérdidas de anchor y de tracking ORB, observar qué hace el sistema actual y repetir los mismos escenarios después de los cambios.

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
codex/pipeline/fase_5_poses_drones_sin_gt/subfases/subfase_5A.md
```

Leer el historial real de 5A y los MD actualizados por 5A antes de actuar.

Documentación mínima de paquetes:

```text
codex/contexto/paquetes/orbslam3_ros2/stereo_slam_node.md
codex/contexto/paquetes/dron_individual/control.md
codex/contexto/paquetes/dron_individual/trayectorias.md
codex/contexto/paquetes/simulacion_dron/scenario_runner_node.md
codex/contexto/paquetes/simulacion_dron/launches.md
```

Leer también el contrato vigente de Fase 4 para saber cómo se publica el estado de anchor y cómo se recupera una relación global.

5A debe haber sustituido cualquier path/topic provisional de este archivo por el real. Si no lo hizo, no ejecutar 5B hasta cerrar esa duda.

## Diagnóstico de partida

En el baseline entregado:

- `StereoSlamNode::PublishLocalPose` publica `orbslam/pose_local` cuando tracking está OK;
- `gen_tray` espera una muestra de `sensor/GT/pose` y `sensor/GT/vel` antes de generar el goal;
- `TrayAction` ya dispone de `absoluto_x`, `absoluto_y`, `absoluto_z`, `absoluto_yaw`;
- para X/Y relativos, `gen_tray` transforma el desplazamiento usando el yaw inicial;
- Z/yaw tienen su propia semántica relativa según el tipo de trayectoria;
- el controlador de fuerzas usa una pose/velocidad medida y no conoce directamente el significado de los flags absolutos: la semántica del goal se decide principalmente en `gen_tray`/capa de trayectoria;
- si el corrector global histórico no tiene `world_T_local_map`, deja de publicar pose global;
- no existe una fuente inercial alternativa prevista para continuar navegando cuando ORB pierde tracking.

5A debe haber verificado si estos puntos siguen vigentes tras Fases 2–4.

No hay todavía evidencia de cómo responde el sistema completo a:

```text
anchor perdido con ORB OK
tracking ORB LOST
anchor obtenido a mitad de goal local
pose global perdida a mitad de goal world
```

La primera parte de 5B es producir esa línea base antes de cambiar la lógica.

## Archivos permitidos a modificar

La lista exacta debe quedar cerrada por 5A. Baseline probable:

```text
src/dron_individual/src/control_tray/gen_tray.cpp
src/dron_individual/src/control_tray/control_calcular_fuerzas.cpp       # solo si hace falta integrar estado/fuente, no sustituir GT final aún
src/dron_individual/action/TrayAction.action                            # solo si 5A demuestra que el contrato actual no puede expresar el frame congelado/estado
src/dron_individual/launch/generar_dron.launch.py
src/dron_individual/config/tray_dron.yaml

orbslam3_ros2/.../StereoSlamNode*                                      # wrapper, si 5A fijó tracking/map_epoch/status aquí
orbslam3_msgs/msg/*                                                     # solo interfaz mínima acordada por 5A

src/simulacion_dron/src/control_tray/scenario_runner_node.cpp
src/simulacion_dron/config/*
src/simulacion_dron/launch/*
codex/archivos_auxiliares/trayectorias/*

codex/contexto/paquetes/dron_individual/
codex/contexto/paquetes/orbslam3_ros2/
codex/contexto/paquetes/orbslam3_msgs/
codex/contexto/paquetes/simulacion_dron/
codex/pipeline/fase_5_poses_drones_sin_gt/
```

Si Fase 2 movió estos paquetes a otros roots, usar los paths actualizados por 5A.

## Archivos prohibidos

```text
ORB_SLAM3/**                         # salvo autorización explícita posterior
src/orbslam3_multi/**                # 5C
src/orbslam3_server/**               # 5C/5D
build/**
install/**
log/**
```

No implementar todavía el cálculo global de 5C–5E, el suavizado de 5F, el estimador completo de velocidad de 5G ni la eliminación final de GT de 5H.

## Funciones, clases o nodos que hay que localizar

5A debe haber confirmado nombres reales. Baseline:

```text
StereoSlamNode::GrabStereo
StereoSlamNode::PublishLocalPose
StereoSlamNode::UpdateMapEpochFromCurrentMap
tracking state expuesto por wrapper [nombre real fijado en 5A]

Clase_Servicio_Accion::handle_goal
Clase_Servicio_Accion::handle_cancel
Clase_Servicio_Accion::handle_accepted
Clase_Servicio_Accion::execute
Clase_Servicio_Accion::pose_actual_callback
Clase_Servicio_Accion::vel_actual_callback

control_calcular_fuerzas::callback_pose      [nombre real de clase fijado en 5A]
control_calcular_fuerzas::callback_vel
control_calcular_fuerzas::enviar_fuerzas

scenario_runner_node
```

Topics/actions mínimos:

```text
orbslam/pose_local
<tracking_status_topic fijado por 5A>
<pose/localization status fijado por 5A>
AccionTrayectoria
AccionTrayectoria/_action/feedback
sensor/GT/pose          # solo baseline/métrica, no decisión funcional nueva
sensor/GT/vel           # solo baseline/métrica, no decisión funcional nueva
```

## Cambios requeridos

1. Introducir o consolidar un estado de localización que distinga inequívocamente:

```text
GLOBAL_VALID
LOCAL_ONLY
LOCALIZATION_LOST
```

   El nombre del mensaje/topic puede variar según 5A, pero debe incluir al menos `map_epoch`, validez local/global y timestamp/edad suficiente para no confundir datos stale.

2. Garantizar que una pose local ORB válida sigue disponible al sistema de trayectoria aunque no exista anchor. No bloquear la aceptación de un movimiento relativo únicamente porque no haya `world_T_local`.

3. Implementar la regla absoluta→relativa cuando no hay global:

```text
si global_pose_valid == false:
    effective_absoluto_x   = false
    effective_absoluto_y   = false
    effective_absoluto_z   = false
    effective_absoluto_yaw = false
```

   Los valores `target_pose`, `tx`, `ty`, `tz`, `tyaw` no se descartan. Se procesan exactamente con la rama que ya correspondería a `absoluto_*=false` para ese tipo de trayectoria.

4. No modificar el goal original en memoria de manera que después parezca que el cliente envió flags distintos. Registrar la interpretación efectiva con un marker claro, por ejemplo el nombre definitivo fijado por 5A:

```text
[F5B-GOAL-FRAME] mode=LOCAL_ONLY absolute_flags_in=... effective_relative=true
```

5. Al aceptar un goal, capturar el frame/modo de ejecución y congelarlo durante toda la trayectoria:

```text
execution_frame = LOCAL_MAP | WORLD
accepted_map_epoch = ...
```

6. Si el goal fue aceptado en local y aparece un anchor:
   - no regenerar la trayectoria;
   - no cambiar las referencias en curso a world;
   - continuar con la misma pose local hasta resultado/cancelación;
   - habilitar global únicamente para el siguiente goal.

7. Si el goal fue aceptado en `WORLD` y se pierde pose global pero ORB sigue válido:
   - cancelar/interrumpir el goal actual con motivo explícito;
   - no reinterpretar el target absoluto restante como relativo;
   - tomar el estado local actual como base para una nueva maniobra de seguridad/local posterior;
   - pasar a `LOCAL_ONLY`.

8. Si ORB pasa a `LOST`:
   - marcar inmediatamente `LOCALIZATION_LOST`;
   - cancelar el goal normal activo;
   - no seguir publicando una pose anterior como si fuese actual;
   - conservar un buffer corto de los últimos estados locales válidos y timestamps para 5G;
   - activar un hook/estado de recuperación ciega, pero no es obligatorio completar el algoritmo físico final hasta 5G si aún no existe velocidad estimada fiable.

9. Si ORB recupera tracking:
   - abandonar el estado de pérdida;
   - no reanudar automáticamente la trayectoria cancelada;
   - quedar en `LOCAL_ONLY` salvo que Fase 3/4 confirme relación global válida;
   - si se recupera global, cambiar a `GLOBAL_VALID` para futuros goals.

10. Tratar `map_epoch` como frontera dura. Una pose/estado/corrección de otro epoch no puede reactivar un goal ni declarar global válida.

11. Crear instrumentación mínima y no bloqueante para poder distinguir en logs:

```text
estado anterior -> estado nuevo
motivo de transición
frame congelado del goal
goal cancelado por pérdida global
tracking LOST / RECOVERED
anchor GLOBAL_AVAILABLE sin cambio del goal local activo
```

12. Mantener GT únicamente para comparar durante simulación/baseline. Ningún nuevo `if`, transición o aceptación/rechazo puede usar GT.

13. Antes de modificar, ejecutar las pruebas baseline. Después repetir exactamente los mismos escenarios con el código modificado y conservar ambas entradas cronológicas en historial.

## Cambios prohibidos

- No exigir pose global para despegar o realizar una búsqueda relativa.
- No ignorar un goal absoluto al estar desanclado: se usa su valor con semántica relativa.
- No convertir a world una trayectoria local ya iniciada cuando aparece un anchor.
- No continuar una trayectoria world usando un target global si se ha perdido la relación con `world`.
- No reutilizar indefinidamente la última pose conocida como pose actual.
- No usar `sensor/GT/pose` o `sensor/GT/vel` para decidir `LOCAL_ONLY`, `GLOBAL_VALID`, `LOST` o recuperación.
- No implementar navegación ciega prolongada.
- No inventar una dirección de recuperación cuando no existe historial local válido; 5G debe tratar ese caso explícitamente.
- No introducir matching global, selección de KFs o correcciones del servidor en esta subfase.
- No cambiar el controlador dinámico o sus ganancias para ocultar fallos de pose.

## Paquetes a compilar

5A debe ajustar la lista según el ownership real. Baseline probable:

```bash
./codex/herramientas/build_selected_packages.sh orbslam3_msgs orbslam3 dron_individual simulacion_dron
```

Si no se modifica interfaz/wrapper, excluirlos del build y compilar solo paquetes afectados.

Ante fallo:

```bash
./codex/herramientas/reduce_build_log.sh
```

Leer únicamente el reducido.

## Pruebas Gazebo requeridas

### Prueba 1 — Baseline antes de modificaciones: sin anchor con ORB válido

Objetivo: observar qué hace el sistema actual cuando no existe pose global.

Secuencia:

1. arrancar un dron con ORB-SLAM3;
2. impedir temporalmente el anchor usando el mecanismo de Fase 4 configurado para prueba, sin tocar GT como entrada funcional;
3. esperar tracking local válido;
4. enviar un goal relativo sencillo;
5. enviar un goal con `absoluto_* = true` y registrar el comportamiento actual;
6. no modificar código entre esta ejecución y su registro en historial.

La línea base no tiene que “pasar”: sirve para comparar.

### Prueba 2 — Post-cambio: absoluto tratado como relativo sin anchor

Escenario mínimo:

```text
submapa no anclado
ORB tracking OK
```

Enviar un goal con valores conocidos y flags absolutos activados. Debe ejecutarse con la semántica exacta de `absoluto_*=false`.

Caso recomendado:

```text
+1 m en Z
+360° yaw relativo / búsqueda
```

si la representación actual de yaw/TrayAction permite ese giro; 5A debe fijar el goal exacto compatible con el generador.

Comprobar que el dron se mueve sin depender de una pose world.

### Prueba 3 — Anchor aparece durante trayectoria local

1. iniciar goal en `LOCAL_ONLY`;
2. durante la trayectoria, provocar/permitir observación de fiducial;
3. comprobar transición a disponibilidad global;
4. comprobar que el goal actual conserva `execution_frame=LOCAL_MAP` hasta terminar;
5. enviar después un nuevo goal absoluto y comprobar que ya puede interpretarse en `world`.

### Prueba 4 — Pérdida global durante trayectoria absoluta

1. iniciar con `GLOBAL_VALID`;
2. aceptar un goal absoluto;
3. invalidar/perder la relación global manteniendo ORB tracking;
4. comprobar cancelación/interrupción del goal world;
5. comprobar transición a `LOCAL_ONLY`;
6. enviar un nuevo movimiento relativo y comprobar que es aceptable.

### Prueba 5 — Baseline y post-cambio de pérdida total ORB

Ejecutar dos entradas separadas de historial:

**A. Antes de cambios:** provocar una zona sin landmarks y observar qué hace el sistema actual.

**B. Después de cambios:** repetir el mismo escenario y comprobar:

```text
tracking -> LOST
trayectoria normal cancelada
LOCALIZATION_LOST activo
no se publica pose stale como válida
buffer reciente conservado
hook de recuperación preparado
```

La recuperación física completa se valida en 5G.

### Prueba 6 — Recuperación de tracking

1. entrar en `LOCALIZATION_LOST`;
2. devolver al campo visual una zona reconocible mediante el mecanismo de prueba;
3. comprobar `LOST -> LOCAL_ONLY` o `LOST -> GLOBAL_VALID` según relación global disponible;
4. comprobar que la trayectoria cancelada no se reanuda automáticamente.

## Patrones de reducción de logs

### Baseline

```text
TRACKING|LOST|RELOCAL|pose_local|anchor|global_pose|AccionTrayectoria|GOAL|RESULT|sensor/GT|ERROR|FATAL|Segmentation fault|Killed
```

### Post-cambio

Los nombres exactos deben quedar fijados por 5A; incluir como mínimo markers equivalentes a:

```text
F5B-STATE|F5B-GOAL-FRAME|F5B-GLOBAL-LOST|F5B-TRACKING-LOST|F5B-TRACKING-RECOVERED|GOAL|RESULT|success|ERROR|FATAL|Segmentation fault|Killed
```

No concluir por ausencia de líneas en un patrón demasiado estrecho: regenerar el reducido con los markers reales. Nunca abrir el log completo.

## Criterio de éxito

La subfase se considera `CONSEGUIDA` solo si:

1. existe una máquina de estados verificable `GLOBAL_VALID / LOCAL_ONLY / LOCALIZATION_LOST`;
2. con ORB válido y sin anchor, el dron puede iniciar un movimiento relativo;
3. un goal con flags absolutos, estando sin global, conserva sus valores pero se ejecuta exactamente por las ramas relativas;
4. el frame/modo del goal queda congelado desde su aceptación;
5. obtener anchor durante un goal local no cambia ese goal a world;
6. perder pose global durante un goal world lo interrumpe y permite pasar a local si ORB sigue válido;
7. perder tracking ORB cancela navegación normal y no deja una pose stale marcada como válida;
8. recuperar tracking no reanuda automáticamente el goal cancelado;
9. `map_epoch` impide mezclar estado viejo y nuevo;
10. existen ejecuciones baseline y post-cambio comparables en historial;
11. no se ha usado GT para ninguna transición o decisión funcional nueva;
12. build y todas las pruebas requeridas terminan sin errores graves no explicados;
13. documentación de paquetes e historial quedan actualizados.

## Criterio de fallo o parcial

- `NO CONSEGUIDA`: el dron sigue necesitando pose global para moverse, un absoluto se ignora en lugar de tratarse como relativo, una trayectoria cambia de frame a mitad, se siguen usando poses stale como válidas o se usa GT funcionalmente.
- `PARCIAL`: funcionan los modos local/global pero falta una de las transiciones de pérdida/recuperación o la instrumentación no permite demostrarla.
- `BLOQUEADA`: el wrapper no expone tracking/map_epoch y 5A no pudo definir una interfaz mínima sin una dependencia externa faltante.

La recuperación ciega puede quedar solo preparada en 5B; no marcarla como “validada” hasta 5G.

## Documentación a actualizar

```text
codex/contexto/00_CONTEXTO_COMPACTACION.md
codex/contexto/paquetes/dron_individual/
codex/contexto/paquetes/orbslam3_ros2/
codex/contexto/paquetes/orbslam3_msgs/        # si se modifica interfaz
codex/contexto/paquetes/simulacion_dron/
codex/pipeline/fase_5_poses_drones_sin_gt/pipeline_fase_5_RESUMEN.md
codex/pipeline/fase_5_poses_drones_sin_gt/historial/INDEX.md
codex/pipeline/fase_5_poses_drones_sin_gt/historial/por_subfase/historial_5B.md
codex/pipeline/fase_5_poses_drones_sin_gt/historial/por_subfase/historial_5B_RESUMEN.md
```

El historial debe conservar separadamente la ejecución baseline y cada ejecución posterior. No reescribir una pérdida pre-cambio como si hubiera sido post-cambio.
