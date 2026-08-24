# Subfase 5D — Transporte versionado de correcciones servidor→dron

## Transporte Server→Dron exclusivamente a través del wrapper

El transporte de correcciones globales desde Servidor debe terminar en `orbslam3`
como frontera cross-group. Si el estimador acaba en `dron_individual`, el wrapper
reexpone los datos mediante un contrato **local dentro de Dron**. Actualizar aristas,
tipo, QoS, versionado y actividad live en `system_architecture`.

## Estado

```text
sin hacer
```

## Objetivo técnico

Definir e implementar el contrato ROS 2 que transporta desde el Servidor al Dron las correcciones por KeyFrame producidas en 5C, de forma ligera, versionada, segura frente a `map_epoch`/revisiones obsoletas y sin exigir una petición-respuesta por cada frame de ORB-SLAM3.

La comunicación debe permitir que el dron mantenga un cache local de correcciones y continúe publicando pose estimada a la frecuencia de `orbslam/pose_local` aunque las correcciones del servidor lleguen a menor frecuencia.

El contrato final debe contener información suficiente para verificar, como mínimo:

```text
drone_id
map_epoch
keyframe_id
map_revision / pose_revision
timestamp
L_T_KF o datos equivalentes para comprobar/seleccionar
W_T_KF o C_KF
modo/origen/validez
```

No se permite aplicar una corrección de otro epoch ni una revisión anterior a una ya aceptada.

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
codex/pipeline/fase_5_poses_drones_sin_gt/subfases/subfase_5C.md
```

Historial real de 5A–5C y documentación:

```text
codex/contexto/paquetes/orbslam3_msgs/00_summary.md
codex/contexto/paquetes/orbslam3_server/00_summary.md
codex/contexto/paquetes/orbslam3_server/global_map_server.md
codex/contexto/paquetes/orbslam3_server/launches.md
codex/contexto/paquetes/orbslam3_multi/00_summary.md
codex/contexto/paquetes/dron_individual/00_summary.md
```

Leer la decisión vigente de Fase 2 sobre copias/source-of-truth de `orbslam3_msgs`. Si existen copias completas del paquete en Dron/Servidor/Simulación, cualquier cambio de interfaz debe aplicarse de forma coherente a todas las copias exigidas y compilar sus consumidores; no mantener variantes divergentes del mismo `.msg`.

## Diagnóstico de partida

El baseline entregado contiene interfaces históricas:

```text
MapCorrection.msg
  header
  drone_id
  drone_name
  world_frame
  local_map_frame
  world_t_local_map
  map_sequence

CorrectedKeyFrame.msg
  keyframe_id
  original_local_pose
  original_global_pose
  corrected_global_pose

CorrectedKeyFrameArray.msg
  header
  drone_id
  drone_name
  world_frame
  local_map_frame
  keyframes[]
```

Estas interfaces no incluyen de forma explícita toda la identidad/revisión necesaria para el control final de Fase 5, especialmente `map_epoch` dentro de la corrección por KF.

Además, el launch activo del servidor entregado indicaba que esos publishers no estaban conectados al backend vigente. 5A y 5C deben haber reconciliado el estado actual antes de ejecutar 5D.

El objetivo no es reenviar toda la nube/mapa ni publicar un array completo grande a frecuencia alta. Deben enviarse estados/correcciones acotados y suficientes para que el dron seleccione la corrección vigente sin bloquearse.

## Archivos permitidos a modificar

5A debe actualizar paths exactos post-Fase 2. Baseline:

```text
src/orbslam3_msgs/msg/MapCorrection.msg
src/orbslam3_msgs/msg/CorrectedKeyFrame.msg
src/orbslam3_msgs/msg/CorrectedKeyFrameArray.msg
src/orbslam3_msgs/msg/*                       # nuevo mensaje solo si está justificado
src/orbslam3_msgs/CMakeLists.txt
src/orbslam3_msgs/package.xml

src/orbslam3_server/src/global_map_server.cpp
src/orbslam3_server/include/orbslam3_server/global_map_server.hpp
src/orbslam3_server/launch/global_orb_map_server.launch.py
src/orbslam3_server/CMakeLists.txt
src/orbslam3_server/package.xml

src/dron_individual/launch/*                  # solo consumidor/cache ligero si 5A fijó aquí el receptor
orbslam3_ros2/...                             # solo si el receptor pertenece al wrapper por decisión de 5A

src/simulacion_dron/launch/*
src/simulacion_dron/src/control_tray/scenario_runner_node.cpp

codex/contexto/paquetes/orbslam3_msgs/
codex/contexto/paquetes/orbslam3_server/
codex/contexto/paquetes/dron_individual/
codex/contexto/paquetes/simulacion_dron/
codex/pipeline/fase_5_poses_drones_sin_gt/
```

Si Fase 2 mantiene varias copias de `orbslam3_msgs`, la lista debe enumerarlas todas antes de modificar.

## Archivos prohibidos

```text
ORB_SLAM3/**
src/orbslam3_multi/**              # salvo adaptación mínima de API ya acordada en 5C
src/dron_individual/src/control_tray/control_calcular_fuerzas.cpp  # 5H
src/dron_individual/src/control_tray/gen_tray.cpp                  # salvo integración de status ya acordada en 5B
src/simulacion_dron/src/graficar/**                                # 5F/5G
build/**
install/**
log/**
```

No convertir el transporte en un protocolo request/reply por frame.

## Funciones, clases o nodos que hay que localizar

5A/5C deben haber fijado nombres reales. Baseline:

```text
GlobalMapServer
punto que recibe el ChangeSet/resultado de 5C
punto de commit de optimización
publisher ROS por drone/namespace

MapCorrection
CorrectedKeyFrame
CorrectedKeyFrameArray

consumer/cache del lado Dron [nombre real fijado por 5A]

launch global_orb_map_server.launch.py
launch del Dron que arranca ORB/estimador
```

Topics a fijar en 5A/5D, evitando nombres globales sin namespace por dron:

```text
/dron_X/<correction_topic>
/dron_X/<correction_status_topic>      # si se separa estado
```

## Cambios requeridos

1. Elegir, basándose en 5A/5C, una de estas estrategias documentadas:
   - extender los mensajes históricos;
   - crear un mensaje específico de corrección por KF;
   - mantener un array batch acotado más un campo de revisión.

   No mantener dos contratos funcionales equivalentes en paralelo sin una razón de compatibilidad explícita.

2. Garantizar identidad completa en wire:

```text
drone_id
map_epoch
keyframe_id
```

3. Incluir una revisión monotónica o comparable que permita al dron descartar mensajes atrasados. Debe aclararse el scope de la revisión:

```text
por submapa
por KF
por mapa global
```

   y no mezclar scopes en una comparación inválida.

4. Incluir `header.stamp`/timestamp de origen suficiente para diagnóstico y cálculo de edad. El dron no debe reemplazar ese timestamp por el de recepción al decidir obsolescencia.

5. Transportar `C_KF` directamente si 5C ya la calcula, salvo que haya una razón justificada para transportar `L_T_KF + W_T_KF` y calcularla en el dron. Prioridad: mínima carga embarcada sin perder trazabilidad.

6. Si el dron necesita seleccionar por proximidad local como fallback, incluir `L_T_KF` o la pose local del KF en el record. No obligar al dron a reconstruirla consultando el servidor.

7. Publicar actualizaciones por evento:
   - corrección de KF nueva;
   - `W_T_KF` cambia por optimización;
   - anchor/relocalización cambia la relación global;
   - nuevo epoch invalida cache anterior.

8. No depender exclusivamente de la llegada de un nuevo delta para enviar una corrección que cambió por una optimización. Si `GlobalPoseStore` cambia, debe existir un trigger de publicación aunque no llegue otro delta inmediatamente.

9. Mantener publicación acotada:
   - enviar solo correcciones nuevas/cambiadas o un batch limitado;
   - no reenviar cientos/miles de KFs a cada pose local;
   - definir límite/cache y comportamiento al superarlo;
   - si se usa refresh periódico, que sea de seguridad y no la única vía de actualización.

10. Elegir QoS en función de 5A y de pruebas de pérdida/reordenación. Requisito funcional independiente del QoS:
   - no permitir backlog antiguo que aplique correcciones stale después de una nueva;
   - `KeepLast`/cola acotada;
   - el consumidor debe validar epoch/revisión incluso si DDS entrega reliable.

11. En el dron, mantener un cache indexado por identidad completa. Al cambiar `map_epoch`:

```text
invalidar correcciones del epoch anterior para el modo actual
```

   sin borrar necesariamente historia diagnóstica.

12. Rechazar con markers claros:

```text
epoch_mismatch
revision_older
non_finite
unknown_drone
unknown_kf_scope
invalid_quaternion
```

13. Añadir instrumentación equivalente a:

```text
[F5D-TX] drone=... epoch=... kf=... revision=... reason=...
[F5D-RX] drone=... epoch=... kf=... revision=... age_ms=...
[F5D-RX-REJECT] reason=revision_older|epoch_mismatch|non_finite
[F5D-CACHE] entries=... current_epoch=...
```

14. Medir tamaño de mensaje/bandwidth aproximado y ritmo de publicación en prueba larga. El objetivo es demostrar que la frecuencia del servidor no determina la frecuencia de pose del dron.

15. Crear una prueba de reordenación/duplicado de mensajes. Puede hacerse mediante test de componente o publicador de prueba sin modificar la red real.

16. Crear una prueba de pérdida temporal de correcciones: el cache mantiene la última válida para 5E, pero el estado de edad debe quedar visible. No convertir una corrección stale en una pose “nueva”.

17. No usar GT en el publisher ni en el receptor. GT puede registrar error externo en fases superiores.

## Cambios prohibidos

- No usar servicios síncronos por cada `pose_local`.
- No enviar `OrbMap` completo como mecanismo de corrección de pose.
- No aplicar mensajes de otro epoch aunque el `keyframe_id` coincida.
- No confiar solo en orden DDS; validar revisión en aplicación.
- No ampliar indefinidamente la cola/cache.
- No enviar todas las correcciones a alta frecuencia si no cambiaron.
- No usar GT para marcar una corrección como correcta/incorrecta.
- No meter lógica de optimización en el receptor del dron.
- No hacer que la pérdida de un mensaje detenga la publicación local de ORB.
- No romper la política de copias/interfaces fijada en Fase 2.

## Paquetes a compilar

Lista exacta según Fase 2. Baseline si cambia interfaz:

```bash
./codex/herramientas/build_selected_packages.sh orbslam3_msgs orbslam3_multi orbslam3_server dron_individual
```

Si `orbslam3_msgs` está duplicado por grupos, compilar cada grupo/overlay de forma separada según Fase 2 y comprobar que todos usan el mismo contrato.

## Pruebas Gazebo requeridas

### Prueba 1 — Publicación/recepción de correcciones por primer anchor

1. arrancar un dron sin anchor;
2. generar KFs/deltas;
3. adquirir anchor;
4. comprobar que el servidor emite correcciones versionadas;
5. comprobar que el dron las recibe/cachea;
6. comprobar `drone_id`, `map_epoch`, `kf_id`, revisión y stamps.

No hace falta todavía usar la corrección para publicar pose global final: eso pertenece a 5E.

### Prueba 2 — Optimización sin nuevo delta inmediato

1. disponer de un KF corregido en cache;
2. provocar una optimización/revisit que cambie `W_T_KF`;
3. evitar depender de un delta posterior como trigger;
4. comprobar que el servidor publica revisión nueva;
5. comprobar que el dron reemplaza la entrada anterior.

### Prueba 3 — Reordenación y duplicados

Con un test publisher o harness:

```text
revision 10 -> aceptar
revision 12 -> aceptar
revision 11 -> rechazar
revision 12 duplicada -> ignorar/no degradar
```

Repetir con epoch distinto y mismo local `kf_id`.

### Prueba 4 — Pérdida temporal de comunicación de correcciones

1. con cache válido, interrumpir temporalmente el flujo de correcciones;
2. comprobar que el cache no se borra inmediatamente;
3. comprobar que la edad aumenta y se marca stale según política;
4. reanudar y comprobar actualización a revisión nueva;
5. no depender de GT para detectar la caída.

### Prueba 5 — Prueba larga multi-dron de carga

Ejecutar el escenario de dos drones con mapa/optimización activos y medir:

```text
mensajes TX/RX por dron
tamaño medio/máximo
correcciones por segundo
cache entries
rechazos stale/epoch
edad p50/p95 si la instrumentación lo permite
```

El comando exacto debe haber sido actualizado por 5A.

## Patrones de reducción de logs

```text
F5D-TX|F5D-RX|F5D-RX-REJECT|F5D-CACHE|map_epoch|revision|optimization|anchor|relocal|ERROR|FATAL|Segmentation fault|Killed
```

Para carga larga, reducir por `drone_id` y por markers `F5D-*` antes de leer.

## Criterio de éxito

La subfase se considera `CONSEGUIDA` solo si:

1. el contrato wire incluye identidad completa de submapa/KF y revisión suficiente;
2. el servidor publica correcciones nuevas por delta/cambio world y también tras optimización sin esperar un nuevo frame;
3. el dron mantiene un cache acotado y versionado;
4. mensajes atrasados, duplicados y de otro epoch no sobrescriben una corrección más nueva;
5. la pérdida temporal de correcciones no obliga a un round-trip por pose ni detiene ORB;
6. el bandwidth/tamaño se mide y no muestra un diseño de reenvío masivo por frame;
7. no se usa GT;
8. todas las copias de interfaces exigidas por Fase 2 permanecen compatibles;
9. build y pruebas pasan sin errores graves no explicados;
10. historial y documentación se actualizan.

## Criterio de fallo o parcial

- `NO CONSEGUIDA`: se mezclan epochs, una revisión vieja puede pisar una nueva, se necesita servicio por frame, se bloquea el dron por latencia de servidor o las copias de mensajes divergen.
- `PARCIAL`: transporte correcto pero el tamaño/frecuencia resulta excesivo o falta un trigger de actualización tras optimización.
- `BLOQUEADA`: la política de interfaces de Fase 2 no está resuelta o un consumidor necesario no está disponible para compilar.

## Documentación a actualizar

```text
codex/contexto/00_CONTEXTO_COMPACTACION.md
codex/contexto/paquetes/orbslam3_msgs/
codex/contexto/paquetes/orbslam3_server/
codex/contexto/paquetes/orbslam3_multi/       # si cambia API de 5C
codex/contexto/paquetes/dron_individual/       # receptor/cache
codex/contexto/paquetes/simulacion_dron/
codex/pipeline/fase_5_poses_drones_sin_gt/pipeline_fase_5_RESUMEN.md
codex/pipeline/fase_5_poses_drones_sin_gt/historial/INDEX.md
codex/pipeline/fase_5_poses_drones_sin_gt/historial/por_subfase/historial_5D.md
codex/pipeline/fase_5_poses_drones_sin_gt/historial/por_subfase/historial_5D_RESUMEN.md
```
