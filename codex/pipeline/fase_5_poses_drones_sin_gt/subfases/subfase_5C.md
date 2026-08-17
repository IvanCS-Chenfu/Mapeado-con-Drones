# Subfase 5C — Cálculo servidor de correcciones globales por KeyFrame

## Estado

```text
sin hacer
```

## Objetivo técnico

Construir en el lado Servidor la primera estrategia funcional para obtener, a partir del mapa raw y de `GlobalPoseStore`, correcciones globales de KeyFrames de cada submapa y dejarlas listas para su envío al dron.

Para un KF identificado de forma estable como:

```text
(drone_id, map_epoch, local_kf_id)
```

el backend debe poder recuperar:

```text
L_T_KF = pose local raw del KF en el map_local de ORB
W_T_KF = pose world aceptada/corregida del mismo KF en GlobalPoseStore
```

y calcular:

```text
C_KF = W_T_KF * inverse(L_T_KF)
```

`C_KF` es la transformación que el estimador embarcado de 5E aplicará a una `pose_local` actual cuando ese KF sea la referencia seleccionada.

La estrategia de esta subfase es deliberadamente revisable. Se implementa, se prueba en corto y largo, y posteriormente 5F comprueba el error de pose. Si 5F demuestra que la elección/corrección de KFs es insuficiente, se reabre 5C y se sustituye el método sin considerar el orden del pipeline como una prohibición para volver atrás.

Método preferido:

1. usar el `reference_keyframe_id` real de ORB-SLAM3 si 5A confirmó que el wrapper puede publicarlo;
2. mantener como fallback una selección de KFs corregidos cercanos/relevantes del mismo submapa;
3. no inyectar estas correcciones dentro del mapa interno de ORB-SLAM3.

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
codex/pipeline/fase_5_poses_drones_sin_gt/subfases/subfase_5B.md
```

Historial real:

```text
codex/pipeline/fase_5_poses_drones_sin_gt/historial/INDEX.md
codex/pipeline/fase_5_poses_drones_sin_gt/historial/por_subfase/historial_5A_RESUMEN.md
codex/pipeline/fase_5_poses_drones_sin_gt/historial/por_subfase/historial_5B_RESUMEN.md
```

si existen tras ejecutar esas subfases.

Documentación de paquetes:

```text
codex/contexto/paquetes/orbslam3_multi/00_summary.md
codex/contexto/paquetes/orbslam3_multi/raw_map_database.md
codex/contexto/paquetes/orbslam3_multi/global_pose_store.md
codex/contexto/paquetes/orbslam3_multi/optimization_manager.md
codex/contexto/paquetes/orbslam3_multi/pose_graph_builder.md
codex/contexto/paquetes/orbslam3_server/00_summary.md
codex/contexto/paquetes/orbslam3_server/global_map_server.md
codex/contexto/paquetes/orbslam3_server/global_pose_corrector.md
codex/contexto/paquetes/orbslam3_ros2/stereo_slam_node.md
```

Leer también las subfases de Fase 3 que definan la ingesta raw, `GlobalPoseStore`, optimización y commits, y el contrato vigente de Fase 4 para anchors/relocalización.

## Diagnóstico de partida

El baseline entregado ya contiene la matemática de una estrategia histórica en `global_pose_corrector`:

```text
T_world_current = T_world_kf_corrected * inv(T_local_kf) * T_local_current
```

Ese nodo selecciona el KF corregido más cercano en coordenadas locales y usa fallback rígido `world_T_local * T_local_current` cuando no encuentra un KF apto.

Sin embargo, el launch activo entregado del servidor indica que no publica todavía las interfaces históricas de corrección y no lanza el corrector. Por tanto 5C no debe limitarse a “activar” código antiguo sin reconciliarlo con la arquitectura actual de Fase 3/4.

Además:

- `RawMapDatabase` debe seguir siendo la fuente de `L_T_KF` raw;
- `GlobalPoseStore` debe seguir siendo la autoridad de `W_T_KF` aceptada;
- una optimización puede mover distintos KFs de forma no representable por una única `world_T_local` rígida;
- un delta puede reenviar KFs antiguos al cambiar su hash, por lo que el orden de llegada no es una secuencia temporal fiable;
- la identidad de un KF no puede reducirse a `local_kf_id` ni a `(drone_id, local_kf_id)` sin `map_epoch`.

5A debe haber concretado si el reference KF real está disponible. Si no lo está, esta subfase implementará la estrategia inicial de candidatos cercanos y dejará instrumentación suficiente para evaluar su calidad en 5F.

## Archivos permitidos a modificar

5A debe sustituir esta lista por paths post-Fase 2. Baseline probable:

```text
src/orbslam3_multi/include/orbslam3_multi/*        # nuevo/actual componente backend de correcciones si hace falta
src/orbslam3_multi/src/*                           # cálculo puro, tests
src/orbslam3_multi/CMakeLists.txt
src/orbslam3_multi/package.xml

src/orbslam3_server/include/orbslam3_server/global_map_server.hpp
src/orbslam3_server/src/global_map_server.cpp
src/orbslam3_server/CMakeLists.txt
src/orbslam3_server/package.xml

codex/contexto/paquetes/orbslam3_multi/
codex/contexto/paquetes/orbslam3_server/
codex/pipeline/fase_5_poses_drones_sin_gt/
```

Modificar `orbslam3_msgs` solo si 5A decidió que el contrato interno entre backend/adapter necesita ya una interfaz ROS en 5C; preferiblemente dejar el transporte ROS para 5D.

## Archivos prohibidos

```text
ORB_SLAM3/**
orbslam3_ros2/**                   # salvo ajuste ya autorizado de reference KF pendiente de 5A
src/dron_individual/**             # 5E/5H
src/simulacion_dron/src/graficar/**# 5F/5G
build/**
install/**
log/**
```

No modificar poses raw dentro de `RawMapDatabase` ni escribir correcciones globales de vuelta a ORB-SLAM3.

## Funciones, clases o nodos que hay que localizar

5A debe haber fijado los nombres reales. Baseline:

```text
RawMapDatabase
RawMapDatabase::<getter de KeyFrame raw>
RawKeyFrameId
RawSubmapId
ChangeSet / resultado de commit raw

GlobalPoseStore
GlobalPoseStore::GetWorldPose
GlobalPoseStore::<API de revision/stats si existe>
commit de optimizacion/propagacion que cambia poses world

GlobalMapServer
callback de OrbMap delta
punto posterior a commit raw
punto de commit de optimizacion
punto de reanclaje/relocalizacion de submapa

GlobalPoseCorrector::LocalPoseCallback          # referencia matematica histórica, no ownership objetivo
CorrectedKeyFrameData                           # referencia histórica
```

Si 5A determinó un nuevo componente backend para pose, usar ese nombre y documentarlo antes de implementar.

## Cambios requeridos

1. Crear o reutilizar una función/backend puro que, dada una identidad completa de KF, obtenga de forma coherente:

```text
L_T_KF raw
W_T_KF aceptada
pose/global revision asociada
```

   La captura debe respetar las reglas de snapshots/locks de Fase 3 y no bloquear ingesta mientras se hacen cálculos triviales sobre conjuntos grandes.

2. Calcular:

```text
C_KF = W_T_KF * inverse(L_T_KF)
```

   rechazando transformaciones no finitas, cuaterniones inválidos o IDs sin correspondencia.

3. Mantener identidad completa en toda estructura/cache:

```text
(drone_id, map_epoch, local_kf_id)
```

   Nunca indexar correcciones solo por `keyframe_id` si el scope no garantiza epoch.

4. Definir cuándo una corrección debe considerarse materialmente nueva:
   - KF raw nuevo/cambiado;
   - primera `W_T_KF` disponible;
   - cambio de `W_T_KF` por optimización/propagación;
   - cambio de anchor/relocalización que afecta al submapa;
   - nuevo `map_epoch`.

5. Integrarse con los eventos de Fase 3 sin hacer trabajo pesado por cada pose local. El cálculo de `C_KF` se dispara por cambios del mapa, no por cada frame de cámara.

6. Si 5A confirmó `reference_keyframe_id`:
   - garantizar que la corrección de ese KF puede materializarse tan pronto como `GlobalPoseStore` tenga `W_T_KF`;
   - registrar si el reference KF pedido todavía no existe en el servidor;
   - no bloquear al dron esperando síncronamente.

7. Si no hay reference KF:
   - construir una estrategia inicial de conjunto de KFs corregidos relevantes del mismo epoch;
   - conservar `L_T_KF` junto a la corrección para que 5E pueda seleccionar por proximidad local;
   - limitar el tamaño de la salida/cache con un criterio documentado;
   - no inferir “KF actual” por mayor ID o último elemento del delta.

8. Mantener un fallback rígido del submapa solo si Fase 3/4 todavía produce una transformación global rígida válida y 5A la ha confirmado. Debe etiquetarse como fallback y no sustituir silenciosamente una corrección por KF.

9. Exponer al adapter ROS de 5D una estructura equivalente a:

```text
submap_id
keyframe_id
L_T_KF
W_T_KF o C_KF
pose_revision
source/mode
```

   No fijar todavía el formato wire si 5D lo hará mediante un mensaje nuevo.

10. Añadir markers específicos y acotados, nombres finales fijados en 5A, equivalentes a:

```text
[F5C-CORRECTION-BUILD]
  drone_id=...
  map_epoch=...
  kf=...
  revision=...
  source=GLOBAL_POSE_STORE
  finite=true

[F5C-CORRECTION-UPDATE]
  reason=RAW_CHANGE|GLOBAL_POSE_CHANGE|ANCHOR|RELOCALIZATION

[F5C-CORRECTION-REJECT]
  reason=missing_raw|missing_world_pose|non_finite|epoch_mismatch
```

11. Crear tests deterministas de matemáticas/identidad:
   - transformación identidad;
   - traslación conocida;
   - rotación conocida;
   - mismo `local_kf_id` en dos epochs;
   - `W_T_KF` cambia y `L_T_KF` permanece raw;
   - pose no finita se rechaza.

12. Crear una prueba larga de observación, sin declarar el algoritmo definitivo, para medir:
   - cuántos KFs reciben corrección;
   - latencia desde cambio world hasta corrección disponible;
   - frecuencia de fallbacks;
   - distancia al KF elegido si se usa estrategia cercana.

13. No usar GT para seleccionar el mejor KF. GT puede registrarse únicamente para análisis externo posterior en 5F.

14. Si el método parece funcionar en 5C pero 5F revela error elevado, documentar expresamente que 5C debe reabrirse y repetir sus pruebas con la nueva estrategia.

## Cambios prohibidos

- No modificar `RawMapDatabase` para que guarde poses optimizadas en lugar de raw.
- No sobrescribir poses internas de ORB-SLAM3.
- No realizar matching/RANSAC en el dron.
- No pedir al servidor una pose por cada frame.
- No seleccionar KFs usando GT o distancia GT.
- No usar únicamente `world_T_local` si existen deformaciones/correcciones por KF disponibles.
- No considerar `local_kf_id` globalmente único.
- No asumir que el último delta contiene únicamente KFs nuevos ni que el mayor ID es el reference KF actual.
- No añadir una segunda autoridad de poses world fuera de `GlobalPoseStore`.
- No elegir una estrategia más compleja que la inicial sin evidencia; 5F es la puerta de calidad.

## Paquetes a compilar

Baseline, si se modifican backend y adapter:

```bash
./codex/herramientas/build_selected_packages.sh orbslam3_multi orbslam3_server
```

Añadir consumidores reales solo si cambia una interfaz compartida.

Ante fallo:

```bash
./codex/herramientas/reduce_build_log.sh
```

## Pruebas Gazebo requeridas

### Prueba 1 — Test determinista de corrección por KF

Antes de Gazebo, ejecutar tests de componente creados en `orbslam3_multi`/backend con transformaciones conocidas.

Debe demostrar matemáticamente:

```text
C_KF * L_T_KF == W_T_KF
```

con tolerancia numérica acordada y casos de rotación/traslación.

### Prueba 2 — Un dron, primer anchor y correcciones

Secuencia:

1. arrancar ORB, servidor y Fase 4;
2. mantener el submapa inicialmente sin anchor;
3. dejar que se generen varios KFs raw;
4. adquirir un fiducial/anchor;
5. comprobar que los KFs con world pose producen correcciones del mismo `(drone_id,map_epoch)`;
6. comprobar que antes del anchor no se inventan `W_T_KF`.

### Prueba 3 — Cambio por optimización/revisit

1. partir de submapa anclado;
2. provocar una corrección/optimización válida de Fase 3/4;
3. comprobar que `W_T_KF` cambia en `GlobalPoseStore`;
4. comprobar que `C_KF` se recalcula sin tocar `L_T_KF` raw;
5. comprobar revisión nueva y motivo correcto.

### Prueba 4 — Dos epochs con IDs locales repetidos

Provocar o reproducir un cambio de `map_epoch` donde reaparezca un `local_kf_id` ya usado.

Debe existir una corrección distinta por identidad completa y ninguna contaminación cruzada.

### Prueba 5 — Prueba larga de estrategia inicial

Usar el escenario representativo alrededor de la casa/edificio con al menos dos drones si el coste lo permite.

Objetivo de 5C: no decidir aún el error final contra GT, sino observar estabilidad de selección/materialización y detectar huecos de corrección/fallbacks. La calidad geométrica final se audita en 5F.

El comando exacto y YAML deben haber sido cerrados por 5A. Baseline orientativo:

```bash
./codex/herramientas/run_simulation.sh \
  --prueba fase5_5C_long \
  --launch "ros2 launch simulacion_dron multi_dron.launch.py" \
  --post-scenario-wait-sec 30
```

## Patrones de reducción de logs

### Tests/Prueba corta

```text
F5C-CORRECTION-BUILD|F5C-CORRECTION-UPDATE|F5C-CORRECTION-REJECT|GlobalPoseStore|map_epoch|anchor|optimization|ERROR|FATAL|Segmentation fault|Killed
```

### Prueba larga

```text
F5C-CORRECTION|reference|nearest|fallback|map_epoch|revision|GLOBAL_POSE_STORE|optimization|relocal|ERROR|FATAL|Segmentation fault|Killed
```

Sustituir por markers reales fijados por 5A. Reducir por dron/epoch si el log sigue siendo grande.

## Criterio de éxito

La subfase se considera `CONSEGUIDA` si:

1. cada corrección está indexada por identidad completa `(drone_id,map_epoch,kf_id)`;
2. `L_T_KF` procede de raw y no cambia por optimización global;
3. `W_T_KF` procede de `GlobalPoseStore`;
4. `C_KF` satisface la composición esperada en tests deterministas;
5. nuevos anchors/poses world y optimizaciones actualizan correcciones sin esperar a una pose por frame;
6. un epoch nuevo no reutiliza correcciones del anterior;
7. el método preferido/fallback definido por 5A está instrumentado y funciona en prueba corta y larga;
8. no se usa GT para selección/cálculo;
9. no se bloquea ingesta/publicación principal por este cálculo;
10. build/tests/Gazebo pasan sin errores graves no explicados;
11. historial y docs de paquetes quedan actualizados.

Esta conclusión no garantiza todavía que la pose final tenga error bajo: esa validación pertenece a 5F y puede reabrir 5C.

## Criterio de fallo o parcial

- `NO CONSEGUIDA`: se mezclan epochs, se modifica raw, se usa GT, `C_KF` no reproduce `W_T_KF` en el KF o una optimización no genera actualización.
- `PARCIAL`: matemática/identidad correctas pero faltan correcciones para una fracción significativa de KFs o el fallback domina la prueba larga.
- `BLOQUEADA`: falta la API real de `GlobalPoseStore`, el wrapper/reference KF o Fase 4 necesarios y 5A no pudo resolverlo.

Si 5F posteriormente muestra error incompatible con control y la causa apunta a esta estrategia, reabrir 5C aunque figure previamente `CONSEGUIDA` y añadir una nueva entrada de historial; no borrar la anterior.

## Documentación a actualizar

```text
codex/contexto/00_CONTEXTO_COMPACTACION.md
codex/contexto/paquetes/orbslam3_multi/
codex/contexto/paquetes/orbslam3_server/
codex/pipeline/fase_5_poses_drones_sin_gt/pipeline_fase_5_RESUMEN.md
codex/pipeline/fase_5_poses_drones_sin_gt/historial/INDEX.md
codex/pipeline/fase_5_poses_drones_sin_gt/historial/por_subfase/historial_5C.md
codex/pipeline/fase_5_poses_drones_sin_gt/historial/por_subfase/historial_5C_RESUMEN.md
```

Si 5C cambia la estrategia tras evidencia de 5F, actualizar también `subfase_5C.md` para que el contrato vigente describa el método nuevo, manteniendo intentos previos solo en historial.
