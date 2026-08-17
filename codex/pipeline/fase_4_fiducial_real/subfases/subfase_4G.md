# Subfase 4G — Interpretación de tags, caras y cubos en el servidor

## Estado

```text
sin hacer
```

## Dependencia

`4A` y `4F`.

## Objetivo técnico

Mover toda la semántica física del fiducial al servidor: conocer qué `tag_id` pertenece a qué `object_id`, la transformación `object_T_tag` de cada cara y la pose global conocida `world_T_object`. A partir de cada `camera_T_tag` sincronizado con un KF, construir candidatos coherentes de pose global de cámara.

El wrapper seguirá enviando una lista plana de tags y no debe modificarse para agrupar cubos.

## Comportamiento esperado

Ejemplo:

```text
KF 37 recibe tags [101, 102, 405]
config servidor:
  101 -> object 1
  102 -> object 1
  405 -> object 4

servidor:
  object 1: dos candidatos desde dos caras
  object 4: un candidato
```

Las dos caras del objeto 1 no se cuentan como dos cubos. El servidor conserva evidencia por tag para poder diagnosticar incoherencias.

## Contexto obligatorio a leer


```text
AGENTS.md
codex/contexto/00_CONTEXTO_COMPACTACION.md
codex/contexto/CONTEXTO_MINIMO_ACTUAL.md
codex/contexto/08_POLITICA_TOKENS_DOCUMENTACION.md
codex/pipeline/PIPELINE_MAESTRO.md
codex/pipeline/fase_4_fiducial_real/pipeline_fase_4_RESUMEN.md
codex/pipeline/fase_4_fiducial_real/pipeline_fase_4.md
```


Además:

```text
subfases/subfase_4A.md
subfases/subfase_4F.md
src/orbslam3_server/src/global_map_server.cpp
config/launch actual de orbslam3_server
```

## Diagnóstico de partida

El servidor actual usa `FiducialConfig {id, world_T_fiducial, radius_m}` y parámetros planos `fiducials.ids/x/y/z/yaw/radius`. No modela caras ni `tag_id`. La estructura debe evolucionar a un mapa de objetos y tags sin introducir esta semántica en `orbslam3_multi` salvo el resultado ya normalizado que necesite el backend.

## YAML del servidor

Crear una configuración local reducida, ruta propuesta:

```text
src/orbslam3_server/config/fiducials.yaml
```

Esquema conceptual:

```yaml
objects:
  - object_id: 1
    world_pose: {x: 0.0, y: 9.0, z: 1.0, roll: 0.0, pitch: 0.0, yaw: 0.0}
    tags:
      - tag_id: 101
        object_T_tag: ...
      - tag_id: 102
        object_T_tag: ...
```

No leer directamente `simulacion_dron/config/fiducials.yaml` en runtime. Los campos compartidos deben mantenerse consistentes mediante documentación/test.

## Archivos permitidos a modificar

```text
src/orbslam3_server/config/fiducials.yaml
src/orbslam3_server/include/orbslam3_server/global_map_server.hpp
src/orbslam3_server/src/global_map_server.cpp
src/orbslam3_server/launch/global_orb_map_server.launch.py
src/orbslam3_server/CMakeLists.txt
src/orbslam3_server/package.xml
codex/contexto/paquetes/orbslam3_server/
```

Se puede añadir un parser/componente separado si evita engordar `global_map_server.cpp`; localizar primero la arquitectura vigente.

## Archivos prohibidos

```text
ORB_SLAM3/
ORB_SLAM3_ROS2/
src/simulacion_dron/config/fiducials.yaml       # no leer/modificar desde servidor como atajo
src/orbslam3_multi/src/fiducial_anchor_manager.cpp   # anchor se adapta en 4H
```

## Funciones, clases o nodos que hay que localizar

```text
FiducialConfig actual
LoadFiducialConfig
FindContainingFiducial
ProcessFiducialsForDelta
callback sincronizado creado en 4F
```

Los nombres de nuevos tipos (`FiducialObjectConfig`, `FiducialTagConfig`, etc.) son propuestas; si existen tipos equivalentes, reutilizarlos.

## Cambios requeridos

1. Crear parser del YAML local del servidor con `object_id`, `world_T_object` y lista de tags con `object_T_tag`.
2. Construir índices `tag_id -> object_id/config` para lookup O(1) o equivalente.
3. Validar globalmente que un `tag_id` no pertenezca a dos objetos.
4. Para cada observación sincronizada calcular:

```text
world_T_tag    = world_T_object * object_T_tag
world_T_camera = world_T_tag * inverse(camera_T_tag)
```

5. Conservar `tag_id`, `object_id`, error de reproyección y fuente para auditoría.
6. Agrupar dentro del mismo KF los candidatos que comparten `object_id`.
7. Si varias caras del mismo cubo son coherentes, mantener un candidato lógico del cubo. Baseline obligatorio: seleccionar como representante el tag válido con menor error de reproyección; la fusión matemática multi-cara es opcional y no necesaria para cerrar 4G.
8. Si varias caras del mismo cubo son incompatibles por encima de umbrales de consistencia, no elegir silenciosamente una; marcar la observación para rechazo/diagnóstico que se formalizará en 4J.
9. Si el KF observa cubos diferentes, conservar un candidato lógico por cubo y comprobar que sus `world_T_camera` son comparables.
10. Añadir logs `FID-OBJECT-MAP`, `FID-OBJECT-CANDIDATE`, `FID-OBJECT-MULTIFACE`.
11. Crear un test de consistencia entre YAML de simulación y servidor para IDs/poses compartidas cuando ambos estén disponibles, sin que uno importe al otro en runtime.
12. Dejar inactiva la antigua lógica de radio GT para la ruta visual; no usar `FindContainingFiducial` sobre observaciones nuevas.

## Cambios prohibidos

- No agrupar en wrapper.
- No usar la pose GT del dron para elegir cubo.
- No inferir el cubo por cercanía espacial del dron.
- No tratar cada cara del mismo cubo como fiducial lógico diferente en backend.
- No fusionar ciegamente candidatos incompatibles.
- No anclar aún si 4H no está implementada.
- No modificar thresholds de revisit para esconder incoherencias de cara.

## Paquetes a compilar

```bash
./codex/herramientas/build_selected_packages.sh orbslam3_server orbslam3_multi
```

`orbslam3_multi` solo si un tipo compartido real cambia.

## Pruebas Gazebo requeridas

### Prueba 1 — Dos caras del mismo cubo

Producir un KF con dos tags visibles del mismo `object_id`. El servidor debe registrar dos tags pero un único grupo lógico de objeto y dos estimaciones `world_T_camera` coherentes dentro del umbral acordado.

### Prueba 2 — Dos cubos distintos

Producir un KF que vea tags de dos `object_id`. Deben aparecer dos grupos lógicos y ambos deben referirse al mismo KF exacto.

### Prueba 3 — Tag desconocido

Inyectar/detectar un ID no presente en config. Debe rechazarse como `unknown_tag` sin afectar los demás tags válidos del batch.

## Patrones de reducción de logs

```text
FID-OBJECT|object_id|tag_id|MULTIFACE|unknown_tag|world_T_camera|consisten|inconsisten|ERROR|FATAL|Segmentation fault|Killed
```

## Criterio de éxito

1. El servidor compila y carga su YAML local.
2. Cada tag se resuelve a un único objeto o se rechaza.
3. Dos caras del mismo cubo se reconocen como el mismo `object_id`.
4. Cubos distintos se mantienen como restricciones diferentes del mismo KF.
5. `world_T_camera` se calcula con convención validada y sin GT.
6. Configs incoherentes/IDs duplicados se rechazan.
7. La ruta visual no usa el radio GT legacy.

## Criterio de fallo o parcial

- `NO CONSEGUIDA`: tag-cubo ambiguo, transformaciones invertidas o uso de GT para resolver asociación.
- `PARCIAL`: lookup correcto pero falta manejo multi-cara/multi-cubo coherente.
- `BLOQUEADA`: no existe una convención de frames acordable a partir de la configuración/calibración disponible.

## Riesgos

- invertir `object_T_tag`;
- pose global del cubo distinta entre simulación y servidor;
- seleccionar por reproyección una cara con orientación de textura incorrecta;
- duplicar IDs entre objetos.

## Documentación a actualizar

```text
codex/contexto/paquetes/orbslam3_server/
codex/pipeline/fase_4_fiducial_real/historial/por_subfase/historial_4G.md
codex/pipeline/fase_4_fiducial_real/historial/por_subfase/historial_4G_RESUMEN.md
```
