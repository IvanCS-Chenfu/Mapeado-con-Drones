# Subfase 4A — Definición y configuración de fiduciales multicara

## Estado

```text
sin hacer
```

## Dependencia

Ninguna subfase anterior de Fase 4.

## Objetivo técnico

Definir de forma inequívoca el objeto fiducial que utilizará el proyecto: un cubo estático con una marca planar en cada cara habilitada. La configuración debe fijar dimensiones, familia, tamaño físico de marca, `object_id`, `tag_id` independientes por cara, pose global del cubo y transformación rígida de cada tag respecto al centro del cubo.

Esta subfase define geometría y contrato de configuración. No genera todavía modelos Gazebo, no detecta imágenes y no modifica la lógica de anchor del servidor.

## Comportamiento esperado

- Cada cubo tiene un `object_id` lógico.
- Cada cara habilitada tiene un `tag_id` globalmente único.
- Dos caras del mismo cubo nunca comparten `tag_id`.
- La familia baseline es `AprilTag 36h11`.
- El tamaño de tag tiene una definición física única y documentada.
- `world_T_object` y `object_T_tag` se pueden reconstruir sin interpretar nombres de cara de forma implícita.
- La cara inferior queda deshabilitada por defecto, pero el esquema puede soportarla.

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


Leer además:

```text
codex/contexto/paquetes/simulacion_dron/00_summary.md
codex/contexto/paquetes/orbslam3_server/00_summary.md
codex/contexto/paquetes/orbslam3_msgs/00_summary.md
```

Si alguno de estos MD no existe, `planificador_fase` debe localizar el equivalente con búsqueda estática antes de implementar.

## Diagnóstico de partida

El servidor actual representa fiduciales simulados mediante listas de parámetros `fiducials.ids/x/y/z/yaw/radius` y decide una observación a partir de la pose GT del dron dentro de un radio. Ese modelo no representa tags visibles ni caras de un cubo y debe quedar como legacy hasta que las subfases posteriores completen la sustitución.

La configuración nueva debe separar:

```text
objeto físico lógico  != marca planar de una cara
object_id             != tag_id
```

Un ejemplo de intención, no de valores definitivos de escenario:

```yaml
fiducial_models:
  standard_cube:
    shape: box
    size: {x: 0.40, y: 0.40, z: 0.40}
    marker:
      family: APRILTAG_36H11
      size_m: 0.30
      surface_offset_m: 0.002
    enabled_faces:
      top: true
      bottom: false
      front: true
      back: true
      left: true
      right: true
```

## Archivos permitidos a modificar

```text
src/simulacion_dron/config/fiducials.yaml                 # nuevo, configuración de simulación
src/simulacion_dron/CMakeLists.txt                        # instalar config si hace falta
src/simulacion_dron/package.xml                           # solo si se añade dependencia real de validación
codex/contexto/paquetes/simulacion_dron/
codex/pipeline/fase_4_fiducial_real/
```

Si el paquete usa otra organización de `config/`, localizarla antes y documentar la ruta real.

## Archivos prohibidos

```text
ORB_SLAM3/
ORB_SLAM3_ROS2/
src/orbslam3_multi/
src/orbslam3_server/src/global_map_server.cpp
src/orbslam3_msgs/msg/
src/dron_individual/
build/
install/
log/
```

No sustituir todavía el fiducial GT del servidor.

## Funciones, clases o nodos que hay que localizar

```text
simulacion_dron/config/sim_dron.yaml
parámetros fiducials.* existentes en global_map_server
carga/instalación actual de YAML en simulacion_dron
```

No existe todavía una clase de configuración fiducial nueva conocida. `planificador_fase` debe localizar el mecanismo de YAML vigente antes de decidir si conviene un parser C++, Python o launch.

## Cambios requeridos

1. Crear un esquema YAML estable para modelos y objetos fiduciales.
2. Definir `object_id` único por cubo y `tag_id` único por cara habilitada.
3. Fijar `family: APRILTAG_36H11` como baseline de Fase 4.
4. Definir `marker.size_m` con semántica inequívoca: lado físico exterior usado por PnP y por la impresión/generación del tag.
5. Definir dimensiones XYZ del cubo y verificar que `size_m` cabe en cada cara donde se habilita.
6. Definir `world_T_object` mediante pose completa `x,y,z,roll,pitch,yaw` o representación equivalente documentada.
7. Definir explícitamente `object_T_tag`/orientación de cada cara. No depender únicamente de `front/top/...` sin convertirlo a una transformación conocida.
8. Definir el sentido vertical de la textura de cada cara para evitar espejos o rotaciones de 90/180 grados.
9. Añadir `surface_offset_m` pequeño y positivo para que 4B pueda evitar z-fighting sin alterar el tamaño físico usado por visión.
10. Validar IDs duplicados, tamaños no positivos, caras fuera del cubo, poses no finitas y familias no soportadas.
11. Documentar que los YAML reducidos del wrapper y servidor se crearán en subfases posteriores y no deberán leer directamente este YAML de otro grupo.
12. Añadir un marcador de validación o salida determinista, por ejemplo `FID-CONFIG-VALID`, que permita contar objetos/tags y detectar errores sin abrir Gazebo.

## Cambios prohibidos

- No asignar el mismo `tag_id` a varias caras.
- No usar el radio GT actual como geometría del fiducial visual.
- No meter pose GT del dron en el YAML nuevo.
- No decidir todavía algoritmos de fusión multi-tag.
- No crear mensajes ROS nuevos en esta subfase.
- No cambiar los umbrales de optimización de Fase 3.
- No copiar todos los parámetros a otros paquetes por comodidad.

## Paquetes a compilar

Si solo se añade configuración/documentación, compilar el paquete para validar instalación:

```bash
./codex/herramientas/build_selected_packages.sh simulacion_dron
```

Si se crea un validador compilado dentro del paquete, queda incluido en el mismo build.

## Pruebas Gazebo requeridas

### Prueba 1 — Validación de configuración sin spawn

No requiere movimiento de drones. Ejecutar el validador o ruta dry-run acordada sobre un YAML con al menos dos cubos y varias caras.

Resultado esperado:

```text
objetos > 0
tags > objetos
tag_id únicos
transformaciones finitas
marker.size_m compatible con dimensiones
```

### Prueba 2 — Configuración inválida

Usar una copia temporal de prueba con un `tag_id` duplicado y otra con un tamaño imposible. Ambas deben rechazarse explícitamente sin modificar el baseline.

No es obligatorio abrir Gazebo en 4A; la aparición física se valida en 4B.

## Patrones de reducción de logs

```text
FID-CONFIG|object_id|tag_id|duplicate|size_m|face|transform|ERROR|FATAL|Segmentation fault|Killed
```

## Criterio de éxito

1. `simulacion_dron` compila/instala la configuración.
2. Existe un YAML reproducible con al menos un cubo multicara.
3. Cada cara habilitada usa un `tag_id` diferente.
4. Las transformaciones `world_T_object` y `object_T_tag` quedan definidas sin ambigüedad.
5. El tamaño físico usado por PnP queda documentado en metros.
6. Configuraciones inválidas producen error explícito.
7. No se usa GT ni se altera todavía el backend.
8. La documentación e historial futuro se actualizan al ejecutar.

## Criterio de fallo o parcial

- `NO CONSEGUIDA`: IDs ambiguos, tamaño físico indefinido, transforms no reproducibles o YAML no validable.
- `PARCIAL`: el esquema existe pero faltan validaciones críticas o una convención de cara/frame.
- `BLOQUEADA`: la familia/diccionario no existe en la versión de OpenCV disponible y no se ha acordado alternativa.

## Riesgos

- confundir `object_id` con `tag_id`;
- usar distinta interpretación de `size_m` entre textura, PnP e impresión;
- invertir normales de caras o reflejar texturas;
- repetir IDs entre cubos;
- introducir parámetros globales duplicados sin control de consistencia.

## Documentación a actualizar

```text
codex/contexto/paquetes/simulacion_dron/
codex/pipeline/fase_4_fiducial_real/pipeline_fase_4_RESUMEN.md
codex/pipeline/fase_4_fiducial_real/historial/por_subfase/historial_4A.md
codex/pipeline/fase_4_fiducial_real/historial/por_subfase/historial_4A_RESUMEN.md
```

Los archivos de historial anteriores se crean solo cuando haya una ejecución real; no deben prellenarse.
