# Subfase 4B — Generación y spawn de fiduciales multicara en Gazebo

## Estado

```text
sin hacer
```

## Dependencia

`4A` — Definición y configuración de fiduciales multicara.

## Objetivo técnico

Generar representaciones visuales correctas de cada `tag_id`, construir para cada `object_id` un modelo SDF estático con un cubo central y una superficie marcada por cara habilitada, y crear automáticamente las instancias en Gazebo a partir del YAML de 4A.

La posición de los cubos no debe codificarse manualmente en `house_1.world`. La fuente de instancias debe ser el YAML y el spawn debe reutilizar el patrón ROS/Gazebo ya empleado por `generador_URDF.cpp` con `/spawn_entity`.

## Comportamiento esperado

- Al arrancar la simulación aparecen todos los cubos configurados.
- Cada cubo es estático y no cae ni colisiona de manera dinámica con el dron salvo que se decida explícitamente lo contrario.
- Cada cara muestra exactamente el `tag_id` configurado, sin espejo ni giro inesperado.
- Las marcas son visibles desde arriba y desde los laterales correspondientes.
- Cambiar la pose de un objeto en YAML cambia su pose en Gazebo sin editar el world.
- No existe z-fighting apreciable entre la cara del cubo y la textura del tag.

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
codex/pipeline/fase_4_fiducial_real/subfases/subfase_4A.md
codex/contexto/paquetes/simulacion_dron/00_summary.md
```

Localizar antes de implementar:

```text
src/simulacion_dron/src/generar_URDF/generador_URDF.cpp
src/simulacion_dron/launch/multi_dron.launch.py
src/simulacion_dron/worlds/house_1.world
src/simulacion_dron/CMakeLists.txt
```

## Diagnóstico de partida

`simulacion_dron` ya puede arrancar Gazebo y generar drones mediante `/spawn_entity`, pero no existe un objeto visual fiducial multicara derivado del nuevo YAML. El servidor actual simula fiduciales con regiones GT y no existe correlación visual entre esas regiones y un modelo Gazebo.

## Archivos permitidos a modificar

Rutas propuestas, a confirmar contra el layout real antes de editar:

```text
src/simulacion_dron/src/fiducials/fiducial_spawner.cpp
src/simulacion_dron/scripts/generate_fiducial_textures.py
src/simulacion_dron/models/fiducials/
src/simulacion_dron/config/fiducials.yaml
src/simulacion_dron/launch/multi_dron.launch.py
src/simulacion_dron/CMakeLists.txt
src/simulacion_dron/package.xml
codex/contexto/paquetes/simulacion_dron/
```

Si se elige generar el SDF completamente en memoria, la carpeta `models/fiducials/` puede reducirse a texturas/materiales; registrar la decisión en historial.

## Archivos prohibidos

```text
ORB_SLAM3/
ORB_SLAM3_ROS2/
src/orbslam3_server/
src/orbslam3_multi/
src/orbslam3_msgs/
src/dron_individual/
build/
install/
log/
```

## Funciones, clases o nodos que hay que localizar

```text
generador_URDF.cpp
/spawn_entity
gazebo_msgs/srv/SpawnEntity
multi_dron.launch.py
generate_launch_description
```

El nodo nuevo puede denominarse `fiducial_spawner`; si ya existe una utilidad equivalente, reutilizarla y documentar su nombre real.

## Cambios requeridos

1. Generar una imagen PNG lossless y determinista por cada `tag_id` habilitado usando exactamente `AprilTag 36h11`.
2. Conservar bordes y proporciones del marcador; no redimensionar con interpolación que difumine celdas binarias.
3. Construir un SDF por cubo con una geometría central `box`, colisión definida y `static=true`.
4. Crear una visual separada por cara marcada para poder asignar su textura y orientación independientemente.
5. Colocar cada visual a `surface_offset_m` de su superficie para evitar z-fighting.
6. Aplicar exactamente `object_T_tag`/convención de cara definida en 4A; no volver a inferir orientaciones con una convención diferente.
7. Crear un nodo/spawner que cargue `fiducials.yaml`, valide configuración, espere `/spawn_entity` y cree nombres Gazebo únicos como `fiducial_cube_<object_id>`.
8. Rechazar de forma explícita texturas ausentes, IDs duplicados, modelos desconocidos y respuestas de spawn fallidas.
9. Integrar el spawner en `multi_dron.launch.py` después de que Gazebo/factory estén disponibles y antes de que el escenario de movimiento dependa de los tags.
10. Añadir logs estructurados por objeto/cara, por ejemplo `FID-SPAWN-OBJECT`, `FID-SPAWN-TAG`, `FID-SPAWN-ERROR`.
11. Mantener la generación de texturas reutilizable para 4L, de modo que las marcas impresas y simuladas puedan derivarse del mismo ID/familia.

## Cambios prohibidos

- No editar manualmente `house_1.world` para cada instancia como solución principal.
- No usar el mismo tag en varias caras.
- No hacer los cubos dinámicos para compensar errores de pose.
- No implementar detector de imagen en esta subfase.
- No modificar parámetros fiduciales del servidor GT.
- No aplicar filtros visuales o compresión JPEG a los tags.
- No adelantar agrupación tag-cubo del servidor.

## Paquetes a compilar

```bash
./codex/herramientas/build_selected_packages.sh simulacion_dron
```

Si el script de texturas usa `cv2.aruco`, validar también que la dependencia runtime esté instalada; no instalar paquetes globales silenciosamente.

## Pruebas Gazebo requeridas

### Prueba 1 — Spawn de un cubo multicara

Configurar un único cubo con `top + cuatro laterales`. Ejecutar:

```bash
./codex/herramientas/run_simulation.sh \
  --prueba fase4_4B_single_cube \
  --launch "ros2 launch simulacion_dron multi_dron.launch.py" \
  --post-scenario-wait-sec 10
```

Comprobar visualmente desde Gazebo:

1. dimensiones del cubo;
2. pose global;
3. ID/textura correcta de cada cara;
4. ausencia de espejos;
5. visibilidad desde arriba y lateral;
6. ausencia de z-fighting.

### Prueba 2 — Múltiples cubos desde YAML

Configurar al menos dos objetos con poses/orientaciones diferentes. Deben aparecer con nombres únicos y sin tocar el world.

### Prueba 3 — Error controlado

Eliminar temporalmente una textura o duplicar un nombre de entidad en una copia de configuración. El spawner debe producir un error identificable y no declarar éxito silencioso.

## Patrones de reducción de logs

```text
FID-SPAWN|FID-CONFIG|spawn_entity|fiducial_cube|tag_id|texture|SDF|ERROR|FATAL|Segmentation fault|Killed
```

## Criterio de éxito

1. `simulacion_dron` compila.
2. Las texturas se generan para los IDs configurados.
3. Uno y varios cubos aparecen mediante `/spawn_entity` desde YAML.
4. Todas las caras tienen orientación/ID correctos.
5. No hay z-fighting relevante.
6. El world no contiene posiciones duplicadas manualmente como fuente primaria.
7. Los errores de configuración/spawn quedan visibles en logs.
8. No se ha introducido lógica de detección ni GT nuevo.

## Criterio de fallo o parcial

- `NO CONSEGUIDA`: cubos ausentes, tags reflejados/ambiguos, spawn no reproducible o pose diferente al YAML.
- `PARCIAL`: spawn correcto pero persisten problemas visuales de alguna cara o generación manual no automatizada.
- `BLOQUEADA`: Gazebo/factory no disponible o la versión de OpenCV no puede generar el diccionario acordado y no hay alternativa aprobada.

## Riesgos

- rotaciones 90/180° entre caras;
- normales invertidas;
- texturas con interpolación;
- rutas de material no instaladas por CMake;
- spawner ejecutado antes de `/spawn_entity`;
- nombres de entidad duplicados.

## Documentación a actualizar

```text
codex/contexto/paquetes/simulacion_dron/
codex/pipeline/fase_4_fiducial_real/pipeline_fase_4_RESUMEN.md
codex/pipeline/fase_4_fiducial_real/historial/por_subfase/historial_4B.md
codex/pipeline/fase_4_fiducial_real/historial/por_subfase/historial_4B_RESUMEN.md
```
