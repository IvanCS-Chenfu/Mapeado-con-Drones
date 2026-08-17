# Subfase 1A — Entorno base de Gazebo

## Estado

```text
realizado
```

## Dependencia

Ninguna subfase anterior.

## Objetivo técnico

Crear un launch de `simulacion_dron` que arranque una única instancia de Gazebo Classic con el mundo seleccionado en YAML, cargue `libgazebo_ros_factory.so` y deje disponibles el reloj de simulación y `/spawn_entity` para las subfases posteriores.

Esta subfase no inserta drones, no arranca control, no abre la GUI y no inicia ORB-SLAM3 ni el servidor global.

## Comportamiento esperado

Al ejecutar el launch:

1. se abre Gazebo una sola vez;
2. el mundo configurado se resuelve desde `simulacion_dron/worlds/`;
3. el proceso permanece activo sin cierre inmediato;
4. `/spawn_entity` aparece como servicio;
5. `/clock` avanza cuando la simulación está en marcha;
6. un nombre de mundo inexistente produce un error explícito y no una caída silenciosa.

## Contexto obligatorio a leer

```text
AGENTS.md
codex/contexto/00_CONTEXTO_COMPACTACION.md
codex/contexto/CONTEXTO_MINIMO_ACTUAL.md
codex/contexto/08_POLITICA_TOKENS_DOCUMENTACION.md
codex/pipeline/PIPELINE_MAESTRO.md
codex/pipeline/fase_1_control_dron/pipeline_fase_1_RESUMEN.md
codex/pipeline/fase_1_control_dron/pipeline_fase_1.md
codex/contexto/paquetes/simulacion_dron/00_summary.md
```

Como referencia de implementación, localizar en `multi_dron.launch.py` el arranque actual:

```text
gazebo --verbose <world_path> -s libgazebo_ros_factory.so
```

## Diagnóstico de partida

La implementación de referencia arranca Gazebo desde `simulacion_dron/launch/multi_dron.launch.py`, lee `world.activar` de `sim_dron.yaml` y construye la ruta `<nombre>.world`. El launch actual también integra drones, GUI y componentes de fases posteriores; para esta subfase debe existir una ruta de prueba que aísle el entorno base.

Los mundos disponibles en el paquete de referencia incluyen:

```text
simulacion_dron/worlds/empty.world
simulacion_dron/worlds/house_1.world
```

## YAML obligatorio

Archivo principal:

```text
src/simulacion_dron/config/sim_dron.yaml
```

Parámetro mínimo:

```yaml
/**:
  ros__parameters:
    world.activar: "empty"
```

Reglas:

- el valor es el nombre del archivo sin `.world`;
- debe existir en `simulacion_dron/worlds/`;
- no representa hardware del dron, pero sí forma parte del perfil de simulación;
- no se debe fijar una ruta absoluta de una máquina concreta;
- una futura selección por argumento de launch debe conservar el YAML como valor por defecto documentado.

## Archivos permitidos a modificar

```text
src/simulacion_dron/launch/gazebo.launch.py                 # si se crea un launch dedicado
src/simulacion_dron/launch/multi_dron.launch.py             # integración posterior sin duplicar Gazebo
src/simulacion_dron/config/sim_dron.yaml
src/simulacion_dron/worlds/empty.world
src/simulacion_dron/worlds/house_1.world
src/simulacion_dron/CMakeLists.txt
src/simulacion_dron/package.xml
codex/contexto/paquetes/simulacion_dron/
codex/pipeline/fase_1_control_dron/
```

## Archivos prohibidos

```text
src/dron_individual/src/control_tray/
src/lib_tray/
src/orbslam3_multi/
src/orbslam3_server/
src/orbslam3_msgs/
ORB_SLAM3/
orbslam3_ros2/
build/
install/
log/
```

## Funciones, nodos y recursos a localizar

```text
generate_launch_description
ExecuteProcess
gazebo
libgazebo_ros_factory.so
world_path
world.activar
simulacion_dron/worlds/
```

Si existe un launch dedicado ya creado, usarlo como autoridad y evitar crear una segunda ruta equivalente.

## Cambios requeridos

1. Proporcionar un launch invocable con `ros2 launch simulacion_dron ...` que arranque solo Gazebo y el soporte ROS necesario.
2. Resolver el mundo desde el share del paquete mediante sustituciones de launch, no con rutas absolutas.
3. Leer `world.activar` desde `sim_dron.yaml` o aceptar un argumento que use ese valor como default.
4. Cargar `libgazebo_ros_factory.so`, porque `1C` utilizará `/spawn_entity`.
5. Asegurar que `worlds/`, `launch/` y `config/` se instalan mediante CMake.
6. Emitir mensajes de error útiles si el YAML no puede leerse o el mundo no existe.
7. Evitar que el launch base abra RViz2, browser, GUI, ORB-SLAM3, servidor global o drones.

## Cambios prohibidos

- No crear todavía URDF/Xacro.
- No insertar entidades.
- No arrancar `generador_URDF`.
- No añadir plugins de motores, GT o cámaras.
- No usar un world externo sin documentar origen y licencia.
- No iniciar más de un proceso Gazebo desde launches incluidos.
- No considerar “aparece una ventana” como única evidencia.

## Paquetes a compilar

```bash
./codex/herramientas/build_selected_packages.sh simulacion_dron
```

Si falla:

```bash
./codex/herramientas/reduce_build_log.sh
```

Leer únicamente el log reducido generado.

## Pruebas requeridas

### Prueba 1 — Arranque con mundo vacío

Configuración:

```yaml
world.activar: "empty"
```

Comando recomendado:

```bash
./codex/herramientas/run_simulation.sh \
  --prueba fase1_1A_empty \
  --launch "ros2 launch simulacion_dron gazebo.launch.py" \
  --post-scenario-wait-sec 5
```

Comprobaciones:

```bash
ros2 service list | rg '^/spawn_entity$'
ros2 topic hz /clock
```

Observación visual: Gazebo permanece abierto con `empty.world`.

### Prueba 2 — Segundo mundo válido

Cambiar el YAML a `house_1` o pasar el argumento acordado y repetir. El mundo debe cargar sin modificar código.

### Prueba 3 — Mundo inexistente

Usar temporalmente un nombre que no exista. La prueba debe fallar con un diagnóstico identificable y sin afirmar que Gazebo quedó operativo.

## Patrones de reducción de logs

```text
gazebo|Gazebo|world|libgazebo_ros_factory|spawn_entity|clock|ERROR|FATAL|Segmentation fault|Killed
```

Si falta información, regenerar el reducido con patrones más concretos. Nunca abrir el log completo.

## Criterio de éxito

La subfase se considera `CONSEGUIDA` si:

1. `simulacion_dron` compila;
2. Gazebo arranca desde launch con dos mundos válidos seleccionables por configuración;
3. solo existe una instancia Gazebo;
4. `/spawn_entity` está disponible;
5. `/clock` avanza;
6. la ruta del mundo no depende de una ubicación absoluta;
7. un mundo inválido produce un fallo explicable;
8. la documentación del paquete y el historial futuro quedan actualizados.

## Criterio de fallo o parcial

- `NO CONSEGUIDA`: no compila, Gazebo se cierra, el mundo no carga o falta `/spawn_entity`.
- `PARCIAL`: Gazebo arranca, pero la selección de mundo, el reloj o la gestión de errores no cumple el contrato.
- `BLOQUEADA`: falta Gazebo Classic o una dependencia externa imprescindible que no puede instalarse dentro del alcance acordado.

## Riesgos

- mezclar Gazebo Classic con Gazebo Sim/Ignition;
- cargar dos procesos Gazebo por includes recursivos;
- depender de rutas del equipo del autor;
- usar un mundo pesado para el smoke test;
- confundir el nodo `clock` auxiliar con `/clock` de Gazebo.

## Documentación a actualizar al ejecutar

```text
codex/contexto/00_CONTEXTO_COMPACTACION.md
codex/contexto/paquetes/simulacion_dron/
codex/pipeline/fase_1_control_dron/pipeline_fase_1_RESUMEN.md
codex/pipeline/fase_1_control_dron/historial/por_subfase/historial_1A.md
codex/pipeline/fase_1_control_dron/historial/por_subfase/historial_1A_RESUMEN.md
```
