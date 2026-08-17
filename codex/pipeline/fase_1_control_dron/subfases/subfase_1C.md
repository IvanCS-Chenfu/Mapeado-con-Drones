# Subfase 1C — Nodo generador e inserción de uno o varios drones

## Estado

```text
realizado
```

## Dependencias

```text
1A — Gazebo y /spawn_entity
1B — Xacro/URDF físico válido
```

## Objetivo técnico

Crear el nodo `generador_URDF` que lee configuración física y de simulación, procesa `dron_plugins.xacro`, llama a `/spawn_entity` y crea una instancia con nombre y namespace propios. Integrarlo en un launch multi-dron capaz de generar uno o más drones dentro de una única instancia de Gazebo.

## Comportamiento esperado

- `dron.numero: 1` crea una entidad.
- `dron.numero: N` crea N entidades `namespace_base_1 ... namespace_base_N`.
- Cada entidad y cada nodo de dron quedan en un namespace independiente.
- Gazebo se abre una sola vez.
- El Xacro recibe el mismo `hardware.yaml` para todas las instancias salvo que se acuerde una configuración por dron.
- La zona de spawn procede de YAML.
- Un fallo de Xacro o spawn se registra con el dron afectado.

## Contexto obligatorio a leer

```text
AGENTS.md
codex/pipeline/fase_1_control_dron/pipeline_fase_1_RESUMEN.md
codex/pipeline/fase_1_control_dron/subfases/subfase_1A.md
codex/pipeline/fase_1_control_dron/subfases/subfase_1B.md
codex/contexto/paquetes/simulacion_dron/00_summary.md
codex/contexto/paquetes/dron_individual/00_summary.md
```

## Diagnóstico de partida

La implementación de referencia contiene:

```text
src/simulacion_dron/src/generar_URDF/generador_URDF.cpp
src/simulacion_dron/launch/multi_dron.launch.py
src/dron_individual/launch/generar_dron.launch.py
src/simulacion_dron/config/sim_dron.yaml
src/dron_individual/config/hardware.yaml
```

`generador_URDF`:

- declara parámetros físicos, actuadores y sensores;
- obtiene el namespace del nodo;
- espera `/spawn_entity`;
- ejecuta `xacro` mediante `popen`;
- usa el namespace sin `/` como nombre de entidad;
- selecciona `x` e `y` aleatorios dentro de `dron.spawn_box`;
- usa `z = 0.15` y `reference_frame = world`;
- finaliza después de recibir la respuesta del servicio.

## YAML obligatorio de simulación

```text
src/simulacion_dron/config/sim_dron.yaml
```

Contrato mínimo:

```yaml
/**:
  ros__parameters:
    dron.numero: 2
    dron.namespace_base: "dron"
    dron.spawn_box: [x1, x2, y1, y2]   # m en world
    world.activar: "empty"
```

Validaciones:

1. `dron.numero >= 1`.
2. `namespace_base` no vacío y apto para nombres ROS.
3. `spawn_box` tiene cuatro números finitos.
4. `x1 <= x2` y `y1 <= y2`.
5. La caja no sitúa deliberadamente drones dentro de obstáculos.
6. La altura inicial debe parametrizarse o documentarse si permanece fija.
7. Con varios drones debe minimizarse el riesgo de solapamiento; si se usa aleatoriedad, la semilla y la política de reintento deben quedar documentadas.

## Relación con `hardware.yaml`

El launch pasa a cada `generador_URDF`:

```text
hardware.yaml
sim_dron.yaml
use_sim_time
drone_id
drone_name
```

Los valores físicos siguen siendo arbitrarios. Cambiar el número de drones no debe cambiar silenciosamente masa, geometría o sensores.

## Archivos permitidos a modificar

```text
src/simulacion_dron/src/generar_URDF/generador_URDF.cpp
src/simulacion_dron/launch/multi_dron.launch.py
src/simulacion_dron/launch/gazebo.launch.py
src/simulacion_dron/config/sim_dron.yaml
src/simulacion_dron/urdf/dron_plugins.xacro
src/simulacion_dron/CMakeLists.txt
src/simulacion_dron/package.xml
src/dron_individual/launch/generar_dron.launch.py
src/dron_individual/config/hardware.yaml
codex/contexto/paquetes/simulacion_dron/
codex/contexto/paquetes/dron_individual/
```

## Archivos prohibidos

```text
src/dron_individual/src/control_tray/
src/lib_tray/
src/simulacion_dron/src/plugins/
src/orbslam3_multi/
src/orbslam3_server/
ORB_SLAM3/
orbslam3_ros2/
```

No implementar todavía movimiento, sensores o GUI.

## Funciones, clases y nodos a localizar

```text
Clase_Cliente
generador_URDF
read_xacro
enviar_datos
gazebo_msgs::srv::SpawnEntity
/spawn_entity
request->name
request->robot_namespace
request->reference_frame
PushRosNamespace
GroupAction
IncludeLaunchDescription
generate_launch_description
```

## Cambios requeridos

1. Leer y validar el YAML antes de indexar sus vectores.
2. Resolver `dron_plugins.xacro` desde el share del paquete.
3. Procesar Xacro con todos los parámetros físicos necesarios.
4. Detectar fallo de `popen`, salida vacía y código de retorno de Xacro.
5. Esperar `/spawn_entity` con una política de timeout o cancelación documentada; evitar espera infinita sin diagnóstico.
6. Asignar nombre de entidad y `robot_namespace` únicos.
7. Generar N grupos con `PushRosNamespace` y un solo Gazebo.
8. Pasar `drone_id`, `drone_name` y `use_sim_time` de forma consistente.
9. Evitar arrancar en esta prueba componentes de fases posteriores que oculten un fallo de spawn.
10. Emitir logs identificables por namespace y resultado del servicio.

## Cambios prohibidos

- No crear una copia del Xacro por dron.
- No usar nombres de entidad repetidos.
- No abrir Gazebo dentro de cada launch individual.
- No asumir que la respuesta de `/spawn_entity` es correcta solo porque el future terminó.
- No usar rutas absolutas.
- No usar GT para decidir la posición inicial.
- No declarar compatibilidad multi-dron si los topics quedan fuera de namespace.

## Paquetes a compilar

```bash
./codex/herramientas/build_selected_packages.sh dron_individual simulacion_dron
```

## Pruebas Gazebo requeridas

### Prueba 1 — Un dron

YAML:

```yaml
dron.numero: 1
dron.namespace_base: "dron"
dron.spawn_box: [-1.0, 1.0, -1.0, 1.0]
world.activar: "empty"
```

Comando:

```bash
./codex/herramientas/run_simulation.sh \
  --prueba fase1_1C_un_dron \
  --launch "ros2 launch simulacion_dron multi_dron.launch.py" \
  --post-scenario-wait-sec 10
```

Comprobar:

```text
entidad dron_1
namespace /dron_1
un único proceso Gazebo
modelo visible
```

### Prueba 2 — Varios drones

Usar al menos `dron.numero: 2` y una caja suficientemente grande. Verificar:

```text
dron_1, dron_2, ...
/gazebo único
/gazebo/spawn o /spawn_entity único
nodos y topics namespaced
entidades no sobrescritas
```

### Prueba 3 — Error controlado

Provocar un fallo reproducible de Xacro o nombre duplicado y comprobar que el log identifica el namespace y el motivo.

## Patrones de reducción de logs

```text
generador_URDF|spawn_entity|Robot model insertado|Fallo al insertar|URDF vacío|xacro|Servicio no disponible|namespace|dron_[0-9]+|ERROR|FATAL|Segmentation fault|Killed
```

## Criterio de éxito

1. Los paquetes compilan.
2. Un dron se inserta correctamente.
3. Dos o más drones se insertan en una única instancia Gazebo.
4. Los nombres de entidad y namespaces son únicos.
5. La zona de spawn procede de YAML y se valida.
6. El Xacro recibe la configuración física completa.
7. Los fallos de servicio/Xacro no se confunden con éxito.
8. La documentación de los paquetes queda sincronizada.

## Criterio de fallo o parcial

- `NO CONSEGUIDA`: no se inserta el dron, se abre más de un Gazebo, se sobrescriben entidades o los namespaces colisionan.
- `PARCIAL`: un dron funciona pero la inserción múltiple, validación YAML o diagnóstico de errores no cumple.
- `BLOQUEADA`: `/spawn_entity` no existe pese a cumplir 1A o falta una dependencia externa crítica.

## Riesgos

- posiciones aleatorias solapadas;
- espera infinita del servicio;
- `popen` sin comprobar código de salida;
- indexación de vectores YAML cortos;
- nombre vacío al ejecutar el nodo sin namespace;
- launch actual cargado con elementos del mapa sparse que no pertenecen a esta subfase.

## Documentación a actualizar al ejecutar

```text
codex/contexto/paquetes/simulacion_dron/
codex/contexto/paquetes/dron_individual/
codex/pipeline/fase_1_control_dron/historial/por_subfase/historial_1C.md
codex/pipeline/fase_1_control_dron/historial/por_subfase/historial_1C_RESUMEN.md
```
