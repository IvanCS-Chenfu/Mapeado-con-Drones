# Subfase 2C — Reorganizar YAML, parámetros, launch y recursos de configuración

<!-- ACUERDOS_CIERRE_F2_2026_08_24_START -->
## Acuerdo definitivo de configuración para ejecutar 2C

> **Vigencia:** acuerdo cerrado el 2026-08-24. Este bloque prevalece sobre cualquier
> frase anterior incompatible del mismo documento. No borra ni reescribe evidencia
> histórica; distingue siempre entre estado actual, deuda conocida y arquitectura objetivo.

### Modelo de ownership
No imponer la regla antigua “toda réplica debe ser parcial” como absoluta. La regla es:
- duplicado accidental: prohibido;
- réplica parcial declarada: permitida con lista de claves;
- réplica completa de deployment profile: permitida solo si está declarada, justificada
  y guardada. `global_map` Servidor↔Simulación permanece completo.

### Caja negra Dron
Simulación no puede abrir `dron_individual/config/hardware.yaml`. Los datos necesarios
para Gazebo se trasladan/replican en Simulación según su ownership real:
- propiedades exclusivas de Gazebo/modelo: propiedad de Simulación;
- intrínsecos Dron usados también por Simulación: réplica declarada;
- parámetros de control/actuadores de Dron: propiedad de Dron; replica solo las claves
  que Simulación consuma.

`actuadores.conversor.fuerza2torque` es un ejemplo de dato Dron consumido también por
el modelo simulado y debe tratarse como réplica declarada, no como dos propietarios.

### Reloj e identidad
Standalone Dron/Servidor: `use_sim_time=false`. Simulación pasa `true`.
`drone_count`, namespace e identidad/selección por ejecución deben tener autoridad
visible en launch y no competir silenciosamente con YAML operacionales.

### Deudas concretas que se corregirán al implementar
- eliminar `control.tray.usar_veltrap` y su launch argument tras un `rg` completo local;
- corregir `<mass value="1.0"/>` para usar `fisico_cuerpo_masa`;
- definir bootstrap/preflight del `ORBvoc.txt` completo;
- conservar `body_T_camera` como intrínseco Dron y documentar transporte futuro;
- no mover/refactorizar el fiducial actual: Fase 4 es su propietaria;
- no retirar GT de control: Fase 5 es su propietaria.

### Artefactos
La prohibición es que los paquetes funcionales escriban/lean runtime desde `src/`.
`codex/archivos_auxiliares` queda permitido como evidencia diagnóstica de Codex.
<!-- ACUERDOS_CIERRE_F2_2026_08_24_END -->

## Estado

```text
SIN HACER
Dependencia: 2A completada y 2B con builds suficientes para trabajar
Objetivo: ownership claro, configuración reutilizable y sin duplicados internos
```

## Objetivo técnico

Reorganizar los archivos YAML y su carga desde launch para que:

- cada parámetro tenga un propietario lógico;
- un mismo dato no se repita dentro de un grupo;
- los paquetes de un grupo puedan reutilizar YAML de otros paquetes del mismo
  grupo mediante recursos instalados;
- ningún paquete cargue directamente YAML de otro grupo;
- cuando un grupo necesite un dato de otro, exista una réplica parcial local y
  verificable;
- los parámetros físicos completos del dron se consuman desde un único YAML;
- los parámetros de control, trayectoria, cámara, ORB-SLAM3, servidor,
  simulación y debug queden separados por responsabilidad;
- todos los flags de debug sean independientes y `false` por defecto;
- los escenarios runtime residan en Simulación y no en `codex`;
- los resultados generados dejen de escribirse dentro de `src/`.

## Contexto obligatorio a leer

```text
AGENTS.md
codex/contexto/00_CONTEXTO_COMPACTACION.md
codex/pipeline/fase_2_separacion_paquetes/pipeline_fase_2.md
codex/pipeline/fase_2_separacion_paquetes/subfases/subfase_2B.md
codex/contexto/paquetes/dron_individual/*.md
codex/contexto/paquetes/orbslam3_ros2/*.md
codex/contexto/paquetes/orbslam3_server/*.md
codex/contexto/paquetes/orbslam3_multi/*.md
codex/contexto/paquetes/simulacion_dron/*.md
```

Después inventariar físicamente todos los `.yaml`, `.yml`, launch, Xacro, URDF,
configuración de RViz, escenarios y parámetros declarados en C++/Python.

## Diagnóstico de partida conocido

El snapshot entregado muestra al menos estas mezclas:

- `dron_individual/config/hardware.yaml` contiene geometría, masas e inercias
  por pieza, conversión de actuadores, GT y cámara simulada;
- `dron_individual/config/tray_dron.yaml` contiene masa/inercia total, geometría,
  gravedad, ganancias, conversión de actuadores y parámetros de trayectoria;
- `dron_individual/config/vision.yaml` usa strings para booleanos de ORB-SLAM3;
- existen calibraciones ORB-SLAM3 y vocabulario tanto en Dron como en
  Simulación;
- `simulacion_dron/config/sim_dron.yaml` mezcla número de drones, namespace,
  zona de spawn y mundo;
- `multi_dron.launch.py` declara numerosos parámetros de servidor y debug como
  argumentos de launch;
- algunas salidas de debug apuntan a `src/codex/archivos_auxiliares/`;
- el escenario largo típico vive actualmente bajo `codex/archivos_auxiliares/`.

La reorganización debe partir de consumidores reales; no basta con renombrar
archivos.

## Reglas permanentes

### 1. Propiedad dentro de un grupo

Cada parámetro tiene un único YAML propietario dentro de su grupo. Otros
paquetes del grupo lo cargan desde launch, no crean otra copia.

Ejemplo:

```text
src/dron/dron_individual/config/physical.yaml
```

Puede ser cargado por launch de `dron_individual` y del wrapper si necesitan
alguna clave, siempre que ambos estén en `dron`.

### 2. Prohibición de carga entre grupos

No se permite:

```text
Servidor -> cargar YAML instalado por Dron
Dron -> cargar YAML instalado por Servidor
Dron/Servidor -> cargar YAML instalado por Simulación
```

Simulación tampoco carga directamente un YAML de Dron o Servidor por ruta
física. Si necesita el dato, crea una réplica parcial local.

### 3. Réplica parcial local

Formato de nombre:

```text
<nombre_yaml_origen>_<grupo_origen>.yaml
```

Ejemplo:

```text
Dron:       physical.yaml
Simulación: physical_dron.yaml
```

La réplica contiene solo parámetros realmente usados por el grupo receptor.
Cada archivo debe declarar en comentarios o metadatos:

```text
origen conceptual
grupo de origen
paquete y YAML de origen
claves replicadas
consumidores locales
regla de igualdad
política de modificación
```

No copiar configuraciones enteras “por si acaso”.

### 4. No duplicación semántica

Dos claves con nombres distintos pero el mismo significado siguen siendo un
duplicado. Antes de crear un parámetro nuevo, localizar todos los consumidores
y elegir un propietario.

### 5. Carga desde launch

Los nodos reciben listas ordenadas de YAML y overrides explícitos. Los launch:

- localizan recursos con `get_package_share_directory`, `FindPackageShare` o
  mecanismo equivalente;
- no usan `../../` ni rutas absolutas al repositorio;
- no abren YAML de otro grupo;
- instalan todos los recursos necesarios;
- aplican overrides por entorno de forma visible.

## Arquitectura objetivo de YAML

La lista es objetivo lógico, no obligación de crear archivos vacíos. Cada YAML
solo se crea si existen parámetros de esa responsabilidad.

### Grupo Dron

#### `physical.yaml`

Propietario sugerido:

```text
src/dron/dron_individual/config/physical.yaml
```

Responsabilidad:

- `mass_total` del dron completo;
- `inertia_total` del dron completo;
- gravedad si se considera propiedad física común;
- dimensiones globales realmente usadas por control/actuadores;
- parámetros físicos inmutables del dron real.

`mass_total` e `inertia_total` son consumidos por trayectoria/control desde
launch. Se eliminan copias con el mismo significado de otros YAML.

No se recalculan en esta fase. En hardware real, `inertia_total` se medirá con
el procedimiento adecuado.

#### `control.yaml`

Responsabilidad:

- ganancias `Kp`, `Kd` o equivalentes;
- límites y saturaciones de control;
- frecuencias propias del controlador;
- parámetros que cambian el comportamiento del control.

No contiene masa ni inercia si ya las recibe desde `physical.yaml`.

#### `trajectory.yaml`

Responsabilidad:

- selección de generador;
- límites de velocidad/aceleración;
- tiempos de aceleración;
- tolerancias y parámetros de ejecución de trayectoria.

No repite propiedades físicas ni ganancias del controlador.

#### `actuators.yaml`

Responsabilidad:

- mezcla o conversión de fuerza/torque;
- configuración de 4/6/8 motores;
- límites de actuadores que pertenezcan al dron.

Si un valor también lo necesita el modelo simulado, Simulación mantiene su
réplica parcial local.

#### `camera.yaml`

Responsabilidad:

- calibración intrínseca/extrínseca propia del dron;
- resolución y baseline del sensor real/objetivo;
- topics y frames de cámara del grupo Dron cuando sean parte del contrato local.

No mezclar parámetros de ORB extractor ni debug de Gazebo.

#### `orbslam3.yaml`

Responsabilidad:

- modo mono/stereo;
- activación del wrapper;
- parámetros ORB-SLAM3 y extractor;
- tracking y publicación local;
- rutas instaladas al vocabulario/configuración.

La calibración puede mantenerse separada si el formato exigido por ORB-SLAM3
lo requiere. La documentación debe explicar la relación entre ambos archivos.

### Grupo Servidor

Separar por propietario algorítmico:

```text
ingestion.yaml
publication.yaml
secondary_worker.yaml
landmark_scoring.yaml
landmark_fusion.yaml
loop_detection.yaml
loop_verification.yaml
pose_graph_optimization.yaml
```

#### `ingestion.yaml`

- topics/servicios de entrada;
- reconciliación delta/snapshot;
- límites de buffers de ingesta;
- parámetros de identidad de drones/submapas.

#### `publication.yaml`

- topics globales;
- frecuencia/heartbeat;
- opciones de KeyFrames, nube y labels;
- parámetros de visualización que pertenezcan al servidor, no al arranque de la
  aplicación de debug.

#### `secondary_worker.yaml`

- capacidad de cola;
- prioridades o límites operativos;
- backpressure/telemetría del worker;
- parámetros de scheduling realmente vigentes.

#### `landmark_scoring.yaml`

- umbrales y pesos del score centralizado;
- criterios de publicación de landmarks.

#### `landmark_fusion.yaml`

- umbrales y reglas de fused tracks;
- aceptación/rechazo de inliers para fusión;
- límites de consistencia de landmarks.

Si existe otro proceso llamado “fusión” con responsabilidad distinta, crear un
YAML adicional con nombre inequívoco solo después de localizar su código.

#### `loop_detection.yaml`

- BoW y filtros baratos;
- límites de candidatos;
- políticas temporales/causales.

#### `loop_verification.yaml`

- matching;
- RANSAC;
- subnubes;
- inliers y errores geométricos.

#### `pose_graph_optimization.yaml`

- selección de vértices/aristas;
- uso de covisibilidad;
- umbrales de aceptación;
- límites de solver;
- validación y rollback;
- exportaciones de grafo solo como parámetros funcionales, mientras su
  activación visual se gobierna desde debug de Simulación.

### Grupo Simulación

#### `simulation.yaml`

- mundo activo;
- reloj simulado;
- comportamiento general de integración;
- parámetros exclusivos de Gazebo.

#### `spawn.yaml`

- número de drones;
- namespace base;
- posiciones iniciales o caja de spawn;
- IDs y disposición del escenario.

El número de drones no puede vivir en Dron ni Servidor.

#### `simulated_sensors.yaml`

- frecuencia de GT;
- ruido y latencia simulados;
- publicación de sensores;
- parámetros de plugins simulados.

#### Réplicas parciales

Ejemplos:

```text
physical_dron.yaml
camera_dron.yaml
actuators_dron.yaml
```

Solo se crean si Simulación consume esas claves. No se copia todo el YAML de
Dron.

#### `debug.yaml`

Contiene un flag independiente por función. Todos los valores predeterminados
son booleanos YAML reales y `false`.

Ejemplo objetivo:

```yaml
/**:
  ros__parameters:
    debug_pipeline_flow_web: false
    debug_open_pipeline_flow_browser: false
    debug_system_architecture_web: false
    debug_open_system_architecture_browser: false
    debug_sparse_global_rviz: false
    debug_flow_telemetry: false
    debug_architecture_telemetry: false
    debug_optimization_animation: false
    debug_pose_graph_dump: false
    debug_verbose_loop: false
    debug_verbose_optimization: false
```

Los nombres definitivos deben seguir la convención del proyecto y tener un
consumidor claro.

## Booleanos y tipos

Corregir strings como:

```yaml
activar: "true"
```

hacia booleanos reales:

```yaml
activar: true
```

solo después de comprobar cómo los consume el launch/nodo. Si el consumidor
espera string por una limitación concreta, corregirlo o documentar la excepción.
No realizar conversiones silenciosas que cambien condiciones de launch.

## Masa, inercia y URDF

### Dron completo

Fuente de verdad:

```text
src/dron/dron_individual/config/physical.yaml
```

Contiene `mass_total` e `inertia_total`. Trayectoria y control los reciben por
launch.

### Modelo simulado por piezas

Las masas e inercias de cuerpo, brazos, motores, cámaras y otros enlaces sirven
al modelo/URDF de Simulación. No se equiparan automáticamente a
`inertia_total`.

La Fase 2 no debe:

- calcular la inercia total desde las piezas;
- validar físicamente el URDF como modelo definitivo del futuro dron real;
- reescribir masivamente el generador URDF;
- eliminar parámetros por pieza sin comprobar su consumidor.

Sí debe:

- mover la propiedad de dichos parámetros al grupo Simulación si solo se usan
  para Gazebo/URDF;
- documentar la diferencia;
- evitar que el controlador lea por accidente la inercia de una pieza.

## Escenarios y salidas

Mover los YAML de escenarios ejecutables desde:

```text
codex/archivos_auxiliares/trayectorias/
```

hacia una ruta instalada por Simulación, por ejemplo:

```text
src/simulacion/simulacion_dron/config/scenarios/
```

Conservar en `codex` solo:

- evidencia histórica;
- resultados reducidos;
- documentación;
- catálogos o referencias a escenarios.

Cambiar las rutas de salida que apuntan a `src/codex/archivos_auxiliares/html`
o `repeticiones` por:

- `log/<grupo>/...`;
- una ruta de resultados configurable;
- un directorio temporal.

No escribir resultados runtime dentro de `src/`.

## Proceso de migración

1. inventariar todos los parámetros y consumidores;
2. crear una tabla `parámetro -> propietario -> consumidores -> política`;
3. detectar duplicados semánticos;
4. acordar nombres finales cuando el código sea ambiguo;
5. crear los YAML nuevos en cambios pequeños;
6. adaptar launch para componerlos;
7. adaptar nodos solo cuando sea necesario para nombres/tipos;
8. eliminar claves antiguas después de comprobar que no tienen consumidores;
9. instalar todos los YAML;
10. mover escenarios y corregir rutas;
11. ejecutar smoke tests de carga de parámetros;
12. actualizar documentación de cada paquete tocado.

No mover todos los parámetros en un único cambio sin build intermedio.

## Archivos probables

```text
src/dron/dron_individual/config/*.yaml
src/dron/dron_individual/launch/*.py
src/dron/orbslam3_ros2/config/**
src/dron/orbslam3_ros2/launch/**

src/servidor/orbslam3_server/config/*.yaml
src/servidor/orbslam3_server/launch/*.py
src/servidor/orbslam3_multi/config/*.yaml

src/simulacion/simulacion_dron/config/**
src/simulacion/simulacion_dron/launch/*.py
src/simulacion/simulacion_dron/urdf/**
src/simulacion/simulacion_dron/src/generar_URDF/**
src/simulacion/simulacion_dron/CMakeLists.txt
src/simulacion/simulacion_dron/package.xml

src/codex/archivos_auxiliares/**
src/codex/contexto/paquetes/**
```

La lista es orientativa. Confirmar físicamente cada consumidor.

## Cambios prohibidos

- No cargar un YAML desde otro grupo.
- No copiar todos los parámetros de un YAML entre grupos.
- No duplicar masa total dentro del grupo Dron.
- No calcular ni alterar valores físicos por intuición.
- No fusionar calibración de cámara y parámetros ORB-SLAM3 si sus formatos o
  propietarios son distintos.
- No agrupar todos los debug en un solo flag.
- No dejar flags debug en `true` por defecto.
- No eliminar un parámetro hasta demostrar que no tiene consumidor.
- No usar rutas al árbol fuente.
- No cambiar umbrales algorítmicos como parte de una mera reorganización.
- No mover evidencia histórica fuera de `codex`.

## Builds requeridos

Después de cada bloque:

- compilar los paquetes propietarios y consumidores del YAML;
- usar las bases de grupo definidas en 2B;
- comprobar instalación del archivo bajo `share/<package>/config`;
- cargar parámetros en un smoke launch sin depender del directorio actual.

Al final:

```text
build Dron aislado
build Servidor aislado
build Simulación como overlay
```

## Pruebas requeridas

### Prueba 1 — Validación estática de configuración

Comprobar:

- todos los YAML parsean;
- booleanos y listas tienen tipos esperados;
- no hay claves duplicadas en un mismo grupo;
- cada launch localiza recursos instalados;
- las réplicas parciales declaran origen y claves;
- todos los debug predeterminados son `false`.

### Prueba 2 — Smoke de launch

Arrancar los componentes con tiempo acotado y comprobar:

- no aparecen `ParameterNotDeclaredException`;
- no faltan archivos;
- los nodos reciben `mass_total` e `inertia_total` desde `physical.yaml`;
- Simulación recibe sus réplicas locales;
- procesos debug desactivados no arrancan.

### Prueba 3 — Regresión corta

Ejecutar una trayectoria corta de uno o dos drones antes de la prueba larga de
2D. El objetivo es detectar parámetros ausentes o mal tipados, no cerrar la
fase.

## Patrones de reducción

```text
parameter|Parameter|yaml|YAML|No such file|Failed to parse|type mismatch|not declared|FindPackageShare|ament_index|debug_|scenario|ERROR|FATAL|Segmentation fault
```

## Criterio de éxito

`CONSEGUIDA` solo si:

1. existe una tabla completa de ownership;
2. masa e inercia total tienen una única fuente en Dron;
3. control/trayectoria las reciben sin duplicarlas;
4. los parámetros de URDF por pieza quedan diferenciados y documentados;
5. ningún grupo carga YAML de otro grupo;
6. las réplicas parciales contienen solo claves necesarias;
7. los launch usan recursos instalados;
8. los escenarios runtime están en Simulación;
9. los resultados no se escriben dentro de `src/`;
10. cada debug tiene su flag y todos están `false` por defecto;
11. builds y smoke tests pasan.

## Criterio de parcial, fallo o bloqueo

`PARCIAL` si la estructura principal está migrada pero queda un YAML legacy con
consumidor real, claramente documentado y sin duplicación silenciosa.

`NO CONSEGUIDA` si hay parámetros duplicados con valores distintos, carga entre
grupos, debug activo por defecto, masa/inercia ambiguas o launch que dependen de
rutas fuente.

`BLOQUEADA` solo si un componente externo impide saber qué parámetros consume y
no existe documentación o ejecución que permita inferirlo con seguridad.

## Documentación de cierre

Al ejecutar, actualizar:

```text
codex/contexto/paquetes/dron_individual/*.md
codex/contexto/paquetes/orbslam3_ros2/*.md
codex/contexto/paquetes/orbslam3_multi/*.md
codex/contexto/paquetes/orbslam3_server/*.md
codex/contexto/paquetes/simulacion_dron/*.md
codex/contexto/04_TOPICS_SERVICES_ACTIONS.md
codex/contexto/05_MAPA_PAQUETES.md
codex/pipeline/fase_2_separacion_paquetes/historial/por_subfase/historial_2C.md
codex/pipeline/fase_2_separacion_paquetes/historial/por_subfase/historial_2C_RESUMEN.md
```

Cada MD de paquete debe explicar para qué sirve cada YAML, parámetros
importantes, propietarios, consumidores y qué puede o no modificar Codex.
