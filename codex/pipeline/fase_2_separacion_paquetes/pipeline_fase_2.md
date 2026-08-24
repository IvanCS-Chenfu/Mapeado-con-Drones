# Pipeline Fase 2 — Separación de paquetes, configuración y despliegue

Resumen de entrada:

```text
codex/pipeline/fase_2_separacion_paquetes/pipeline_fase_2_RESUMEN.md
```

## Estado

```text
FASE: CONSEGUIDA
Preparación documental: CERRADA
Acuerdo cerrado: sí
Autorización funcional de ejecución: EJECUTADA
Prueba acordada: dos drones completan la vuelta al edificio con debug visual activo
Dudas abiertas: ninguna
Revisión visual humana de prueba 200: confirmada correcta
Layout final de system_architecture: validado en escritorio y viewport estrecho
```

Este pipeline conserva el contrato ejecutado. Los resultados viven en
`RESULTADO_FINAL_FASE_2.md` y en `historial/por_subfase/`.

## Objetivo general

Transformar el workspace actual en tres grupos físicamente diferenciados dentro
de `src/`, manteniendo el comportamiento funcional existente:

```text
src/dron/
src/servidor/
src/simulacion/
```

El resultado debe permitir:

1. instalar y compilar el software de `dron` sin paquetes de `servidor` ni de
   `simulacion`;
2. instalar y compilar el software de `servidor` sin paquetes de `dron` ni de
   `simulacion`;
3. construir `simulacion` como integración explícita de las instalaciones de
   `dron` y `servidor`;
4. localizar todos los recursos mediante el índice de paquetes instalado, sin
   rutas frágiles al árbol fuente;
5. entender y modificar la configuración mediante YAML con ownership claro;
6. mantener documentación suficiente para que Codex no tenga que redescubrir
   la nueva distribución en cada sesión;
7. visualizar la arquitectura completa y la actividad de comunicación entre
   paquetes;
8. impedir regresiones mediante guardas automáticas.

## Principios acordados

### Tres grupos ejecutables

La distribución objetivo es:

```text
src/
├── dron/
│   ├── dron_individual/
│   ├── lib_tray/
│   ├── ORB_SLAM3/
│   ├── orbslam3_ros2/
│   └── orbslam3_msgs/
│
├── servidor/
│   ├── orbslam3_multi/
│   ├── orbslam3_server/
│   └── orbslam3_msgs/
│
├── simulacion/
│   └── simulacion_dron/
│
├── codex/
└── mi_tfg/                 # legacy conservado temporalmente
```

`codex` permanece en `src/codex/`; no es un cuarto grupo de despliegue.
`mi_tfg` permanece temporalmente en la raíz como paquete legacy y queda fuera
de los builds y dependencias de los tres grupos. `ORB_SLAM3_MULTI` fue retirado
completamente antes de ejecutar Fase 2.

Los nombres de paquetes ROS 2 no se cambian en Fase 2 salvo que exista una
imposibilidad técnica demostrada. La ruta física puede cambiar sin renombrar el
paquete declarado en `package.xml`.

`ORB_SLAM3` y `orbslam3_ros2` deben localizarse y moverse cuando estén
disponibles en el workspace real. Si su contenido no está accesible durante la
preparación, se conservan como contratos de ruta y no se inventan cambios.

### Interfaces duplicadas deliberadamente

No existe una carpeta compartida de interfaces. El paquete completo
`orbslam3_msgs` aparece en:

```text
src/dron/orbslam3_msgs/
src/servidor/orbslam3_msgs/
```

Ambas copias conservan:

- el mismo nombre de paquete;
- los mismos `.msg`, `.srv` y futuros `.action`;
- manifests y reglas de generación compatibles;
- la misma semántica de campos.

La copia canónica es:

```text
src/servidor/orbslam3_msgs/
```

La copia de `dron` es una réplica controlada. No se permite modificarlas de
forma independiente. La subfase 2G debe comparar contenido y fallar si existe
divergencia.

Esta duplicación impide un build global ingenuo de todo `src/`, porque `colcon`
puede descubrir dos paquetes con el mismo nombre. Por ello los builds se hacen
por grupo.

### Independencia y dependencia permitida

```text
Dron       -> no depende de Servidor ni Simulación
Servidor   -> no depende de Dron ni Simulación
Simulación -> puede depender de Dron y Servidor
```

La independencia se demuestra en entornos limpios. Un build de `dron` no puede
heredar accidentalmente `install/servidor`, y un build de `servidor` no puede
heredar `install/dron`.

`simulacion` se construye y ejecuta como overlay de ambos grupos. Al coexistir
dos instalaciones de `orbslam3_msgs`, la copia de `servidor` debe tener
precedencia intencionada y las guardas deben haber demostrado previamente que
las dos copias son idénticas.

### Comunicación

El proyecto permanece en simulación. Los componentes se comunican directamente
mediante ROS 2:

- topics;
- services;
- actions;
- TF cuando corresponda.

Bluetooth u otro transporte físico puede mencionarse como posible evolución,
pero no se implementa ni se diseña en esta fase.

## Estrategia de compilación

### Idea principal

La subfase 2B elimina completamente:

```text
build/
install/
log/
```

Después crea espacios independientes:

```text
build/
├── dron/
├── servidor/
└── simulacion/

install/
├── dron/
├── servidor/
└── simulacion/

log/
├── dron/
├── servidor/
└── simulacion/
```

Los comandos exactos deben encapsularse en herramientas de `codex/herramientas/`
y usar, cuando sea compatible con la versión instalada de `colcon`:

```text
--base-paths
--build-base
--install-base
--log-base
```

Cada paquete se compila individualmente en orden topológico, con exactamente un
paquete por invocación. Entre paquetes se carga el prefijo local del mismo
grupo. El build limita tanto los workers de `colcon` como el paralelismo interno
de CMake/Make para evitar picos de CPU y memoria. El orden concreto no se
inventa: se obtiene de los manifests, `colcon list` y las dependencias reales.

### Errores que forman parte de la subfase

Si un paquete no compila después del movimiento, 2B debe localizar y corregir,
cuando sea necesario:

- `package.xml` incompleto;
- `CMakeLists.txt` incompleto;
- `#include` o rutas de headers;
- exports de librerías e interfaces;
- dependencias de build, exec o test no declaradas;
- recursos no instalados en `share/<package>`;
- launch que dependen del directorio de trabajo;
- rutas a YAML, mundos, modelos, vocabulario o scripts;
- scripts que asumen `install/setup.bash` único;
- dependencias accidentales de otro grupo.

Las correcciones deben preservar el comportamiento. Si el primer error revela
una decisión funcional no acordada, la autorización se suspende y se consulta
al usuario.

### Alternativas si falla la idea principal

La separación de bases es la opción principal. Solo si existe un bloqueo real,
documentado y no corregible de forma razonable, se prueban estas alternativas:

1. **Workspaces temporales por grupo.** El código sigue en las tres carpetas,
   pero cada build usa un árbol temporal o enlaces controlados que exponen un
   solo grupo.
2. **Bases únicas reutilizadas.** Se limpia `build/install/log` antes de pasar
   de un grupo a otro. Es más simple, pero no conserva los tres resultados a la
   vez.
3. **Selección y exclusión explícitas.** Se usa `--packages-select`,
   `--packages-up-to` o mecanismos equivalentes, asegurando que la segunda
   copia de `orbslam3_msgs` no se descubre.

No se acepta como solución “compilar unos paquetes y omitir otros” sin demostrar
por qué el grupo completo no puede construirse.

## Arquitectura de configuración YAML

### Regla de grupo

Un launch o nodo solo puede cargar YAML instalados dentro de su mismo grupo.
No se permiten rutas directas desde `dron` a `servidor`, desde `servidor` a
`dron`, ni desde cualquiera de ellos a `simulacion`.

Dentro de un mismo grupo, un launch puede cargar YAML pertenecientes a otro
paquete del grupo mediante `ament_index`/`FindPackageShare`.

### Réplicas declaradas entre grupos

Si un grupo necesita un dato conceptual de otro, crea un YAML local parcial:

```text
<nombre_original>_<grupo_origen>.yaml
```

Ejemplo:

```text
src/dron/dron_individual/config/physical.yaml
src/simulacion/simulacion_dron/config/physical_dron.yaml
```

`physical_dron.yaml` contiene solo las claves consumidas por Simulación. Cada
réplica debe documentar:

- grupo de origen;
- paquete y YAML de origen;
- claves replicadas;
- consumidores locales;
- si deben coincidir exactamente;
- política de modificación.

La subfase 2G compara únicamente las claves declaradas como replicadas.

Una copia completa solo se admite cuando representa un `deployment profile`
explícito. `global_map` Servidor-Simulación es la única excepción vigente y la
guarda exige igualdad exacta mientras ambas copias representen el mismo perfil.

### Sin duplicados dentro de un grupo

Un dato tiene un solo propietario dentro de cada grupo. Por ejemplo,
`mass_total` no se repite en `physical.yaml`, `control.yaml` y
`trajectory.yaml`. Los launch componen varios YAML y entregan a cada nodo los
parámetros que necesita.

### Masa e inercia

El grupo `dron` debe disponer de un YAML físico que contenga al menos:

```text
mass_total
inertia_total
```

Estos parámetros son la fuente de verdad del dron completo para trayectoria y
control. Los nodos deben recibirlos desde launch; no deben conservar otra copia
con el mismo significado en YAML de control o trayectoria.

No se confunden con las masas e inercias por enlace usadas en el modelo
simulado/URDF. La Fase 2:

- no calcula `inertia_total` a partir de las piezas;
- no asume que el URDF actual represente el futuro dron real;
- no rediseña profundamente el modelo;
- documenta claramente la diferencia entre parámetros globales de control y
  propiedades por enlace de Simulación.

### Familias de YAML

La organización final se decide tras inventariar consumidores reales, pero debe
tender a responsabilidades separadas como las siguientes.

**Dron:**

```text
physical.yaml
control.yaml
trajectory.yaml
actuators.yaml
camera.yaml
orbslam3.yaml
safety.yaml                 # solo si existen parámetros reales de seguridad
```

**Servidor:**

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

Si el código contiene otro proceso de fusión distinto, se crea un YAML con
nombre inequívoco después de localizar su propietario; no se crean archivos
vacíos por anticipado.

**Simulación:**

```text
simulation.yaml
spawn.yaml
simulated_sensors.yaml
physical_dron.yaml          # réplica parcial, si se necesita
camera_dron.yaml            # réplica parcial, si se necesita
debug.yaml
config/scenarios/*.yaml
```

Los nombres definitivos pueden ajustarse mecánicamente para reflejar mejor los
componentes existentes, sin romper las reglas de ownership.

### Política de edición por Codex

La documentación de cada YAML debe clasificar sus parámetros:

| Clase | Ejemplos | Regla |
|---|---|---|
| Fijo/protegido | masa, inercia, geometría, calibración física | No modificar sin acuerdo explícito |
| Ajustable por subfase | `Kp`, `Kd`, umbrales, límites, frecuencias algorítmicas | Modificar solo si la subfase lo requiere y lo prueba |
| Debug | RViz, webs, dumps, animaciones, logs verbose | Puede activarse para una prueba autorizada; volver a `false` |
| Identidad/contrato | topics, frames, IDs, nombres de interfaces | Cambiar solo mediante subfase de contrato explícita |

### Debug

Cada función dispone de su propio flag. No se agrupan acciones distintas en un
único booleano. Como mínimo deben quedar separados, cuando existan:

```text
debug_pipeline_flow_web
debug_open_pipeline_flow_browser
debug_system_architecture_web
debug_open_system_architecture_browser
debug_sparse_global_rviz
debug_flow_telemetry
debug_architecture_telemetry
debug_optimization_animation
debug_pose_graph_dump
debug_verbose_loop
debug_verbose_optimization
```

Todos los flags quedan `false` en `debug.yaml` por defecto. Los launch no deben
arrancar procesos desactivados.

### Recursos y resultados

- Los YAML se instalan en `share/<package>/config`.
- Los escenarios ejecutables dejan `codex/archivos_auxiliares/` y pasan a una
  ruta lógica dentro de `simulacion_dron/config/scenarios/`.
- `codex` conserva documentación y evidencia histórica, no la fuente runtime
  de escenarios.
- Los resultados generados no se escriben dentro de `src/`; usan `log/`, una
  ruta configurable de resultados o un directorio temporal.

## Diagrama arquitectónico

Se crea una herramienta nueva, separada del visualizador interno del mapa:

```text
simulacion_dron/web/
├── pipeline_flow/          # existente, detalle ORB-SLAM3/mapa global
└── system_architecture/    # nuevo, grupos, paquetes y comunicaciones
```

El nuevo diagrama debe:

- mostrar todos los paquetes;
- encerrar visualmente los paquetes de cada grupo en un contenedor claramente
  rotulado;
- representar topics, services, actions y dependencias relevantes;
- indicar dirección, tipo de interfaz, productor, consumidor y namespace;
- permitir una vista estática sin ROS 2 activo;
- iluminar conexiones con actividad real cuando su debug esté habilitado;
- usar telemetría ligera, acotada y no bloqueante;
- seguir funcionando como documentación si la telemetría falla;
- no gobernar el pipeline ni enviar comandos.

La definición del grafo debe residir en un archivo declarativo independiente de
la lógica de renderizado. Se recomiendan dos modos o capas:

```text
comunicaciones ROS 2 en runtime
dependencias de build/configuración
```

## Secuencia de subfases

### 2A — Crear los grupos y mover los paquetes

Crea la estructura, copia `orbslam3_msgs`, mueve los paquetes y corrige solo lo
necesario para que el árbol sea coherente. No reorganiza aún todos los YAML ni
declara que el sistema funciona.

### 2B — Limpiar y compilar paquete a paquete

Borra artefactos, implementa la estrategia principal de bases separadas,
compila cada grupo en aislamiento y corrige manifests, includes, exports,
instalación de recursos y rutas necesarias.

### 2C — Reorganizar YAML y launch

Define ownership, elimina duplicados internos, mueve `mass_total` e
`inertia_total` al YAML físico, crea réplicas declaradas entre grupos, separa
debug y traslada escenarios ejecutables.

### 2D — Comprobar el funcionamiento

Valida descubrimiento, arranque, interfaces, parámetros, namespaces y la prueba
integrada de dos drones rodeando el edificio. La prueba base usa debug en
`false`.

### 2E — Actualizar contexto de Codex

Actualiza índices, mapas de paquetes, documentación de componentes, rutas,
herramientas y la política permanente de YAML y grupos.

### 2F — Crear el diagrama estático y en vivo

Añade `system_architecture` sin sustituir `pipeline_flow`, documenta la
arquitectura y comprueba actividad real con flags independientes.

### 2G — Guardas y cierre

Añade verificaciones automáticas, repite builds y prueba final, elimina rutas
obsoletas solo cuando su sustitución esté validada y deja el handoff de Fase 2.

## Prueba oficial de cierre

Escenario:

```text
prueba_tipica_rodeo_edificio_dos_fiduciales.yaml
```

El escenario debe residir, tras 2C, en el paquete de Simulación. Dos drones
completan la vuelta alrededor del edificio con el flujo habitual del proyecto.

La validación incluye:

1. build aislado de `dron`;
2. build aislado de `servidor`;
3. build de `simulacion` como overlay;
4. arranque del launch oficial;
5. ejecución completa de los dos drones;
6. topics, services, actions, TF, namespaces y parámetros correctos;
7. mapa y control con comportamiento equivalente al previo;
8. ausencia de rutas inexistentes, paquetes duplicados descubiertos o recursos
   cargados desde `src/`;
9. logs reducidos sin errores graves no explicados;
10. RViz2, `pipeline_flow`, `system_architecture`, sus navegadores, telemetría
    arquitectónica y logs de Fase 3 activos durante la prueba oficial;
11. comprobación negativa separada de que los defaults `false` no arrancan
    procesos de debug.

La comunicación se mantiene en ROS 2 directo. No se prueba Bluetooth.

## Criterio de cierre de Fase 2

La fase se marca `CONSEGUIDA` solo si:

- el árbol físico coincide con los tres grupos acordados;
- `dron` y `servidor` compilan en aislamiento real;
- `simulacion` compila y ejecuta usando ambos prefijos;
- las dos copias de `orbslam3_msgs` son idénticas y la canónica está declarada;
- no existen dependencias de Gazebo o GT en grupos donde no correspondan,
  salvo una excepción explícita, justificada y documentada;
- los YAML tienen propietario, consumidores y política de edición;
- no hay duplicados internos de parámetros con el mismo significado;
- las réplicas parciales y los `deployment profiles` completos están declarados
  y verificados según su categoría;
- todos los flags de debug son independientes y `false` por defecto;
- la vuelta al edificio con dos drones termina correctamente;
- los visualizadores son independientes, opcionales y no bloqueantes;
- la documentación de Codex refleja la distribución real;
- las guardas detectan las regresiones arquitectónicas acordadas.

`PARCIAL` solo es válido si la separación funciona pero falta una evidencia o
guarda obligatoria claramente identificada. `NO CONSEGUIDA` si un grupo no
compila de forma aislada, la simulación no completa la prueba o la configuración
queda ambigua/duplicada.

## Exclusiones permanentes de la fase

- No implementar transporte físico.
- No crear una carpeta común de interfaces.
- No convertir las copias de `orbslam3_msgs` en paquetes con nombres distintos.
- No cambiar campos de mensajes por la separación física.
- No usar rutas `../../` entre grupos.
- No cargar directamente YAML de otro grupo.
- No recalcular la inercia completa desde el URDF.
- No introducir parámetros duplicados para “hacer que compile”.
- No activar debug por defecto.
- No escribir resultados runtime dentro de `src/`.
- No mezclar `system_architecture` con `pipeline_flow`.
- No borrar evidencia histórica ni crear resultados ficticios en los MD de
  subfase.
