# Subfase 2B — Limpiar artefactos y compilar paquete a paquete por grupos

## Estado

```text
SIN HACER
Dependencia: 2A conseguida o árbol equivalente validado
Idea principal de build: bases separadas por grupo
Alternativas: permitidas solo tras demostrar un bloqueo real
```

## Objetivo técnico

Eliminar por completo los artefactos heredados de compilaciones anteriores y
demostrar que:

- `dron` compila sin `servidor` ni `simulacion`;
- `servidor` compila sin `dron` ni `simulacion`;
- `simulacion` compila como integración explícita de ambos grupos;
- cada paquete puede compilarse individualmente en orden topológico;
- los manifests, `#include`, exports, recursos instalados y rutas son correctos;
- ningún residuo de un build anterior oculta una dependencia no declarada.

Los errores encontrados forman parte del alcance: deben diagnosticarse y
corregirse cuando la corrección preserve el acuerdo funcional.

## Contexto obligatorio a leer

```text
AGENTS.md
codex/contexto/00_CONTEXTO_COMPACTACION.md
codex/contexto/08_POLITICA_TOKENS_DOCUMENTACION.md
codex/contexto/09_LOGS_Y_SUBLOGS.md
codex/herramientas/USO_HERRAMIENTAS.md
codex/pipeline/fase_2_separacion_paquetes/pipeline_fase_2.md
codex/pipeline/fase_2_separacion_paquetes/subfases/subfase_2A.md
codex/contexto/paquetes/*/00_summary.md
```

Leer además `CMakeLists.txt`, `package.xml` y documentación específica de los
paquetes que fallen. No abrir código completo como primera medida.

## Diagnóstico de partida conocido

El snapshot entregado contiene varias señales que un build limpio debe revisar:

- `dron_individual/CMakeLists.txt` declara dependencias de Gazebo/Xacro aunque
  el grupo Dron debe terminar aislado de Simulación;
- `dron_individual/package.xml` no enumera todas las dependencias usadas por su
  `CMakeLists.txt` y ejecutables;
- `simulacion_dron` incluye launch de `dron_individual` y
  `orbslam3_server`, lo cual es correcto para el grupo integrador;
- varios recursos se cargan mediante `get_package_share_directory`, pero
  existen también rutas hacia `src/codex` que deben eliminarse en 2C;
- `orbslam3_msgs` estará instalado dos veces, una por Dron y otra por Servidor;
- `ORB_SLAM3` puede requerir una compilación propia distinta de `colcon` y no
  debe compilarse con paralelismo excesivo sin control.

Esta lista no sustituye el diagnóstico real de los primeros errores.

## Limpieza inicial obligatoria

Antes de compilar:

1. registrar el estado actual y la siguiente acción en
   `00_CONTEXTO_COMPACTACION.md`;
2. confirmar que no hay procesos usando artefactos del workspace;
3. borrar completamente:

```text
build/
install/
log/
```

4. no borrar ningún archivo bajo `src/` como parte de la limpieza;
5. no editar manualmente contenidos generados dentro de `build/install/log`.

La limpieza inicial es global. Después se recrean bases separadas por grupo.

## Estrategia principal

### Estructura de artefactos

```text
build/dron/
build/servidor/
build/simulacion/

install/dron/
install/servidor/
install/simulacion/

log/dron/
log/servidor/
log/simulacion/
```

### Comandos base orientativos

Dron:

```bash
colcon --log-base log/dron build \
  --base-paths src/dron \
  --build-base build/dron \
  --install-base install/dron \
  --packages-select <paquete>
```

Servidor:

```bash
colcon --log-base log/servidor build \
  --base-paths src/servidor \
  --build-base build/servidor \
  --install-base install/servidor \
  --packages-select <paquete>
```

Simulación, después de cargar los prefijos de los otros grupos:

```bash
source /opt/ros/<distro>/setup.bash
source install/dron/local_setup.bash
source install/servidor/local_setup.bash

colcon --log-base log/simulacion build \
  --base-paths src/simulacion \
  --build-base build/simulacion \
  --install-base install/simulacion \
  --packages-select <paquete>
```

Los comandos exactos deben adaptarse a la versión instalada de ROS 2/`colcon`
y quedar encapsulados en scripts. No se obliga al usuario a recordar la línea
completa.

## Aislamiento del entorno

### Dron

Cada build de Dron comienza desde un shell limpio con solo:

```text
/opt/ros/<distro>
install/dron de los paquetes ya compilados del mismo grupo
```

No debe existir en `AMENT_PREFIX_PATH`, `CMAKE_PREFIX_PATH` ni variables
equivalentes ningún prefijo de Servidor o Simulación.

### Servidor

Cada build de Servidor comienza desde un shell limpio con solo:

```text
/opt/ros/<distro>
install/servidor de los paquetes ya compilados del mismo grupo
```

No debe cargar Dron o Simulación.

### Simulación

Simulación sí carga:

```text
/opt/ros/<distro>
install/dron
install/servidor
install/simulacion de paquetes ya compilados
```

El orden de prefijos debe quedar documentado. La copia canónica de
`orbslam3_msgs` es la de Servidor; debe tener precedencia intencionada. Antes de
integrar se comprueba que ambas copias son idénticas.

## Orden paquete a paquete

No fijar un orden puramente alfabético. Obtenerlo mediante:

- `package.xml`;
- `CMakeLists.txt`;
- `colcon list` y herramientas de grafo disponibles;
- documentación de paquetes;
- primer error real de dependencia.

Orden orientativo, pendiente de confirmación física:

**Dron:**

```text
orbslam3_msgs
lib_tray
ORB_SLAM3          # build externo/controlado, si procede
orbslam3_ros2
                    # nombre ROS declarado: verificar

dron_individual
```

**Servidor:**

```text
orbslam3_msgs
orbslam3_multi
orbslam3_server
```

**Simulación:**

```text
simulacion_dron
```

Si `dron_individual` no depende en build del wrapper, el orden puede variar,
pero la decisión se registra. No se omite un paquete para fingir independencia.

## Flujo por paquete

Cada invocación debe contener exactamente un paquete en `--packages-select`.
No se permite agrupar varios paquetes aunque pertenezcan al mismo grupo. La
herramienta limita workers de `colcon` y el paralelismo interno de CMake/Make;
el build especial de `ORB_SLAM3` aplica el mismo límite.

Para cada paquete:

1. registrar el paquete y comando exacto;
2. compilar solo ese paquete en su grupo;
3. si falla, reducir el log con la herramienta correspondiente;
4. identificar el primer error real, no el último error en cascada;
5. corregir la causa mínima;
6. repetir el build del paquete;
7. cargar el `local_setup.bash` del grupo;
8. comprobar que el paquete aparece en el prefijo esperado;
9. ejecutar tests locales si existen;
10. registrar resultado antes de pasar al siguiente paquete.

No compilar toda la fase de nuevo ante cada error si un build pequeño permite
validar la corrección.

## Correcciones permitidas

Se deben realizar cuando sean necesarias:

### Manifests

- añadir dependencias realmente usadas a `package.xml`;
- clasificar correctamente `build_depend`, `exec_depend`, `test_depend` o
  `<depend>`;
- eliminar dependencias exclusivas de otro grupo cuando no exista consumidor;
- exportar runtime de interfaces y librerías.

### CMake

- añadir `find_package` real;
- corregir `ament_target_dependencies`;
- corregir `target_link_libraries` y exports;
- instalar headers, librerías, ejecutables y recursos;
- generar interfaces y enlazar typesupport correctamente;
- evitar referencias a rutas fuente fijas.

### Código

- corregir `#include` rotos por la ubicación o por exports incorrectos;
- usar includes públicos instalados;
- eliminar includes no usados que introduzcan dependencia prohibida;
- corregir nombres de paquete/ruta solo si el comportamiento no cambia.

### Launch y recursos

- localizar recursos mediante `ament_index`/`FindPackageShare`;
- instalar launch, config, worlds, models, URDF, RViz y web;
- conservar permisos de scripts Python;
- evitar que un launch funcione solo desde la raíz del repositorio.

### Herramientas

Crear o adaptar herramientas equivalentes a:

```text
codex/herramientas/build_dron.sh
codex/herramientas/build_servidor.sh
codex/herramientas/build_simulacion.sh
codex/herramientas/build_fase_2_completa.sh
codex/herramientas/source_simulacion.sh
```

Los nombres pueden ajustarse a las convenciones existentes, pero deben ofrecer
una interfaz clara y registrar los logs en el grupo correcto.

## Correcciones que requieren detenerse

Suspender la autorización y consultar si el fallo exige:

- cambiar campos o semántica de mensajes;
- renombrar un paquete ROS 2;
- mover un paquete a otro grupo distinto del acordado;
- introducir una dependencia de Dron hacia Servidor o viceversa;
- cambiar algoritmos de control, SLAM, fusión u optimización;
- reemplazar la estrategia de interfaces duplicadas;
- rediseñar la compilación externa de ORB-SLAM3 de forma material.

## Dependencias prohibidas a detectar

### Dron

No debe necesitar para compilar:

```text
simulacion_dron
orbslam3_multi
orbslam3_server
gazebo_ros/gazebo_msgs si no son usados por código embarcado
mundos, modelos o plugins de Gazebo
sensor/GT/* como requisito de arranque de build
```

La presencia actual de una dependencia no significa que pueda borrarse sin
revisar. Localizar primero el consumidor y trasladarlo a Simulación cuando
corresponda.

### Servidor

No debe necesitar para compilar:

```text
dron_individual
lib_tray
simulacion_dron
gazebo_ros/gazebo_msgs
plugins o modelos
```

Puede usar `orbslam3_msgs` desde su copia local canónica.

### Simulación

Puede depender de paquetes instalados de Dron y Servidor. No debe contener
copias completas de esos paquetes para conseguirlo.

## Dos copias de `orbslam3_msgs`

Antes de compilar Simulación:

1. comparar las copias;
2. abortar si divergen;
3. compilar cada copia en su grupo;
4. comprobar el prefijo activo con herramientas ROS 2;
5. documentar el orden de source;
6. confirmar que productores y consumidores generan el mismo type support.

No compilar todo `src/` con ambos paquetes visibles.

## Alternativas autorizadas si la estrategia principal falla

### Alternativa 1 — Workspaces temporales por grupo

Crear árboles temporales de build que expongan solo:

```text
src/dron
src/servidor
src/simulacion
```

El código fuente sigue en las carpetas acordadas. Los árboles temporales no son
una cuarta fuente de verdad y se regeneran mediante script.

### Alternativa 2 — Reutilizar bases únicas

Compilar un grupo, validar y guardar evidencia; limpiar `build/install/log`;
compilar el siguiente grupo. Es menos cómoda para integrar, pero evita
incompatibilidades de herramientas antiguas.

### Alternativa 3 — Selección/exclusión explícita

Usar mecanismos de selección de paquetes con una sola copia visible. Debe
quedar demostrado que no se descubren los dos `orbslam3_msgs`.

### Condición para cambiar

No usar una alternativa por el primer error. Antes debe existir:

- comando exacto fallido;
- log reducido;
- causa atribuida a la separación de bases, no a un manifest o ruta;
- explicación de por qué la corrección razonable no basta;
- registro del cambio de estrategia.

## Tests locales

Ejecutar los tests existentes de cada paquete cuando compile. Como mínimo:

- tests de `lib_tray`;
- tests deterministas de `orbslam3_multi`;
- test de orden del worker de `orbslam3_server`;
- tests del wrapper/ORB-SLAM3 disponibles;
- lint o tests de instalación de recursos cuando existan.

No se exige Gazebo en esta subfase, salvo un smoke de launch estrictamente
necesario para comprobar instalación. La prueba funcional completa es 2D.

## Patrones de reducción de logs

```text
Starting >>>|Finished <<<|Failed <<<|Summary:|CMake Error|fatal error:|undefined reference|Could not find|package .* not found|duplicate package|No such file|ImportError|ERROR|FATAL|Segmentation fault|Killed
```

Para ORB-SLAM3 incluir además:

```text
cc1plus|undefined symbol|Pangolin|Sophus|ORB_SLAM3
```

No abrir el log completo; ampliar patrones si falta evidencia.

## Criterio de éxito

`CONSEGUIDA` solo si:

1. se borraron los artefactos heredados;
2. todos los paquetes de Dron compilan uno a uno sin prefijos externos;
3. todos los paquetes de Servidor compilan uno a uno sin prefijos externos;
4. Simulación compila después de cargar Dron y Servidor;
5. todos los tests locales obligatorios pasan o un fallo preexistente queda
   demostrado y acordado como exclusión;
6. manifests, includes y recursos instalados son coherentes;
7. no hay dos copias divergentes de `orbslam3_msgs`;
8. los scripts dejan reproducible la estrategia usada;
9. cada build y corrección está registrada en historial.

## Criterio de parcial, fallo o bloqueo

`PARCIAL` si todos los grupos principales compilan pero falta un paquete externo
no entregado o un test no funcional claramente identificado.

`NO CONSEGUIDA` si Dron o Servidor solo compilan cargando el otro grupo, si se
omite un paquete del grupo sin acuerdo, si Simulación no enlaza ambos prefijos o
si se ocultan dependencias con artefactos antiguos.

`BLOQUEADA` solo si una dependencia externa imprescindible no está disponible y
no existe stub, paquete instalado o contrato suficiente para continuar sin
inventar cambios.

## Documentación de cierre

Actualizar al ejecutar:

```text
codex/contexto/00_CONTEXTO_COMPACTACION.md
codex/contexto/paquetes/<paquete>/00_summary.md
codex/contexto/paquetes/<paquete>/<componente>.md
codex/herramientas/USO_HERRAMIENTAS.md
codex/pipeline/fase_2_separacion_paquetes/historial/por_subfase/historial_2B.md
codex/pipeline/fase_2_separacion_paquetes/historial/por_subfase/historial_2B_RESUMEN.md
```

No declarar independencia basándose solo en un build incremental.
