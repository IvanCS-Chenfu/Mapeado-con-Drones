# Subfase 2A — Crear los tres grupos y trasladar los paquetes

## Estado

```text
SIN HACER
Contrato documental: CERRADO
Ejecución: requiere autorización explícita conforme a AGENTS.md
Dudas abiertas: ninguna
```

## Objetivo técnico

Crear dentro de `src/` los grupos `dron`, `servidor` y `simulacion`, trasladar
cada paquete a su grupo acordado y duplicar de forma controlada el paquete
completo `orbslam3_msgs` entre Dron y Servidor.

La subfase debe cambiar la ubicación física sin cambiar la identidad ROS 2, la
semántica de las interfaces ni el comportamiento funcional de los nodos.

## Resultado objetivo

```text
src/
├── dron/
│   ├── dron_individual/
│   ├── lib_tray/
│   ├── ORB_SLAM3/
│   ├── orbslam3_ros2/
│   └── orbslam3_msgs/
├── servidor/
│   ├── orbslam3_multi/
│   ├── orbslam3_server/
│   └── orbslam3_msgs/
├── simulacion/
│   └── simulacion_dron/
└── codex/
```

La copia canónica de las interfaces queda en:

```text
src/servidor/orbslam3_msgs/
```

## Contexto obligatorio a leer

Antes de preparar o ejecutar:

```text
AGENTS.md
codex/contexto/00_CONTEXTO_COMPACTACION.md
codex/contexto/CONTEXTO_MINIMO_ACTUAL.md
codex/contexto/08_POLITICA_TOKENS_DOCUMENTACION.md
codex/contexto/05_MAPA_PAQUETES.md
codex/pipeline/PIPELINE_MAESTRO.md
codex/pipeline/fase_2_separacion_paquetes/pipeline_fase_2_RESUMEN.md
codex/pipeline/fase_2_separacion_paquetes/pipeline_fase_2.md
codex/contexto/paquetes/*/00_summary.md
```

Usar los MD de paquete antes de abrir código. Abrir manifests, launch o código
solo para resolver rutas y dependencias concretas.

## Diagnóstico de partida conocido

El workspace actual sitúa los paquetes directamente bajo `src/`. La separación
conceptual existe en documentación, pero no está representada por la estructura
física.

Paquetes conocidos:

```text
dron_individual
lib_tray
ORB_SLAM3
orbslam3_ros2        # el paquete ROS declarado puede llamarse orbslam3
orbslam3_msgs
orbslam3_multi
orbslam3_server
simulacion_dron
```

`orbslam3_msgs` es consumido por el wrapper y por el servidor. El usuario ha
acordado copiar el paquete completo en `dron` y `servidor`, no crear una carpeta
común ni mover interfaces a otros paquetes.

`ORB_SLAM3` y el wrapper pueden no estar disponibles en todos los snapshots
entregados. En el workspace real deben localizarse antes de moverlos. No se
inventan rutas ni archivos ausentes.

## Alcance

Incluye:

- inventario físico de los paquetes y recursos bajo `src/`;
- creación de las tres carpetas;
- traslado mediante operaciones que conserven el contenido;
- copia completa de `orbslam3_msgs` a ambos grupos;
- declaración de la copia canónica de Servidor;
- correcciones mecánicas inmediatas de rutas que impidan descubrir el árbol;
- revisión inicial de scripts y documentación que contengan rutas absolutas o
  rutas directas antiguas;
- comprobación estática de que no se perdió ningún archivo.

No incluye todavía:

- build limpio completo, que pertenece a 2B;
- reorganización profunda de YAML, que pertenece a 2C;
- prueba integrada, que pertenece a 2D;
- actualización exhaustiva de documentación, que pertenece a 2E;
- implementación del diagrama, que pertenece a 2F;
- guardas finales, que pertenecen a 2G.

## Ownership acordado

| Grupo | Paquete/recurso | Motivo |
|---|---|---|
| `dron` | `dron_individual` | Control, trayectoria y lógica embarcada |
| `dron` | `lib_tray` | Librería consumida por el control del dron |
| `dron` | `ORB_SLAM3` | Frontend visual local |
| `dron` | `orbslam3_ros2` | Wrapper ROS 2 ejecutado por cada dron |
| `dron` | `orbslam3_msgs` | Contratos requeridos por el wrapper y el dron |
| `servidor` | `orbslam3_multi` | Backend algorítmico global |
| `servidor` | `orbslam3_server` | Adaptador/coordinador ROS 2 central |
| `servidor` | `orbslam3_msgs` | Copia canónica de contratos |
| `simulacion` | `simulacion_dron` | Gazebo, modelos, plugins, escenarios y visualización |
| fuera de grupos | `codex` | Contexto, pipeline, herramientas y evidencia |

## Inventario previo obligatorio

Antes del primer movimiento:

1. ejecutar una búsqueda física de directorios de paquete;
2. registrar para cada paquete:
   - ruta actual;
   - nombre en `package.xml`;
   - tipo de build;
   - dependencias directas;
   - recursos instalados;
   - launch que lo referencian;
3. localizar copias adicionales de paquetes con el mismo nombre;
4. localizar symlinks, submódulos o rutas externas;
5. localizar referencias textuales a `src/<paquete>`;
6. localizar rutas a `build/`, `install/` y `log/` que asuman una estructura
   única;
7. comprobar el estado de Git y no revertir cambios del usuario.

Búsquedas orientativas:

```bash
find src -name package.xml -o -name COLCON_IGNORE
rg -n "src/(dron_individual|lib_tray|ORB_SLAM3|orbslam3_ros2|orbslam3_msgs|orbslam3_multi|orbslam3_server|simulacion_dron)" src
rg -n "get_package_share_directory|FindPackageShare|ament_index" src
```

No abrir logs completos durante esta auditoría.

## Pasos de implementación

### 1. Crear las carpetas

Crear:

```text
src/dron/
src/servidor/
src/simulacion/
```

No crear `src/interfaces`, `src/shared` ni una cuarta carpeta equivalente.

### 2. Mover paquetes no duplicados

Mover cada paquete al grupo acordado conservando:

- nombre de directorio;
- contenido completo;
- permisos ejecutables;
- symlinks válidos;
- historial Git cuando sea posible mediante `git mv`.

No renombrar el nombre del paquete en `package.xml`.

### 3. Copiar `orbslam3_msgs`

Usar la copia actual como base y terminar con:

```text
src/servidor/orbslam3_msgs/   # canónica
src/dron/orbslam3_msgs/       # réplica
```

Las copias deben ser byte a byte equivalentes en los archivos de contrato y
compatibles en manifests. Si el movimiento se hace primero a Servidor, la copia
de Dron debe derivarse de esa canónica.

No modificar campos de mensajes durante esta subfase.

### 4. Resolver descubrimiento básico

Comprobar por separado que `colcon` puede descubrir los paquetes de cada grupo
sin descubrir el otro:

```text
base path: src/dron
base path: src/servidor
base path: src/simulacion
```

Esta comprobación no sustituye al build de 2B.

### 5. Corregir únicamente roturas mecánicas inmediatas

Se pueden corregir rutas necesarias para que el árbol sea legible, por ejemplo:

- scripts que no encuentran el paquete por una ruta fija;
- enlaces Markdown rotos por el movimiento;
- herramientas que enumeran directorios de paquete por ruta física;
- referencias de configuración que solo necesitan cambiar el prefijo físico.

La localización runtime de recursos debe tender a `ament_index`, pero la
reestructuración completa de launch/YAML se reserva para 2C.

### 6. Comprobar integridad

Comparar inventario antes/después:

- mismo número de archivos por paquete, salvo la copia intencionada;
- mismos tamaños/hashes donde no hubo modificación;
- ningún paquete perdido;
- ninguna copia accidental adicional;
- `codex` permanece en su ruta;
- `mi_tfg` permanece temporalmente en la raíz como legacy y no se incorpora a
  ningún grupo;
- `ORB_SLAM3_MULTI` no existe, porque fue retirado antes de Fase 2.

## Archivos probables a modificar

```text
src/codex/herramientas/*.sh
src/codex/herramientas/*.py
src/codex/contexto/CODEX_INDEX.yaml
src/codex/contexto/05_MAPA_PAQUETES*.md
src/codex/pipeline/PIPELINE_MAESTRO.md
```

En esta subfase solo se modifican si la ruta antigua impide continuar o dejar
un checkpoint coherente. La actualización documental exhaustiva se hace en 2E.

Los paquetes se mueven completos. No debe editarse código algorítmico salvo una
corrección mecánica inevitable y documentada.

## Archivos y cambios prohibidos

- No crear carpeta de interfaces compartidas.
- No dejar una tercera copia de `orbslam3_msgs` en la raíz de `src/`.
- No cambiar nombres/campos de `.msg` o `.srv`.
- No cambiar nombres ROS de paquetes.
- No mezclar paquetes de Simulación dentro de Dron o Servidor.
- No copiar `dron_individual` o `orbslam3_server` completos a Simulación.
- No borrar `codex` ni moverlo dentro de uno de los grupos.
- No reorganizar todavía todos los parámetros YAML.
- No borrar `build/install/log` en 2A; la limpieza controlada pertenece a 2B.
- No declarar que el sistema compila o funciona sin ejecutar 2B y 2D.

## Verificación requerida

### Estática

- existen exactamente las tres carpetas acordadas;
- cada paquete está en su grupo;
- existen exactamente dos copias intencionadas de `orbslam3_msgs`;
- la copia canónica está declarada como la de Servidor;
- no quedan paquetes principales en la raíz antigua;
- las rutas de los archivos movidos son válidas;
- no hay referencias obvias a directorios inexistentes;
- `git diff --check` no muestra errores de formato.

### Descubrimiento

Ejecutar comprobaciones equivalentes a:

```bash
colcon list --base-paths src/dron
colcon list --base-paths src/servidor
colcon list --base-paths src/simulacion
```

Cada salida debe contener solo los paquetes permitidos del grupo. La copia con
el mismo nombre en otro grupo no debe ser descubierta.

## Patrones para diagnóstico

```text
Duplicate package names|duplicate package|package not found|No such file|ament_index|COLCON|ERROR|FATAL
```

Si una herramienta genera log completo, reducirlo antes de leerlo.

## Criterio de éxito

`CONSEGUIDA` solo si:

1. el árbol objetivo existe;
2. todos los paquetes fueron preservados;
3. `orbslam3_msgs` existe en Dron y Servidor y no en la raíz antigua;
4. la copia de Servidor está documentada como canónica;
5. el descubrimiento por grupo no mezcla dominios;
6. no se cambió comportamiento funcional;
7. el checkpoint de continuidad registra que 2B es la siguiente acción.

## Criterio de parcial, fallo o bloqueo

`PARCIAL` si el árbol está creado pero un paquete externo no disponible no pudo
moverse y quedó documentado con su ruta objetivo.

`NO CONSEGUIDA` si se pierde contenido, hay paquetes en grupos incorrectos,
`colcon` descubre ambas copias de `orbslam3_msgs` en el mismo build o se cambia
la identidad ROS sin acuerdo.

`BLOQUEADA` solo si falta físicamente un paquete imprescindible y no existe una
forma segura de preservar su ubicación contractual.

## Documentación de cierre

Al cerrar 2A, actualizar como mínimo:

```text
codex/contexto/00_CONTEXTO_COMPACTACION.md
codex/pipeline/fase_2_separacion_paquetes/pipeline_fase_2_RESUMEN.md
codex/pipeline/fase_2_separacion_paquetes/historial/por_subfase/historial_2A.md
codex/pipeline/fase_2_separacion_paquetes/historial/por_subfase/historial_2A_RESUMEN.md
```

Los historiales se crean solo cuando la subfase se ejecute. Este contrato no
incluye resultados anticipados.
