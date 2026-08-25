# Detalle largo importado - Subfase 4A

Este archivo conserva el detalle del contrato revisado importado desde `Fase_4_completa_4A_4I_muy_detallada.zip`. El contrato ejecutable corto esta en `../subfase_4A.md`.

# Subfase 4A — Contrato geométrico y configuración de objetos fiduciales visuales

## Estado

```text
CONSEGUIDA el 2026-08-24; ver `../../historial/por_subfase/historial_4A_RESUMEN.md`
```

Las condiciones preparatorias descritas a continuacion se cumplieron antes de
la ejecucion. Se conservan como detalle del contrato, no como trabajo pendiente.

## Condición previa extraordinaria y snapshot de referencia

Esta subfase queda **diseñada conceptualmente**, pero **NO debe implementarse todavía** sin cerrar antes la preparación conversada y recibir autorización funcional explícita. El snapshot de `main` usado para reconciliar este documento es `4424a586330ca0e54814824fae26bad9daed8232` (`docs: integrar y reconciliar cierre de fase 2`). Ese SHA es únicamente una referencia documental, no una base congelada de implementación.

Antes de ejecutar 4A será obligatorio:

1. volver a leer `AGENTS.md`, `codex/contexto/00_CONTEXTO_COMPACTACION.md` y `codex/contexto/CONTEXTO_MINIMO_ACTUAL.md`;
2. comprobar que Fase 2 está realmente cerrada y releer su resumen/handoff final;
3. volver a inspeccionar `main` y el layout real de `dron/`, `servidor/` y `simulacion/`;
4. volver a leer ADR 0009 y cualquier ADR posterior que afecte a ownership, réplicas, interfaces o deployment;
5. reconciliar este contrato con `pipeline_fase_4.md`, `pipeline_fase_4_RESUMEN.md` y las guardas vigentes;
6. explicar alcance, archivos probables, riesgos y pruebas;
7. recibir autorización explícita posterior antes de modificar código/YAML/launch, compilar o simular.

Los ZIP de sandbox estudiados durante la preparación de Fase 4 son únicamente material de referencia. No constituyen evidencia de ejecución y **no deben descomprimirse sobre el repositorio como parche completo**, porque contienen snapshots anteriores a la separación final de Fase 2.

El `AGENTS.md` vigente añade además una obligación transversal: cualquier subfase futura que cambie configuración/replicas, interfaces, relaciones cross-group o deployment debe actualizar `system_architecture`, sus metadatos y guardas/tests cuando aplique, manteniendo toda instrumentación completamente dormida con el debug desactivado.

## Dependencia

Ninguna subfase anterior de Fase 4.

4A define el contrato que necesitarán 4B, 4D, 4G y las pruebas posteriores.

## Objetivo técnico

Definir de forma única y reproducible la geometría de los objetos fiduciales visuales del proyecto.

El término funcional será **objeto fiducial**, no "cubo" como restricción de arquitectura. El único tipo geométrico baseline de Fase 4 será un `box` rectangular definido directamente mediante tres dimensiones independientes:

```text
size_x
size_y
size_z
```

Por tanto:

```text
size_x = size_y = size_z  -> cubo
alguna dimensión diferente -> prisma rectangular
```

No se introducirán catálogos de modelos reutilizables (`models -> objects`) porque no son necesarios para el objetivo del TFG y complicarían el contrato. Cada objeto declara directamente sus dimensiones `x/y/z`.

Cada objeto puede tener como máximo una marca AprilTag por cara. Cada marca se identifica mediante un `tag_id` globalmente único y puede tener un `size_m` distinto al de cualquier otra marca.

4A no crea todavía texturas ni modelos Gazebo, no modifica ORB-SLAM3, no ejecuta detección visual y no cambia el anchor del servidor.

## Decisiones cerradas

Las siguientes decisiones se consideran cerradas para el diseño A-D:

1. El único `shape` baseline es `box`.
2. Cada objeto se define directamente mediante `size_m.x`, `size_m.y` y `size_m.z`.
3. No existe una capa de `models` reutilizables.
4. El frame del objeto está en el centro geométrico del `box`.
5. Las caras reconocidas son exactamente:

```text
pos_x
neg_x
pos_y
neg_y
pos_z
neg_z
```

6. Hay como máximo un AprilTag por cara.
7. Una cara puede estar habilitada o deshabilitada.
8. `neg_z` debe estar soportada por el esquema, aunque permanecerá deshabilitada en el escenario baseline inicial.
9. Cada cara habilitada tiene un `tag_id` globalmente único.
10. El ID `0` es válido si pertenece al diccionario AprilTag seleccionado.
11. Dos tags pueden compartir el mismo `size_m`; lo que no puede repetirse es `tag_id`.
12. Cada tag está siempre centrado en su cara.
13. No se soporta desplazamiento configurable dentro de la cara.
14. La orientación interna del tag sobre su cara es siempre la orientación canónica `0°` definida en este documento.
15. No se soporta una rotación configurable 90/180/270° del tag dentro de su cara.
16. La familia baseline es `APRILTAG_36H11` mediante el diccionario equivalente de OpenCV.
17. `size_m` lo introduce el usuario y representa **la longitud física, en metros, de cada arista del cuadrado AprilTag usado por PnP**.
18. `size_m` solo debe ser finito y estrictamente positivo.
19. **No se validará que `size_m` quepa dentro de la cara del objeto.**
20. **No se generará error ni warning por falta de margen respecto al borde.** Si el usuario configura una marca mayor que la cara, esa configuración se acepta deliberadamente.
21. La pose global del objeto admite `x/y/z/roll/pitch/yaw` aunque el escenario baseline utilice rotación nula.
22. En las pruebas iniciales se usarán tres objetos cúbicos, aunque el código y el YAML deberán soportar prismas rectangulares desde el principio.

## Semántica exacta de `size_m`

Para una marca configurada como:

```yaml
size_m: 0.30
```

la interpretación obligatoria es:

```text
lado físico del cuadrado AprilTag = 0.30 m
```

Es el valor que se usará en 4D para crear los cuatro puntos 3D del cuadrado de PnP.

No significa:

- tamaño de la cara del objeto;
- tamaño de un margen blanco exterior;
- tamaño de la textura PNG en píxeles;
- diámetro de una región de detección;
- distancia al centro del objeto.

La textura puede generarse a cualquier resolución de píxeles adecuada, pero cuando se represente físicamente en Gazebo deberá ocupar exactamente `size_m × size_m` metros.

Si `size_m` es mayor que la superficie de la cara, el sistema no lo corrige ni lo limita. Esa decisión pertenece al usuario que configura el escenario.

## Identidad: `object_id` y `tag_id`

Se mantienen dos identidades diferentes:

```text
object_id = identidad lógica del objeto físico
 tag_id   = identidad visual codificada en una cara
```

Ejemplo:

```text
object_id = 2
pos_x -> tag_id 201
neg_x -> tag_id 202
pos_y -> tag_id 203
neg_y -> tag_id 204
pos_z -> tag_id 205
```

Todos esos tags pertenecen al mismo objeto físico, pero el wrapper de 4D no utilizará esa relación. El conocimiento `tag_id -> object_id` queda reservado al servidor para 4G.

El programa tampoco debe inferir `object_id` a partir de las centenas del ID. El patrón `101..105`, `201..205`, `301..305` es únicamente una convención humana del escenario baseline. La relación real siempre se obtiene de la configuración.

## Convención general de transformaciones

En toda la Fase 4:

```text
A_T_B = transformación que expresa el frame B en coordenadas del frame A
```

El frame `object` se sitúa en el centro geométrico del `box` y utiliza los ejes XYZ del propio objeto.

La pose global se expresa como:

```yaml
world_T_object:
  translation_m: [x, y, z]
  rotation_rpy_deg: [roll, pitch, yaw]
```

La composición RPY seguirá la convención ya utilizada por el proyecto/sandbox:

```text
R = Rz(yaw) * Ry(pitch) * Rx(roll)
```

La implementación deberá comprobar esta convención contra Gazebo/ROS antes de dar 4A/4B por cerradas.

## Frame de cada tag y orientación canónica 0°

Para todos los tags:

```text
+Z_tag = normal de la marca que apunta hacia fuera del objeto
+Y_tag = dirección visual "arriba" de la marca
+X_tag = completa el sistema dextrógiro
```

Como el tag siempre está centrado y su orientación interna siempre es 0°, `object_T_tag` no necesita escribirse manualmente para cada cara: puede derivarse de forma determinista únicamente a partir de:

```text
dimensiones x/y/z
+ nombre de cara
+ tag centrado
+ orientación canónica fija
```

`surface_offset_m` **no forma parte de `object_T_tag` ni del contrato físico/global**. Si 4B necesita un pequeño desplazamiento exclusivamente gráfico para evitar z-fighting en Gazebo, ese valor tendrá ownership de Simulación y se tratará como artefacto de renderizado, no como geometría global que el servidor deba interpretar o transmitir al dron.

La orientación canónica queda fijada así:

| Cara | `+Z_tag` en object | `+Y_tag` en object | `+X_tag` en object |
|---|---|---|---|
| `pos_x` | `+X` | `+Z` | `+Y` |
| `neg_x` | `-X` | `+Z` | `-Y` |
| `pos_y` | `+Y` | `+Z` | `-X` |
| `neg_y` | `-Y` | `+Z` | `+X` |
| `pos_z` | `+Z` | `+Y` | `+X` |
| `neg_z` | `-Z` | `+Y` | `-X` |

Las tres columnas deben formar siempre una base ortonormal dextrógira.

La traslación semántica del centro del tag se obtiene a partir del semieje correspondiente de la caja: el plano físico del tag se considera pegado a la superficie ideal del `box`. Cualquier offset gráfico de Gazebo queda fuera de esta transformación semántica y debe mantenerse mínimo, explícito y medido para que no se convierta en una fuente oculta de error. Si un offset gráfico necesario llegara a ser suficientemente grande como para afectar a la precisión, la solución correcta será revisar la técnica de renderizado, no contaminar silenciosamente `object_T_tag` con un parámetro exclusivo de Gazebo.

## Esquema YAML objetivo

Nombre recomendado para la configuración física/lógica:

```text
fiducial_objects.yaml
```

No debe reutilizarse ciegamente el `global_map/fiducials.yaml` legacy de Fase 3, porque ese archivo pertenece al mecanismo fiducial GT y contiene parámetros con otra semántica. Durante la transición ambos contratos pueden coexistir, pero ninguna clave nueva debe duplicar otra con el mismo significado dentro del mismo perfil. La retirada/fusión del legacy se hará en las subfases posteriores cuando deje de ser necesario para regresión.

`fiducial_objects.yaml` contiene la **verdad física/lógica necesaria para interpretar una instalación de tags**: familia, objetos, dimensiones, pose global, caras habilitadas, `tag_id` y `size_m`. No contiene parámetros puramente gráficos de Gazebo ni parámetros de ajuste del algoritmo detector que no describan la instalación física.

Ejemplo conceptual de un objeto:

```yaml
schema_version: 1
family: APRILTAG_36H11

objects:
  - object_id: 1
    shape: box
    size_m:
      x: 0.40
      y: 0.40
      z: 0.40
    world_T_object:
      translation_m: [0.0, 9.0, 1.0]
      rotation_rpy_deg: [0.0, 0.0, 0.0]
    faces:
      pos_x:
        enabled: true
        tag_id: 101
        size_m: 0.30
      neg_x:
        enabled: true
        tag_id: 102
        size_m: 0.30
      pos_y:
        enabled: true
        tag_id: 103
        size_m: 0.30
      neg_y:
        enabled: true
        tag_id: 104
        size_m: 0.30
      pos_z:
        enabled: true
        tag_id: 105
        size_m: 0.30
      neg_z:
        enabled: false
```

El formato definitivo puede ajustar nombres menores al implementarse, pero no podrá cambiar las decisiones semánticas cerradas sin volver a discutirlas.

### Configuración exclusiva de Simulación

Si Gazebo necesita parámetros como:

```text
surface_offset_m
resolución de textura
material/renderer
```

no se añadirán por comodidad al contrato global si el Servidor no los consume semánticamente. Se ubicarán en configuración propia de `simulacion_dron` (por ejemplo, un bloque/archivo de rendering cuya ruta exacta se decidirá al implementar). Esto aplica la separación `semantic ownership / authority / deployment source` de ADR 0009.

### Configuración de política del detector

Parámetros como:

```text
corner_refinement
pose_solver
max_reprojection_error_px
intervalos/timeouts del cliente
```

no forman parte de la geometría del objeto. En 4D serán controlados por Servidor y suministrados al Dron mediante el servicio de configuración acordado. No se copiarán a un YAML local del wrapper.

## Escenario baseline cerrado

La prueba principal inicial utilizará tres objetos.

### Objeto 1

```text
object_id = 1
pose       = (x=0, y=+8.5, z=1, roll=0, pitch=0, yaw=0)
size       = 0.40 x 0.40 x 0.40 m
pos_x      = tag 101
neg_x      = tag 102
pos_y      = tag 103
neg_y      = tag 104
pos_z      = tag 105
neg_z      = deshabilitada
size_m tags = 0.30 m baseline
```

### Objeto 2

```text
object_id = 2
pose       = (x=0, y=-8.5, z=1, roll=0, pitch=0, yaw=0)
size       = 0.40 x 0.40 x 0.40 m
pos_x      = tag 201
neg_x      = tag 202
pos_y      = tag 203
neg_y      = tag 204
pos_z      = tag 205
neg_z      = deshabilitada
size_m tags = 0.30 m baseline
```

### Objeto 3

```text
object_id = 3
pose       = (x=+8.5, y=0, z=1, roll=0, pitch=0, yaw=0)
size       = 0.40 x 0.40 x 0.40 m
pos_x      = tag 301
neg_x      = tag 302
pos_y      = tag 303
neg_y      = tag 304
pos_z      = tag 305
neg_z      = deshabilitada
size_m tags = 0.30 m baseline
```

Aunque el escenario principal use el mismo tamaño de tag, 4D deberá incluir una prueba específica con tamaños distintos para demostrar que el servicio `tag_id -> size_m` se utiliza realmente.

## Zona fiducial acordada

El perfil inicial declara `min_distance_m=1.0` y `max_distance_m=5.0`. Son
parámetros configurables por el usuario: Servidor conserva la autoridad
semántica y Simulación aporta el perfil local del deployment con las guardas de
ADR 0009. No se derivan del baseline ni cambian el scoring 3R; decidir si ambos
rangos deben compartir parámetros queda como revisión futura.

Las detecciones fuera del rango siguen llegando al Servidor y se conservan para
diagnóstico y para el posible consumo de Fase 6, pero no pueden crear anchor ni
una tarea fiducial funcional.

## Aplicación estricta de ADR 0009

Se conserva el ADR 0009 vigente tras la reconciliación de Fase 2. A partir de ahora deben distinguirse explícitamente tres preguntas para cada dato:

```text
semantic ownership     -> quién posee conceptualmente el dato
authority/control      -> quién decide su valor
deployment source      -> desde qué copia/perfil local lo carga una ejecución
```

Y se distinguen tres categorías de duplicación:

```text
duplicado accidental/semántico -> PROHIBIDO
réplica parcial declarada       -> PERMITIDA con claves exactas y regla de igualdad
réplica completa declarada      -> PERMITIDA solo como deployment profile justificado y guardado
```

La réplica completa `global_map` Servidor↔Simulación ya está aceptada por ADR 0009 y sus guardas. Los nuevos archivos de Fase 4 deben integrarse en esa política de forma explícita; no pueden aparecer dos YAML parecidos sin declarar qué relación tienen.

### Clasificación prevista para Fase 4

**Contrato físico/global fiducial** (`family`, `object_id`, dimensiones, `world_T_object`, cara, `tag_id`, `size_m`):

- autoridad funcional: Servidor/global mapping;
- consumido por Servidor para interpretar observaciones y por Simulación para materializar el escenario;
- en despliegue Gazebo puede existir una copia/perfil local bajo `simulacion_dron`, siguiendo la política declarada de deployment;
- si la copia de Servidor y la de Simulación representan el mismo perfil validado, una guarda debe exigir la igualdad acordada;
- si representan perfiles intencionalmente distintos (por ejemplo real `disabled/not configured` frente a escenario simulado con tres objetos), la divergencia debe ser explícita, documentada y reconocida por las guardas como perfiles distintos, no tolerada accidentalmente.

**Parámetros exclusivamente gráficos de Gazebo** (`surface_offset_m`, detalles de material/render):

- ownership y consumo: Simulación;
- no deben replicarse al Servidor ni enviarse al Dron si no existe un consumidor semántico real.

**Política de detección controlada por Servidor** (solver, refinamiento, umbral preliminar, etc.):

- authority: Servidor;
- consumo en Dron mediante el contrato Server→Dron definido en 4D;
- el Dron no mantiene una copia YAML de esos valores.

Esta última ruta coincide exactamente con el contrato futuro ya incorporado a ADR 0009:

```text
valor controlado por Servidor y consumido en Dron
    -> cliente Dron al arrancar
    -> servicio de configuración Servidor
    -> valor local en RAM del Dron
```

4A no implementa todavía ese servicio; únicamente deja clasificados los datos que 4D distribuirá.

## Impacto obligatorio en `system_architecture`

4A cambia el contrato de configuración y puede introducir una nueva réplica/perfil declarada. Por la regla transversal de Fase 2 deberá actualizarse, al ejecutar la subfase, la capa `config/replica` y la metadata declarativa de `system_architecture` para reflejar:

- propietario/authority del contrato fiducial visual;
- fuentes de deployment Servidor/Simulación;
- tipo de réplica o divergencia de perfil;
- archivos/configuración realmente instalados.

4A no crea una arista runtime nueva, por lo que no debe inventarse telemetría live de tráfico. Cualquier instrumentación de debug añadida para visualización debe quedar completamente inactiva cuando `system_architecture` esté apagado.

## Validador requerido

4A debe disponer de un validador determinista, preferiblemente automatizable en tests, que compruebe al menos:

### Raíz y versión

- raíz YAML válida;
- `schema_version` soportada;
- `family` existe y representa el diccionario físico configurado;
- `objects` es una lista.

### Objetos

- `object_id` entero válido;
- `object_id` único;
- `shape == box`;
- `size_m.x/y/z` numéricos, finitos y `> 0`;
- `translation_m` con tres valores finitos;
- `rotation_rpy_deg` con tres valores finitos;
- `faces` solo contiene nombres soportados.

### Tags habilitados

- como máximo un tag por cara por construcción del esquema;
- `tag_id` entero y no negativo;
- `tag_id` debe existir realmente en `DICT_APRILTAG_36h11`;
- `tag_id` globalmente único entre todas las caras habilitadas;
- `size_m` numérico, finito y `> 0`;
- `family` compatible con el diccionario baseline disponible.

### Comprobación explícitamente prohibida

El validador **NO** debe rechazar ni advertir por:

```text
tag.size_m > ancho de la cara
 tag.size_m > alto de la cara
 margen pequeño respecto al borde
```

Eso es una decisión deliberada del usuario.

## Configuración sin tags

Para un despliegue real/standalone puede existir una configuración sin objetos o sin tags habilitados.

Eso no debe tratarse automáticamente como corrupción del archivo. Debe representar explícitamente:

```text
fiducials disabled/not configured
```

La simulación baseline de 4A/4B sí deberá contener los tres objetos acordados y 15 tags habilitados.

## Archivos previstos a modificar al ejecutar 4A

Las rutas exactas se revalidarán después de Fase 2. Con el layout actual, la intención es trabajar principalmente sobre:

```text
simulacion/simulacion_dron/config/fiducial_objects.yaml       # deployment profile Gazebo, si esta es la convención final
simulacion/simulacion_dron/config/<rendering_fiducial>.yaml   # solo parámetros exclusivos de Gazebo, si hace falta
simulacion/simulacion_dron/test/...                           # validador/tests/guardas
simulacion/simulacion_dron/CMakeLists.txt                     # instalación/tests si procede
simulacion/simulacion_dron/package.xml                        # solo dependencia real necesaria
servidor/orbslam3_server/config/fiducial_objects.yaml         # perfil servidor/real o réplica declarada
codex/contexto/...
codex/pipeline/fase_4_fiducial_real/...
metadata/guardas de system_architecture que correspondan
```

No copiar rutas `src/...` de los ZIP antiguos sin comprobar el layout final.

## Archivos y áreas prohibidas en 4A

```text
dron/ORB_SLAM3/
dron/orbslam3_ros2/
dron/orbslam3_msgs/
servidor/orbslam3_multi/
servidor/orbslam3_server/src/global_map_server.cpp
simulacion/simulacion_dron/worlds/house_1.world como fuente manual de poses
build/
install/
log/
```

4A no debe adelantar 4B, 4C, 4D, 4G ni 4H.

## Cambios requeridos

1. Introducir el nuevo contrato visual `fiducial_objects.yaml` sin mezclarlo con el fiducial GT legacy.
2. Eliminar del esquema del sandbox la capa de `models`.
3. Declarar dimensiones directamente por objeto.
4. Soportar `x != y != z` desde el primer día.
5. Mantener los tres cubos baseline indicados arriba.
6. Fijar las seis caras posibles.
7. Fijar una marca máxima por cara.
8. Fijar tag centrado y orientación 0°.
9. Derivar `object_T_tag` de la cara y geometría, evitando configurar una transformación redundante que pueda contradecir el nombre de cara.
10. Permitir `tag_id = 0`.
11. Validar unicidad global de IDs.
12. Validar que el ID existe en el diccionario real seleccionado.
13. Fijar semántica exacta de `size_m` como longitud de lado del cuadrado.
14. Permitir tamaños repetidos.
15. Permitir tamaños mayores que la cara sin warning/error.
16. Mantener pose global completa XYZ+RPY.
17. Soportar configuración vacía para despliegue real como `disabled/not configured`.
18. Emitir salida determinista resumida para pruebas.
19. Clasificar cada archivo/clave según `semantic ownership`, `authority/control` y `deployment source/profile`.
20. Registrar cualquier réplica nueva en las guardas de Fase 2; no aceptar duplicados semánticos silenciosos.
21. Mantener `surface_offset_m` y otros parámetros puramente gráficos fuera del contrato global.
22. Actualizar la capa `config/replica` de `system_architecture` y sus tests/metadata si el contrato final introduce nuevas fuentes o réplicas.

## Logs recomendados

```text
[FID-CONFIG-VALID]
[FID-CONFIG-DISABLED]
[FID-CONFIG-ERROR]
[FID-CONFIG-OBJECT]
[FID-CONFIG-TAG]
```

No imprimir matrices completas por defecto.

## Pruebas requeridas

### Prueba 1 — Baseline de tres objetos

El validador debe aceptar exactamente el escenario de 3 objetos/15 tags cerrado arriba.

Resultado esperado:

```text
objects=3
enabled_tags=15
unique_object_ids=true
unique_tag_ids=true
shape=box
family=APRILTAG_36H11
```

### Prueba 2 — Prisma rectangular soportado

Crear una copia temporal de test con, por ejemplo:

```text
x=0.60
y=0.40
z=0.25
```

Debe validarse correctamente. No es necesario usarla en el escenario principal.

### Prueba 3 — Tamaño mayor que la cara

Crear una copia temporal donde un tag tenga `size_m` mayor que las dimensiones de su cara.

Resultado obligatorio:

```text
VALID
```

Esta prueba existe expresamente para evitar que una futura refactorización reintroduzca una restricción que el usuario ha rechazado.

### Prueba 4 — `tag_id` duplicado

Dos caras habilitadas con el mismo ID deben producir error explícito.

### Prueba 5 — ID fuera de diccionario

Un ID que no pueda generarse con `DICT_APRILTAG_36h11` debe rechazarse antes de Gazebo.

### Prueba 6 — ID cero

`tag_id: 0` debe ser aceptado si el diccionario lo soporta.

### Prueba 7 — Tamaño inválido

`size_m <= 0`, NaN o infinito debe rechazarse.

### Prueba 8 — Configuración vacía de despliegue real

Debe reconocerse como:

```text
fiducials disabled/not configured
```

sin presentarlo como fallo del SLAM.

### Prueba 9 — Ownership/réplicas y `system_architecture`

Ejecutar las guardas de configuración vigentes con el contrato nuevo. Debe quedar demostrado que:

```text
no hay YAML cross-group leído directamente
no hay duplicado semántico accidental
las réplicas/perfiles están declarados
la igualdad se exige cuando representan el mismo perfil
la divergencia solo se acepta cuando el deployment distinto está documentado
```

Revisar además la capa `config/replica` de `system_architecture`: debe representar la realidad declarada y no crear ninguna arista runtime fiducial que todavía no exista.

## Paquetes a compilar

Si 4A solo incorpora YAML y tests Python, el objetivo es validar la instalación/configuración del paquete afectado mediante las herramientas vigentes después de Fase 2.

Comando actual orientativo:

```bash
./codex/herramientas/build_selected_packages.sh simulacion_dron
```

No ejecutar este comando hasta cerrar Fase 2 y volver a comprobar las herramientas.

## Criterio de éxito

4A solo puede marcarse como `realizado` si:

1. el esquema final está implementado e instalado;
2. soporta cajas rectangulares;
3. el escenario baseline de tres cubos queda representado exactamente;
4. las caras e IDs son inequívocos;
5. `size_m` tiene la semántica acordada;
6. no se impone que el tag quepa en la cara;
7. `object_T_tag` puede reconstruirse de forma determinista;
8. el ID 0 está permitido;
9. duplicados e IDs imposibles se detectan;
10. ADR 0009 se respeta distinguiendo ownership, authority y deployment source;
11. cualquier réplica queda declarada y cubierta por una guarda, y no hay duplicados semánticos accidentales;
12. los parámetros exclusivos de Gazebo no contaminan la configuración global;
13. configuración vacía queda como `disabled/not configured`;
14. `system_architecture` refleja el nuevo contrato de configuración sin inventar aristas runtime;
15. no se ha tocado todavía tracking, detección, anchor ni GT funcional;
16. build/tests/logs reales quedan registrados en historial.

## Criterio de fallo o parcial

- `NO CONSEGUIDA`: geometría ambigua, IDs repetidos, frames no reproducibles, `size_m` sin semántica única o YAML no validable.
- `PARCIAL`: esquema creado pero faltan pruebas esenciales o la derivación de frames no queda inequívoca.
- `BLOQUEADA`: Fase 2 cambia el layout/ownership de forma incompatible y requiere revisar este contrato antes de implementar.

## Riesgos principales

- mezclar `object_id` con `tag_id`;
- volver a introducir `models` innecesarios;
- duplicar semánticamente configuración visual sin declarar una réplica/perfil según ADR 0009;
- mezclar parámetros puramente gráficos de Gazebo con el contrato físico/global;
- hacer que el wrapper conozca geometría global;
- interpretar `size_m` de forma distinta en generación, PnP e impresión;
- invertir la normal de alguna cara;
- rotar la textura 90/180° respecto al frame matemático;
- asumir que un tag debe caber en la cara pese a que el contrato lo permite expresamente;
- hardcodear el máximo de ID en vez de consultar/validar el diccionario disponible.

## Documentación a actualizar al ejecutar

```text
codex/contexto/...
codex/contexto/paquetes/simulacion_dron/...
codex/contexto/paquetes/orbslam3_server/... si se añade perfil standalone
codex/contexto/03_ARQUITECTURA_ACTUAL.md / metadata de system_architecture si aplica
codex/pipeline/fase_4_fiducial_real/pipeline_fase_4.md
codex/pipeline/fase_4_fiducial_real/pipeline_fase_4_RESUMEN.md
codex/pipeline/fase_4_fiducial_real/historial/por_subfase/historial_4A.md
codex/pipeline/fase_4_fiducial_real/historial/por_subfase/historial_4A_RESUMEN.md
```

Los historiales solo se crean/rellenan cuando exista una ejecución real.
