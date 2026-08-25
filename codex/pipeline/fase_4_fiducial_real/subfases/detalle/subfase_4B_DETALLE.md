# Detalle largo importado - Subfase 4B

Este archivo conserva el detalle del contrato revisado importado desde `Fase_4_completa_4A_4I_muy_detallada.zip`. El contrato ejecutable corto esta en `../subfase_4B.md`.

# Subfase 4B — Generación visual y spawn de objetos fiduciales en Gazebo

## Estado

```text
CONSEGUIDA el 2026-08-24; ver historial 4B
```

La preparacion y autorizacion descritas a continuacion ya se completaron. Se
conservan como trazabilidad del contrato. La revision visual fue perfecta y la
ruta tipica revisada se ejecutara como regresion del bloque 4C+4D.

## Condición previa extraordinaria y reconciliación con Fase 2

4B está diseñada, pero no debe implementarse sin cerrar antes la preparación conversada y recibir autorización funcional explícita. El snapshot documental consultado al preparar esta revisión es `main@4424a586330ca0e54814824fae26bad9daed8232`. Antes de ejecutar debe releerse el estado final de Fase 2 y comprobar que sus guardas, herramientas y deployment no han cambiado.

Antes de ejecutar se debe volver a inspeccionar `main`, `AGENTS.md`, el layout de `simulacion_dron`, `multi_dron.launch.py`, las herramientas de build/simulación, ADR 0009, `system_architecture` y el contrato 4A definitivo.

Los scripts y SDF del ZIP de sandbox son material reutilizable conceptualmente, no archivos para sobrescribir el proyecto actual. En especial, no debe sustituirse el `multi_dron.launch.py` actual por el snapshot antiguo del ZIP.

## Dependencia

```text
4A — Contrato geométrico y configuración de objetos fiduciales visuales
```

No se ejecutará 4B sobre un YAML cuyo contrato 4A no haya sido validado.

## Objetivo técnico

Convertir la configuración 4A en objetos visuales reales dentro de Gazebo:

```text
fiducial_objects.yaml
        |
        +--> validar configuración
        |
        +--> generar textura AprilTag por tag_id
        |
        +--> construir SDF del box y sus caras marcadas
        |
        +--> /spawn_entity
        |
        +--> confirmar que todos los objetos existen antes de comenzar la prueba
```

El mundo `house_1.world` no será la fuente primaria de posiciones fiduciales. Cambiar la pose/dimensiones/caras de un objeto en el YAML debe cambiar lo que aparece en Gazebo sin editar manualmente el `.world`.

4B no detecta tags desde la cámara y no calcula PnP. La comprobación matemática completa de `camera_T_tag` corresponde a 4D.

## Decisiones cerradas heredadas de 4A

- objeto geométrico: `box`;
- dimensiones directas `x/y/z` por objeto;
- puede ser cúbico o rectangular;
- una marca máxima por cara;
- marca siempre centrada;
- orientación interna siempre 0°;
- caras `pos_x`, `neg_x`, `pos_y`, `neg_y`, `pos_z`, `neg_z`;
- `neg_z` soportada, deshabilitada en baseline;
- familia `APRILTAG_36H11`;
- `tag_id` único global, ID 0 permitido;
- `size_m` es el lado físico del cuadrado;
- `size_m` puede ser mayor que la cara y 4B debe representarlo tal cual;
- colisión física del objeto habilitada;
- el escenario baseline tiene tres cubos de 0.40 m en las poses acordadas.

## Escenario baseline

```text
object 1: (0,+8.5,1), RPY=(0,0,0), 0.40 x 0.40 x 0.40 m
object 2: (0,-8.5,1), RPY=(0,0,0), 0.40 x 0.40 x 0.40 m
object 3: (+8.5,0,1), RPY=(0,0,0), 0.40 x 0.40 x 0.40 m
```

Cada uno utiliza cinco caras activas:

```text
pos_x
neg_x
pos_y
neg_y
pos_z
```

IDs:

```text
objeto 1 -> 101,102,103,104,105
objeto 2 -> 201,202,203,204,205
objeto 3 -> 301,302,303,304,305
```

Baseline visual:

```text
tag size_m = 0.30 m
```

## Generación de texturas

La idea del sandbox se conserva como base:

1. usar OpenCV `aruco` con `DICT_APRILTAG_36h11`;
2. generar una imagen lossless por `tag_id`;
3. usar PNG sin JPEG;
4. evitar cualquier interpolación que difumine las celdas binarias;
5. utilizar una resolución de generación suficientemente alta, pero desacoplada de `size_m` físico;
6. generar el `tag_id` exacto de la cara;
7. volver a detectar offline el PNG generado antes de considerarlo válido;
8. fallar de forma explícita si OpenCV no dispone del diccionario o del ID solicitado.

La resolución en píxeles puede ser un parámetro de la herramienta; no modifica la medida física en Gazebo.

Ejemplo conceptual:

```text
tag_id=101
PNG=800x800 px
visual Gazebo=0.30x0.30 m
```

La relación píxel/metro se resuelve al mapear la textura al plano visual.

## Margen visual para redetección offline

Para verificar un PNG generado puede ser necesario colocarlo temporalmente sobre un canvas blanco, ya que el detector necesita separación respecto al borde de la imagen.

Ese margen de test no cambia `size_m` ni debe incrustarse silenciosamente en la medida física que usa PnP.

## Construcción del SDF

Cada `object_id` se transforma en un SDF independiente, preferiblemente generado en memoria para no mantener cientos de archivos SDF manuales.

Estructura conceptual:

```text
model fiducial_object_<object_id>
  static = true

  link body
    collision body_collision
      box size = [x,y,z]

    visual body_visual
      box size = [x,y,z]

    visual tag_<tag_id>_<face>
      plane
      size = [size_m,size_m]
      pose = derivada de la cara
      material = textura tag_<tag_id>.png
```

### Cuerpo físico

El cuerpo debe:

- ser estático;
- tener las dimensiones exactas del YAML;
- tener colisión de caja;
- no caer por gravedad;
- no atravesarse por el dron como solución normal;
- tener un visual neutro que no interfiera con la marca.

### Plano de cada tag

Cada cara habilitada crea una visual separada.

Debe cumplir:

- centro de la marca = centro geométrico de la cara;
- orientación = la convención 0° de 4A;
- normal exterior correcta;
- tamaño exacto `size_m × size_m`;
- material/textura correspondiente a su `tag_id`;
- sin colisión propia adicional, salvo que una necesidad futura lo justifique;
- sin filtros de textura que borren bordes;
- sin espejado.

## Tags mayores que la cara

El SDF **NO** debe recortar automáticamente una marca al tamaño de la cara.

Si:

```text
cara = 0.20 x 0.20 m
tag  = 0.30 x 0.30 m
```

la visual del tag sigue teniendo:

```text
0.30 x 0.30 m
```

y queda centrada en esa cara.

No se emite error ni warning por este motivo. Cualquier solapamiento o aspecto poco realista es responsabilidad de la configuración elegida por el usuario.

## Offset gráfico y z-fighting

El sandbox introdujo un pequeño desplazamiento de la superficie visual hacia fuera para evitar z-fighting. Se conserva la posibilidad técnica, pero **se reclasifica tras ADR 0009**:

```text
surface_offset_m = parámetro exclusivo de renderizado Gazebo
```

No pertenece al contrato físico/global de `fiducial_objects.yaml`, no lo necesita el Servidor para interpretar tags y no se transmite al Dron. Su ownership es Simulación.

Baseline orientativo del prototipo:

```text
surface_offset_m ~= 0.001 m
```

El valor final debe ser el mínimo que elimine el artefacto visual. La pose semántica `object_T_tag` continúa definida sobre la superficie ideal de la caja. Esto implica que el offset de rendering es una aproximación gráfica conocida; debe medirse y mantenerse despreciable frente a las tolerancias de validación. Si fuese necesario aumentar el offset hasta un valor que altere materialmente la geometría, debe cambiarse la técnica de renderizado en lugar de introducir ese parámetro en la geometría global por comodidad.

La configuración de ese offset puede vivir en un bloque/archivo exclusivo de `simulacion_dron`; no debe duplicar `object_id`, `tag_id`, `size_m` o poses si no es necesario.

## Orientación visual: qué hay que validar realmente en 4B

No basta con comprobar que "hay un tag en la cara".

Por cada cara debe comprobarse que:

```text
cara correcta
ID correcto
normal correcta
arriba visual correcto
sin rotación 90° inesperada
sin rotación 180° inesperada
sin espejo
```

En 4B esta verificación es visual/geométrica desde Gazebo porque todavía no existe el detector funcional de 4D.

Cuando 4D esté disponible se añadirá la corroboración matemática completa: detectar una marca desde una cámara con pose conocida y comprobar que la `camera_T_tag` obtenida es compatible con la transformación derivada del objeto.

## Spawner

Se reutilizará el patrón probado del sandbox y del generador de drones:

```text
rclpy/rclcpp node
   -> valida YAML
   -> genera texturas
   -> espera /spawn_entity
   -> crea un request por objeto
   -> comprueba respuesta
   -> finaliza con resultado explícito
```

Nombre recomendado:

```text
fiducial_spawner
```

La ruta/nombre exactos se confirmarán tras Fase 2.

### Nombres Gazebo

Formato recomendado:

```text
fiducial_object_<object_id>
```

Deben ser únicos y deterministas.

### Timeout obligatorio

El prototipo del ZIP usa una espera que puede quedar bloqueada indefinidamente tras lanzar una petición.

La implementación final debe tener timeout acotado tanto para:

- esperar la disponibilidad de `/spawn_entity`;
- esperar la respuesta de cada `SpawnEntity`.

Un timeout produce error explícito y evita declarar la subfase como correcta.

## Orden de arranque de la simulación

La prueba no debe depender de una espera fija arbitraria como única garantía.

Objetivo:

```text
Gazebo/factory listo
     ↓
fiducial_spawner
     ↓
TODOS los objetos spawneados con éxito
     ↓
escenario_runner / movimiento de drones
```

Se elegirá el mecanismo concreto después de inspeccionar el `multi_dron.launch.py` definitivo de Fase 2. Puede resolverse mediante eventos de launch, finalización correcta de un proceso one-shot u otro mecanismo equivalente.

La condición semántica es obligatoria:

> la trayectoria de validación no empieza hasta que el spawn fiducial ha terminado correctamente.

El escenario baseline mantiene el cuadrado de trayectoria en coordenadas ±10.
No crea goals sobre los centros fiduciales a ±8.5: así deja separación respecto
a los objetos colisionables y mantiene las caras próximas dentro del rango
fiducial inicial de 1–5 m.

## Uso de ADR 0009 y deployment

4B debe aplicar la distinción vigente de Fase 2 entre:

```text
semantic ownership
authority/control
deployment source/profile
```

El spawner pertenece a Simulación y solo puede cargar configuración instalada/local del deployment de Simulación. No debe abrir rutas al árbol fuente de Servidor ni Dron.

Para el contrato físico/global `fiducial_objects.yaml`, el deployment Gazebo puede mantener la copia/perfil de Simulación declarada conforme a ADR 0009. Si esa copia representa el mismo perfil que la del Servidor, la guarda correspondiente exige igualdad. Si el perfil simulado difiere deliberadamente del perfil standalone/real, la divergencia debe estar declarada como tal.

Cuando `multi_dron.launch.py` arranque posteriormente el `fiducial_config_server` de 4D, se reutilizará el mecanismo de deployment aceptado en Fase 2: Simulación selecciona/pasa al proceso del Servidor el perfil local de deployment que corresponde al escenario Gazebo, sin que un nodo abra directamente YAML de otro grupo por rutas de código fuente.

Los parámetros puramente gráficos (`surface_offset_m`, material, resolución de generación si se configura) permanecen únicamente en Simulación y no forman parte de las réplicas globales salvo necesidad funcional demostrada.

## Impacto obligatorio en `system_architecture`

4B cambia el deployment de Simulación al añadir el spawn de objetos fiduciales y posiblemente un proceso/ejecutable nuevo dentro de `simulacion_dron`. Al ejecutar:

- actualizar la metadata declarativa del paquete `simulacion_dron`;
- reflejar en la capa `deployment` que el escenario crea objetos fiduciales antes del movimiento;
- no inventar una arista cross-group nueva si todavía no existe;
- si se añade telemetría live del spawner para el visualizador, producirla solo cuando el debug de `system_architecture` esté activo; con debug `false` no debe construirse/serializarse/publicarse ningún evento específico.

## Archivos previstos a modificar

Rutas actuales orientativas, a revalidar al cerrar Fase 2:

```text
simulacion/simulacion_dron/config/fiducial_objects.yaml
simulacion/simulacion_dron/config/<rendering_fiducial>.yaml   # si hace falta para parámetros exclusivos de Gazebo
simulacion/simulacion_dron/scripts/generate_fiducial_textures.py   # o ruta equivalente
simulacion/simulacion_dron/scripts/fiducial_spawner.py             # o C++ equivalente
simulacion/simulacion_dron/launch/multi_dron.launch.py
simulacion/simulacion_dron/CMakeLists.txt
simulacion/simulacion_dron/package.xml
simulacion/simulacion_dron/test/...
codex/contexto/paquetes/simulacion_dron/...
codex/contexto/03_ARQUITECTURA_ACTUAL.md / metadata de system_architecture si aplica
codex/pipeline/fase_4_fiducial_real/...
metadata/tests de system_architecture que correspondan
```

No es obligatorio conservar una carpeta `models/fiducials/` permanente si el SDF y materiales se generan en runtime de forma controlada.

## Áreas prohibidas en 4B

```text
dron/ORB_SLAM3/
dron/orbslam3_ros2/
dron/orbslam3_msgs/
servidor/orbslam3_server/src/global_map_server.cpp
servidor/orbslam3_multi/
OrbMap.msg
tracking ORB
anchor GT/visual
```

No se añade detector visual en 4B.

## Cambios requeridos

1. Portar las ideas útiles del generador del sandbox al paquete real de simulación.
2. Generar texturas únicamente para caras habilitadas.
3. Verificar cada textura mediante redetección offline.
4. Construir SDF dinámico de `box` con colisión.
5. Derivar automáticamente la pose de cada plano desde la cara y la convención 4A.
6. Mantener tag centrado/orientación 0° sin parámetros adicionales.
7. No recortar tags grandes.
8. Aplicar un offset gráfico mínimo únicamente en Simulación si hace falta para evitar z-fighting, sin incorporarlo a `object_T_tag`.
9. Hacer spawn mediante `/spawn_entity`.
10. Añadir timeout y tratamiento de error.
11. Integrar el spawn en el launch actual sin reemplazar por una versión antigua del ZIP.
12. Garantizar que el movimiento no empieza antes de que el spawn termine.
13. Respetar el deployment profile/guardas de ADR 0009 y no cargar YAML cross-group mediante rutas fuente.
14. Actualizar metadata/deployment de `system_architecture` cuando corresponda.
15. Mantener logs estructurados y poco ruidosos.

## Logs recomendados

```text
[FID-TEXTURE-GENERATED]
[FID-TEXTURE-VERIFIED]
[FID-TEXTURE-ERROR]
[FID-SDF-BUILT]
[FID-SPAWN-WAIT]
[FID-SPAWN-SERVICE-READY]
[FID-SPAWN-OBJECT]
[FID-SPAWN-TAG]
[FID-SPAWN-SUCCESS]
[FID-SPAWN-ERROR]
[FID-SPAWN-ALL-DONE]
```

## Pruebas requeridas

### Prueba 1 — Generación offline

Generar las 15 texturas baseline y redetectarlas sin Gazebo.

Esperado:

```text
101..105 -> correctos
201..205 -> correctos
301..305 -> correctos
0 errores
```

### Prueba 2 — Un objeto

Spawnear solo el objeto 1.

Comprobar:

- pose `(0,+8.5,1)`;
- tamaño 0.40 m;
- colisión;
- cinco caras activas;
- IDs correctos;
- `neg_z` sin tag;
- orientación visual correcta;
- ausencia de espejo;
- z-fighting no apreciable.

### Prueba 3 — Tres objetos baseline

Spawnear los tres objetos y comprobar nombres únicos, poses y 15 texturas.

### Prueba 4 — Prisma rectangular

Con una copia temporal de configuración, usar `x != y != z` y comprobar que la geometría visual/colisión se crea correctamente.

### Prueba 5 — Tag mayor que la cara

Con una copia temporal, configurar `size_m` mayor que la cara.

Esperado:

```text
spawn correcto
visual conserva size_m configurado
no se recorta
no se rechaza
```

### Prueba 6 — Orientación de cada cara

Inspeccionar al menos `+X`, `-X`, `+Y`, `-Y`, `+Z` y verificar la orientación 0° definida en 4A.

No se considera suficiente que el ID sea legible; debe comprobarse también el sentido vertical de la textura.

### Prueba 7 — Error de spawn

Probar de forma controlada una entidad duplicada o fallo equivalente. Debe aparecer un error explícito y no `ALL-DONE` falso.

### Prueba 8 — Timeout

Comprobar que la ausencia de `/spawn_entity` o una respuesta inexistente no deja el proceso colgado indefinidamente.

### Prueba 9 — Gating de trayectoria

Demostrar en logs que el escenario de movimiento comienza después de:

```text
FID-SPAWN-ALL-DONE
```

Después debe completar la trayectoria cuadrada a ±10 sin colisionar con los
tres objetos. La inspección visual corresponde al usuario y no requiere enviar
capturas por el chat.

### Prueba 10 — Deployment y `system_architecture`

Comprobar que el spawner usa únicamente recursos instalados/configuración local del deployment de Simulación y que no abre rutas fuente a Dron/Servidor.

Con `system_architecture` activo, la metadata/deployment debe reflejar la nueva capacidad de spawn sin inventar relaciones cross-group. Con el debug de arquitectura desactivado, cualquier telemetría específica del spawner debe quedar completamente dormida.

## Paquetes a compilar

Comando orientativo actual:

```bash
./codex/herramientas/build_selected_packages.sh simulacion_dron
```

Se revalidará después de Fase 2.

## Criterio de éxito

4B está realizada únicamente cuando:

1. las texturas se generan y redetectan correctamente;
2. el SDF se deriva del YAML, no de posiciones hardcodeadas en el world;
3. se soportan cajas rectangulares;
4. los tres cubos baseline aparecen correctamente;
5. las cinco caras activas de cada cubo tienen los IDs correctos;
6. las orientaciones visuales coinciden con la convención 0°;
7. los objetos tienen colisión;
8. un `size_m` mayor que la cara sigue representándose sin rechazo;
9. no existe z-fighting relevante;
10. el spawner no puede bloquearse indefinidamente;
11. la trayectoria espera al spawn completo;
12. la configuración de rendering permanece propia de Simulación y no contamina el contrato global;
13. el deployment/metadata de `system_architecture` queda coherente con el nuevo spawner;
14. no se ha añadido detección, PnP, GT nuevo ni anchor;
15. build, simulación, revisión visual y logs quedan documentados.

## Criterio de fallo o parcial

- `NO CONSEGUIDA`: objetos/tags ausentes, IDs equivocados, textura reflejada, orientación incorrecta, pose distinta al YAML o spawn no reproducible.
- `PARCIAL`: objetos aparecen, pero queda alguna cara/orientación/z-fighting o gating sin resolver.
- `BLOQUEADA`: Fase 2 cambia el mecanismo de launch/spawn y obliga a readaptar la integración antes de ejecutar.

## Riesgos

- copiar `multi_dron.launch.py` antiguo del ZIP y perder cambios de Fase 2/3;
- confundir resolución PNG con tamaño físico;
- textura girada respecto al frame matemático;
- plano con normal invertida;
- offset gráfico demasiado grande y convertido de facto en un error geométrico;
- rutas de material no instaladas;
- bloqueo esperando `/spawn_entity`;
- movimiento iniciado demasiado pronto;
- recortar por error un tag mayor que su cara pese al contrato de 4A.

## Documentación a actualizar al ejecutar

```text
codex/contexto/paquetes/simulacion_dron/...
codex/pipeline/fase_4_fiducial_real/pipeline_fase_4.md
codex/pipeline/fase_4_fiducial_real/pipeline_fase_4_RESUMEN.md
codex/pipeline/fase_4_fiducial_real/historial/por_subfase/historial_4B.md
codex/pipeline/fase_4_fiducial_real/historial/por_subfase/historial_4B_RESUMEN.md
```
