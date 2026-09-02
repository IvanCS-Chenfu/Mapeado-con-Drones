# Pipeline Fase 7 — GUI 3D de operación y supervisión multi-dron

Resumen de entrada:

```text
codex/pipeline/fase_7_gui/pipeline_fase_7_RESUMEN.md
```

## Estado

```text
FASE: SIN HACER
Preparación documental: CERRADA
Acuerdo cerrado: sí
Autorización funcional de ejecución: PENDIENTE
Prueba final acordada: operar y observar el sistema multi-dron desde la GUI propia, con datos reales de Fases 3–6, Gazebo abierto y sin depender de RViz2
Dudas abiertas: ninguna en el contrato actual; cualquier duda funcional nueva no cubierta debe consultarse al usuario
```

Este pipeline es un **contrato documental**. No contiene resultados reales de build, simulación ni historial. Las carpetas de historial se entregan vacías de forma intencionada y solo se crearán MD de historial cuando exista una ejecución real.

## Objetivo general

Crear desde cero una aplicación de escritorio propia para el servidor que permita observar y operar el sistema multi-dron en tiempo real. La GUI debe tener como elemento principal un viewport 3D comparable funcionalmente al uso básico de RViz2, pero implementado por el proyecto y sin incrustar RViz2 ni depender de él.

La aplicación debe mostrar en un mismo frame global `world` los productos que ya existen o existirán en el proyecto: nube sparse global, poses estimadas de los drones, KeyFrames, fiduciales, trayectoria actual de cada dron y, cuando la Fase 8 exista, nube densa. A la derecha debe mostrar tarjetas de estado/tarea de cada dron y, debajo, permitir crear órdenes manuales `GO_TO`, `CAPTURE_SPARSE` y `CAPTURE_DENSE` mediante los contratos de misión de Fase 6.

La GUI es un **consumidor y operador**, no la autoridad del mapa, pose, tareas, trayectorias o reconstrucción densa. Si la GUI se cierra, falla o se ralentiza, el pipeline ROS 2 debe continuar funcionando de forma segura.

## Arquitectura acordada

### Aplicación propia de escritorio

Tecnología objetivo:

```text
C++
+ ROS 2 / rclcpp
+ Qt Widgets
+ OpenGL
```

La versión mayor concreta de Qt se decide en 7A según el entorno real. No se crea una página web, no se incrusta RViz2 y no se usa otro visualizador como núcleo de la aplicación.

La arquitectura cerrada usa dos paquetes ROS 2 dentro de:

```text
src/servidor/
```

con nombres de contrato propuestos:

```text
multidron_gui_lib  -> lógica, modelos, render, widgets y tests unitarios
multidron_gui      -> orquestación, ejecutable y launch ROS 2/Qt
```

Si al ejecutar 7A el workspace real ya contiene una abstracción equivalente, se reutiliza en lugar de duplicarla. La GUI no se integra dentro del proceso `global_map_server`; debe ser un proceso/nodo independiente para que sus dependencias gráficas, bloqueos o cierre no afecten al servidor de mapa. Separar librería y ejecutable evita un monolito difícil de testear.

## Ciclo iterativo con Fase 6

Por decisión de trabajo posterior a Fase 5, Fases 6 y 7 avanzarán en bucle:

```text
avanzar Fase 7 hasta una dependencia real de Fase 6
  -> volver a Fase 6 y crear el contrato/telemetría/acción necesario
  -> repetir hasta cerrar ambas fases
```

Con Fases 1–5 disponibles, el primer tramo probable de Fase 7 es `7A`–`7F`,
`7H` y partes testeables/sintéticas de `7L`. `7G` requiere trayectoria vigente
de Fase 6, `7I` progreso/tarea real, `7J` `GO_TO`, `7K` tareas manuales de
captura y `7M` la integración completa posterior a Fase 6. Cada salto entre
fases conserva la puerta de preparación/autorización de `AGENTS.md`.

### Separación de threads y datos

Arquitectura conceptual obligatoria:

```text
ROS 2 topics / services / actions
                |
                v
          RosDataBridge
                |
                v
          GuiDataModel
      caches/snapshots RAM
                |
                v
          thread Qt/GUI
                |
                v
        Scene3D + OpenGL
```

Los callbacks ROS actualizan modelos/cachés en memoria. No renderizan. El thread de Qt consume snapshots coherentes y actualiza la interfaz.

La cámara puede dibujarse a 30–60 FPS o al ritmo razonable del equipo aunque la nube sparse solo cambie cuando llega una revisión nueva. Los buffers de GPU se actualizan cuando cambian sus datos, no se reconstruyen innecesariamente en cada frame.

No se requiere una base de datos persistente para la GUI. El `GuiDataModel` es principalmente estado en memoria y puede descartar revisiones antiguas cuando ya no son necesarias para la vista actual.

## Layout principal acordado

Distribución conceptual:

```text
┌──────────────────────────────────────────────────────────────────────┐
│ controles de capas / score / opciones visuales                     │
├───────────────────────────────────────────────┬──────────────────────┤
│                                               │ tarjetas de drones   │
│                                               │                      │
│                                               │ scroll vertical      │
│              VIEWPORT 3D                      │ si no caben todas    │
│                                               │                      │
│                                               ├──────────────────────┤
│                                               │ añadir tarea         │
├───────────────────────────────────────────────┴──────────────────────┤
│ información de entidad seleccionada                                 │
└──────────────────────────────────────────────────────────────────────┘
```

El viewport ocupa la zona central/izquierda y es el elemento principal. El bloque de creación de tareas debe seguir siendo accesible aunque el listado de drones sea largo; la lista de tarjetas es la zona que debe tener scroll.

## Viewport 3D

Debe existir un frame `world` común y un grid horizontal de referencia. La cámara debe permitir al menos:

- rotación/orbit con ratón;
- pan/desplazamiento;
- zoom con rueda;
- resize sin deformar la proyección;
- navegación fluida mientras llegan datos ROS;
- selección de entidades mediante picking.

El renderer debe estructurarse por capas independientes, no mediante un bloque monolítico. Capas baseline:

```text
GridLayer
SparseMapLayer
DenseMapLayer             # preparada en Fase 7, datos reales en Fase 8
DronePoseLayer
KeyFrameLayer
FiducialLayer
TrajectoryLayer
SelectionLayer
```

Se pueden añadir `AnchorLayer`, `LoopLayer`, `GoalLayer` o `ConflictLayer` si los productores reales ya entregan datos suficientemente claros y su inclusión no desvía las subfases obligatorias. No se inventan datos para llenar capas.

## Controles superiores

Como mínimo deben permitir activar/desactivar:

- nube sparse;
- nube densa, aunque permanezca sin datos antes de Fase 8;
- poses de drones;
- KeyFrames;
- trayectorias actuales;
- fiduciales;
- cualquier capa adicional acordada que se implemente realmente.

### Score de MapPoints

El umbral de score de Fase 7 es **solo visual**. No cambia `LandmarkScoreManager`, no filtra la base del mapa y no modifica qué puntos existen o publica funcionalmente el backend para otros consumidores.

Deben coexistir dos funciones:

1. filtrar visualmente MapPoints por `score >= umbral_gui`;
2. colorear los puntos visibles según un gradiente score bajo → rojo, score medio → amarillo, score alto → verde.

El rango/normalización debe derivarse del contrato real del score. No asumir `[0,1]` si el código vigente usa otra escala. La misma puntuación debe conservar significado visual estable; no normalizar dinámicamente por el mínimo/máximo del frame si eso cambia el color de un punto solo porque aparecieron otros puntos.

## Datos visibles y ownership

La GUI debe consumir la fuente canónica real de cada dato tras ejecutar Fases 3–6:

- mapa sparse global y score: Fase 3 / servidor de mapa;
- KeyFrames globales: Fase 3;
- fiduciales/anchors visibles: Fase 4 y estado global aceptado;
- pose global estimada/estado de tracking: Fase 5, nunca GT funcional;
- tareas, progreso y trayectoria actual: Fase 6;
- nube densa: Fase 8.

No se fijan en este documento nombres nuevos de topics si una fase previa ya creó uno equivalente. 7A debe inventariar el workspace **real posterior a Fases 3–6** y congelar los nombres que utilizará la GUI.

Si un producto no tiene aún un topic adecuado pero el dato ya existe en su productor, se debe ampliar el productor de forma mínima y no bloqueante o volver a la subfase propietaria para crear un contrato canónico. No se deriva información incorrecta dentro de la GUI solo para evitar tocar la fase de origen.

## Tarjetas de drones

Debe crearse dinámicamente una tarjeta por dron detectado/participante. La tarjeta muestra como mínimo, cuando los datos reales existen:

- identificador/nombre del dron;
- estado de pose/tracking útil para operación;
- nombre/tipo de la tarea actual;
- estado de tarea;
- barra de progreso;
- información relevante proporcionada por el productor, sin inventar batería, latencia u otros campos inexistentes.

La lista tiene scroll vertical si no caben todas las tarjetas.

### Progreso

La GUI **no calcula por su cuenta** el progreso de una tarea de mapeo. Lo representa desde el contrato de Fase 6.

Se acepta progreso discreto: por ejemplo incrementos por secciones/hitos de cobertura. No se exige que la barra sea suave ni que cambie cada frame.

Cuando una tarea sencilla como `GO_TO` pueda proporcionar un porcentaje real/geométrico con cambios mínimos en su productor, Fase 6 puede publicarlo y la GUI lo representa. Si el productor no dispone de progreso fiable, no se simula mediante tiempo transcurrido.

## Dron perdido o stale

Si un dron queda perdido, stale o sin pose nueva válida, la GUI conserva la
última pose `world` válida, muestra etiqueta `PERDIDO` o estado equivalente y
dibuja ejes/representación con mayor transparencia. No reemplaza la pose por
cero, no oculta silenciosamente el dron y no propaga una pose falsa como si
fuera actual. Al recuperarse una pose válida, vuelve a la representación normal.

## Trayectoria actual

La GUI muestra para cada dron **la trayectoria que va a ejecutar actualmente**. Si el planificador ha generado una recta, se dibuja esa recta; si ha generado una curva/polinomio, se representa esa curva con muestreo suficiente.

No es requisito baseline mostrar el historial completo de trayectorias propuestas/reservadas/rechazadas. La visualización operativa principal es el camino futuro vigente del dron.

Si Fase 6 no publica la trayectoria actual en una representación recuperable, debe reabrirse la subfase propietaria de Fase 6 y añadir telemetría canónica; no reconstruir la trayectoria en GUI a partir de posiciones pasadas.

## Selección de entidades

El picking se diseña genéricamente. El requisito mínimo es seleccionar un MapPoint y mostrar su posición `x,y,z` en `world`, pero la infraestructura debe admitir también KFs, drones, fiduciales y otras entidades implementadas.

Al seleccionar una entidad:

- se resalta de forma visible;
- aparece un panel/campo de información;
- para MapPoints se muestran al menos `x`, `y`, `z` y score si está disponible;
- para otros tipos se muestran únicamente campos reales y útiles.

Seleccionar un MapPoint **no crea automáticamente una tarea** ni rellena obligatoriamente el objetivo. Sirve para inspeccionar y orientar manualmente al operador.

## Creación de tareas desde GUI

El bloque inferior derecho permite elegir dron e introducir siempre una pose absoluta:

```text
x, y, z, yaw   en frame world
```

Tipos baseline:

```text
GO_TO
CAPTURE_SPARSE
CAPTURE_DENSE
```

La GUI utiliza el contrato de tareas del servidor creado en Fase 6. No manda un `TrayAction` directamente al dron y no evita planner, obstacle avoidance, reservas o reglas de prioridad existentes.

No se añaden en esta fase controles de `pause`, `resume` ni `cancel` desde la GUI.

### `GO_TO`

El dron debe ir a la pose `(x,y,z,yaw)` en `world` usando el comportamiento de Fase 6. La semántica de prioridad/preempción es la ya acordada en Fase 6: no crear una vía paralela por ser una orden gráfica.

### `CAPTURE_SPARSE`

Secuencia funcional:

```text
usuario elige dron + x,y,z,yaw
        -> tarea via sistema Fase 6
        -> dron llega a la pose usando GO_TO/planner/reservas
        -> se considera estabilizada/llegada según contrato real
        -> se espera creación NATURAL de un nuevo KF de ORB-SLAM3
        -> KF válido asociado al dron/epoch actual -> COMPLETED
        -> no aparece KF dentro del límite acordado -> FAILED
```

No se modifica ORB-SLAM3 ni se fuerza la creación de KeyFrame. Si falla, el operador puede mandar otra tarea para alejarse, variar posición/orientación e intentar obtener un KF desde otra vista.

El timeout/tolerancias exactos deben reutilizar parámetros existentes cuando sea posible. Si no existe una convención previa y elegir un valor cambia el comportamiento funcional, Codex debe preguntarlo durante la preparación de 7K.

### `CAPTURE_DENSE`

La GUI y el contrato deben quedar preparados para:

```text
GO_TO pose absoluta
-> adquirir la vista orientada por yaw
-> generar/incorporar una contribución densa de esa vista
```

La reconstrucción/captura densa real pertenece a Fase 8. Fase 7 no implementa Open3D/TSDF ni inventa una nube densa. Antes de Fase 8 la capacidad debe aparecer como no disponible o ser rechazada explícitamente por backend; nunca informar éxito falso. La forma visual exacta de “no disponible” puede resolverse durante 7K si el backend real obliga a elegir entre varias alternativas.

## Nube densa y Fase 8

Fase 7 crea `DenseMapLayer`, su control de visibilidad y la infraestructura de buffers necesaria para nubes grandes. Se valida con datos sintéticos o replay controlado.

Fase 8 será responsable de producir la nube densa global real, su versionado y reintegración. La conexión futura no debe obligar a rediseñar la ventana ni el renderer.

## Puerta de validación visual hacia Fases 3–6

Regla transversal obligatoria:

```text
La GUI es consumidor y herramienta de verificación.
No es una capa de corrección de errores de fases anteriores.
```

Durante 7E–7M se debe comprobar visualmente que los productos de las fases anteriores tienen sentido. Ejemplos:

- nube sparse duplicada/desplazada/incoherente -> revisar Fase 3;
- KFs/submapas mal alineados -> revisar Fase 3;
- fiduciales con pose/orientación incoherente -> revisar Fase 4;
- pose estimada que salta, queda stale o usa frame incorrecto -> revisar Fase 5;
- trayectoria representada no coincide con la trayectoria realmente vigente -> revisar Fase 6;
- tarea/progreso reportado no refleja el estado real -> revisar Fase 6.

Antes de culpar al productor, se debe comprobar que el mensaje recibido por la GUI es realmente incorrecto. Si el mensaje es correcto y el renderer lo transforma/pinta mal, el defecto es Fase 7.

Si se demuestra que el dato de origen es incorrecto o el contrato previo es insuficiente:

1. detener la subfase de Fase 7;
2. registrar `Autorizacion funcional: SUSPENDIDA` en `00_CONTEXTO_COMPACTACION.md` si el arreglo implica decisión funcional;
3. explicar al usuario qué se observa y por qué pertenece a una fase anterior;
4. volver a la fase/subfase propietaria y corregir allí con su preparación/autorización;
5. repetir después la prueba visual de Fase 7.

No se aceptan offsets visuales, filtros ad hoc o transformaciones “para que se vea bien” que oculten un error real del backend.

## Regla de dudas durante ejecución

El contrato no intenta adivinar todos los estados que aparecerán al integrar el sistema. Ejemplo explícito acordado: si un dron está perdido y existen varias representaciones razonables para su pose en GUI, Codex no elige arbitrariamente ocultarlo, congelarlo o pintarlo de otra forma.

Ante cualquier duda funcional no resuelta:

- parar antes de implementar esa decisión;
- explicar el estado real encontrado;
- presentar alternativas y consecuencias;
- preguntar al usuario;
- continuar solo tras cerrar el nuevo acuerdo.

Las decisiones puramente técnicas que no cambian comportamiento funcional —estructura de VBO, algoritmo de picking, clase auxiliar, layout interno de memoria— pueden resolverse por ingeniería y documentarse.

## Subfases

| ID | Nombre | Salida principal |
|---|---|---|
| `7A` | Arquitectura, dependencias y contratos ROS 2 | Inventario real de productores, paquete GUI, threading, topics/actions/services y ownership cerrados. |
| `7B` | Ventana principal y layout operativo | Main window con toolbar, viewport reservado, panel derecho con scroll, tareas e inspector. |
| `7C` | Modelo de datos y bridge ROS 2 asíncrono | Callbacks ROS desacoplados de Qt, caches/snapshots y lifecycle limpio. |
| `7D` | Motor 3D, grid y cámara | `Scene3DWidget` OpenGL fluido con orbit/pan/zoom y arquitectura de layers. |
| `7E` | Nube sparse, umbral y gradiente de score | MapPoints globales eficientes, filtro visual y rojo→amarillo→verde por score. |
| `7F` | Drones, KeyFrames y fiduciales | Capas 3D con datos reales de Fases 3–5 y toggles independientes. |
| `7G` | Trayectoria actual de cada dron | Curva/recta futura vigente publicada por Fase 6 y representada por dron. |
| `7H` | Picking e inspector genérico | Selección de MapPoints y arquitectura extensible a KFs/drones/fiduciales. |
| `7I` | Tarjetas de drones y progreso de tareas | Cards dinámicas, scroll y progreso real/discreto desde Fase 6. |
| `7J` | Creación de `GO_TO` | Formulario world absoluto y envío seguro a `task_server` en Fase 6. |
| `7K` | `CAPTURE_SPARSE` y preparación `CAPTURE_DENSE` | Captura sparse sin forzar KF y contrato denso preparado para Fase 8. |
| `7L` | `DenseMapLayer` y rendimiento | Capa densa preparada, buffers grandes y pruebas sintéticas de rendimiento. |
| `7M` | Integración, validación visual y cierre | Prueba multi-dron completa, regresiones cruzadas y GUI desacoplada/robusta. |

## Prueba final de Fase 7

La prueba final debe arrancar el sistema multi-dron real posterior a Fase 6,
Gazebo y la GUI propia. No se requiere RViz2 para operar ni validar la GUI.

Secuencia mínima:

1. arrancar servidor, drones y simulación con datos de mapa/pose/tareas reales;
2. abrir la GUI y comprobar grid/cámara/interacción;
3. visualizar sparse y alternar capas;
4. cambiar umbral de score y activar color por score sin afectar al backend;
5. ver poses, KFs y fiduciales en posiciones coherentes;
6. hacer que al menos dos drones se muevan y comprobar sus trayectorias actuales;
7. revisar tarjetas/progreso y scroll con suficiente número de drones o tarjetas sintéticas de test;
8. seleccionar MapPoints y otras entidades implementadas;
9. enviar un `GO_TO` desde GUI y comprobar el lifecycle real;
10. ejecutar `CAPTURE_SPARSE`: éxito cuando aparezca un KF natural válido y caso de fallo controlado sin KF;
11. comprobar que `CAPTURE_DENSE` no informa éxito real antes de Fase 8;
12. cerrar la GUI mientras el sistema continúa funcionando;
13. reabrirla y reconstruir la vista desde topics/estado vigente sin requerir reiniciar el pipeline.

Toda anomalía visual debe clasificarse primero como **dato de origen** o **representación GUI** y aplicarse la puerta de retorno a Fases 3–6 cuando corresponda.

## Exclusiones globales

- No incrustar ni depender de RViz2.
- No crear una aplicación web.
- No usar GT para pose, mapa, selección de tareas, trayectoria o representación funcional; GT solo puede seguir existiendo como métrica externa en simulación.
- No ejecutar algoritmos de SLAM, fusión, optimización, planificación o reconstrucción densa dentro de la GUI.
- No derivar progreso de tareas por tiempo transcurrido si el productor no lo conoce.
- No forzar KeyFrames modificando ORB-SLAM3 para `CAPTURE_SPARSE`.
- No implementar la nube densa real de Fase 8.
- No saltarse `task_server`/`task_manager`, planner ni reservas enviando
  `TrayAction` directo desde la GUI.
- No introducir correcciones visuales para ocultar defectos de fases anteriores.
- No crear historiales ficticios.

## Criterio de cierre de Fase 7

La fase se considera `CONSEGUIDA` solo si:

1. la GUI propia compila y arranca como proceso independiente del backend;
2. el viewport 3D es navegable y fluido con grid y layers;
3. sparse, score, drones, KFs, fiduciales y trayectorias actuales se representan desde datos reales y coherentes;
4. el filtro/gradiente de score son exclusivamente visuales;
5. el picking permite inspeccionar al menos MapPoints con coordenadas world;
6. las tarjetas escalan a N drones mediante scroll y muestran progreso reportado, no inventado;
7. `GO_TO` y `CAPTURE_SPARSE` pasan por la cadena real de Fase 6;
8. `CAPTURE_DENSE` queda preparado sin fingir una implementación de Fase 8;
9. cerrar/fallar la GUI no bloquea mapa, control, tareas ni servidor;
10. cualquier regresión real de Fases 3–6 detectada por la GUI ha sido corregida en su fase de origen y revalidada;
11. no existe dependencia funcional de GT ni RViz2;
12. documentación e historial real quedan sincronizados tras las ejecuciones.
