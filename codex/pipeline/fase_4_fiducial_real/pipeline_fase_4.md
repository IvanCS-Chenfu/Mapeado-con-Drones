# Pipeline Fase 4 — Sustitución del fiducial GT por fiducial visual

## 0. Naturaleza de este documento

Este documento consolida el diseño acordado durante la conversación y sustituye conceptualmente el plan antiguo 4A–4L. **No autoriza por si solo nuevas implementaciones**. Los bloques 4A+4B, 4C+4D, 4E+4F y 4G+4H fueron preparados, autorizados y ejecutados. La Fase 4 queda cerrada con ese alcance; 4I se aplaza como regresión opcional futura y conservará su propia puerta de autorización. El snapshot `main@4424a586330ca0e54814824fae26bad9daed8232` queda solo como referencia histórica de la conversación que generó este diseño.

Antes de ejecutar cualquier subfase será obligatorio:

1. comprobar el SHA actual de `main`;
2. releer `AGENTS.md`, contexto compacto/mínimo y cierre real de Fase 2;
3. releer ADR 0009 y ADR posteriores;
4. reconciliar rutas y símbolos con el código que realmente se compila;
5. revisar `system_architecture`, guardas de duplicación/configuración y herramientas de build por grupo;
6. explicar cambios, riesgos y pruebas;
7. recibir autorización explícita posterior.

No copiar los ZIP de sandbox completos sobre el repositorio. Solo pueden servir como referencia de diseño/código puntual después de comparar con `main`.

## 1. Objetivo general

La Fase 3 demostró el comportamiento del backend fiducial usando información GT. Fase 4 no pretende reescribir ese backend, sino sustituir la **fuente de observación** por una fuente visual ligada al KF exacto.

Objetivo funcional final:

```text
GT fiducial funcional = OFF

cámara simulada
    ↓
ORB-SLAM3 crea KF
    ↓
evento exacto 4C
    ↓
worker AprilTag 4D
    ↓
batch ROS 4E
    ↓
sincronización exacta 4F
    ↓
semántica/fusión 4G
    ↓
world_T_camera_target
    ↓
FiducialAnchorManager existente
    ↓
anchor/revisit/optimization de Fase 3
```

## 2. Invariantes arquitectónicos

- `submap = (drone_id, map_epoch)`.
- identidad de KF = `(drone_id, map_epoch, local_keyframe_id)`.
- `RawMapDatabase` sigue crudo; Fase 4 no escribe pose optimizada allí.
- `GlobalPoseStore` conserva autoridad global.
- fiduciales son observaciones absolutas, nunca loops.
- GT no decide anchor, residual ni optimización.
- Dron no carga YAML de Servidor/Simulación.
- `orbslam3_msgs` mantiene dos copias controladas Dron/Servidor según las guardas de Fase 2.
- la réplica completa `global_map` Servidor↔Simulación se conserva conforme ADR 0009.
- `system_architecture` debe actualizarse cuando una subfase cambie interfaces, relaciones cross-group, réplicas o deployment.
- instrumentación web/debug queda completamente dormida cuando está desactivada.

## 3. Configuración y ownership

ADR 0009 separa:

```text
semantic ownership
control/authority
deployment source/profile
```

Aplicación a Fase 4:

### Dron/wrapper
Conoce únicamente lo necesario para percepción:

```text
family
parámetros de detector
lista tag_id -> size_m
intrínsecos/distorsión efectivos
```

No conoce:

```text
object_id
face
object_T_tag
world_T_object
world_T_tag
world_T_camera
política de anchor
```

### Servidor
Es autoridad funcional sobre:

```text
política detector entregada al dron
semántica tag -> fiducial lógico
geometría del fiducial
pose global conocida del objeto
zona segura
fusión/selección de candidatos
anchor/revisit/optimization backend
```

### Simulación
Posee únicamente lo que es específico de Gazebo/render:

```text
spawn
SDF/material
surface offset para z-fighting
texturas
propiedades visuales/colisión de simulación
```

## 4. Contrato geométrico baseline

Objeto físico: `box` rectangular con dimensiones directas `x/y/z`, sin capa reusable de `models`.

Escenario baseline:

```text
object 1: center=(0,+8.5,1), size=(0.40,0.40,0.40), RPY=(0,0,0)
object 2: center=(0,-8.5,1), size=(0.40,0.40,0.40), RPY=(0,0,0)
object 3: center=(+8.5,0,1), size=(0.40,0.40,0.40), RPY=(0,0,0)
```

Caras soportadas:

```text
pos_x neg_x pos_y neg_y pos_z neg_z
```

Baseline: `pos_x`, `neg_x`, `pos_y`, `neg_y`, `pos_z` habilitadas; `neg_z` deshabilitada.

IDs baseline:

```text
object 1: 101..105
object 2: 201..205
object 3: 301..305
```

`size_m=0.30 m` baseline. `size_m` significa **longitud física del lado del cuadrado AprilTag**. Puede ser mayor que la cara del `box`; no se rechaza ni se avisa por ello. ID 0 es válido para AprilTag si pertenece al diccionario usado.

Cada tag está centrado y tiene orientación interna fija de 0°. `object_T_tag` se deriva determinísticamente en Servidor/Simulación a partir de dimensiones+cara+convención, no se envía al dron.

## 5. Flujo completo de datos

### 5.1 Creación de KF

ORB-SLAM3 crea el KF en `Tracking`. Un evento one-shot de valor permite al wrapper saber que **esa llamada a `TrackStereo`** creó el KF X. No se usa delta temporal ni búsqueda del KF más cercano.

### 5.2 Detección

El wrapper clona la imagen izquierda exacta del frame KF y la encola sin bloquear. Un único worker por dron detecta todos los tags válidos y produce por tag:

```text
tag_id
camera_T_tag
quality_score
reprojection_error_px
tag_area_px2
métrica de ambigüedad si aplica
tiempos de detección/PnP para debug
```

### 5.3 Publicación

Solo cuando existe `N>0` se publica un batch fiducial. No se envía un mensaje vacío por cada KF.

### 5.4 Sincronización

Servidor casa el batch exclusivamente con la clave exacta del KF. Si batch llega antes del delta, queda pendiente. Nunca se reasocia por timestamp.

### 5.5 Interpretación por fiducial

Servidor agrupa tags por `object_id`/fiducial lógico. Para cada grupo:

1. valida tags conocidos;
2. calcula distancia individual `||translation(camera_T_tag)||`;
3. si cualquier tag del grupo está fuera del rango configurado, todo ese fiducial queda no apto para anchor en ese KF;
4. conserva las detecciones aunque no sean aptas;
5. transforma cada tag a una estimación común del objeto/cámara usando geometría del servidor;
6. comprueba consistencia entre caras;
7. descarta outliers según política documentada;
8. fusiona poses coherentes mediante media ponderada apropiada en SE(3)/quaternions, no promediando ángulos Euler de forma directa;
9. obtiene una calidad global del fiducial.

### 5.6 Varios fiduciales en un mismo KF

Servidor puede obtener múltiples candidatos lógicos. Sin embargo, el tratamiento validado en Fase 3 conserva **un fiducial funcional por KF**.

Selección:

```text
candidatos aptos para anchor
        ↓
mayor calidad global
        ↓
desempates deterministas
        ↓
1 candidato funcional
```

Los demás quedan como `detected_secondary`. No se usan para crear múltiples controles/anchors del mismo KF. Su utilidad futura para Fase 6 queda preservada, pero Fase 4 no diseña la tarea “buscar fiducial”.

### 5.7 Backend

El candidato seleccionado genera `world_T_camera_target` y alimenta la API existente de `FiducialAnchorManager` junto con `local_T_camera` del KF exacto.

El backend ya calcula conceptualmente:

```text
world_T_local = world_T_camera_target * inverse(local_T_camera)
```

Si el submapa no está anclado crea anchor; si ya lo está, aplica la lógica de revisit/residual/task ya validada en Fase 3. No se vuelve a aplicar `body_T_camera` a una pose ya expresada en cámara.

## 6. Zona fiducial configurable y uso futuro

El perfil inicial usa 1–5 m, pero ambos límites son parámetros explícitos del
usuario. No se derivan del baseline ni cambian el rango de scoring 3R.

Regla acordada:

```text
para un mismo fiducial lógico:
si TODOS los tags observados están entre min_distance_m y max_distance_m -> candidato potencial
si ALGUNO está fuera -> todo el fiducial NO apto para anchor/optimization en ese KF
```

El perfil inicial declara `min_distance_m=1.0` y `max_distance_m=5.0`. Ambos
valores son configurables por el usuario, con Servidor como autoridad semántica
y Simulación como perfil de deployment guardado. No se derivan del baseline ni
alteran el scoring 3R; revisar una posible unificación queda para el futuro.

Esto **no** implica descartar la detección. Una observación lejana puede ser valiosa más adelante para Fase 6, por ejemplo para decidir que el dron debe acercarse a un fiducial. Fase 4 solo debe conservar la información y no darle autoridad de anchor.

Si un KF ve dos fiduciales diferentes, la regla se aplica por fiducial lógico. Un fiducial lejano no invalida automáticamente otro fiducial independiente que sí sea apto.

## 7. Tráfico de red

Se ha considerado fusionar en el dron para reducir mensajes. Se rechaza como baseline porque:

- normalmente hablamos de 1–3 poses por KF con fiducial, no de todos los frames;
- el coste es pequeño comparado con mapas/deltas;
- enviar geometría tag→fiducial al dron rompería la separación semántica;
- Servidor perdería capacidad para detectar una cara incoherente;
- la optimización prematura no está justificada sin métricas.

4H medirá bytes/mensajes reales. Solo si aparece un problema objetivo de tráfico se reabrirá esta decisión.

## 8. Subfases

La ejecución cerrada comprende los bloques `4A+4B`, `4C+4D`, `4E+4F` y
`4G+4H`. La regresión `4I` queda aplazada y requerirá autorización funcional
propia si se retoma.

### 4A — Contrato geométrico
Ver `subfases/subfase_4A.md`.

### 4B — Spawn visual
Ver `subfases/subfase_4B.md`.

### 4C — Evento exacto KF
Ver `subfases/subfase_4C.md`.

### 4D — Config remota + worker + PnP
Ver `subfases/subfase_4D.md`.

### 4E — Contrato ROS
Ver `subfases/subfase_4E.md`.

### 4F — Sincronización exacta
Ver `subfases/subfase_4F.md`.

### 4G — Interpretación/fusión/selección
Ver `subfases/subfase_4G.md`.

### 4H — Reemplazo GT y prueba integral
Ver `subfases/subfase_4H.md`.

### 4I — Regresión ESP32-CAM
Ver `subfases/subfase_4I.md`.

## 9. Reorganización del plan antiguo

Se eliminan como subfases independientes las antiguas 4J, 4K y 4L y cambia el significado de la antigua 4I. Sus pruebas útiles no se pierden: se redistribuyen. El detalle está en el resumen y en 4H/4I.

## 10. Criterios de cierre de fase

Se exige, como mínimo:

1. tres objetos baseline reproducibles en Gazebo;
2. evento KF exacto sin heurística temporal;
3. worker no bloqueante con configuración Server→Dron;
4. detección multi-tag y PnP por `size_m` individual;
5. batch no vacío por KF y topic separado de deltas;
6. asociación exacta a `RawMapDatabase`;
7. fusión multicara y zona segura en Servidor;
8. un único fiducial funcional seleccionado por KF;
9. anchor/revisitas/optimizer existentes funcionando sin GT funcional;
10. prueba multi-dron completa en 4H;
11. medición de tráfico y backlog;
12. `system_architecture`, guardas, docs e historial real actualizados.

La regresión de cámara ESP32-CAM de 4I es una ampliación futura y no forma
parte del criterio de cierre decidido para esta ejecución de Fase 4.
