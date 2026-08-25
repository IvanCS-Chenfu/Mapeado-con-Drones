# Pipeline Fase 4 — Fiducial visual sin Ground Truth — RESUMEN ACTUALIZADO

## Estado

```text
Fase 4: CONSEGUIDA Y CERRADA (alcance ejecutado 4A-4H)
Diseño documental: RECONCILIADO desde el zip revisado 4A-4I
Subfases ejecutadas: 4A ... 4H
Bloques ejecutados: 4A+4B; 4C+4D; 4E+4F; 4G+4H
4A: CONSEGUIDA
4B: CONSEGUIDA
4C: CONSEGUIDA
4D: CONSEGUIDA; prueba 208 tecnica correcta y aceptada por el usuario
4E: CONSEGUIDA
4F: CONSEGUIDA
4G: CONSEGUIDA
4H: CONSEGUIDA
4I: APLAZADA; regresion opcional futura, fuera del cierre actual
Historial real: `historial/INDEX.md`
Snapshot documental de referencia: main@4424a586330ca0e54814824fae26bad9daed8232
```

Este ZIP es un **contrato de trabajo**, no evidencia de implementación. Antes de tocar código habrá que aplicar la puerta de preparación, releer `AGENTS.md`, el cierre definitivo de Fase 2, ADR 0009, el código real y el SHA actual de `main`.

El primer bloque 4A+4B ya se ejecuto con autorizacion explicita. Las pruebas
201/202 validan contrato, spawn, readiness, trayectoria ±10 y cierre limpio;
La revision visual de 201 fue perfecta. La trayectoria tipica queda ajustada al
cuadrado ±10 con paradas cardinales `(0,±10)` y `(±10,0)`; la prueba 205 la
recorrio parcialmente hasta la interrupcion solicitada durante el paso 8.

El bloque 4C+4D ya fue implementado. La prueba 205 demostro el evento exacto de
KF y la deteccion real; 206 mostro que un cierre forzado de HighGUI dentro de
`stereo` tambien mataba el wrapper. HighGUI vive ahora en un proceso ROS
independiente. La 208 completo la trayectoria con 79 eventos visuales, cierres
por timeout y deltas posteriores de ambos wrappers. 4C queda conseguida y 4D
queda conseguida por confirmacion expresa del usuario tras la 208.

El bloque 4E+4F publica batches visuales tipados y los sincroniza con el KF raw
exacto en ambos ordenes de llegada. La prueba tipica 210 completo 68/68
matches sin expulsiones ni conflictos; la 211 verifico la arista
wrapper→Servidor live en ambos grafos con otros 18/18 matches. La capacidad
pending inicial 10 alcanzo un pico 7 en la prueba larga.

Repeticion visual 212: el YAML incorpora seis yaw relativos alrededor de
`±180°`, pero la prueba no llego a ejecutarlos. Un loop previo activo una
hard failure de commit y el mission gate bloqueo el paso 5. El ajuste angular
queda pendiente de revision visual; 4E+4F no se reabren por esta incidencia.

Correccion y prueba 213: se elimina el latch `secondary_blocking_failure_`, sin
eliminar logs ni metricas de fallo. El servidor compila y pasa 12/12 targets;
la trayectoria completa 17/17 pasos y 22/22 goals, y cada optimizacion libera
el backpressure al terminar. Los visualizadores registran 74/74 PUB/SHOW. La
evidencia tecnica es correcta y el usuario da 4A-4F por concluidas. La calidad
de optimizacion loop no queda aceptada: por las derivas visibles y nueve
rechazos tardios, la prueba 213 se revisara de nuevo en la subfase 3Q.

## Objetivo de Fase 4

Sustituir el fiducial simulado/GT de Fase 3 por una cadena visual realista:

```text
objeto AprilTag visible en Gazebo
        ↓
KF exacto creado por ORB-SLAM3
        ↓
worker visual del wrapper
        ↓
tag_id + camera_T_tag + calidad
        ↓
topic fiducial separado de orb_map_delta
        ↓
Servidor sincroniza con RawMapDatabase
        ↓
Servidor interpreta tag→fiducial y fusiona caras
        ↓
world_T_camera_target
        ↓
FiducialAnchorManager existente
        ↓
anchor / revisit / optimización ya existente
```

La ruta GT fiducial se conservó únicamente durante la transición de 4G. Tras
validar la cadena visual, 4H la eliminó por completo de código, configuración,
replay y grafos. Fase 4 no conserva GT fiducial ni siquiera para métricas; el GT
de debug que pueda existir en Fase 5 pertenece a otro contrato.

## Subfases vigentes

| Subfase | Responsabilidad cerrada |
|---|---|
| **4A** | Contrato geométrico de objetos `box`, tags por cara, IDs, tamaños, poses a ±8,5 m y validación. |
| **4B** | Generar AprilTags, spawnear objetos estáticos/colisionables y mantener la trayectoria segura a ±10 m. |
| **4C** | Evento one-shot exacto de creación de KF y conservación de la imagen izquierda exacta. |
| **4D** | Servicio Server→Dron de configuración mínima, worker asíncrono, cola acotada, detección/PnP y calidad. |
| **4E** | Mensajes ROS 2 y publicación de un batch no vacío por KF con `0..N` tags detectados (solo se publica si N>0). |
| **4F** | Recepción en Servidor y asociación exacta batch↔KF, tolerando llegada fuera de orden, duplicados y expulsión FIFO acotada por dron. |
| **4G** | Semántica en Servidor: tag→fiducial, rango configurable inicialmente 1–5 m, consistencia, fusión ponderada multicara, selección de un fiducial funcional por KF y generación de `world_T_camera_target`. |
| **4H** | Conectar la observación visual al `FiducialAnchorManager` existente, apagar el GT funcional y ejecutar la prueba integral multi-dron de toda la sustitución. |
| **4I** | Repetir/regresar la cadena con una cámara simulada cuyas especificaciones correspondan a la ESP32-CAM objetivo; cualquier fallo se corrige en la subfase propietaria. |

## Reorganización respecto al plan antiguo 4A–4L

Las antiguas 4I–4L ya no se conservan como subfases independientes:

- la antigua **4I (revisitas/optimización)** se absorbe en 4H porque esa arquitectura ya está implementada en Fase 3 y solo hay que demostrar que acepta la nueva fuente visual;
- la antigua **4J (rechazos/robustez)** se reparte donde corresponde: 4D visión, 4F identidad/sincronización, 4G semántica/consistencia y 4H regresión integral;
- la antigua **4K (integración multi-dron)** se fusiona con 4H y pasa a ser su prueba de salida fuerte;
- la antigua **4L (cámara física)** se aplaza. La nueva 4I valida primero la arquitectura con un perfil de cámara simulada equivalente a ESP32-CAM. Una validación física podrá añadirse en una fase futura si existe hardware.

## Decisiones centrales

- Dron es caja negra y no conoce `object_id`, caras, `object_T_tag` ni `world_T_object`.
- El servicio de 4D entrega solo lo necesario para detectar: familia/política detector y `tag_id -> size_m`.
- Un KF con tags publica **un solo mensaje** con todas las observaciones; un KF sin tags no publica mensaje fiducial.
- Cada tag lleva `camera_T_tag`, `quality_score` y métricas originales explicables.
- El servidor recibe todos los tags aunque alguno esté fuera de la zona fiducial.
- La zona fiducial se configura con mínimo y máximo; el perfil inicial declara
  explícitamente `[1,5] m`. No se deriva del baseline ni modifica el scoring 3R,
  cuya posible unificación queda como revisión futura.
- La zona segura se evalúa por fiducial lógico: si **cualquiera de los tags observados de ese fiducial** está fuera del rango configurado, ese fiducial queda `detected_not_anchor_eligible` completo para ese KF.
- Las detecciones no aptas para anchor no se destruyen: pueden ser útiles en Fase 6 para futuras tareas de búsqueda/acercamiento.
- Si un KF ve varias caras del mismo fiducial, el servidor verifica coherencia y hace una fusión ponderada de poses usando calidad; no promedia RPY de forma ingenua.
- Si un KF ve varios fiduciales lógicos, el servidor los interpreta todos pero entrega **solo uno** al tratamiento funcional actual, porque Fase 3 está diseñada como un fiducial por KF.
- Se selecciona entre candidatos aptos el fiducial con mayor calidad global; los demás quedan como `detected_secondary` para diagnóstico/futuro uso.
- El backend ya dispone de `world_T_camera_target`; Fase 4 no debe rediseñar innecesariamente la lógica de anchor/revisit/optimization.
- `body_T_camera` no forma parte necesaria de la cadena de anchor visual si ya trabajamos con `world_T_camera_target` y `local_T_camera`.
- `pipeline_flow` y `system_architecture` se actualizan en el bloque 4E+4F para
  representar la arista wrapper→Servidor y su actividad runtime real.

## Criterio global de cierre

Fase 4 se considera conseguida porque 4H demostró, ya sin ruta GT fiducial,
que dos drones, los tres objetos baseline y las detecciones visuales atraviesan
correctamente 4A→4H. Por decisión del usuario, 4I queda aplazada como regresión
opcional futura con perfil ESP32-CAM y no condiciona este cierre.
