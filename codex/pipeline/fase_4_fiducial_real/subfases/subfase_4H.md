# Subfase 4H — Sustitución efectiva del fiducial GT y prueba integral multi-dron

## Estado

```text
CONSEGUIDA
Preparacion: cerrada
Autorizacion funcional: concedida y consumida
```

## Dependencia

`4G` completada. Además 4A–4F deben estar realmente validadas, porque 4H es la puerta integral de toda la cadena.

## Por qué 4H se simplifica respecto al MD antiguo

El backend actual ya dispone de una observación con `world_T_camera_target` y `FiducialAnchorManager::Evaluate()` ya calcula `world_T_local` a partir de ese objetivo y de `local_T_camera`. También distingue:

```text
AnchorCreated
RevisitWithinThreshold
OptimizationRequired
```

Por tanto Fase 4 **no debe reescribir** revisitas/optimizer ni introducir un backend nuevo. El trabajo principal es adaptar la entrada visual de 4G y demostrar que sustituye a GT.

La antigua subfase 4I de revisitas/optimización se absorbe aquí como **validación de regresión**, no como reimplementación.

## Objetivo técnico

Conectar el `selected_primary_fiducial` de 4G al tratamiento existente:

```text
world_T_camera_target_fused
+ local_T_camera del KF exacto
+ identidad del fiducial/visita
        ↓
FiducialAnchorManager existente
        ↓
AnchorCreated / RevisitWithinThreshold / OptimizationRequired
```

Durante la prueba funcional, la ruta fiducial GT debe estar desactivada. Puede
seguir presente como transición hasta que la cadena visual pase; después se
eliminan su código, configuración, suscripción y representación en los grafos.

## Cámara vs cuerpo

No insertar una conversión innecesaria a body.

Para anchor:

```text
world_T_local = world_T_camera_target * inverse(local_T_camera)
```

`body_T_camera` pertenece a calibración del dron y será relevante para pose del cuerpo/otras fases, pero no hay que aplicarlo dos veces a una observación que ya está expresada en cámara.

## Adaptación mínima del tipo backend

Revisar la estructura vigente de `FiducialObservation`. Solo cambiar campos que sean necesarios para representar origen visual y semántica `object_id`.

Mantener:

- `arrival_id`/auditoría;
- `RawKeyFrameId` exacto;
- `fiducial_visit_id` visual por `(drone_id,map_epoch,object_id)`;
- `world_T_camera_target`;
- `source=visual_fiducial`;
- calidad/distancia que el backend realmente necesite.

No conservar campos GT como requisito funcional. Si quedan por compatibilidad legacy, la ruta visual no debe depender de ellos.

## Visitas visuales y evaluación de todos los KFs

Todos los KFs que 4G entregue con primary se envían a
`FiducialAnchorManager::Evaluate()`. `fiducial_visit_id` no deduplica KFs ni
evita que el manager mida su error: solo agrupa controles consecutivos del
mismo objeto.

El Servidor mantiene la visita por `(drone_id,map_epoch,object_id)`. Conserva
el mismo ID mientras el hueco entre observaciones primary no supere el parámetro
configurable inicial de `2.0 s`; tras un hueco mayor crea una visita nueva. Un
KF dentro de umbral produce `RevisitWithinThreshold`; uno fuera puede producir
`OptimizationRequired` mediante la lógica existente.

## Un fiducial funcional por KF

4G ya ha seleccionado el primary. 4H no vuelve a comparar varios objects ni crea varios anchors del mismo KF.

Las observaciones secundarias permanecen fuera del manager funcional actual.

## Primer anchor

Si el submapa no tiene anchor y el primary visual es válido:

1. obtener `local_T_camera` del KF exacto;
2. evaluar `world_T_camera_target`;
3. calcular `world_T_local`;
4. commit mediante APIs/autoridad existentes de `GlobalPoseStore`;
5. registrar qué object/tag(s) originaron la fusión;
6. promover hard/control según política existente.

No modificar la pose raw del KF en `RawMapDatabase`.

## Revisitas

Si ya existe anchor:

- usar la misma observación visual normalizada;
- conservar thresholds de Fase 3 inicialmente;
- residual bajo -> `RevisitWithinThreshold`;
- residual alto -> `OptimizationRequired` y task existente;
- no tratar fiducial como loop;
- no cambiar thresholds para hacer pasar la prueba.

La finalidad es demostrar que la fuente visual entra en un mecanismo ya validado.

## Rechazos/robustez que 4H debe verificar

La antigua 4J no existe como subfase independiente. 4H comprueba end-to-end que los rechazos ya implementados en sus capas funcionan:

- detector ausente/fallido no bloquea SLAM;
- batch fuera de orden se resuelve;
- fiducial fuera del rango configurado no crea anchor;
- multiface incoherente no crea anchor falso;
- secondary no crea segundo control;
- pérdida total de tags permite seguir publicando pose/deltas;
- GT deshabilitado no impide anchor visual.

## system_architecture

Esta es la transición funcional:

```text
ANTES:
GT -> Servidor -> fiducial anchor

DESPUÉS:
wrapper visual -> Servidor -> fiducial anchor
```

Al conseguir 4H, `system_architecture` debe dejar de representar GT como camino
funcional de fiducial. Fase 4 elimina también su uso como métrica/debug; el GT
de debug reservado para Fase 5 es independiente.

## Prueba integral de salida — absorbe la antigua 4K

4H no se cierra con “un tag produjo AnchorCreated”. Debe ejecutar una prueba completa y suficientemente larga.

### Escenario baseline

- 2 drones;
- 3 objetos fiduciales acordados;
- 5 caras habilitadas por objeto;
- recorrido alrededor del edificio o trayectoria equivalente validada;
- ruta GT fiducial funcional OFF;
- Gazebo, RViz2 y ventanas de tags activas durante 5 s;
- grafos web desactivados durante la prueba larga y comprobados en un smoke
  corto separado.

### Debe demostrarse

1. ambos wrappers obtienen configuración 4D;
2. objetos 4B existen antes de empezar trayectoria;
3. cada dron genera KFs y el evento 4C es correcto;
4. se detectan tags reales de Gazebo;
5. 4E publica batches no vacíos;
6. 4F sincroniza con KFs exactos;
7. 4G agrupa/fusiona/filtra;
8. al menos los casos previstos de fiducial primary se procesan;
9. submapas se anclan sin GT funcional;
10. revisitas usan la lógica existente;
11. si aparece naturalmente un residual alto, la task de optimización existente
    se activa y conserva validación/rollback; no se fuerza ni se cambian
    thresholds para provocarlo;
12. `orb_map_delta`, pose y mapa global siguen estables;
13. no hay contaminación entre `drone_id/map_epoch`;
14. se miden colas, drops y tráfico fiducial.

## Qué significa “probar todos los fiduciales”

No es necesario crear una subfase K separada, pero la prueba 4H debe evitar concluir a partir de un único ID. Se deben recorrer/observar los tres objetos baseline y suficientes caras para detectar errores de orientación/ID/configuración específicos.

## Métricas de tráfico

Medir como mínimo:

```text
batches publicados por dron
tags por batch
bytes estimados/reales del topic
tasa máxima/media
comparación cualitativa con orb_map_delta
worker_queue drops
pending 4F max
```

La decisión de enviar tags individuales se reabre solo si las métricas muestran un problema real.

## Retirada de GT fiducial

La validación se hace primero con la ruta GT presente pero funcionalmente
desactivada. Cuando la cadena visual pase, se retira la ruta fiducial GT y se
repite la regresión necesaria para demostrar que no queda dependencia oculta.

## Regresión de Fase 3

Como se está reemplazando la fuente, verificar explícitamente que siguen funcionando:

- primer anchor;
- control/hard KF;
- residual pequeño;
- task por residual alto si el escenario la produce naturalmente;
- prioridad worker secundaria;
- post-apply;
- rollback ante solución inválida si el escenario alcanza ese caso;
- publicación global estable.

No reimplementar ninguna de estas piezas si ya funciona.

No se fuerza un residual alto ni un rollback para cerrar 4H. El criterio es que
la cadena visual se comporte como la antigua ruta GT; cualquier diferencia real
se diagnostica antes de decidir cambios adicionales.

## Logs principales

```text
KF-EVENT
FID-DETECT
FID-BATCH-PUB
FID-SYNC
FID-OBJECT-FUSED
FID-OBJECT-PRIMARY
FID-VIS-ANCHOR
FID-REVISIT
FID-TASK
POST-APPLY
ROLLBACK
SCENARIO-RUNNER
```

## Criterio de éxito

- la cadena A→H funciona con dos drones;
- GT fiducial funcional está OFF;
- anchors visuales reales de simulación;
- revisitas/optimization existentes aceptan fuente visual;
- un solo fiducial funcional por KF;
- secundarios no interfieren;
- pérdidas/rechazos no bloquean SLAM;
- mapa final estable;
- tráfico medido y razonable;
- `system_architecture` representa la nueva realidad.

## Criterio de fallo

Cualquier anchor dependiente de GT, asociación a KF incorrecto, fusión incoherente aceptada, doble fiducial funcional en un KF, bloqueo del tracking, cross-drone contamination o corrupción del raw map impide cerrar 4H.

## Resultado

Prueba 216 completa la trayectoria sin ruta GT fiducial con 52/52 primary y
los tres objetos. Smoke 217 valida ambos grafos live. La ruta GT fiducial se
elimina de codigo, configuracion, replay y arquitectura; se conserva solo el GT
independiente de control/Fase 5. Evidencia en
`historial/por_subfase/historial_4H_RESUMEN.md`.
