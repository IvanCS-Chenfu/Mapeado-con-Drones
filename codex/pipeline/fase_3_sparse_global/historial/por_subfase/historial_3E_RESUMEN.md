# Historial 3E - resumen

Leer este archivo antes de `historial_3E.md` cuando haya que hablar o trabajar
sobre 3E.

## Estado vigente

`CONSEGUIDA`. Implementación, live y replay completos el 2026-08-12. El usuario
acepta la evidencia técnica de los dos anchors y decide no modificar el pulso
breve `first anchor`; volverá a observarlo en la prueba de la siguiente subfase.

## Implementación activa

- ring GT de exactamente 50 muestras por dron, mutex/callback group propios y
  executor de dos threads;
- asociación temporal solo de KFs nuevos, con `max_dt=1.0 s`;
- adaptador GT simulado separado de la API normalizada de
  `FiducialAnchorManager`;
- primer anchor atómico y exactamente un hard KF por submapa;
- observaciones posteriores journalizadas y diferidas a 3H;
- record v2 compatible con v1 y replay sin GT live;
- grafo 8 nodos/8 aristas con manager, sin nodo para `GroundTruthBuffer`;
- cero publishers globales; RViz2 pertenece a 3F.

## Evidencia vigente

- build de tres paquetes: exit 0; tests focales 29/29;
- prueba 91 intento 1: `NO CONSEGUIDA`, Gazebo murió antes de los goals y 3E no
  se ejercitó; el intento se conserva;
- prueba 91 intento 2: `success=true`, anchors para `(2,1)/kf18` y `(1,0)/kf23`
  en fiducial 2; final 22 observaciones, 2 anchors, 61 poses, 2 hard y 20
  diferidas;
- record `rawdb_prueba_3e.record`: 150 entradas raw y 22 observaciones;
- replay 92: reproduce transformaciones y contadores exactos, procesa 150
  entradas con `max_active=1` y no crea subscriptions GT live;
- web: marcador `topology=8_nodes_8_edges`; contrato Python 8/8;
- el usuario vio servidor->manager, pero no manager->store. Los dos commits sí
  constan en backend; el pulso web dura solo 240 ms y ocurre únicamente dos
  veces, frente a 22 observaciones fiduciales;
- auditoría estática: solo publishers de flow/backpressure, ninguno de mapa.

## Cierre conversado

El usuario considera posible que el pulso `first anchor` se le pasara y no
solicita cambios. La ausencia de publishers globales confirma el criterio
técnico de RViz2 vacío. 3E se cierra y la visibilidad de esa arista se observará
de nuevo en la prueba de 3F.

## Errores históricos que no deben repetirse

- executor monohilo o callback GT haciendo trabajo de bases;
- buffers enormes con borrado lineal;
- manager acoplado a GT/Gazebo;
- marcar todos los KFs próximos como hard;
- usar GT como pose final general o como loop;
- ocultar un intento fallido al repetir la simulación.

## Detalle

`historial_3E.md`.
