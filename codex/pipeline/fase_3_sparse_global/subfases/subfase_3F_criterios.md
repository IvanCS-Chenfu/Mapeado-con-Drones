# Subfase 3F - Criterios de cierre

## CONSEGUIDA

3F solo puede cerrarse como `CONSEGUIDA` cuando:

1. Compilan `orbslam3_multi`, `orbslam3_server` y `simulacion_dron`.
2. Pasan tests unitarios e integrados de score, builder y serializacion ROS.
3. `LandmarkScoreManager` es la unica autoridad de score y usa solo datos ORB
   en 3F.
4. `GlobalMapBuilder` mantiene caches e indices incrementales; no hace scan
   global de geometria ante cambios parciales.
5. Cada MapPoint publicado tiene KF asociado world utilizable y score.
6. La reproyeccion usa `inverse(T_local_kf)` y `T_world_kf`.
7. El fallback geometrico de submapa permanece en cero.
8. No aparecen submapas sin anchor.
9. El primer anchor libera todo el backfill del submapa.
10. Un KF movido o score cambiado actualiza solo IDs dependientes.
11. Un dirty set vacio no provoca build ni publish nuevos.
12. Nube y KFs tienen la misma revision/timestamp.
13. `/global_sparse_cloud` contiene XYZ, score, RGB temporal e identidad de
    submapa acordada.
14. `/global_keyframes` muestra frustums con color estable por submapa.
15. El grafo web representa las aristas reales acordadas sin inventar flujo.
16. El replay con 100 ms es determinista y la simulacion completa termina.
17. El usuario confirma que RViz2 y el grafo muestran el comportamiento
    esperado, incluida una nueva oportunidad de observar `first anchor`.
18. No existe publication worker, timer pesado, ACK visual ni publicacion desde
    commits secundarios.
19. El flujo principal conserva `max_active=1` y backpressure 8/2 de 3C.
20. Logs, historial, docs de paquetes y checkpoint quedan sincronizados.

## PARCIAL

Marcar `PARCIAL` cuando la arquitectura y los datos sean correctos, pero falte
una evidencia obligatoria recuperable, por ejemplo confirmacion visual del
usuario, replay repetido o un detalle de color/marker que no corrompa el mapa.
Debe indicarse exactamente que criterio falta y no ocultar pruebas fallidas.

## NO CONSEGUIDA

Marcar `NO CONSEGUIDA` si ocurre cualquiera de estos casos:

- no compila o la simulacion/replay obligatorio falla;
- se publica un submapa sin anchor;
- nube o KFs mezclan revisiones;
- se usa GT para score/geometria funcional;
- se reconstruye toda la geometria por cada delta sin justificacion;
- se publica un MP sin KF world mediante fallback rigido;
- el servidor calcula/modifica scores;
- un commit secundario publica o espera RViz2/web;
- la cache conserva puntos/markers eliminados;
- el grafo muestra aristas que no corresponden a eventos reales;
- el pipeline depende de que RViz2 o el navegador esten abiertos.

## Riesgos y mitigacion

- La serializacion completa de `PointCloud2` sigue siendo O(N): medirla y
  mantener incremental el calculo geometrico.
- Una asociacion KF inestable moveria puntos sin causa: conservar el observador
  mientras siga valido y probar replay determinista.
- IDs de markers pueden colisionar: namespace por submapa y asignacion estable.
- El replay puede ser demasiado rapido: usar el retardo acordado de 100 ms solo
  en pruebas visuales.
- El gradiente temporal podria confundirse con score backend: documentar que
  RGB es presentacion del servidor y `score` es el dato canonico.

## Transferencia a Fase 7

En 7E, `SparseMapLayer` pasa a calcular rojo-amarillo-verde desde `score`. Tras
validar esa ruta se retira del servidor la responsabilidad de generar RGB de
presentacion. No se cambia `LandmarkScoreManager`, el score ni la geometria al
hacer la transferencia.

## Documentacion obligatoria tras implementar

Actualizar como estado vigente, no como mera nota historica:

```text
codex/contexto/paquetes/orbslam3_multi/00_summary.md
codex/contexto/paquetes/orbslam3_multi/landmark_score_manager.md
codex/contexto/paquetes/orbslam3_multi/global_map_builder.md
codex/contexto/paquetes/orbslam3_multi/sparse_global_backend.md
codex/contexto/paquetes/orbslam3_server/00_summary.md
codex/contexto/paquetes/orbslam3_server/global_map_server.md
codex/contexto/paquetes/simulacion_dron/00_summary.md
codex/contexto/paquetes/simulacion_dron/pipeline_flow_visualizer.md
codex/pipeline/fase_3_sparse_global/historial/por_subfase/historial_3F.md
codex/pipeline/fase_3_sparse_global/historial/por_subfase/historial_3F_RESUMEN.md
codex/pipeline/fase_3_sparse_global/historial/INDEX.md
codex/contexto/01_ESTADO_ACTUAL_RESUMEN.md
codex/contexto/07_ULTIMA_SESION.md
codex/contexto/00_CONTEXTO_COMPACTACION.md
```
