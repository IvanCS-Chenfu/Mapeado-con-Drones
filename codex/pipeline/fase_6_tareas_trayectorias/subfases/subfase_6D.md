# Subfase 6D - Mapa voxel global y evidencia reversible

## Estado

```text
sin hacer
```

## Dependencia

6A/6C y fronteras publicas reales de mapa/pose de Fases 3-5.

## Objetivo tecnico

Construir en `task_lib` un mapa voxel navegable incremental que se corrija al
mover/eliminar KFs, MPs o depth asociado, sin acceder a `orbslam3_multi`.

## Modelo

Separar estrictamente `occupancy/free evidence` de `coverage evidence`.

- MapPoint: indicio debil de superficie/ocupacion, ponderable por score.
- Depth endpoint: ocupacion fuerte.
- Depth ray: FREE.
- Trayectoria realmente estimada: FREE fuerte en swept volume.
- KeyFrame: referencia geometrica/coverage; nunca obstaculo por si solo.

Toda contribucion conserva procedencia `(source,drone_id,map_epoch,kf/chunk,
pose_revision)` y permite add/remove/move. No se guarda solo una probabilidad
saturada imposible de deshacer.

Depth se transforma al KF de referencia y se agrupa en `LocalVoxelSubmap[KF]`.
Si cambia `W_T_KF`, se retira la rasterizacion world anterior y se reintegra el
submapa local. El pitch actual queda incorporado en `Kref_T_C`.

## Cambios requeridos

1. Implementar `VoxelMapWorker` como unico writer y snapshots de lectura.
2. Definir chunks/voxel hash, acumuladores reversibles y fuentes identificables.
3. Auditar bootstrap/deltas/revisiones/culling/reset publicos de `orbslam3_server`.
4. Integrar add/remove/move de MP y KF sin acoplamiento interno.
5. Preparar depth chunks ligados a KF y FREE de trayectoria real.
6. Emitir `MapChangeEvent` con revision, chunks/voxels, AABB y tipo de cambio.
7. Congelar `voxel_size` por mision desde handshake; medir su valor.
8. Alimentar grafo con queue/revision/changed voxels sin eventos por voxel.

## Limites

No usar ausencia de MPs como FREE, no usar KF como obstacle, no consumir GT y no
depender de Fase 8. El filtrado de otros drones observados por depth queda como
mejora futura salvo evidencia que lo haga imprescindible.

## Pruebas

- Add/remove/move por fuente y varias contribuciones en un voxel.
- Depth ray/endpoint, pitch y reintegracion al mover KF.
- Culling/reset/epoch y evidencia FREE realmente recorrida.
- Separacion occupancy/coverage y lista exacta de voxeles afectados.
- Rendimiento/memoria multi-dron; GUI+Gazebo y grafo muestran revisiones.

## Criterio de exito

No quedan fantasmas tras correcciones, las fuentes se deshacen aisladamente,
coverage no altera occupancy y cada cambio produce una revision incremental.

## Dependencias abiertas

La politica ante desaparicion definitiva del KF se decide tras auditar F5:
identidad historica o reparentado. No inventarla durante preparacion.
