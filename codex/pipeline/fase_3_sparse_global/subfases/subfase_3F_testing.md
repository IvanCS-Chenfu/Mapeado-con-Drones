# Subfase 3F - Testing acordado

## Paquetes

Compilar como minimo:

```text
orbslam3_multi
orbslam3_server
simulacion_dron
```

Los builds deben ejecutarse con `build_selected_packages.sh`. Si falla, reducir
el log antes de leerlo y corregir el primer error real.

## Pruebas unitarias de `LandmarkScoreManager`

1. Mismo input ORB produce siempre el mismo score.
2. Score queda en `[0,1]`.
3. `is_bad` produce score cero e invalidacion publica.
4. Cambio de `found_ratio` u observaciones afecta solo al ID indicado.
5. Descriptor nulo/no nulo aplica el bonus acordado.
6. Repetir un evento identico es idempotente y no crea revision falsa.
7. IDs de drones/epochs distintos nunca colisionan.

## Pruebas unitarias de `GlobalMapBuilder`

1. Submapa no anclado produce cero KFs y cero puntos.
2. Primer anchor incorpora todos los KFs/MPs acumulados del submapa.
3. KF nuevo en submapa anclado aparece incrementalmente.
4. Mover un KF reproyecta solo sus MapPoints asociados.
5. Cambiar un score actualiza solo los puntos indicados sin reproyectar XYZ.
6. MP/KF eliminado desaparece de cache y salida.
7. Se prefiere el KF de referencia en la primera asociacion valida.
8. El observador fallback permanece estable aunque el de referencia aparezca
   despues; solo cambia si deja de ser valido.
9. MP sin observador world se omite y nunca usa fallback de submapa.
10. Nube y KFs comparten revision/timestamp.
11. Dirty set vacio no reconstruye ni publica.
12. Ordenes distintas de IDs producen el mismo resultado determinista.

## Pruebas del adaptador ROS

- Validar schema, offsets y `point_step` de `PointCloud2`.
- Reconstruir `map_epoch` desde `map_epoch_low/high`.
- Verificar `score` y RGB para scores `0`, `0.5` y `1`:
  rojo, amarillo y verde respectivamente.
- Confirmar que el servidor copia score, no lo recalcula.
- Confirmar frustums con IDs estables, color por submapa y `DELETE` correcto.
- Comprobar QoS y que abrir RViz2 tarde recupera la ultima vista.

## Replay determinista

Reutilizar `rawdb_prueba_3e.record`. El replay debe poder introducir `100 ms`
entre entradas exclusivamente como parametro de prueba para que el flujo sea
observable en RViz2 y web.

Abrir:

- servidor en modo replay;
- RViz2;
- visualizador web.

Validar:

1. mismas entradas y anchors que 3E;
2. scores creados solo para MPs afectados;
3. builder inicialmente vacio para submapas no anclados;
4. backfill al primer anchor de cada submapa;
5. nube y frustums aparecen con revisiones coherentes;
6. replay repetido produce mismos conteos, asociaciones y revision final;
7. no se consume GT live para construir mapa o score.

## Simulacion integrada

La trayectoria acordada es:

1. iniciar Gazebo, wrappers, servidor, RViz2 y grafo web;
2. esperar tracking valido de ambos drones;
3. enviar ambos drones al fiducial 2;
4. mantenerlos el tiempo suficiente para crear ambos anchors;
5. desplazarlos despues hacia `x=-8`;
6. mantener espera visual para observar nuevas incorporaciones.

### RViz2 esperado

- Antes del anchor no aparece ningun dato de submapas no anclados.
- Al anclarse cada submapa aparece su backfill de KFs y MapPoints.
- Los KFs son frustums y cada submapa tiene color propio estable.
- Los MapPoints recorren rojo-amarillo-verde segun score.
- Tras moverse hacia `x=-8`, nube y KFs crecen o se actualizan en vivo.
- No quedan marcadores obsoletos ni nubes duplicadas por revision.

### Grafo web esperado

- Se observa otra vez `first anchor` cuando corresponda.
- `RawMapDatabase -> LandmarkScoreManager` se activa por score raw.
- `LandmarkScoreManager -> GlobalMapBuilder` se activa por score dirty.
- Raw y poses activan sus aristas propias hacia el builder.
- Build y ambas publicaciones a RViz2 muestran el mismo flow/revision.
- Un delta sin cambios publicables no simula un flujo completo hasta RViz2.

## Diagnostico de logs

Nunca abrir el log completo. Reducir primero con patrones equivalentes a:

```text
F3F-|F3E-FIRST-ANCHOR|PRIMARY-WORKER|RAWDB-|POSESTORE-|
PIPELINE-FLOW|global_sparse_cloud|global_keyframes|ERROR|FATAL|
Segmentation fault|Killed
```

Extraer sublogs especificos para:

- score y revision;
- dirty sets y elementos recalculados;
- anchors/backfill;
- publicaciones cloud/KFs;
- cola principal/backpressure;
- errores y cierres.

## Evidencia que debe reportarse

- scores min/mean/max y cantidad tracked/bad;
- submapas anclados/no anclados;
- KFs/MPs cacheados, recalculados y omitidos;
- asociaciones a KF de referencia/fallback;
- contador de fallback geometrico igual a cero;
- revisiones raw/pose/score/build/publicadas;
- conteos de puntos y markers por publicacion;
- `PrimaryWorker max_active=1` y estado final de cola;
- resultado del replay repetido;
- observacion del usuario en RViz2 y grafo web.
