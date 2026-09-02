# Resumen — Fase 8: Nube densa global multi-dron

## Estado

```text
sin hacer
Preparación documental: cerrada
Autorización de ejecución: pendiente
Historial: vacío; no existen ejecuciones reales en este ZIP
```

## Objetivo

Crear en servidor una reconstrucción densa global a partir de estéreo, relacionada con el mapa sparse pero sin fusionar ambos en una única nube ni modificar datos raw de ORB-SLAM3. La geometría densa se construye como subnubes locales a KFs, voxels/blocks derivados y capturas estacionarias correctivas.

## Arquitectura

```text
Dron: captura/transporte mínimo
  -> wrapper envía L/R exactas cuando nace un KF
  -> topics L/R normales para capturas/seguridad

Servidor:
  dense_map_server -> coordinación ROS
  dense_map_multi  -> algoritmos dense
  orbslam3_multi   -> poses/grafo/raw sparse
```

Paquetes nuevos propuestos:

```text
src/servidor/dense_map_multi/
src/servidor/dense_map_server/
```

## Decisiones cerradas

- Todo disparity/depth/nube/fusión se calcula en servidor.
- DenseKF clave `(drone_id,map_epoch,kf_id)` y nube local al propio KF.
- No se guardan imágenes L/R permanentemente.
- La pose `world` de una subnube siempre se deriva de la pose global vigente del KF.
- Sparse y dense son representaciones independientes pero acopladas.
- Dos escalas voxel:
  - occupancy gruesa del ROI: UNKNOWN/FREE/OCCUPIED + evidencia/confianza;
  - geometría fina sparse por blocks/voxels, apta para TSDF-like.
- Depth que no genera nube puede actualizar occupancy mediante raycast.
- El buffer sparse simetrico de Fase 3P es un prototipo acotado de
  proyeccion/oclusion para scoring, no una fuente dense. Al preparar 8C se
  revisara si sus interfaces o telemetria son reutilizables y si el depth real
  por KF puede mejorar posteriormente la evidencia de visibilidad de 3P.
- Occupancy no caduca por tiempo; nueva evidencia puede corregir una creencia antigua.
- 8E prueba primero todos los KFs como dense y movimiento real; 8F reduce redundancia y 8G mejora calidad.
- 8I reintegra contribuciones fusionadas tras cambios de poses.
- 8J obtiene registros dense; 8K usa el mismo optimizador de Fase 3. No hay segundo pose optimizer.
- `SparseDenseCorrectionDatabase` refina MPs solo en la salida de servidor; `RawMapDatabase` y ORB local quedan intactos.
- Tras mapping, 8N detecta huecos/errores y `task_server` de Fase 6 asigna las
  tareas correctivas.
- Captura HQ parada usa topics L/R normales; no necesita KF nuevo.
- Recaptura de DenseKF malo sustituye atómicamente su nube; la antigua se borra tras commit correcto.
- Zona sin KF se integra por patches rígidos asociados a KFs, no punto-a-punto.
- Planificación usa sparse+dense+occupancy, pero depth local sigue evitando obstáculos nuevos.
- Compresión de imágenes se decide en 8R después de medir tráfico.
- GUI real se usa desde 8E; multi-dron está presente desde la DB/fusión, no como parche final.

## Secuencia

```text
8A  calibración/frames + paquetes dense
8B  imágenes L/R exactas del KF
8C  disparity/depth/DenseSubcloud local
8D  DenseKeyFrameDatabase
8E  baseline todos los KFs + GUI + optimizaciones
8F  selección DenseKF
8G  calidad/filtros
8H  occupancy + DenseFusionMap
8I  reintegración por revisiones
8J  registro/fusión dense-dense
8K  constraints dense en grafo Fase 3
8L  corrección derivada de MapPoints
8M  revalidación tras optimización
8N  cobertura/calidad -> necesidades de tarea
8O  captura HQ estacionaria
8P  recaptura/reemplazo + patches
8Q  planificación/occupancy/depth local
8R  rendimiento/red/memoria
8S  exportación
8T  integración final multi-dron
```

## Prueba final

La posible realimentacion de depth de 8C hacia scoring/fusion 3P requiere una
preparacion funcional propia: debe demostrar alineacion temporal, disponibilidad
por KF y coste acotado, sin convertir Fase 8 en requisito para el mapa sparse.

Dos o más drones realizan mapping sparse con DenseKF oportunistas; el sistema corrige/reintegra tras optimizaciones, fusiona solapes, analiza el ROI, genera tareas correctivas estacionarias, usa occupancy+dense+sparse en planificación, mantiene depth local para obstáculos nuevos y exporta un mapa final sin GT funcional. Si una fase anterior produce datos incorrectos, se vuelve a esa fase y se corrige antes de cerrar Fase 8.
