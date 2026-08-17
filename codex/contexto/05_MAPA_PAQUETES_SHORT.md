# 05 — Mapa de paquetes (Resumen rápido)

Resumen: Índice rápido de paquetes y responsabilidades. Leer este archivo antes
de abrir `05_MAPA_PAQUETES.md` completo.

Dónde buscar:
- Contexto mínimo: `codex/contexto/CONTEXTO_MINIMO_ACTUAL.md`
- Política tokens: `codex/contexto/08_POLITICA_TOKENS_DOCUMENTACION.md`
- Paquetes: `codex/contexto/paquetes/` (buscar `SUMMARY.md` dentro de cada paquete)
- Subfases activas: `codex/pipeline/fase_3_sparse_global/subfases/`
- Historial por subfase activa: `codex/pipeline/fase_3_sparse_global/historial/por_subfase/`

Líneas clave (una por paquete):
- `orbslam3_msgs`: Contrato ROS 2 entre wrapper, servidor y corrector.
- `orbslam3_ros2`: Wrapper estéreo ORB-SLAM3 (publica pose local, OrbMap delta).
- `orbslam3_multi`: backend 3L con raw/poses/score/builder y optimizacion fiducial privada.
- `orbslam3_server`: workers principal/secundario, replay/backpressure y publishers cloud/KFs.
- `dron_individual`: Control por dron y acciones.
- `simulacion_dron`: Gazebo, RViz2, launches live/replay y grafo web 3L 16/25.
- `lib_tray`: Generación de trayectorias.

Para detalles completos abrir `05_MAPA_PAQUETES.md`.
