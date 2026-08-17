# 00_summary — dron_individual

Resumen: Lógica por dron: control, generación de trayectorias (`TrayAction`), launches del wrapper ORB-SLAM3 y configuración física/vision.

Interfaces/Entradas: recibe GT y sensores simulados; action `TrayAction`.

Ejecutables principales: `gen_tray`, `control_calcular_fuerzas`, `aplicar_fuerzas_dron`, `control_dron`.

Config/Launch: `config/*.yaml`, `launch/` con `orbslam_use.launch.py`.

Relación: consume `lib_tray`, es lanzado por `simulacion_dron`.

Las calibraciones `config/orbslam/orbslam_mono.yaml` y
`config/orbslam/orbslam_stereo.yaml` fijan `loopClosing: 0`. ORB-SLAM3 conserva
tracking, `LocalMapping`, fusión local, BoW y covisibilidad, mientras el
servidor mantiene la autoridad sobre loops y optimizaciones globales.

Perfil multi-dron vigente desde 3G: camaras 480x360 a 20 Hz, calibracion
coherente con baseline 0.057 m y 900 features. `generar_dron.launch.py` acepta
`orb_vocabulary_path`; el launch superior multi-dron selecciona el vocabulario
L5 compacto, mientras el launch individual conserva el vocabulario L6 completo
por defecto. Los procesos ORB limitan arenas glibc con `MALLOC_ARENA_MAX=2`.

Detalles en `launches.md`, `control.md` y archivos de config del paquete.
