# Snapshot legacy2 de orbslam3_server

Instantanea creada el 2026-08-10 al rehacer la subfase 3B.

Contiene el estado completo del servidor anterior a la reconstruccion:

- `src/`: servidor global, corrector de poses y tests existentes;
- `include/`: cabeceras del servidor anterior;
- `launch/`: launch activo anterior y launch historico `_antiguo`;
- `CMakeLists.txt.snapshot` y `package.xml.snapshot`: configuracion original.

Este arbol es solo de consulta. No se compila, instala ni se incluye desde los
targets activos. Los nombres de los metadatos estan alterados deliberadamente
para impedir que Colcon lo descubra como un paquete independiente.
