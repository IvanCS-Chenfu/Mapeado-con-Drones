# Snapshot legacy2 de orbslam3_multi

Instantanea creada el 2026-08-10 al rehacer la subfase 3B.

Contiene el estado completo de la implementacion multi-dron anterior a la
reconstruccion:

- `src/`: implementaciones, tests y ejecutables auxiliares existentes;
- `include/`: todas las cabeceras publicas existentes;
- `CMakeLists.txt.snapshot` y `package.xml.snapshot`: configuracion original.

La carpeta `legacy/` anterior permanece separada e intacta. Este nuevo arbol es
solo de consulta: no se compila, instala ni se incluye desde targets activos.
Los metadatos se renombraron para impedir que Colcon lo descubra como paquete.
