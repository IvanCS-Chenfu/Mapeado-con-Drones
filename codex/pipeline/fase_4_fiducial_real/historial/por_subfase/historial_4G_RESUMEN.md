# Resumen 4G

Estado agregado: **CONSEGUIDA**.

Servidor interpreta cada batch sincronizado mediante `FiducialObjectInterpreter`:
configuracion `yaml-cpp`, rango por tag, fusion robusta SE(3), primary
determinista, visitas por intervalos y FIFO reciente 50 por dron. Las pruebas
214/215 cubren los tres objetos. La carrera temporal detectada en 214 quedo
corregida y probada en 215. No usa GT, no cambia scoring 3R ni poses raw.
