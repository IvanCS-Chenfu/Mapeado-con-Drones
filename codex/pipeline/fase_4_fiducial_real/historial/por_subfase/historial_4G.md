# Historial 4G

## 2026-08-25 - Interpretacion visual de objetos fiduciales

- objetivo: convertir batches 4F en objetos fiduciales sin GT;
- cambios: `FiducialObjectInterpreter`, integracion en `GlobalMapServer`,
  CMake/package, launch, YAML y tests;
- comportamiento: carga `yaml-cpp`, rango `[1,5] m` por tag, fusion robusta,
  primary unico, FIFO 50 y visitas configurables;
- builds: `orbslam3_server` y `simulacion_dron`, correctos;
- tests: 5 gtests nuevos; servidor 73 y Simulacion 85 tests sin fallos; guarda 15/15;
- prueba 214: 80 batches/80 primary, 94 tags, cero rechazos y tres objetos;
- incidencia: el primer algoritmo creaba visitas con timestamps fuera de orden;
- correccion: intervalos temporales y test 10.0, 11.0, 10.5, 13.1 s;
- prueba 215: 73/73 primary, 83 tags, tres objetos y visitas correctas;
- conclusion: CONSEGUIDA.
