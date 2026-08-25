# Historial 4A

## 2026-08-24 - Implementacion y validacion contractual

- objetivo intentado: definir y validar el contrato geometrico de los tres objetos fiduciales baseline;
- archivos modificados: perfiles `fiducial_objects.yaml`, rendering, spawner, launch, tests y metadata declarativa;
- paquetes compilados: `simulacion_dron` y `orbslam3_server`, un paquete por invocacion;
- resultado de build: ambos correctos; rebuild de `simulacion_dron` correcto tras ajustes de estilo y cierre;
- pruebas: contratos directos 38/38; CTest final 10/10 en ambos paquetes; guarda completa final previa 15/15;
- intentos preservados: CTest sandbox sin ejecucion por `LastTest.log`; CTest real 9/10 por flake8; guarda 14/15 por `__pycache__`; pytest sin ejecucion por rutas mal nombradas;
- correcciones: estilo mecanico, retirada de caches generadas y repeticion con rutas correctas;
- evidencia positiva: perfiles instalados identicos, IDs 101-105/201-205/301-305, objetos a ±8.5 m y rango `[1,5] m`;
- conclusion: CONSEGUIDA;
- siguiente paso recomendado: usar el contrato como entrada de 4C+4D sin cambiar su ownership.

## 2026-08-24 - Pruebas Gazebo 201 y 202

- objetivo intentado: demostrar que el contrato genera assets y SDF reales antes de declarar readiness;
- pruebas Gazebo: 201 completa y 202 smoke de cierre;
- patrones de reduccion: `FID-TEXTURE`, `FID-SDF`, `FID-SPAWN`, `SCENARIO-RUNNER`, GUI/debug y errores;
- evidencia positiva: en ambas pruebas, 15 texturas redetectadas, 3 SDF, 3 spawn y `FID-SPAWN-ALL-DONE` antes de readiness;
- evidencia negativa: la 201 descubrio doble shutdown de `rclpy` al cleanup; no afecto al escenario;
- correccion: `ExternalShutdownException` y `KeyboardInterrupt` son cierre normal, con shutdown condicionado a `rclpy.ok()`;
- resultado posterior: prueba 202 con spawner terminado limpiamente, sin traceback ni `FID-SPAWN-ERROR`;
- conclusion: CONSEGUIDA.
