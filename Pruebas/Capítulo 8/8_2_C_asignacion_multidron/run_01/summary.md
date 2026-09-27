# Prueba 8.2-C - asignacion multidron

Resultado: **INTERRUMPIDA POR EL USUARIO**.

- Escenario preparado: D1 `(0, -10, 1, 90)` a `(-2, -10, 1, 90)`; D2 `(0, -10, 1.3, 90)` a `(2, -10, 1.3, 90)`.
- Modo: dos drones, GT, mision autonoma y `mission_house.yaml`.
- La simulacion se detuvo durante la ventana de observacion posterior al escenario.
- El primer capturador no cargo las interfaces ROS; el relanzado con overlays correctos fue detenido antes de generar el resumen procesado.
- No se infiere ningun resultado de asignacion ni se usa este intento como evidencia de 8.2-C.

El log bruto y el reducido central se conservan como trazabilidad; `run_01` no debe borrarse al crear una repeticion.
