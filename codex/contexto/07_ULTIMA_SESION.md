# Ultima sesion

## Resumen

Prueba 7.8.1-B del Capitulo 7, perfil trapezoidal: **CONSEGUIDA** en `run_01`.
Con 7.8.1-A cúbica ya conseguida, queda completada la comparación de ambos
perfiles cortos ORB.

## Validacion

Sin cambio de código ni build adicional. Un dron siguió la misma aproximación
GT, tomó autoridad ORB en la secuencia 2840 y ejecutó
`tipo_trayectoria=1` hasta `(-7,-10,1,90°)`. El escenario y wrapper terminaron
con código 0 y `[SIM-DONE] success=true`; la duración trapezoidal fue 13.666 s
y el gate final quedó a 0.070 m.

Las 668 muestras evaluadas fueron 100 % ORB, con cero fallback y cero
eventos no OK. RMSE de seguimiento: 0.0715 m; RMSE ORB-GT
alineado: 0.0363 m. Se generaron CSV, resumen y seis
figuras, incluidas las comparativas GT-trayectoria y error 3x4.

## Pendiente

Trabajo activo: no. La siguiente prueba del capítulo es 7.8.2, deriva,
revisita fiducial y optimización durante el control.
