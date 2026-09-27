# Prueba 7.8.2 - notas de evidencia

## Fuente exclusiva

Las figuras y metricas vigentes de 7.8.2 proceden **solo** de `run_01`.
Los intentos `run_02` a `run_15` quedan excluidos y no se usan en este analisis
ni en la sintesis 7.8.3.

## Resultado medido

- El log reducido confirma la terminacion de los seis movimientos programados.
- La telemetria registra revisita al fiducial 1 a los `123.540 s` desde la
  autoridad ORB; calidad `0.7212` y distancia `1.0330 m`.
- Se sincronizaron 6.689 muestras ORB durante `146.890 s`, con 156 cambios de
  KF, cero fallbacks y cero muestras de tracking no OK.
- El error global W maximo observado fue `0.8330 m`.
- No hay marcadores `F3E`, `F3Q` o `F3K` en el log reducido de `run_01`.
  Por tanto, esta ejecucion no demuestra una optimizacion ni un commit del
  servidor y tampoco permite una comparativa W antes/despues de commit.

## Figuras

Los artefactos reproducibles se encuentran en `processed/`:

- `figura_1_trayectoria_xy_w.png`: el mayor salto consecutivo de ORB W posterior a la revisita se resalta en rojo como `salto W observado`.
- `figura_2_error_global_w.png`: azul, media movil temporal de 2 s con huecos interpolados solo para visualizacion; gris tenue, error W instantaneo. No se dibujan marcas verticales de KF.
- `figura_3_error_local_o.png`
- `figura_4_zoom_revisita_fiducial_1.png`: tres paneles X/Y/Z, cada uno con ORB W, GT W y ORB O.

## Conclusion

**PARCIAL.** La ejecucion valida el recorrido ORB, los cambios de KF, la
revisita fiducial y la continuidad observada de O durante el recorrido. No
aporta evidencia suficiente para atribuir una correccion global a una
optimizacion/commit del servidor.
