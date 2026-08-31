# Resultado Fase 5

## Estado

`CONSEGUIDA` dentro del dominio visual validado. El cierre post-limpieza de 5J
tambien esta conseguido.

## Objetivo

Publicar y consumir pose/velocidad estimadas de cada dron en un frame local
continuo y un frame global corregible, permitiendo control sin GT cuando ORB
dispone de evidencia suficiente.

## Arquitectura final

`orbslam3_ros2 -> navigation_state_mux -> gen_tray/control`. ORB estima y
propaga; el mux selecciona fuente; trayectoria y control consumen una sola
interfaz.

## Contratos de frames

- `O_T_B`: continuo, causal y usado por control.
- `W_T_B`: global, revisable y usado para absolutos/visualizacion.
- las optimizaciones globales no se reinyectan en el mapa local ORB.

## Estado final de subfases

5A-5G consiguieron contratos, estado local/global y transporte. 5H consiguio la
integracion. 5I consiguio la estabilizacion en movimiento. 5J consolido y
valido el cierre.

## Implementacion final

Estado ORB a 50 Hz, reference KF real, `B_T_C` correcta, predictores causales
angular/translacional, gravedad en O, `MIDPOINT_DYNAMIC`, buffers fisicos y
source lock por goal.

## Que se valido

Hover, movimientos simples y tres rutas favorables ORB consecutivas. La
evidencia visual de 351 demostro degradacion previa a la perdida.

## Que no se exige

No se exige completar con ORB una ruta que atraviese fachadas visualmente
pobres ni implementar todavia planificacion por evidencia.

## Limitaciones conocidas

ORB depende de textura, profundidad, cobertura y geometria. Las optimizaciones
de Fase 3 pueden mover W y conservan sus propias limitaciones.

## Dominio valido de funcionamiento

Movimiento lento, escena estereo favorable, tracking `OK`, anchor valido y
estado causal fresco.

## Evidencia ORB

Se conservan metricas del mismo frame para diagnostico y para derivar la futura
politica GOOD/DEGRADING/POOR.

## Pruebas finales

350R-352 fijan baseline/causalidad; 353-355 aportan el cierre funcional 3/3.
Tras la limpieza, 357 y 358 repiten con exito la ruta ORB productiva. La 356 no
cuenta como regresion ORB porque no obtuvo anchor, pero valida debug OFF.

## GT_FALLBACK

Permanece temporalmente en Fase 5 para completar tareas cuando ORB no esta
anclado o se pierde. No corrige el mapa ni el estimador y debe retirarse en
Fase 6.

## Debug

`debug_fase_5=false` bloquea telemetria extensa. Con master activo, los
subflags habilitan control o evidencia visual. Warnings y errores reales siguen
visibles.

## Deuda transferida a Fase 6

Clasificacion persistente de evidencia, planificacion anticipada, tareas
relativas de recuperacion y eliminacion progresiva del fallback GT.

## Condiciones para reabrir Fase 5

Reabrir solo si ORB diverge con evidencia visual buena, si se rompe causalidad
o frames, o si una regresion equivalente a las finales falla sin degradacion
visual previa.
