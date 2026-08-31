# Explicacion de Fase 5

## A. Objetivo inicial

Controlar cada dron con una pose estimada, separando continuidad local y
correcciones globales, sin usar GT como pose final.

## B. Arquitectura de frames

O es el frame local continuo del controlador. W es el frame global corregible.
`O_T_B` gobierna control; `W_T_B` permite goals absolutos y visualizacion.

## C. Integracion inicial

5A-5G construyeron contratos, consulta global y mensaje comun. 5H conecto
`NavigationState` con mux, trayectoria y controlador, con fuente atomica por
goal y `GT_FALLBACK` temporal.

## D. Frecuencias

ORB entregaba medidas alrededor de 20 Hz y control trabajaba a 50 Hz. Mantener
la ultima muestra generaba tirones y derivadas discontinuas. El wrapper paso a
publicar estado propagado a 50 Hz.

## E. Extrinseca

La composicion inicial de camara y body estaba invertida. La correccion de
`B_T_C` alineo pose, keyframes y control sin compensaciones en `gen_tray`.

## F. Desfase temporal angular

R y omega debian representar el mismo horizonte. Filtrar saltos sin mantener
esa coherencia reducia sintomas, pero podia inyectar energia en el controlador.

## G. Diagnosticos con GT

Las baterias GT aislaron causalidad; nunca definieron el producto. Demostraron
que R/omega y p/v contenian problemas independientes y guiaron las correcciones.

## H. Predictor angular

Omega se estima causalmente y se propaga usando torque, ecuaciones de cuerpo
rigido e inercia compuesta real. Los buffers no consumen muestras futuras.

## I. Problema translacional

La derivacion multipunto amplificaba ruido y aceleracion (`A_HAT_AMPLIFICATION`).
El fallo se manifesto especialmente al frenar y mantener hover.

## J. Gravedad

La gravedad debe expresarse en O. `g_O = O_R_W * g_W` se congela por epoch para
no mover el frame continuo durante optimizaciones globales.

## K. MIDPOINT_DYNAMIC

Reconstruye velocidad en el instante medio entre dos poses aceptadas y propaga
hasta el presente con thrust, gravedad y estado angular coherentes.

## L. Buffers

Thrust y torque mantienen una muestra predecesora ZOH, eliminan futuro no
causal y sobreviven a reset visual porque la actuacion fisica no desaparece.

## M. Hover estable

330/331 validaron hover ORB real sin fallback y con energia angular neta
disipativa.

## N. Trayectorias simples

X, Y, Z y yaw se probaron por separado. Una prueba Y invalida colisiono con un
fiducial; otras revelaron que distancia y calidad visual condicionan ORB.

## O. Ruta representativa

La ruta de dos fachadas reprodujo fallos que no aparecian en movimientos cortos
y permitio separar problemas internos de degradacion visual.

## P. STALE_RAW_HISTORY

Cambios de reference KF podian dejar historia raw incompatible. La politica
causal invalida o reinicia la historia visual afectada sin borrar buffers
fisicos.

## Q. Pulso de validez

Una invalidacion aislada podia disparar fallback. La validez se hizo coherente
con el estado publicado y la politica de source lock.

## R. Bateria 349 GT/ORB

Los cruces p/v frente a R/omega demostraron errores suficientes en ambos
bloques. Los overrides se retiraron al terminar el diagnostico.

## S. Evidencia visual

El recibo de tracking sella inliers, matches, profundidad, disparidad,
cobertura, reference KF y `Tcr` del mismo frame. El volcado es opcional.

## T. Prueba 351

Con GT gobernando y ORB en observacion, inliers, cobertura y profundidad caen
antes de `RECENTLY_LOST`. El fallo de esa fachada tiene causa visual observable.

## U. Pruebas 352-355

352 valido una ruta visualmente favorable. 353-355 completaron tres ejecuciones
con autoridad ORB, tracking continuo y sin fallback posterior.

## V. Conclusion

Fase 5 queda conseguida en su dominio: ORB puede gobernar movimiento cuando la
evidencia visual es suficiente. No se afirma robustez universal ante escenas
pobres.

## W. Transferencia a Fase 6

Fase 6 debe planificar segun tendencia de evidencia, distinguir GOOD,
DEGRADING y POOR, y retirar gradualmente `GT_FALLBACK`. Los umbrales deben salir
de datos, no de constantes inventadas.
