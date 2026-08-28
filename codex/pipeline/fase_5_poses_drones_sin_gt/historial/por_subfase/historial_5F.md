# Historial de subfase 5F

## 2026-08-27 - Prueba 226

- resultado: NO CONSEGUIDA; YAML relativo no encontrado y colision con propiedad `rclpy.Node.subscriptions`.
- evidencia positiva: PENDING, anchors, push revision 1 y autoridad en ambos drones.

## 2026-08-27 - Prueba 227

- resultado: PARCIAL; ruta aceptada y metricas activas, pero dron 2 pierde tracking y el siguiente goal se rechaza antes de 5G.
- evidencia adicional: loops, revisiones naturales y carrera de doble shutdown detectada.

## 2026-08-27 - Prueba 228

- resultado: PARCIAL; incluso un tramo de 3 m provoca perdida 2->3->0->1 del dron 2 y rechazo posterior.
- correccion: lifecycle idempotente y validacion dirigida sin goals tras la perdida.

## 2026-08-27 - Prueba 229

- resultado: escenario CONSEGUIDO, metrica PARCIAL; `success=true`, revisiones hasta 3 y artefactos completos.
- defecto medidor: latencia invalida por mezclar reloj ROS y stamp; corregida sin alterar poses.

## 2026-08-27 - Prueba 230 vigente

- escenario: exit 0, `success=true`, 171 s, guarda de recursos inactiva.
- builds/tests: `simulacion_dron` correcto y CTest 11/11.
- evidencia: dos anchors hard, loops fusionados, pushes primary/secondary, revisiones hasta al menos 4, perdida real del dron 2 y cierre sin ERROR/FATAL.
- temporal: ~19.16/19.15 Hz; jitter p95 1/3 ms; latencia relativa p95 0.480/0.240 s.
- W posicion MAE: 2.953 m (dron 1), 0.191 m (dron 2).
- W angular MAE: 1.907 rad (dron 1), 2.104 rad (dron 2).
- artefactos: `codex/archivos_auxiliares/metricas/prueba_230/{summary.json,drone_*_samples.csv,drone_*_o_w_gt.png}`.
- interpretacion revisada con el usuario: el error GT->W no tiene por que ser
  cero mientras existe deriva y el backend publica revisiones sucesivas. Los
  agregados actuales mezclan todas las muestras autoritativas y no separan la
  ventana posterior a cada optimizacion ni la revision final; por tanto no
  demuestran por si solos una incoherencia de W. La mejora de posicion O->W es
  visible en ambos drones, especialmente en el dron 2. El error angular grande
  y casi constante debe contrastarse con las convenciones body/camera/GT antes
  de atribuirlo al optimizador.
- conclusion: PARCIAL; instrumentacion conseguida e interpretacion de calidad
  incompleta. Falta estratificar por revision/optimizacion y mantener la puerta
  humana pendiente.

## 2026-08-27 - Pruebas 231-233, preparación visual

- prueba 231: NO CONSEGUIDA antes de Gazebo por faltar permiso ejecutable en el
  nuevo visualizador; corregido mecánicamente y cubierto por rebuild/CTest.
- prueba 232: NO CONSEGUIDA porque solo el dron 2 obtuvo anchor antes del primer
  goal; el gate rechazó al dron 1.
- prueba 233: NO CONSEGUIDA porque ambos drones seguían sin anchor antes del
  primer goal. Confirmó que el YAML necesita moverse mediante GT para alcanzar
  el primer fiducial y detectó una carrera de shutdown del visualizador.
- consecuencia: el harness se redefinió como política GT legacy completa,
  default false y sin usar `NavigationState` para gobernar el movimiento.

## 2026-08-27 - Prueba 234 y revisión visual humana

- objetivo: ejecutar el YAML típico exacto con control GT legacy y observar en
  RViz2 exclusivamente los KFs y `W_T_B` estimados.
- builds/tests: `dron_individual` y `simulacion_dron` correctos; policy 1/1 y
  CTest Simulación 12/12.
- ejecución: exit 0, `success=true`, 17/17 pasos y 22/22 goals absolutos
  aceptados con `legacy_gt_sim=true`; ningún rechazo. Guarda inactiva y mínimo
  MemAvailable 5224.6 MiB.
- visualizador: listo en frame `world`, ambos drones reciben poses globales y
  revisiones sucesivas; cierre limpio sin traceback.
- métricas: 9494/9339 muestras emparejadas y 3820/4144 autoritativas para
  drones 1/2. El dron 1 atraviesa epochs 0, 1 y 2; el dron 2 permanece en 0.
- revisión humana: las poses estimadas están sobre el camino generado por los
  KFs y se recolocan al optimizar. Persisten defectos de optimización que
  pertenecen a Fase 3 y no se corregirán en Fase 5.
- defecto visual: los ejes parpadean con mucha más frecuencia que las pérdidas
  reales. La causa confirmada es que cada cambio de reference KF convierte W en
  `PROVISIONAL` mientras llega la pose autoritativa; `global_valid=false` hace
  que el visualizador borre los markers aunque el tracking siga válido y exista
  `W_T_B` propagada.
- conclusión: PARCIAL. La relación pose estimada/KFs y su actualización tras
  optimización están validadas visualmente; queda acordar si RViz debe mostrar
  también W `PROVISIONAL` y ocultar solo W `INVALID` o pérdida real.

## 2026-08-27 - Prueba 235, corrección antiparpadeo

- cambio: ejes visibles con W `PROVISIONAL` o `AUTHORITATIVE` mientras local y
  continuidad sean válidas; borrado con W `INVALID` o pérdida real.
- validación: build correcto, CTest 12/12 y simulación `success=true` con el
  mismo YAML exacto que 234; guarda inactiva.
- análisis: omitido por petición del usuario.
- conclusión: PARCIAL pendiente de confirmación visual.
