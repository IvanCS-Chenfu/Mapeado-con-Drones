# Historial de subfase 5B

## 2026-08-26 - Implementacion y validacion determinista

- objetivo intentado: crear recibo ORB coherente, continuidad `O_T_B` intra-epoch y política de goals de 5B.
- archivos modificados: core `System`, mensajes, wrapper estéreo, `gen_tray`, launch, runner y escenario de simulación.
- paquetes compilados: `orbslam3_msgs` Dron/Servidor, target `ORB_SLAM3`, `orbslam3`, `dron_individual` y `simulacion_dron`.
- resultado de build: todos correctos; el target core alcanzó 100 % y cada paquete seleccionado terminó 1/1.
- tests: `orbslam3` 2/2, política nueva 1/1 y `simulacion_dron` 10/10. La suite global de `dron_individual` quedó 4/7 por deuda legacy de linters; código/test 5B pasan aislados.
- evidencia positiva: `NavigationState`, estimador continuo y gate disponen de regresiones deterministas.
- evidencia negativa: ninguna funcional; warnings core y deuda de linters preexistentes.
- conclusión: PARCIAL hasta completar Gazebo.

## 2026-08-26 - Prueba 221

- objetivo intentado: fiducial, gate, giro 180 grados y pérdida real.
- pruebas Gazebo/replay: prueba 221, exit 1 y `success=false`.
- patrones: `F5B|SCENARIO-RUNNER|FID|HARD|ANCHOR|RECENTLY_LOST|LOST|ERROR|FATAL`.
- evidencia positiva: wrapper e interfaz arrancaron sin errores de enlace.
- evidencia negativa: timeout de spawn del fiducial 1; ambos ORB quedaron `NOT_INITIALIZED`; el escenario expiró en el paso 1.
- conclusión: NO CONSEGUIDA por despliegue inicial no reproducible.
- siguiente paso recomendado: override de spawn desactivado por defecto y repetición independiente.

## 2026-08-26 - Prueba 222

- objetivo intentado: repetir el escenario con spawn dirigido cerca del fiducial.
- pruebas Gazebo/replay: 7/7 pasos, exit 0, `success=true`, guarda de recursos no activada.
- evidencia positiva: tracking 2, cambios ref-KF sin salto, absoluto rechazado por global inválida, relativos con snapshot y pérdida 2->3->0->1 de ambos drones.
- evidencia negativa: la telemetría del servidor estaba desactivada; observaciones fiduciales válidas no confirmaban por sí solas el commit hard.
- conclusión: PARCIAL por observabilidad incompleta del anchor.

## 2026-08-26 - Prueba 223

- objetivo intentado: observar explícitamente el anchor hard.
- pruebas Gazebo/replay: exit 1; escenario detenido en paso 1.
- evidencia positiva: ambos ORB alcanzaron tracking 2.
- evidencia negativa: timeout transitorio de `fiducial_object_2`; el spawner murió y no publicó readiness.
- conclusión: NO CONSEGUIDA por infraestructura, sin regresión funcional.

## 2026-08-26 - Prueba 224

- objetivo intentado: repetición del escenario tras el timeout transitorio.
- pruebas Gazebo/replay: 7/7 pasos, exit 0, `success=true`, guarda no activada.
- evidencia positiva: repite íntegramente gate, continuidad y pérdida de ambos drones.
- evidencia negativa: se usó el argumento incorrecto `fase3_logs_terminal`; no habilitó marcadores F3 del servidor.
- conclusión: PARCIAL por observabilidad incompleta.

## 2026-08-26 - Prueba 225 de cierre

- objetivo intentado: validación integral con `debug_fase3_logs_terminal:=true`.
- pruebas Gazebo/replay: 7/7 pasos, exit 0, `success=true`; mínimo MemAvailable 6629.6 MiB, máximo ORB PSS 1000.8 MiB y guarda no activada.
- patrones: `F3E-FID|F3O-FID|F5B|SCENARIO-RUNNER|FID-(TAG|BATCH|SPAWN)|RECENTLY_LOST|LOST|ERROR|FATAL`.
- evidencia positiva: anchors hard aplicados en `(1,0)` y `(2,0)` antes de la misión; absoluto rechazado; relativos congelados; ref-KF cambia con paso cero; ambos drones pierden tracking 2->3 y no publican continuidad válida, después reinician 3->0->1.
- evidencia negativa: exit 255 conocido de Gazebo durante cleanup posterior a `success=true`; no afecta el escenario.
- conclusión: CONSEGUIDA.
- siguiente paso recomendado: preparar conversadamente el bloque 5C+5D+5E+5F sobre el HEAD vigente.
