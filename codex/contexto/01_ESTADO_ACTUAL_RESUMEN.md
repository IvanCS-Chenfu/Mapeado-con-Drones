# Estado actual - resumen

## Situacion

```text
Fase 2: CONSEGUIDA
Fase 3: cierre previo conseguido; reabierta únicamente en 3Q
Fase 4: CONSEGUIDA Y CERRADA con alcance 4A-4H
Fase 5: 5A CONSEGUIDA documentalmente; 5B-5H sin hacer
4A: CONSEGUIDA
4B: CONSEGUIDA
4C: CONSEGUIDA
4D: CONSEGUIDA
4E: CONSEGUIDA
4F: CONSEGUIDA
4G: CONSEGUIDA
4H: CONSEGUIDA
4I: APLAZADA; regresion opcional futura
Subfase actual: 3Q A REVISAR tras mejora conservadora y prueba 220
Preparacion 3Q: CERRADA; autorizacion ejecutada
Siguiente punto de entrada: continuar pipeline; reabrir 3Q solo si reaparece el fallo
Trabajo funcional activo: ninguno
Punto de entrada paralelo Fase 5: preparar 5B
Revision visual de prueba 200: confirmada correcta por el usuario
Pendiente de Fase 2: ninguno
Autorizacion 4A+4B: concedida y consumida
```

## Bloque 4A+4B

- tres objetos fiduciales baseline a ±8.5 m, 15 tags y rango `[1,5] m`;
- spawner Gazebo con validacion offline, SDF dinamico y readiness transient-local;
- trayectoria de dos drones a ±10 m: 10/10 goals correctos en prueba 201;
- prueba 202 confirma cierre limpio del spawner;
- Gazebo y RViz2 activos; ambos grafos web desactivados;
- revision visual: confirmada perfecta por el usuario;
- trayectoria tipica: cuadrado ±10 con paradas cardinales, usada parcialmente en 205.

## Bloque 4C+4D

- evento one-shot y recibo exacto de KF, imagen y calibracion implementados;
- servicio con 15 tags, cliente, cola 4/drop-oldest, worker y detector validados;
- prueba 205: tag 202 detectado por ambos drones con error menor de `0.26 px`;
- pruebas 205/206 revelaron contaminacion Snap y que un cierre forzado de
  HighGUI dentro de `stereo` mata el wrapper;
- el wrapper publica ahora imagen anotada latest-only y un proceso ROS separado
  `fiducial_visualizer` posee HighGUI;
- prueba 207: escenario completo y wrappers estables, pero una carrera cerraba
  las ventanas en 3-4 ms antes de hacerlas visibles;
- prueba 208: escenario completo, 79 SHOW, 17 timeouts, cero cierres falsos y
  deltas de ambos wrappers 57 s despues del ultimo cierre;
- el usuario acepta la 208 y da 4C+4D por concluidas.

## Bloque 4E+4F

- topic reliable/volatile KeepLast(32), batch no vacio y solo tags validos;
- identidad y timestamp de KF exactos, con `camera_T_tag` y metricas finitas;
- sidecar pending O(1), FIFO por dron, capacidad configurable 10 y sin TTL;
- digest consumido sustituye el flag `fiducial_batch_consumed`;
- pruebas unitarias, builds y CTests completos correctos;
- prueba 210: trayectoria tipica, 68/68 matches, pico pending 7/10 y cero
  expulsiones/conflictos/rechazos;
- prueba 211: ambos grafos live y 18/18 matches adicionales.

## Bloque 4G+4H

- interpretador en Servidor con `yaml-cpp`, rango por tag, fusion robusta,
  primary unico, visitas por intervalos y FIFO 50;
- todos los KFs primary llegan al `FiducialAnchorManager` existente;
- ruta GT fiducial eliminada de codigo, configuracion, replay y grafos;
- CTest: Servidor 150 y Simulacion 85 tests sin fallos;
- prueba 216: trayectoria completa sin GT, 52/52 primary y tres objetos;
- smoke 217: ambos grafos live; guardas 15/15.

Repeticion visual 212: el YAML incorpora seis yaw relativos y pasa contratos,
pero un loop incompatible con una hard constraint activo un fallo bloqueante
antes del paso 5. Los giros nuevos no se ejecutaron; la repeticion esta
suspendida hasta decidir como tratar el mission gate.

El usuario decide eliminar el latch persistente de fallo. Tras el cambio,
`orbslam3_server` compila y pasa 12/12 targets. La prueba 213 completa 17/17
pasos y 22/22 goals, libera el backpressure tras cada optimizacion y produce
74/74 PUB/SHOW fiduciales. El usuario da 4A-4F por concluidas, pero no acepta
esta ejecucion como validacion de 3Q: hubo derivas visibles y nueve propuestas
loop tardias rechazadas. La prueba 213 queda como reentrada obligatoria de 3Q.

## Correccion 3Q y pruebas 219-220

- ventana segmentada comun para loop/fiducial, sin loop sintetico ni cierre por
  submapa completo;
- solo hard inmovil; KFs internos sin deadband 2 cm y revisitados 5 m/20 grados;
- apoyo 2/4/6, pesos efectivos, consenso temporal 3/60 y umbrales separados;
- builds correctos; CTest `orbslam3_multi` 9/9, Servidor 12/12 y Simulacion 10/10;
- 219 completa 17/17 y 22/22; 22/30 solves hacen commit y cinco fusionan;
- ocho descartes explicables, cero hard/revisit/fatal y cero anchors por loop;
- visualmente una esquina queda corregida por completo y la derecha multi-epoch
  solo parcialmente; `pending` llega a 51 y queda un solve activo al apagar.
- mejora posterior: cascada automatica al aparecer autoridad world y
  recuperacion reciente 1/1 provisional dentro de `0.50 m/0.15 rad/2 m`, con
  fallback 2/4/6 fuera de esa banda;
- 220 completa 17/17 y 22/22, valida la cascada y no activa incorrectamente 1/1;
  el usuario califica el resultado general como excelente;
- defecto residual: `task=1000000005590` encontro constraints consecutivas con
  error world casi `0/1.012/0 m`, selecciono las tres regiones y movio 277 KFs
  de una ventana de 296. Esto no identifica por si solo al candidate central
  como pose incorrecta. El validator admitio 0.289 m de degradacion estructural
  por quedar dentro de sus limites amplios;
- punto de entrada: conservar las tres como `CurrentLoop`, pero no aceptar
  `MaxIterations` como convergencia ni permitir que loops ya satisfechos
  empeoren por la mejora OR; añadir guarda estructural local, sin tocar cascada.

## Fase 5A

- arquitectura reconciliada con `O_T_B` continuo y `W_T_B` corregible;
- reference KF real + `Tcr`; consulta inicial asíncrona y revisiones por push;
- absolutos sin global rechazados y goals activos congelados en `O`;
- `GT_FALLBACK` temporal visible hasta recovery real en Fase 6;
- smoothing no obligatorio y antigua 5I absorbida en 5H;
- sin cambios funcionales, builds, tests o simulaciones.

## Entrega de Fase 2

- grupos fisicos `dron`, `servidor` y `simulacion`;
- builds y prefijos separados por grupo;
- interfaces duplicadas de forma controlada;
- configuracion por dominio y despliegue segun ADR 0009;
- ORBvoc completo instalado desde un bootstrap fuera de `src`;
- observabilidad lazy-gated segun ADR 0010;
- `system_architecture` estatico/live separado de `pipeline_flow`;
- guardas automaticas de arquitectura y documentacion.

## Validacion

- build: 9/9 paquetes, un paquete por invocacion;
- CTest: `lib_tray` 4/4, `orbslam3_multi` 9/9,
  `orbslam3_server` 10/10 y `simulacion_dron` 9/9;
- prueba 199: debug-off, 5/5 pasos y 4/4 goals;
- prueba 200: debug completo, 14/14 pasos y 20/20 goals;
- RViz2 y ambos web activos; guarda de recursos no disparada;
- `system_architecture_bridge` cierra sin el `ValueError` de prueba 198.
- layout final: CTest 9/9, guarda 15/15 y dos viewports inspeccionados.

## Limitaciones

`dron_individual` conserva deuda legacy global de linters, aunque todos los
archivos tocados pasan comprobaciones focales y rebuild. La prueba 200 presenta
un traceback de cleanup de `gui_tray_multi` y el exit 255 conocido de Gazebo,
ambos posteriores a `SIM-DONE`.

Las pruebas 205-207 preservan fallos ya corregidos. La 208 valida el aislamiento
y fue aceptada por el usuario como cierre de 4D.

## Referencias

```text
codex/pipeline/fase_2_separacion_paquetes/RESULTADO_FINAL_FASE_2.md
codex/pipeline/fase_2_separacion_paquetes/historial/INDEX.md
codex/pipeline/fase_4_fiducial_real/historial/INDEX.md
codex/contexto/decisiones/ADR_0009_configuracion_por_dominio_y_despliegue.md
codex/contexto/decisiones/ADR_0010_observabilidad_web_debug_coste_cero.md
```
