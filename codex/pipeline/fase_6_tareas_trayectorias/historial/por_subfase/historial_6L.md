# Historial 6L

## 2026-09-12/13 - Riesgo visual preventivo y reorientacion

- objetivo intentado: detectar soporte ORB pobre direccional, frenar localmente
  y recuperar observacion mediante una subtarea normal.
- archivos modificados: contratos `VisualTrackingEvidence`/`VisualRiskEvent`,
  wrapper estereo, `task_manager`, `task_server`, launch y visor de debug.
- builds: interfaces Dron/Servidor, `orbslam3`, `task_manager`,
  `dron_individual`, `task_server` y `simulacion_dron` correctos.
- pruebas: 689 fallo de infraestructura por una replica de interfaz sin
  reconstruir; 690 detecto que la guarda de 5 s podia preemptar la maniobra;
  691 valida guarda 6 s y visor por evento; 692/693 completan la cadena.
- evidencia positiva: 693 termina `success=true`, sin guarda de recursos;
  muestra `TRACKING_RISK`, `STOP`, limpieza GUI/reserva y
  `REORIENT_FOR_TRACKING` con terminal correcto.
- limitacion: `VISUAL_RETREAT` por recorrido estable, politica persistente de
  precauciones y pruebas de sectores variados siguen pendientes.
- conclusion: PARCIAL.

## 2026-09-13 - Cierre funcional acordado antes de la prueba final

- se elimina `VISUAL_RETREAT` del alcance activo: no se implementara un
  retorno por historial en esta entrega;
- por trayectoria se fija el primer sector que mantiene deficit; un riesgo
  posterior no se combina para cambiar la maniobra ya iniciada;
- STOP local comunica el evento, el servidor registra la precaucion espacial
  ligera y sirve una reorientacion que evita ese sector, sin buscar el maximo
  de inliers ni modificar D*/voxeles;
- se permite una sola reorientacion por `(trajectory_id, sector)`; al terminal
  normal el dron vuelve a la cola de subtareas;
- pendiente: aplicar el contrato, validar los cuatro sentidos y cerrar estado.

## 2026-09-13 - Cierre con prueba 696

- 694 y 695 fallaron antes de la logica visual porque el escenario solicitaba
  una compuerta que el launch no creaba con
  `phase6_execute_coverage_candidates=false`; se preservan como fallos de
  configuracion, no de 6L;
- 696 activo el ejecutor con la compuerta inicialmente cerrada y la abrio con
  `call_set_bool`: el servicio respondio inmediatamente;
- D1 fijo un primer sector TOP pobre en `dstar_1_1`, inicio STOP local, retiro
  su polilinea, mostro el debug por evento y el servidor registro la
  precaucion espacial ligera;
- `reorient_1_2` recibio yaw/pitch, termino normalmente y el servidor reencolo
  la subtarea; una degradacion posterior de otro corredor siguio la politica
  de STOP causal ya existente;
- conclusion: CONSEGUIDA con el alcance acordado. `VISUAL_RETREAT`, politica
  persistente de precauciones y calibracion de escenarios siguen como mejoras
  futuras, no como deuda de cierre.

## 2026-09-13 - Revision de franja vacia y prueba 697

- cambio de contrato: se retira el umbral de pocos inliers por semiplano.
  `visual_risk_empty_region_fraction=0.75` cuenta LEFT `[0,0.75W]`, RIGHT
  `[0.25W,W]`, TOP `[0,0.75H]` y BOTTOM `[0.25H,H]`; solo cero inliers de
  tracking en la franja dirigida durante tres frames activa el protocolo;
- interfaces y consumidores recompilados: replicas `orbslam3_msgs`, wrapper
  ORB, `task_manager`, `dron_individual` y `simulacion_dron`; CTest de
  `dron_individual` 8/8 correcto;
- 697: `success=true`. D1 llego con GT al fiducial 2, se habilito coverage y
  ejecuto rutas D* durante 120 s. Hubo STOPs causales por
  `occupied_or_inflated_corridor`, pero `F6L-FIRST-EMPTY-REGION`,
  `TRACKING_RISK`, precaucion y `REORIENT_FOR_TRACKING` ocurrieron cero veces;
- interpretacion: no aparecio una franja dirigida de 75 % sin inliers durante
  tres frames. La prueba descarta un falso positivo en ese recorrido, pero no
  ejerce la cadena del criterio nuevo;
- conclusion: PARCIAL hasta ejecutar un caso que active una franja vacia real
  y correlacione STOP local, precaucion y reorientacion.

## 2026-09-13 - Salto configurable de 25 grados y pruebas 698/699

- se sustituyo el salto fijo de la maniobra por
  `visual_risk_reorientation_step_deg=25.0`: LEFT/RIGHT modifican yaw y
  TOP/BOTTOM modifican `camera_pitch`, siempre alejandose del primer sector
  pobre; no se introduce seleccion de pared ni orientacion ordinaria;
- `PlanRoute.dispatch_execution=true` y el paso YAML `plan_route` permiten que
  698/699 atraviesen D*, reserva, action, STOP y reorientacion reales sin abrir
  la cola automatica de cobertura;
- 698 intento 1 fallo por un cliente de action que solo se creaba con la cola
  automatica; se corrigio creando los clientes siempre. El intento 2 ejercio
  `F6L` pero termino al pedir un objetivo ocupado. El intento 3 termino
  correctamente, pero solo produjo STOP de corredor ocupado, no visual;
- 699 termino correctamente: D1 se anclo con GT, ejecuto dos ascensos D* y
  mantuvo `map_epoch=0`, pero tampoco alcanzo una franja TOP/BOTTOM vacia. Los
  STOPs observados fueron de corredor ocupado;
- conclusion: la integración y el umbral 0.75 quedan correctos, pero la
  validacion final de yaw/pitch +/-25 grados con evidencia ORB real sigue
  pendiente de un escenario visualmente adverso que active una franja vacia.

## 2026-09-13 - Secuencia terminal y pruebas 700/701

- se corrigio el runner para no solicitar el paso siguiente por una pausa
  temporal: espera el terminal del `TrajectoryPlan` despachado y, si existe,
  el de la reorientacion visual;
- 700: D1/GT llego al fiducial 2 y completo tres rutas X+ consecutivas. Todos
  los planes ordinarios conservaron yaw cercano a `90 grados`, pitch `0` y
  `map_epoch=0`;
- 701 inicial revelo que un `STOP solicitado` por
  `occupied_or_inflated_corridor` se publicaba antes de que concluyera la
  action física. Se incorporo el terminal `STOP completado` sobre el mismo
  `trajectory_id`; la repeticion 701 espero ese terminal antes de cada ascenso
  siguiente y termino `success=true`;
- no hubo `F6L-FIRST-EMPTY-REGION`, `TRACKING_RISK` ni reorientacion en 700 o
  701. El criterio de franja vacia no queda cerrado por estas pruebas y sigue
  pendiente el escenario ORB adverso previsto.

## 2026-09-13 - Metas relativas acumulativas y pruebas 712--718

- el runner acumulativo toma `W_T_B` tras cada terminal y solicita el siguiente
  destino relativo de 2 m. El primer intento real revelo que, si D* tardaba mas
  de un segundo, el runner reenviaba la misma solicitud y despachaba rutas
  duplicadas; se corrigio para esperar siempre el mismo futuro. La logica de
  mapeo normal no depende de este cliente de prueba;
- 713 confirma una sola solicitud por indice, pero alcanza el volumen duro
  nominal antes del riesgo. Se creo el YAML aislado de mision
  `mission_house_visual_risk_test.yaml`, con ROI ampliado para estos ensayos y
  los mismos vetos de ocupacion, inflacion y reservas;
- 715 activo una franja vacia horizontal real, pero el runner clasifico el
  detalle `TRACKING_RISK` como STOP no visual. Se corrigio para reconocer tanto
  ese marcador canonico como el texto legacy `visual_risk`;
- 717 es positiva: D1/GT, fiducial 2, epoch 0 y metas X+ acumulativas. En
  `dstar_1_17` se detecto `sector=2`, fraccion 0.75, cero inliers; STOP y
  `reorient_1_19` finalizaron normalmente. El yaw persistente paso de 90 a
  115 grados y el runner recibio `REORIENT-DONE`;
- 718 repite Z+ con 12 metas y epoch 0. D* y STOPs de corredor funcionaron,
  pero no aparecio franja TOP/BOTTOM vacia ni reorientacion de pitch. Termino
  con `RISK-NOT-OBSERVED`, por lo que no es evidencia de fallo del protocolo;
  solo falta un escenario que active el sector vertical.
- 719 empleo `mission_house_visual_risk_vertical_long_test.yaml`, que eleva
  solo el volumen de prueba hasta `z=62 m`, y encadeno metas Z+ de 2 m. En el
  paso 13, con epoch 0, detecto `sector=4`, fraccion 0.75 y cero inliers.
  El STOP visual activo `reorient_1_26` con yaw cercano a 90 y pitch `-25`;
  `F6M-WAYPOINT-CONTRACT` confirma que esa orientacion llego al dron. Un cambio
  `occupied_or_inflated_corridor` durante el propio giro lo reemplazo por STOP,
  que termino correctamente, y el runner clasifico la reorientacion como no
  completada.
- conclusion: PARCIAL. La activacion horizontal y el giro yaw quedan
  validados; 719 valida la deteccion vertical y la orden de pitch, pero falta
  que una reorientacion vertical alcance terminal sin preemption de corredor.

## 2026-09-13 - Reorientacion vertical local y pruebas 720/721

- se traslada el yaw/pitch correctivo al dron: `task_manager` ejecuta STOP y,
  al completarlo, una `TrayAction` local a XYZ fijo de 5 s; publica inicio y
  terminal de reorientacion. `task_server` solo limpia la ruta/reserva previa,
  mantiene HOLD, persiste la orientacion reportada al terminal y reencola;
  no crea `reorient_*`, reserva, corredor ni puede cancelar el giro local;
- 720 no es evidencia funcional: el ordenador se suspendio. La accion GT al
  fiducial devolvio `success=true`, pero la compuerta posterior de pose expiro;
  se conserva como intento cronologico no concluyente;
- 721 repite limpia la secuencia vertical: D1/GT llega al fiducial 2, mantiene
  `map_epoch=0` y en `dstar_1_18` activa `sector=4`, fraccion 0.75 y cero
  inliers. STOP termina y dispara `F6L-LOCAL-REORIENT-DISPATCH`; cinco segundos
  despues `F6L-LOCAL-REORIENT-FINAL success=true` informa yaw 90 y pitch -25.
  El runner recibe `REORIENT-DONE` y el servidor solo entonces registra
  `F6M-ORIENTATION-UPDATED local=true`;
- no aparece una reserva ni una ruta `reorient_*` para el giro, por lo que no
  existe preemption de corredor sobre una orientacion sin desplazamiento;
- conclusion: CONSEGUIDA para 6L. Permanecen fuera del cierre `VISUAL_RETREAT`,
  veto futuro por precaucion y calibracion con escenarios variados.

## 2026-09-13 - Comprobacion visual de pitch 722

- 722 termino `success=true`: D1/GT llego al fiducial 2, mantuvo XYZ fijo y
  ordeno `pitch=-25 grados` sin activar D*, reservas ni riesgo visual;
- la revision visual del usuario confirma que el joint fisico de camara se
  movio correctamente. Completa la evidencia que 721 aportaba por terminal de
  accion y estado persistido;
- conclusion agregada sin cambios: 6L CONSEGUIDA. Depth, evidencia
  `OCCUPIED/FREE` por depth, mapa denso y orientacion por normales pertenecen
  al bloque posterior, no se adelantan en este cierre.
