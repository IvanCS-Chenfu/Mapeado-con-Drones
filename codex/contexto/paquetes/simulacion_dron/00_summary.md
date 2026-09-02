# 00_summary - simulacion_dron

Los overrides GT/ORB y el servicio shadow usados en los laboratorios 321-349
fueron retirados en 5J. Los YAML historicos se conservan como evidencia, pero
no son escenarios runtime vigentes.

5F incorpora `pose_metrics_node.py`, activable desde `multi_dron.launch.py`,
para generar CSV/JSON/PNG O/W/GT por dron con emparejamiento temporal y
alineacion O fija por epoch. La prueba 230 valida su ejecucion y deja la calidad
W como no aceptada.

Para el diagnóstico angular 5H guarda además
`drone_N_gt_angular.csv`: orientación GT, omega world/body, stamp físico Gazebo
y stamp ROS de recepción. Este par permite mapear ambos relojes offline sin
usar GT en estimación o control. `analyze_f5h_angular_phase.py` produce timeline,
lags, frecuencia/fase, potencias y comparación `tau_ew` real/ideal.
La prueba 264 valida el puente con 323 ciclos ORB sincronizados y permite
separar latencia raw (`~0.08 s`) de la inyeccion de energia del lazo angular.
El analizador de 265 añade trabajo/energia de `tau_er`, `tau_ew` y torque total,
edad local, horizonte/clamp y separación visual/base/predicha.
En 265 la captura confirma horizonte medio `43.2 ms`, pero localiza el desfase
dominante en `visual_q -> base_q`; el hover falla antes que 264 y no se avanza
a etapa 3.
Para 266 el analizador separa error visual-base before/after, cuenta tipos de
reanclaje y añade potencia media, energia por segundo y ventana comun de
comparacion con 265.
La captura 266 deduplica 157 medidas ORB, confirma 118 SMALL_ANCHOR y muestra
la transicion tardia moderate/predict-only/rejected antes del fallback.

La integracion 5H usa `global_drone_pose_visualizer.py`. Consume los
`NavigationState` de cada dron y publica `/global_drone_poses` como
`MarkerArray`: ejes XYZ desde `o_t_body`, exactamente la pose de control, y
etiqueta `[ORB]/[GT]`. El
launch expone `phase5_global_pose_rviz_enabled=false` y el modo de prueba
`use_legacy_gt_goal_policy_for_simulation=false`; este último gobierna solo el
control legacy GT y no alimenta las poses estimadas.

Preparacion 1J vigente: `multi_dron.launch.py` expone
`phase5_navigation_source=gt|orb` y lo propaga al
`navigation_state_mux` de cada dron, separado de la politica de
fallback. En modo GT ORB permanece activo en sombra y los fiduciales siguen
siendo exclusivamente visuales.

Los goals YAML pueden sobrescribir esa fuente con
`navigation_source: None|GT|ORB`. `None` hereda el launch y `ORB` conserva la
politica de fallback configurada. La prueba 369 valida `GT` seguido de `None`:
el segundo goal uso GT_FALLBACK al no existir anchor.

1J incorpora el plugin fisico de pitch y pasos `pitch` en
`scenario_runner_node`. El filtro configurable de 364 (`tau=0.05 s`) resolvio
la oscilacion de velocidad y 365/366 validaron consignas, limites y movimiento.
`camera_pitch_enabled=false` crea ahora topologia fixed sin cargar el servo;
`true` crea el revolute. Cada camara y el rig usan `1e-5 kg`, evitando el
momento descentrado de `~0.0392 Nm` que causaba la deriva de 370. Las pruebas
370R2/371 validan llegada real con ambas topologias y 372 completa el barrido.

Desde 1K, `debug_fase_1=false` silencia `DEBUG/INFO` de los nodos de vuelo y
plugins de motores, GT y pitch. Conserva warnings de seguridad, errores y
resultados. Las pruebas 374/375 validan la conmutacion sin cambiar vuelo ni pitch.

El runner ofrece `wait_for_navigation_pose` para exigir tolerancias XYZ/yaw
sostenidas sobre `NavigationState`. En 372 el gate GT dio `0.0157 m`; ORB tomo
autoridad real antes del barrido y despues perdio tracking, entrando en el
fallback configurado. 1J queda conseguida; la persistencia ORB corresponde al
trabajo posterior de Fase 5/6.

Los markers no dependen de `global_valid`. Solo se actualizan con una muestra
local, continua y con velocidad valida; ante una muestra transitoria no
consumible conservan la ultima pose que tambien conserva el controlador.

`debug_fase_5=false` es la puerta maestra. Con ella activa,
`debug_orb_control_state` se propaga a cada wrapper y controlador. En las
pruebas progresivas de 5H permite correlacionar medida, pose/omega publicadas y
respuesta de control; apagado no emite esa telemetria diagnostica.

Las etapas 1-6 de 5H viven como YAML auxiliares bajo
`codex/archivos_auxiliares/trayectorias/`; no alteran los escenarios instalados.
La etapa 1 se consigue en 258 y la etapa 2 falla en 259, por lo que 3-8 no se
ejecutan.

Paquete de launch, escenarios y observabilidad Gazebo. Integra servidor y, de
forma configurable, RViz2, `pipeline_flow` y `system_architecture`; tambien
ofrece replay sin Gazebo ni GT live.

Desde Fase 7 bloque 1, `multi_dron.launch.py` inicia `multidron_gui` por defecto
con `launch_multidron_gui=true`, mientras `launch_rviz=false` deja RViz2 fuera
del flujo normal. Ambos son overrides independientes y el backend conserva el
modo headless. El launch pasa numero/namespaces de drones, YAML fiducial y
`drone_stale_timeout_sec=1.0`, y sanea el entorno Snap del proceso Qt.

El grafo `system_architecture` usa una topología, metadata y layout declarativos
separados. Su composición sitúa Simulación/Servidor arriba y Dron abajo para
facilitar la lectura de interfaces entre despliegues.

Desde 3T contiene en `config/global_map/` el perfil de parámetros controlables
por el despliegue simulado. Es una copia exacta del perfil del servidor durante
la etapa de simulación y un test contractual impide divergencias o parámetros
sin propietario.

Desde 4A+4B contiene el despliegue visual fiducial de Gazebo:

```text
config/fiducial_objects.yaml -> replica exacta del contrato canonico
config/fiducial_rendering.yaml -> parametros exclusivos de render/spawn
src/fiducials/fiducial_spawner.py -> assets AprilTag, SDF y readiness
```

El spawner redetecta las 15 texturas `DICT_APRILTAG_36h11`, crea tres objetos
estaticos/colisionables y publica `/fiducial_spawn_ready` con QoS reliable y
transient-local. `scenario_runner_node` puede bloquear un escenario mediante
`wait_for_bool` hasta recibir ese estado.

Desde 4D, `multi_dron.launch.py` pasa el perfil fiducial al servicio del
Servidor y propaga a cada wrapper el debug visual y su duracion. El grafo
`system_architecture` incluye la arista runtime servicio Server->wrapper.
Desde 4E+4F ambos grafos incluyen tambien el topic
wrapper→`orbslam3_server`; `runtime.yaml` replica la capacidad pending 10.
Desde 4G+4H, `fiducials.yaml` expone rango, consistencia, gap y FIFO visuales;
`system_architecture` ya no contiene la ruta GT fiducial. Conserva
`sim_to_dron_gt` porque pertenece al control y a la futura Fase 5.

Fase 2 separa configuracion propia de modelo/sensores en `physical_dron.yaml`
y `simulated_sensors.yaml`. `actuators_dron.yaml` es una replica parcial
declarada de Dron. Simulacion no abre YAML operacionales de otro grupo.
`calibration_dron.yaml` replica el `B_T_C` optico frontal de Dron con
`RPY=(-90,0,-90)` para las guardas de despliegue.

## Launches

```text
launch/multi_dron.launch.py -> Gazebo + N drones + servidor + debug opcional
launch/f3c_replay.launch.py -> replay raw 3C
launch/f3d_replay.launch.py -> replay 3D con anchor sintetico
launch/f3e_replay.launch.py -> replay raw + observaciones fiduciales
launch/f3f_replay.launch.py -> replay 3F + RViz2 + web + apertura de pestaña
launch/pipeline_flow_only.launch.py -> diagnostico web aislado
```

`multi_dron.launch.py` activa `spawn_fiducials=true` por defecto. Los objetos
se colocan a ±8.5 m; los escenarios de Fase 4 conservan la ruta a ±10 m.

`multi_dron.launch.py` dispone de perfiles sin duplicar launches:

- `launch_gazebo_gui=false`: usa `gzserver` sin `gzclient`;
- `launch_mission_gui=false`: omite la GUI de mision;
- `fase3_debug.yaml`: RViz2, grafo, navegador y logs `[F3*]` independientes;
- `drone_start_stagger_sec=8.0`: arranque 0/8/16... s por defecto;
- `orb_vocabulary_path`: `ORBvoc.txt` completo por defecto; L5 solo por override.

Fase 5B añade un override de spawn desactivado por defecto y el escenario
`f5b_fiducial_giro_180.yaml`. `scenario_runner_node` admite rechazos esperados
para validar el gate sin detener la secuencia. La prueba 225 confirma anchors,
continuidad/ref-KF y pérdida real de ambos drones tras el giro.

El perfil visual completo se usa con dos drones. Para tres o mas drones y para
fases dense se usa headless y se habilitan solo las vistas necesarias.

## Observabilidad 3P

- RViz2 muestra `/global_sparse_cloud` con `RGB8` y
  `/global_keyframes` como frustums.
- El grafo web tiene 23 nodos y 41 aristas. Ademas del flujo fiducial incluye
  `CovisibilityDatabase`, `LoopDetector`, `LoopBoWIndex`,
  `SubcloudLoopVerifier`, `LoopDecision`, `LoopAnchorConstraintStore` y
  `FusedLandmarkManager` con salidas a covisibilidad, score y builder.
- La arista `SecondaryWorker --retry / LOW--> SecondaryTaskQueue` representa
  solo el nuevo intento real posterior a stale/rollback.
- El flujo secundario conserva iluminacion progresiva por `task_id` desde
  lifecycle `start` hasta `done`; las etapas ya no son pulsos independientes.
- En desktop, principal, poses/anchors y loop/fusion ocupan tres bandas con
  columnas ampliamente separadas; las rutas curvas evitan solapes en retornos
  y diagonales largas. El layout movil vertical permanece independiente.
- `pipeline_flow_browser.py` espera `/health=ready` y abre una sola pestaña
  desde el propio launch; no necesita un comando manual de Codex.
- Los launches limpian variables Snap/VS Code para RViz2 y evitan cargar
  bibliotecas GTK incompatibles.

## Validacion

- contrato web 9/9;
- live 98: intento funcional con bridge 11/18, conservado como no conseguido
  por bloqueos y swap agotada antes de las optimizaciones;
- replay 99: ejecución aislada sin Gazebo sobre 54 deltas;
- live visual 133: escenario completo, dos anchors y minimo disponible 612.3
  MiB sin PSI de memoria;
- prueba 137: tres drones en movimiento, seis goals, tres anchors, 141 KFs
  activos y minimo 878.8 MiB;
- prueba 138: estado normal de dos drones restaurado con Gazebo GUI, RViz2 y
  web, minimo 946.6 MiB y guarda inactiva.
- live 148: intento fallido preservado; hard constraint por carrera de control
  dejo mission gate activo hasta timeout;
- replay 150: reproduce las 1239 entradas y confirma la correccion sin hard;
- live 151: escenario fid2-fid1-fid2 completo y confirmado visualmente;
- replay 153: backlog 3M-3O drenado por completo;
- live 154: escenario secuencial A fiducial/B loop completo, 2 anchors con solo
  1 hard y guard inactivo. RViz2 y web arrancaron; su lectura visual humana
  queda pendiente del usuario.
- prueba 160: escenario tipico completo, bridge 3P listo, servidor y cola
  secundaria cierran limpios y guard de recursos inactiva; el usuario confirma
  RViz2 y grafo web correctos.
- prueba 161: mismo escenario completo, guard inactivo, contrato web 9/9 y
  cierre de servidor/colas limpio. La revision visual humana de esta ejecucion
  aun no se ha comunicado.
- cierre 3T: CTest 8/8; prueba 195 completa el escenario tipico con exit 0,
  guarda de recursos inactiva y perfiles YAML de simulacion cargados; el
  usuario confirma el resultado visual correcto.
- cierre 3S: prueba 196 `success=true`, cuatro goals correctos y servidor
  operativo; con los cuatro flags false no arrancan RViz2, bridge ni navegador
  y no aparece telemetria `[F3*]`.

La validacion automatica de topologia, lifecycle y configuracion 3T esta
conseguida.

La copia `config/global_map/loop_fusion.yaml` incorpora la correccion reabierta
3Q: sin deadband de 2 cm, revisitados 5 m/20 grados, consenso 3/60 y umbrales
separados de convergencia/fusion/commit. Tambien replica la recuperacion tras
perdida 1/1 estricta (`0.50 m`, `0.15 rad`, recorrido maximo `2 m`) con fallback
2/4/6. El contrato de configuracion pasa dentro del CTest 10/10; la prueba 220
termina 17/17 y 22/22 con resultado visual general excelente y un outlier 3Q
residual documentado en su historial.

Detalle: `launches.md`, `scenario_runner_node.md` y
`pipeline_flow_visualizer.md`.

Las pruebas diagnósticas 269-272 usan cuatro YAML equivalentes y el argumento
`f5h_gt_timing_mode` de `multi_dron.launch.py`. `pose_metrics_node.py` conserva
GT angular dual-clock y el analizador existente compara energía y fase sin
introducir GT en el control normal.
El analizador añade desde 276 RMSE/MAE/error máximo y mismatch direccional de
omega; 276-277 validan de forma reproducible el estimador causal con pose GT
20 Hz. La 278 reutiliza `gt_20_delay` y falla con 80 ms; la bateria se detiene
antes de `gt_orb_timing` y ORB real.

`multi_dron.launch.py` propaga `orb_navigation_prediction_mode` a cada dron.
Los escenarios 318-320 validan respectivamente paridad y dos hovers ORB reales.
318 completa el runner pero incumple la ausencia de huecos; conforme al STOP,
319/320 no se han ejecutado.

El diagnostico shadow post-320R añade `f5h_orb_shadow_mode` al launch y
`call_set_bool` al runner. El primer 320R2 fue invalido por ruta YAML relativa.
320R2R completo aproximacion GT, ORB dinamico en sombra y activacion en
frontera; pese a tracking sano y ausencia de fallback, el error de posicion
crecio hasta `~1.63 m`. Hover productivo no validado; 321 no ejecutada.
