# Auditoria previa Fase 1J - pitch comun del rig estereo

Fecha: 2026-09-01.

## Conclusion ejecutiva

El modelo actual fija las dos camaras directamente a `cuerpo`. El baseline y
la geometria estereo son rigidos, pero no existe un link de rig, un joint de
pitch, publicacion de `joint_states` ni una cadena TF dinamica consumible.

El bloqueo funcional principal esta en `StereoSlamNode`: convierte las poses
de camara ORB a poses de body mediante una unica `body_t_camera_` cargada desde
YAML al arrancar. Si se inclinase solo el modelo Gazebo sin sustituir este
contrato, `NavigationState.o_t_body`, `w_t_body`, sus velocidades y el control
interpretarian parte del movimiento de la camara como movimiento del dron.

F3 y F4 trabajan correctamente en pose de camara/KF y no deben convertirse a
body. Necesitan conservar la pose de camara real del instante del KF. La nueva
extrinseca dinamica pertenece a la frontera camara -> body de F5 y a cualquier
nube depth que se quiera expresar fuera del rig.

## Inventario actual

### Modelo y sensores

- `simulacion/simulacion_dron/urdf/dron_plugins.xacro`: las dos camaras usan
  joints `fixed` con padre `cuerpo`; no existe `stereo_rig`.
- Los plugins Gazebo publican `image_raw` y `camera_info` con frame
  `camara_izq`/`camara_der`.
- `generador_URDF` entrega el XML a `/spawn_entity`, pero no publica
  `robot_description` para un `robot_state_publisher`.
- No se localizo productor runtime de `/joint_states`,
  `robot_state_publisher` ni broadcaster TF del arbol del dron.

### Calibracion estereo

- ORB usa intrinsecos y `bf` de
  `dron/dron_individual/config/orbslam/orbslam_stereo.yaml`.
- El baseline efectivo es 0.057 m. El pitch comun no debe cambiar `bf`, los
  intrinsecos ni la transformada izquierda-derecha.
- El wrapper no se suscribe a `camera_info`: carga el modelo efectivo desde
  YAML y desde el recibo de ORB. `camera_info` sigue siendo relevante para
  consumidores ROS externos y para comprobar frames.

### ORB, F3 y F4

- `TrackStereo()` devuelve `Tcw`; el wrapper publica `Twc` como pose legacy de
  camara y exporta cada `OrbKeyFrame.pose` tambien como `Twc`.
- El backend sparse, las nubes y `/global_keyframes` usan poses de camara/KF.
  Esta semantica debe conservarse: inclinar el rig mueve realmente la camara y
  por tanto tambien el frustum y la pose del KF.
- F4 detecta el tag en la imagen izquierda exacta del KF y publica
  `camera_t_tag` con el stamp del KF. El servidor compone
  `world_T_camera` directamente y no usa `body_T_camera`.
- F4 no necesita convertir a body. Si imagen, KF y deteccion siguen siendo del
  mismo instante, el pitch queda contenido en la pose ORB de la camara. Debe
  evitarse una segunda correccion por pitch.

### F5 y control

- `StereoSlamNode` carga una `body_t_camera_` fija desde `calibration.yaml`.
- La usa en dos fronteras criticas:
  - `o_t_camera * inverse(body_t_camera_) -> raw_o_t_body`;
  - `w_t_camera * inverse(body_t_camera_) -> raw_w_t_body`.
- `raw_o_t_body` alimenta pose, velocidades, predictor dinamico, gravedad en O,
  `NavigationState` y finalmente control y trayectoria.
- Con pitch dinamico, ambas composiciones deben usar la extrinseca valida para
  el stamp de la imagen. Usar el ultimo estado recibido sin sincronizacion
  puede inyectar velocidad angular y lineal falsa.
- `/global_drone_poses` y la GUI propia consumen `NavigationState.w_t_body`;
  quedaran correctos cuando la frontera anterior lo sea y no deben aplicar
  pitch por su cuenta.
- En modo productivo `dynamic`, F5 se suscribe a `control/tray/torque` y
  propaga entre medidas con una inercia body fija mediante
  `J*omega_dot=tau-omega x (J*omega)`. Ese topic contiene el torque solicitado
  a los motores, no el wrench neto medido en Gazebo. Un servo de pitch con
  masa/inercia no despreciables aplica al body un torque de reaccion que hoy no
  entraria en el predictor. El error seria mayor al acelerar, frenar o alcanzar
  limites, y podria contaminar tambien la propagacion lineal al orientar el
  thrust con una R body incorrecta. La siguiente medida visual corregiria la
  base, pero la prediccion a 50 Hz entre imagenes quedaria sesgada.

### Depth

- No existe un nodo depth productivo integrado en el pipeline actual.
- Los scripts de `dron_individual/src/vision/` son experimentales y algunos
  publican nubes con frames de cuerpo o mapa sin TF dinamica demostrada. No
  deben reutilizarse tal cual para F6.
- La profundidad estereo dentro de ORB permanece en frame de camara y no cambia
  por el pitch. Toda nube futura en body/O/world debe usar la pose del rig
  sincronizada con la pareja estereo.

## Matriz de impacto

| Zona | Impacto | Accion futura necesaria |
|---|---|---|
| Xacro/URDF | Directo | Crear `stereo_rig`, joint revolute comun y joints fijos a ambas camaras. |
| Gazebo camera plugins | Bajo | Mantener sensores en links del rig y verificar stamps/frames. |
| Baseline/intrinsecos ORB | Ninguno esperado | Conservar y validar 0.057 m, `bf` e intrinsecos. |
| TF/joint state | Directo, hoy ausente | Introducir estado autoritativo y TF dinamica namespaced. |
| `StereoSlamNode` F5 | Critico | Resolver `body_T_camera(stamp)` y definir fallo/retencion si falta. |
| Predictor/velocidad F5 | Critico indirecto | Alimentarlo solo con pose body compensada y probar joint en movimiento. |
| F3 sparse/KFs | Semantica a conservar | Seguir almacenando y publicando pose de camara/KF. |
| F4 fiduciales | Regresion | Conservar `camera_T_tag` y pose de camara del mismo KF. |
| Visualizadores sparse | Bajo | Los frustums siguen siendo de camara. |
| Visualizadores de dron/GUI | Indirecto | Consumir `w_t_body` corregida sin compensacion adicional. |
| Depth/F6 voxel | Directo futuro | Transformar con extrinseca sincronizada y pose correcta. |
| Config/launch | Directo | Parametros, comando, estado y nombres por dron. |

## Riesgos de implementacion

1. Doble compensacion en F3/F4 moveria artificialmente KFs, fiduciales y nube.
2. Usar pitch actual para una imagen anterior crea saltos y velocidades falsas.
3. `camara_izq` se usa como frame aunque ORB espera convencion optica; la
   convencion debe quedar explicita.
4. Las camaras tienen masa. Accionar el joint puede reaccionar fisicamente
   sobre el body; hay que separar ese efecto de un error geometrico.
5. Topics, joints y TF deben estar aislados por namespace multi-dron.
6. F5 necesita una politica definida antes de la primera muestra o si no puede
   interpolar el pitch al stamp de imagen.

## Hallazgos transversales adicionales

### Frames fisicos, opticos y multi-dron

- Los topics de imagen estan namespaced por dron, pero los plugins Gazebo usan
  actualmente `camara_izq`/`camara_der` como `frameName` sin prefijo por dron.
  Al introducir un arbol TF global esos nombres colisionarian entre drones.
- El link fisico de camara mira por `+X`, mientras ORB/OpenCV usa la convencion
  optica `+Z` hacia delante, `+X` a la derecha y `+Y` hacia abajo. Hasta ahora
  `camara_izq` ha mezclado ambos significados porque no existia TF runtime.
  1J debe separar link fisico y frame optico mediante un joint fijo.
- `orbslam_use.launch.py` declara `dron_X/base_link` como `body_frame`, aunque
  el link principal del URDF se llama `cuerpo`. Hoy es metadato; con TF real
  debe existir una relacion explicita y conservarse el contrato publico o
  cambiarse de forma deliberada.

Cadena recomendada, unica por dron:

```text
dron_X/base_link -> dron_X/stereo_rig
                 -> dron_X/camera_left_link
                 -> dron_X/camera_left_optical_frame
```

La rama derecha debe ser fija respecto al mismo rig y usar nombres equivalentes.

### Geometria neutral y fuentes de verdad

- `calibration.yaml` es la calibracion canonica de `dron_individual`; existen
  replicas exactas en simulacion y servidor protegidas por test.
- El Xacro deriva hoy la posicion de camara de las dimensiones del cuerpo:
  aproximadamente `(0.0975, +/-0.0285, 0.03) m`. La calibracion redondea la
  izquierda a `(0.10, 0.03, 0.03) m`; el baseline 0.057 m si coincide.
- Si la nueva TF usa literalmente la geometria Xacro, el pitch neutral ya no
  reproducira exactamente la extrinseca estatica usada por F5. Se recomienda
  que la calibracion canonica defina el origen neutral y el baseline del rig,
  y que el generador los pase al Xacro, evitando aceptar dos geometrías casi
  iguales como fuentes simultaneas.
- La calibracion estatica puede conservar la transformada neutral y los joints
  fijos rig-optical. El angulo medido aporta solo la parte dinamica.

### Masa, colisiones y limites

- `physical_dron.yaml` configura actualmente cada camara con masa `0.00`, pese
  a que el default Xacro es positivo. Un link hijo de joint revolute con masa
  nula no es un cuerpo dinamico valido y Gazebo puede reducirlo o ignorarlo.
- Una masa/inercia positiva pequena tambien cambia levemente masa total, centro
  de masas y torque de reaccion respecto al modelo fijo usado por control/F5.
  Debe parametrizarse y medirse, no suponerse exactamente cero.
- Las camaras tienen collision geometry. Al rotarlas pueden colisionar con el
  propio `cuerpo` y generar impulsos espurios. La geometria es esquematica; se
  recomienda desactivar la autocolision del rig contra el dron en 1J.
- Alcanzar un hard stop a velocidad no nula tambien transmite un impulso. El
  perfil de comando debe frenar antes del limite y saturar el target dentro de
  los limites mecanicos.

### Comando, estado y automatizacion

- No existe `ros2_control`, `controller_manager` ni interfaz actual de joint.
  Tampoco se localizo un plugin Gazebo ROS instalado que satisfaga directamente
  todo el contrato de posicion, velocidad maxima, aceleracion maxima y estado.
- La opcion recomendada es exponer una interfaz ROS neutra para simulacion y
  futuro hardware: comando `trajectory_msgs/JointTrajectory` de un joint y
  estado `sensor_msgs/JointState`, ambos namespaced. Gazebo implementaria el
  actuador, pero F6/F7 no dependerian de APIs del simulador.
- `scenario_runner_node` solo entiende espera, servicios booleanos y movimiento
  del dron. Para probar 1J de forma reproducible necesita una accion de pitch
  con espera por tolerancia de estado; la GUI Tk actual no es necesaria para
  esa validacion y se recomienda dejar su ampliacion a F7.

### Compatibilidad y degradacion controlada

- `dron_individual` puede arrancar fuera de esta simulacion, donde no habra TF
  de pitch. Se recomienda un modo explicito `static|tf`: `static` conserva el
  comportamiento actual y `tf` exige la transformada dinamica.
- En modo `tf`, si falta la extrinseca al stamp de imagen, no se debe usar sin
  aviso la ultima disponible. ORB/F3/F4 pueden conservar la muestra de camara,
  pero la salida body de F5 debe marcarse no valida y usar la politica de
  fallback ya acordada, hasta recuperar sincronizacion.
- El Xacro admite una configuracion mono. Se recomienda que 1J modifique solo
  el rig estereo y mantenga el camino mono fijo como regresion.

### Efectos que no justifican cambios preventivos

- Mover el rig con el body quieto crea movimiento real de camara: pueden crecer
  la frecuencia de KFs, candidatos BoW y carga del servidor. Debe medirse, pero
  no corregirse como si fuese movimiento falso.
- A pitch alto puede perderse textura, verse cuerpo/rotor/suelo o desaparecer
  el fiducial. Los limites mecanicos pueden ser +/-70 grados aunque F6 use un
  rango operativo menor. La velocidad por defecto debe respetar tambien el
  solape visual a 20 Hz.
- No se recomienda ampliar de antemano el predictor F5 con torque del servo.
  Primero se medira reaccion, innovacion de pose y error body durante barridos;
  si el efecto es material, ese resultado suspendera el acuerdo antes de tocar
  el modelo dinamico de F5.

## Decisiones pendientes antes de implementar

- Confirmado: joint `cuerpo -> stereo_rig`, eje `+Y`, positivo hacia abajo,
  neutral `0 deg` y limites configurables con defaults `-70/+70 deg`.
- Confirmado: Codex elegira defaults conservadores y parametrizados para masa,
  inercia, velocidad y aceleracion, y los ajustara por evidencia de tracking y
  reaccion fisica. No se usara masa nula ni actuacion cinematica.
- Confirmado: `JointTrajectory`/`JointState` sera la interfaz estable. 1J solo
  implementara consigna y seguimiento cerrado del joint; la decision autonoma
  de pitch pertenece a Fase 6 y se discutira alli.
- Confirmado: frames unicos por dron, separacion link/optical, conservacion de
  `dron_X/base_link` como contrato publico y calibracion canonica como fuente
  exacta de la geometria neutral.
- Confirmado: el pitch sera activable/desactivable en el dron y desde el launch
  de simulacion. Desactivado conserva extrinseca estatica; activado exige estado
  dinamico sincronizado y fallo cerrado de la salida body si falta.
- Confirmado: automatizacion en scenario YAML, sin ampliar la GUI Tk en 1J,
  autocolision del rig desactivada y camino mono fijo.
- Confirmado: medir primero torque/reaccion y detenerse antes de ampliar F5 si
  el efecto es material.
- Conseguido como entrega previa: selector explicito
  `phase5_navigation_source=gt|orb`, con GT forzado distinguible del fallback y
  prueba ORB estricta sin fallback.

## Selector GT/ORB solicitado para 1J

La auditoria inicial confirmo que `gt_fallback_enabled` no era un selector: ORB
podia recuperar autoridad al quedar cualificado. La entrega previa ya incorpora
en `multi_dron.launch.py` y `navigation_state_mux` el selector separado
`phase5_navigation_source=gt|orb`.

El contrato implementado y validado es:

- `gt`: pose y velocidad GT gobiernan exclusivamente trayectoria y control;
  ORB continua ejecutandose en sombra para comprobar tracking, mapas,
  extrinseca dinamica y fiduciales, pero no realimenta el dron;
- `orb`: ORB es la fuente objetivo de navegacion; `gt_fallback_enabled` sigue
  siendo una politica independiente que decide si una perdida permite fallback
  o produce estado invalido.

El modo `gt` no puede depender de recibir un `NavigationState` ORB para publicar
estado: debe seguir operativo aunque ORB pierda tracking. Tampoco puede entrar
en deteccion de fiduciales, anchors, mapa, optimizacion o pose final. Cambiar de
GT a ORB para la segunda bateria significa cambiar el parametro, no retirarlo.

Orden recomendado de validacion:

1. pitch desactivado y autoridad GT: regresion neutral;
2. pitch activado y autoridad GT: joint, PID, TF, tracking y efectos fisicos;
3. mismas maniobras con autoridad ORB: validar F5 con extrinseca dinamica;
4. barridos de aceleracion/frenado: decidir por evidencia si el torque del rig
   exige ampliar el predictor F5.

Las pruebas 359R y 360 validaron respectivamente GT forzado y ORB estricto. El
intento 359 se conserva como invalido de infraestructura por una replica de
mensajes desactualizada, ya reparada. No se deben inventar valores numericos
para las decisiones restantes del joint.

## Validacion requerida

- Regresion neutral contra F4/F5 actuales.
- Pitch positivo y negativo estaticos: tracking, frustum, fiducial y body.
- Pitch en movimiento: sincronizacion y ausencia de velocidad body ficticia.
- Baseline y transformada izquierda-derecha invariantes en todo el rango.
- `camera_info`, frame IDs, TF y namespaces correctos con dos drones.
- Depth en camara y transformacion a body/O/world coherentes.
- F4: el objeto permanece fijo en world al variar solo el pitch.
- F5: `o_t_body`/`w_t_body` permanecen estables, salvo movimiento fisico real.
- Regresion de una ruta ORB favorable de Fase 5 con pitch neutral.
- Comparacion externa, sin alimentar el estimador con GT, del error angular y
  lineal durante aceleracion/frenado del joint frente al baseline neutral;
  registrar tambien comando/estado/esfuerzo del joint y torque de motores.

## Alcance de esta auditoria

Auditoria estatica de codigo, configuracion y contratos. No se modifico el
runtime, no se genero URDF, no se inspecciono un arbol TF live y no se ejecuto
Gazebo. Esas comprobaciones pertenecen a la implementacion y pruebas de 1J.
