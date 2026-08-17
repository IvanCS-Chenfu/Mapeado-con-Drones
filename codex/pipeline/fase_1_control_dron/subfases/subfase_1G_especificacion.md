# Subfase 1G — Especificación de acción, controlador y mixer

Este archivo complementa `subfase_1G.md`.

## Estado de la subfase

```text
realizado
```

## Contrato `TrayAction`

```text
uint8 tipo_trayectoria     # 0 pol3, 1 veltrap, 2 elipse
geometry_msgs/PoseStamped target_pose
float32 tx
float32 ty
float32 tz
float32 tyaw
bool absoluto_x
bool absoluto_y
bool absoluto_z
bool absoluto_yaw
---
bool success
float32 t_total
---
float32 t_act
Float64MultiArray x
Float64MultiArray y
Float64MultiArray z
Float64MultiArray yaw
```

Cada array de feedback contiene:

```text
[pos, vel, acc, jerk, ratio]
```

### Semántica para pol3

```text
tx, ty, tz, tyaw = tiempos finales por eje [s]
target_pose       = objetivo de posición/yaw
```

### Semántica para veltrap

```text
tx, ty, tz, tyaw no fijan tiempos finales;
las duraciones se derivan de v_max y t_a del YAML.
```

Los campos siguen existiendo por compatibilidad de la interfaz, pero no deben inducir una semántica falsa.

### Semántica para elipse

```text
tx   = rx [m]
ty   = ry [m]
tz   = alpha [rad]
tyaw = tiempo de vuelta [s]
target_pose.position = centro de la elipse
orientation yaw      = yaw fijo u offset para mirar al centro
```

La GUI y documentación deben cambiar etiquetas según `tipo_trayectoria` o explicar claramente esta sobrecarga.

## `gen_tray`

Nodo/action server:

```text
node: gen_tray
action: AccionTrayectoria
inputs: sensor/GT/pose, sensor/GT/vel
output: feedback interno de la action
```

Comportamiento de referencia:

1. acepta tipos `0..2`;
2. al aceptar un goal, aborta el goal activo anterior;
3. espera una nueva muestra de pose y velocidad;
4. captura `x0,y0,z0,yaw0` y velocidades iniciales;
5. crea una clase de `lib_tray`;
6. publica feedback a 30 Hz;
7. normaliza yaw;
8. devuelve `success=true` al alcanzar `t_total`;
9. acepta cancelación.

Requisitos adicionales de seguridad/documentación:

- validar tiempos, radios, valores finitos y quaternion;
- definir timeout de espera de estado o dejarlo como decisión explícita antes de modificar;
- comprobar frescura de GT mediante timestamps;
- evitar que un goal reemplazado pueda marcarse después como success;
- asegurar acceso thread-safe a goal y muestras;
- documentar que `target_pose.header.frame_id` no se usa actualmente para transformar frames; las referencias se interpretan según flags y frame `world`/orientación inicial.

## Objetivos absolutos y relativos

- `x/y` relativos se rotan usando `yaw0` del dron.
- `z` relativo se suma a `z0` en veltrap/elipse; pol3 delega la semántica a `finales_absolutas`.
- yaw relativo se suma a `yaw0` y se normaliza.
- elipse relativa rota su centro y su `alpha` con el yaw inicial.

Estas reglas deben probarse y no inferirse solo de la GUI.

## `control_calcular_fuerzas`

Interfaces:

```text
sub: sensor/GT/pose                       PoseStamped, world
sub: sensor/GT/vel                        TwistStamped, world
sub: AccionTrayectoria/_action/feedback   TrayAction_FeedbackMessage
pub: control/tray/fuerza                  Float64
pub: control/tray/torque                  Vector3Stamped, frame cuerpo
timer: 20 ms
```

YAML:

```yaml
fisico.cuerpo.masa: <masa total>                         # kg
fisico.cuerpo.matriz_inercia: [ixx, iyy, izz, ixy, ixz, iyz]
fisico.constante.gravedad: -9.81                         # m/s²
control.fuerza.kp: 2.0
control.fuerza.kv: 7.0
control.torque.kr: 0.1
control.torque.kw: 0.1
```

La clave `fisico.cuerpo.masa` se usa en el control como **masa total del dron**, aunque en `hardware.yaml` la misma nomenclatura representa la masa del cuerpo central. Esta ambigüedad debe documentarse o corregirse con nombres distintos antes de hardware real.

Cálculo de fuerza:

```text
ep = x - x_des
ev = x_dot - x_dot_des
F_des_world = -Kp*ep - Kv*ev + m*(x_ddot_des - g)
F_des_body = R_act^T * F_des_world
thrust = F_des_body.z
```

Cálculo de actitud/torque:

- eje z deseado alineado con la fuerza;
- yaw deseado construye eje auxiliar;
- se genera `R_des` y sus derivadas;
- errores `er` y `ew`;
- `tau_des = -Kr*er - Kw*ew + ...`.

Riesgos numéricos obligatorios:

- `norm(F_des)` cerca de cero;
- `eje_z × c_yaw` cerca de cero;
- quaternion no normalizado;
- feedback con arrays de menos de cinco elementos;
- estado GT ausente u obsoleto;
- gains/masa/inercia no finitos.

## `aplicar_fuerzas_dron`

Interfaces:

```text
sub: control/tray/fuerza
sub: control/tray/torque
pub: motor/arr_iz
pub: motor/ab_iz
pub: motor/ab_der
pub: motor/arr_der
timer: 20 ms
```

YAML:

```yaml
actuadores.conversor.fuerza2torque: 0.02
fisico.brazos.longitud: 0.25
fisico.brazos.grados: 45.0
```

Debe coincidir con `hardware.yaml`:

```text
tray_dron.fisico.brazos.longitud == hardware.fisico.brazos.dim[0]
tray_dron.fisico.brazos.grados   == hardware.fisico.brazos.grados
tray_dron.actuadores...          == hardware.actuadores...
```

Mixer:

```text
U = [fuerza_total, tau_x, tau_y, tau_z]
f_motores = A^-1 * U
```

Si `A` es casi singular se usa pseudoinversa. El contrato obligatorio termina en cuatro motores.

## YAML completo de control

Ruta:

```text
src/dron_individual/config/tray_dron.yaml
```

Además de parámetros físicos/control:

```yaml
control.tray.usar_veltrap: "true"  # baseline histórico; el action admite tipo por goal
crear.tray.v_max_lin: 0.8          # m/s
crear.tray.v_max_ang: 0.5          # rad/s
crear.tray.t_a: 5.0                # s
```

Reglas:

1. Los valores actuales son arbitrarios.
2. Masa total debe derivarse del modelo: cuerpo + N*brazos + N*motores + cámaras.
3. Inercia total no debe copiarse de la inercia del cuerpo sin justificación.
4. Gains deben ser finitos y no negativos según el diseño.
5. `v_max` y `t_a` positivos.
6. `fuerza2torque`, longitud y ángulo deben coincidir con el Xacro/plugin.
7. Los strings booleanos deben normalizarse o validarse.

## Soporte de motores

Obligatorio:

```text
4 motores: modelo + plugin + mixer + control + pruebas
```

Opcional:

```text
6/8 motores: modelo y plugin manual
```

Fuera de alcance:

```text
mixer 6/8
control estable 6/8
GUI que prometa vuelo 6/8
```

## Cambios requeridos

1. Compilar/generar `TrayAction`.
2. Enlazar `gen_tray` con las tres clases de `lib_tray`.
3. Mantener feedback con cinco elementos por eje.
4. Validar goals antes de crear la trayectoria.
5. Preservar cancelación y reemplazo de goals.
6. Evitar espera infinita de GT sin política documentada.
7. Proteger singularidades numéricas del controlador.
8. Validar coherencia entre YAML físico y de control.
9. Publicar únicamente cuatro consignas para el cierre obligatorio.
10. Definir política ante fuerzas negativas/no realizables y saturación. Si no se implementa saturación, limitar pruebas y registrar la limitación.
11. Mantener `use_sim_time=true` en simulación.
12. Emitir markers/logs suficientes para goal, tipo, inicio, cancelación, resultado y errores.
