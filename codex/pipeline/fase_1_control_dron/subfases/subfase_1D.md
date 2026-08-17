# Subfase 1D — Plugin de fuerzas y torques por motor

## Estado

```text
realizado
```

## Dependencias

```text
1B — links físicos de motores
1C — entidad insertada y topics namespaced
```

## Objetivo técnico

Crear `plugin_actuar_motores`, integrarlo en el Xacro y comprobar desde terminal que publicar consignas de fuerza en los cuatro topics de motor genera fuerza relativa y torque sobre los links correctos, produciendo movimiento observable del cuadricóptero en Gazebo.

La actuación manual puede contemplar 6 u 8 links, pero el requisito de fase y las pruebas obligatorias son para 4 motores.

## Comportamiento esperado

Para cada motor:

```text
std_msgs/msg/Float64
  -> fuerza local (0, 0, F)
  -> torque local (0, 0, ±F * fuerza2torque)
  -> AddRelativeForce + AddRelativeTorque en cada tick
```

Los signos de torque alternan por índice para representar sentidos opuestos de giro. La última consigna recibida se reaplica hasta que llegue otra, incluida una consigna cero.

## Contexto obligatorio a leer

```text
AGENTS.md
codex/pipeline/fase_1_control_dron/subfases/subfase_1B.md
codex/pipeline/fase_1_control_dron/subfases/subfase_1C.md
codex/contexto/paquetes/simulacion_dron/00_summary.md
```

## Diagnóstico de partida

La implementación de referencia se encuentra en:

```text
src/simulacion_dron/src/plugins/plugin_actuar_motores.cpp
src/simulacion_dron/urdf/dron_plugins.xacro
src/dron_individual/config/hardware.yaml
```

El plugin:

- lee siempre `motor1`–`motor4` y `topic1`–`topic4`;
- detecta opcionalmente `motor5`/`motor6` y después `motor7`/`motor8`;
- busca links con `model_->GetLink`;
- crea una suscripción por motor;
- conserva hasta ocho fuerzas/torques;
- no aplica saturación ni caducidad de consigna;
- usa `fuerza2torque` del SDF/Xacro.

## YAML obligatorio

Parámetro:

```yaml
actuadores.conversor.fuerza2torque: 0.02
```

Semántica:

```text
torque_z [N·m] = fuerza_z [N] * conversor [m]
```

El valor actual es arbitrario. Debe ser finito y no negativo. Cuando exista hardware real, se sustituirá por un modelo identificado de motor/hélice o una relación física documentada.

El YAML también debe mantener:

```text
fisico.brazos.numero: 4
fisico.brazos.dim: [longitud, radio]
fisico.brazos.grados
```

porque los nombres y la geometría del modelo determinan dónde actúan las fuerzas.

## Topics obligatorios para cuatro motores

Dentro del namespace de cada dron:

```text
motor/arr_iz
motor/ab_iz
motor/ab_der
motor/arr_der
```

Tipo:

```text
std_msgs/msg/Float64
```

Unidad: Newtons de fuerza local en el eje `+z` del link de motor.

## Archivos permitidos a modificar

```text
src/simulacion_dron/src/plugins/plugin_actuar_motores.cpp
src/simulacion_dron/urdf/dron_plugins.xacro
src/dron_individual/config/hardware.yaml
src/simulacion_dron/CMakeLists.txt
src/simulacion_dron/package.xml
codex/contexto/paquetes/simulacion_dron/
```

## Archivos prohibidos

```text
src/dron_individual/src/control_tray/aplicar_fuerzas_dron.cpp
src/dron_individual/src/control_tray/control_calcular_fuerzas.cpp
src/dron_individual/src/control_tray/gen_tray.cpp
src/lib_tray/
src/orbslam3_multi/
src/orbslam3_server/
```

El mixer y el control pertenecen a `1G`.

## Clases, funciones y tags a localizar

```text
PluginActuarMotores::Load
PluginActuarMotores::OnWrench
PluginActuarMotores::OnUpdate
GZ_REGISTER_MODEL_PLUGIN
model_->GetLink
AddRelativeForce
AddRelativeTorque
<plugin name="plugin_actuar_motores">
<motorN>
<topicN>
<fuerza2torque>
```

## Cambios requeridos

1. Compilar el plugin como shared library e instalarlo.
2. Asociar cada topic con el link correcto en `dron_plugins.xacro`.
3. Crear las cuatro suscripciones dentro del namespace del modelo.
4. Aplicar fuerza y torque relativos en cada actualización de Gazebo.
5. Alternar el signo del torque de reacción de forma coherente con el orden de motores.
6. Registrar link/topic al cargar y emitir error si falta un link.
7. Validar que `fuerza2torque` es finito.
8. Definir explícitamente la política de consignas negativas, cero, máximas y caducidad. Si se conserva la ausencia de límites, debe figurar como limitación y las pruebas usar rangos seguros.
9. Mantener 4 motores como configuración obligatoria; 6/8 son opcionales y no prueban control.

## Cambios prohibidos

- No calcular trayectorias.
- No implementar el mixer de fuerzas totales.
- No usar GT para la actuación.
- No aplicar fuerza al link `cuerpo` si el contrato dice motor individual.
- No asumir que el joint visual de la hélice modela empuje real.
- No ocultar una fuerza máxima hardcodeada.
- No dejar una consigna de prueba elevada activa al cerrar la prueba; publicar cero.

## Paquetes a compilar

```bash
./codex/herramientas/build_selected_packages.sh simulacion_dron dron_individual
```

## Pruebas Gazebo requeridas

### Preparación

```yaml
fisico.brazos.numero: 4
actuadores.conversor.fuerza2torque: 0.02
```

Arrancar un dron en `empty.world` sin activar todavía el control de `1G`.

### Prueba 1 — Consigna común

Publicar el mismo valor pequeño y seguro en los cuatro topics:

```bash
ros2 topic pub --rate 20 /dron_1/motor/arr_iz std_msgs/msg/Float64 "{data: 3.0}"
ros2 topic pub --rate 20 /dron_1/motor/ab_iz  std_msgs/msg/Float64 "{data: 3.0}"
ros2 topic pub --rate 20 /dron_1/motor/ab_der std_msgs/msg/Float64 "{data: 3.0}"
ros2 topic pub --rate 20 /dron_1/motor/arr_der std_msgs/msg/Float64 "{data: 3.0}"
```

El valor exacto debe elegirse con la masa configurada para evitar una aceleración peligrosa. Al terminar, publicar `0.0` en todos.

Resultado esperado: variación vertical sin un yaw sostenido grande causado por signos incorrectos.

### Prueba 2 — Actuación asimétrica

Aplicar una diferencia pequeña entre lados y observar inclinación/rotación coherente. No se exige estabilización.

### Prueba 3 — Aislamiento multi-dron

Con dos drones, publicar únicamente en `/dron_1/motor/*`. `dron_2` no debe recibir esas consignas.

### Prueba opcional — 6/8 motores

Solo comprueba que el plugin se suscribe a los topics adicionales y aplica actuación manual. No forma parte del cierre ni prueba un mixer/control válido.

## Patrones de reducción de logs

```text
PluginActuarMotores|Suscrito a|aplicar wrench|no encuentro link|motor/|fuerza2torque|ERROR|FATAL|Segmentation fault|Killed
```

## Evidencia visual obligatoria

Registrar en el historial futuro:

- valor y frecuencia de cada topic;
- masa total configurada;
- duración de la consigna;
- movimiento observado;
- confirmación de publicación de cero al terminar;
- captura o vídeo si el criterio visual no puede deducirse del log.

## Criterio de éxito

1. El plugin compila y carga.
2. Los cuatro links y topics se resuelven.
3. Las fuerzas manuales mueven el dron en Gazebo.
4. La asimetría produce respuesta angular observable.
5. Dos namespaces quedan aislados.
6. El valor `fuerza2torque` procede de YAML/Xacro.
7. Se documenta la falta de control obligatorio para 6/8.
8. No quedan errores graves ni consignas activas al finalizar.

## Criterio de fallo o parcial

- `NO CONSEGUIDA`: plugin no carga, link inexistente, topics sin efecto o movimiento de otro dron.
- `PARCIAL`: fuerza lineal funciona, pero torque/signos, aislamiento o configuración YAML no cumplen.
- `BLOQUEADA`: incompatibilidad no resuelta con la versión de Gazebo Classic.

## Riesgos

- fuerza ilimitada;
- consigna persistente;
- orden de índices distinto al orden físico;
- torque alterno incorrecto;
- saturación numérica o dron expulsado del mundo;
- confundir soporte del plugin 6/8 con soporte del control.

## Documentación a actualizar al ejecutar

```text
codex/contexto/paquetes/simulacion_dron/
codex/pipeline/fase_1_control_dron/historial/por_subfase/historial_1D.md
codex/pipeline/fase_1_control_dron/historial/por_subfase/historial_1D_RESUMEN.md
```
