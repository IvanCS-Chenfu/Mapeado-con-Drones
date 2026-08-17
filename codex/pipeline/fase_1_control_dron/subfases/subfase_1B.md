# Subfase 1B — Modelo físico Xacro/URDF del dron

## Estado

```text
realizado
```

## Dependencia

`1A` — Entorno base de Gazebo.

## Objetivo técnico

Definir un modelo físico parametrizable del dron mediante Xacro que genere un URDF válido a partir de `hardware.yaml`. El cierre obligatorio corresponde a un cuadricóptero de cuatro motores formado por cuerpo, cuatro brazos y cuatro motores, con geometría visual, colisiones, joints, masas e inercias.

Se puede conservar soporte de modelado para seis u ocho brazos, pero no forma parte del criterio obligatorio de control.

## Comportamiento esperado

- El Xacro se procesa sin Gazebo.
- El URDF contiene `cuerpo`, cuatro links de brazo, cuatro links de motor y sus joints.
- Los parámetros físicos llegan desde YAML y no quedan reemplazados por constantes contradictorias.
- Las masas e inercias son válidas para la simulación.
- Las configuraciones 6/8, si se conservan, solo demuestran generación del modelo; no control.

## Contexto obligatorio a leer

```text
AGENTS.md
codex/contexto/00_CONTEXTO_COMPACTACION.md
codex/pipeline/fase_1_control_dron/pipeline_fase_1_RESUMEN.md
codex/pipeline/fase_1_control_dron/subfases/subfase_1A.md
codex/contexto/paquetes/simulacion_dron/00_summary.md
codex/contexto/paquetes/dron_individual/00_summary.md
```

Leer además los documentos de paquete relativos a Xacro, URDF y configuración física.

## Diagnóstico de partida

El código de referencia contiene:

```text
src/simulacion_dron/urdf/dron.xacro
src/simulacion_dron/urdf/dron_plugins.xacro
src/dron_individual/config/hardware.yaml
```

`hardware.yaml` configura cuerpo, brazos, motores y cámara. Los valores son arbitrarios de simulación. En los Xacro de referencia, la masa del link `cuerpo` aparece fijada a `1.0` en vez de usar siempre `fisico_cuerpo_masa`; la reconstrucción debe comprobar y corregir cualquier incoherencia de ese tipo.

## YAML obligatorio: `hardware.yaml`

Ruta:

```text
src/dron_individual/config/hardware.yaml
```

Parámetros físicos mínimos:

```yaml
fisico.cuerpo.dim: [x, y, z]                         # m
fisico.cuerpo.color: "White"
fisico.cuerpo.masa: 1.0                              # kg
fisico.cuerpo.matriz_inercia: [ixx, iyy, izz, ixy, ixz, iyz]  # kg·m²

fisico.brazos.numero: 4
fisico.brazos.grados: 45.0                           # grados
fisico.brazos.dim: [longitud, radio]                 # m
fisico.brazos.color: "Gray"
fisico.brazos.masa: 0.05                             # kg por brazo
fisico.brazos.matriz_inercia: [ixx, iyy, izz, ixy, ixz, iyz]

fisico.motores.dim: [altura, radio]                  # m
fisico.motores.color: "Green"
fisico.motores.masa: 0.05                            # kg por motor
fisico.motores.matriz_inercia: [ixx, iyy, izz, ixy, ixz, iyz]
```

Reglas de validación:

1. `dim` debe tener el tamaño exacto documentado y componentes positivas.
2. Las masas deben ser finitas y no negativas; para cuerpo, brazos y motores usados dinámicamente deben ser positivas.
3. Las inercias deben tener seis componentes y formar una matriz simétrica físicamente admisible.
4. `fisico.brazos.numero` obligatorio es `4`; valores opcionales permitidos: `6` u `8`.
5. Los colores deben existir como materiales URDF/Gazebo o producir un error claro.
6. Todos los valores actuales deben etiquetarse como baseline arbitrario, no como datos de hardware real.

## Convenciones geométricas

- `cuerpo`: caja centrada en su link.
- `brazo`: cilindro unido mediante joint fijo al cuerpo.
- `motor`: cilindro unido al extremo del brazo mediante joint continuo.
- `fisico.brazos.grados`: orientación de los brazos diagonales respecto al frente.
- Los nombres de links de cuatro motores deben coincidir con los utilizados después por el plugin:

```text
arriba_izquierda_motor
abajo_izquierda_motor
abajo_derecha_motor
arriba_derecha_motor
```

## Archivos permitidos a modificar

```text
src/simulacion_dron/urdf/dron.xacro
src/simulacion_dron/urdf/dron_plugins.xacro        # solo parte física; plugins se validan en 1D/1E
src/dron_individual/config/hardware.yaml
src/simulacion_dron/CMakeLists.txt
src/simulacion_dron/package.xml
codex/contexto/paquetes/simulacion_dron/
codex/contexto/paquetes/dron_individual/
```

## Archivos prohibidos

```text
src/dron_individual/src/control_tray/
src/lib_tray/
src/simulacion_dron/src/plugins/
src/orbslam3_multi/
src/orbslam3_server/
ORB_SLAM3/
orbslam3_ros2/
```

No implementar sensores o actuación en esta subfase, aunque `dron_plugins.xacro` sea el archivo integrado final.

## Funciones, macros y elementos a localizar

```text
<xacro:arg>
<xacro:property>
link cuerpo
link_arm
joint_cuerpo_brazo
link_motor
joint_brazo_motor
brazo_total
xacro:if fisico_brazos_numero == 4/6/8
visual
collision
inertial
```

## Cambios requeridos

1. Mantener `hardware.yaml` como fuente de configuración física.
2. Pasar cada campo del YAML a argumentos Xacro con nombres estables.
3. Crear geometría visual y de colisión para cuerpo, brazos y motores.
4. Aplicar las masas e inercias del YAML, incluida la masa de `cuerpo`.
5. Generar obligatoriamente la variante de cuatro motores.
6. Validar tamaños de vectores antes de indexarlos en el nodo que procese el YAML.
7. Documentar unidades y semántica de cada parámetro.
8. Mantener nombres de links compatibles con las subfases 1D y 1E.
9. Evitar duplicar dos modelos físicos divergentes; si existen `dron.xacro` y `dron_plugins.xacro`, el segundo debe reutilizar o permanecer sincronizado con el primero.

## Cambios prohibidos

- No inventar un dron real.
- No ajustar masas o inercias para “que vuele” sin registrar que son valores de simulación.
- No usar `mass value="1.0"` si el YAML proporciona otro valor.
- No añadir controladores de motor.
- No afirmar soporte controlado de seis u ocho motores.
- No introducir meshes externos sin licencia y ruta reproducible.

## Paquetes a compilar

```bash
./codex/herramientas/build_selected_packages.sh dron_individual simulacion_dron
```

## Tests deterministas requeridos

### Test 1 — Procesamiento Xacro de cuatro motores

```bash
xacro src/simulacion_dron/urdf/dron.xacro \
  fisico_brazos_numero:=4 > /tmp/dron_4.urdf
check_urdf /tmp/dron_4.urdf
```

Comprobar:

- un `cuerpo`;
- cuatro arms;
- cuatro motors;
- joints sin referencias rotas;
- masas e inercias presentes;
- ausencia de errores XML/Xacro.

### Test 2 — Propagación de parámetros

Cambiar temporalmente una dimensión, masa y color en un YAML de prueba y verificar que el URDF generado contiene los valores nuevos. Restaurar el baseline al terminar.

### Test 3 — Variantes opcionales

Generar 6 y 8 solo si se mantiene ese código. El test confirma estructura y nombres, no vuelo ni control.

## Prueba Gazebo requerida

Insertar el URDF del cuadricóptero en el mundo vacío mediante la ruta mínima acordada. Debe aparecer con geometría, orientación y colisiones coherentes, sin plugins funcionales todavía.

```bash
./codex/herramientas/run_simulation.sh \
  --prueba fase1_1B_modelo \
  --launch "ros2 launch simulacion_dron gazebo.launch.py" \
  --post-scenario-wait-sec 5
```

La forma exacta de inserción puede usar `/spawn_entity` de manera manual en esta prueba o la integración de `1C`, sin adelantar su implementación.

## Patrones de reducción de logs

```text
xacro|URDF|check_urdf|link|joint|inertia|mass|spawn_entity|ERROR|FATAL|Segmentation fault|Killed
```

## Criterio de éxito

1. Los paquetes compilan.
2. El Xacro de cuatro motores genera URDF válido.
3. El YAML modifica realmente geometría y parámetros físicos.
4. La masa de cada link usa su parámetro correspondiente.
5. Los nombres de links quedan listos para el plugin de 1D.
6. El modelo puede visualizarse en Gazebo sin errores graves.
7. Se documenta que los datos son arbitrarios y que 6/8 no están controlados.

## Criterio de fallo o parcial

- `NO CONSEGUIDA`: Xacro inválido, links/joints rotos, vectores fuera de rango o parámetros YAML ignorados.
- `PARCIAL`: el modelo de cuatro motores se genera, pero persisten incoherencias de masa/inercia o duplicación entre Xacro.
- `BLOQUEADA`: falta `xacro`/`check_urdf` o una dependencia del modelo que no puede resolverse.

## Documentación a actualizar al ejecutar

```text
codex/contexto/paquetes/simulacion_dron/
codex/contexto/paquetes/dron_individual/
codex/pipeline/fase_1_control_dron/historial/por_subfase/historial_1B.md
codex/pipeline/fase_1_control_dron/historial/por_subfase/historial_1B_RESUMEN.md
```
