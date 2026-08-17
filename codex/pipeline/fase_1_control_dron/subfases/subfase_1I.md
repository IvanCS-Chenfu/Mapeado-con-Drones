# Subfase 1I — Gráficas de trayectorias, estado y error de seguimiento

## Estado

```text
realizado
```

El estado `realizado` indica que el proyecto de referencia ya contiene una primera infraestructura de adquisición y representación en `simulacion_dron/src/graficar/`. No constituye evidencia de que todas las gráficas, namespaces, métricas y pruebas descritas en este contrato se hayan vuelto a ejecutar. Los resultados futuros se registrarán únicamente en `historial/por_subfase/`.

## Dependencias

```text
1E — Ground Truth disponible en simulación
1F — semántica de los perfiles de lib_tray conocida y probada
1G — feedback de TrayAction y control en lazo cerrado
1H — selección de dron y envío reproducible de goals
```

## Objetivo técnico

Crear y validar un subsistema de gráficas que permita observar, durante una trayectoria y al finalizarla, la información necesaria para entender el generador y evaluar el seguimiento del cuadricóptero:

- perfiles deseados de posición, velocidad, aceleración, jerk y ratio de progreso;
- estado real obtenido temporalmente desde Ground Truth;
- comparación entre referencia y estado real;
- errores de posición, velocidad y yaw;
- comportamiento por eje y evolución respecto al tiempo de simulación;
- aislamiento correcto entre drones mediante namespaces.

Las gráficas son instrumentación de simulación y diagnóstico. No forman parte del lazo de control, no pueden modificar consignas y no deben bloquear `gen_tray`, `control_calcular_fuerzas`, `aplicar_fuerzas_dron` ni Gazebo.

En esta fase se admite Ground Truth como medida real de comparación. La subfase debe recordar explícitamente que en la **Fase 5** las gráficas de operación y control deberán consumir la pose/velocidad estimadas sin GT. GT podrá conservarse entonces solo como métrica externa de simulación.

## Comportamiento esperado

Al ejecutar una trayectoria para un dron seleccionado debe ser posible abrir, como mínimo, las siguientes vistas:

1. **Perfil deseado de posición**: `x`, `y`, `z` y `yaw`.
2. **Perfil deseado de velocidad**: `vx`, `vy`, `vz` y `vyaw`.
3. **Perfil deseado de aceleración**: `ax`, `ay`, `az` y `ayaw`.
4. **Perfil deseado de jerk**: `jx`, `jy`, `jz` y `jyaw` cuando el generador lo proporcione.
5. **Ratio de progreso** de cada eje cuando sea significativo para el generador.
6. **Referencia frente a GT** para posición y yaw.
7. **Error de seguimiento** de `x`, `y`, `z` y `yaw`.
8. **Referencia frente a GT de velocidad** y su error, cuando estén activos `sensor/GT/vel` y el índice de velocidad del feedback.

Cada gráfica debe indicar título, magnitud, unidad, eje temporal, dron/namespace, tipo de trayectoria y leyenda inequívoca. No se deben mezclar en un mismo eje magnitudes con unidades incompatibles sin una separación explícita.

## Contexto obligatorio a leer

```text
AGENTS.md
codex/pipeline/fase_1_control_dron/pipeline_fase_1_RESUMEN.md
codex/pipeline/fase_1_control_dron/subfases/subfase_1E.md
codex/pipeline/fase_1_control_dron/subfases/subfase_1F.md
codex/pipeline/fase_1_control_dron/subfases/subfase_1G.md
codex/pipeline/fase_1_control_dron/subfases/subfase_1H.md
codex/contexto/paquetes/simulacion_dron/00_summary.md
codex/contexto/paquetes/dron_individual/00_summary.md
codex/contexto/paquetes/lib_tray/00_summary.md
```

También se debe revisar, como fuente de verdad del código actual:

```text
src/simulacion_dron/src/graficar/graficar.py
src/simulacion_dron/src/graficar/graficar_GT.cpp
src/simulacion_dron/src/graficar/graficar_tray.cpp
src/simulacion_dron/src/graficar/graficar_GTvsTray.cpp
src/simulacion_dron/CMakeLists.txt
src/simulacion_dron/package.xml
src/dron_individual/action/TrayAction.action
src/dron_individual/src/control_tray/gen_tray.cpp
```

La wiki del proyecto puede utilizarse como documentación explicativa, pero el comportamiento y la semántica final deben comprobarse contra el código y las interfaces instaladas.

## Diagnóstico de partida

La implementación de referencia sigue esta arquitectura:

```text
fuente ROS 2
  -> adaptador C++ de datos
      -> Float64MultiArray /numeric_array
      -> UInt8MultiArray  /labels_array
          -> graficar.py
              -> Matplotlib
```

### `graficar.py`

- implementa `MultiArrayPlotter` en Python;
- escucha un array numérico y otro array 2D de etiquetas;
- utiliza tiempo ROS y admite `use_sim_time`;
- detecta la aparición/desaparición del publisher numérico;
- reinicia buffers cuando cambia el número de series;
- mantiene hasta 5000 muestras;
- permite una ventana móvil mediante `window_seconds`;
- actualiza Matplotlib en el hilo principal y ejecuta `rclpy.spin` en otro hilo.

### `graficar_GT.cpp`

- tiene activos únicamente los datos de posición GT `x`, `y`, `z`;
- las suscripciones de velocidad y aceleración están comentadas;
- utiliza actualmente el topic absoluto `/model/sensor/GT/pose`;
- publica en topics globales `/numeric_array` y `/labels_array`.

### `graficar_tray.cpp`

- escucha actualmente `/AccionTrayectoria/_action/feedback` como topic absoluto;
- accede al índice `1` de cada eje, que según `TrayAction.action` significa **velocidad**;
- publica las etiquetas `x`, `y`, `z`, `yaw`, por lo que la etiqueta actual no expresa correctamente la magnitud;
- no permite seleccionar desde configuración posición, velocidad, aceleración, jerk o ratio.

### `graficar_GTvsTray.cpp`

- escucha `sensor/GT/pose` y `AccionTrayectoria/_action/feedback` como topics relativos;
- convierte el quaternion GT a yaw;
- usa el índice `0` del feedback, correspondiente a posición;
- publica actualmente `referencia - real` para `x`, `y`, `z` y `yaw` cada 20 ms;
- deja comentadas las comparaciones de curvas real/referencia y las variantes de velocidad/aceleración;
- puede intentar indexar el vector real antes de haber recibido una muestra GT si llega primero el feedback;
- no normaliza el error angular al atravesar `-pi/pi`.

Los tres adaptadores publican por defecto sobre los mismos topics globales. Esto impide ejecutar varias fuentes o varios drones simultáneamente sin colisiones o mezcla de datos.

## Semántica obligatoria de `TrayAction` para las gráficas

Cada eje de feedback contiene cinco elementos:

```text
índice 0 -> posición
índice 1 -> velocidad
índice 2 -> aceleración
índice 3 -> jerk
índice 4 -> ratio
```

Unidades esperadas:

| Magnitud | `x/y/z` | `yaw` |
|---|---|---|
| Posición | m | rad |
| Velocidad | m/s | rad/s |
| Aceleración | m/s² | rad/s² |
| Jerk | m/s³ | rad/s³ |
| Ratio | adimensional | adimensional |

No se permite elegir índices mediante números mágicos dispersos. La relación índice-magnitud debe centralizarse, validarse y documentarse.

## YAML obligatorio

Crear una configuración específica y lógica para la instrumentación:

```text
src/simulacion_dron/config/graficas.yaml
```

Esquema mínimo esperado:

```yaml
graficas:
  habilitadas: true
  use_sim_time: true

  dron:
    namespace: "dron_1"

  fuente:
    modo: "error_posicion"   # tray_perfil | gt | comparar_posicion | error_posicion | comparar_velocidad | error_velocidad

  topics:
    feedback: "AccionTrayectoria/_action/feedback"
    gt_pose: "sensor/GT/pose"
    gt_vel: "sensor/GT/vel"
    gt_acc: "sensor/GT/acc"
    numeric: "graficas/numeric_array"
    labels: "graficas/labels_array"

  visualizacion:
    ventana_segundos: 20.0   # 0 = histórico completo limitado por buffer
    max_muestras: 5000
    refresco_ms: 50
    titulo: ""
    mostrar_leyenda: true
    backend: "interactive"  # interactive | headless

  muestreo:
    periodo_ms: 20
    edad_maxima_gt_ms: 100

  error:
    normalizar_yaw: true

  exportacion:
    guardar_png: false
    guardar_csv: false
    directorio: ""
```

Reglas del YAML:

- `namespace` debe coincidir con `sim_dron.yaml` y con el launch multi-dron.
- Los topics internos deben ser relativos al namespace; no deben construirse con `/model` o `/AccionTrayectoria` globales.
- Los tiempos se expresan en segundos o milisegundos según el nombre del campo.
- `max_muestras`, periodos y ventanas deben validarse como positivos, salvo los ceros documentados.
- Los umbrales de error, si se añaden, son criterios de prueba y no características físicas del dron.
- No se deben duplicar masa, inercia, geometría o ganancias en `graficas.yaml`. Sus fuentes de verdad siguen siendo `hardware.yaml` y `tray_dron.yaml`.
- Los valores actuales del proyecto son parámetros arbitrarios de simulación, no datos de un dron real.
- Cuando exista hardware real, las gráficas deberán conservar unidades y trazabilidad hacia los YAML físicos, pero nunca alterar esos parámetros.

## Contrato de topics y namespaces

Para el dron configurado como `dron_1`, los topics efectivos deben resolverse de forma equivalente a:

```text
/dron_1/AccionTrayectoria/_action/feedback
/dron_1/sensor/GT/pose
/dron_1/sensor/GT/vel
/dron_1/sensor/GT/acc
/dron_1/graficas/numeric_array
/dron_1/graficas/labels_array
```

Se admite ejecutar una sola fuente de gráfica por pareja `numeric/labels`, o publicar cada grupo en un topic distinto y lanzar una instancia de `graficar.py` por grupo. No se admite que dos productores incompatibles escriban silenciosamente en el mismo topic global.

## Archivos permitidos a modificar

```text
src/simulacion_dron/src/graficar/graficar.py
src/simulacion_dron/src/graficar/graficar_GT.cpp
src/simulacion_dron/src/graficar/graficar_tray.cpp
src/simulacion_dron/src/graficar/graficar_GTvsTray.cpp
src/simulacion_dron/config/graficas.yaml
src/simulacion_dron/launch/graficas.launch.py
src/simulacion_dron/launch/multi_dron.launch.py
src/simulacion_dron/CMakeLists.txt
src/simulacion_dron/package.xml
codex/contexto/paquetes/simulacion_dron/
codex/contexto/paquetes/dron_individual/
```

`dron_individual` y `lib_tray` pueden leerse para confirmar la semántica de la interfaz. Solo deben modificarse si se identifica un defecto que impida publicar datos coherentes y se cierra previamente el acuerdo funcional correspondiente.

## Archivos prohibidos

```text
src/orbslam3_multi/
src/orbslam3_server/
ORB_SLAM3/
orbslam3_ros2/
orbslam3_msgs/
```

No modificar `TrayAction.action` para resolver una carencia que pueda solucionarse consumiendo correctamente sus cinco elementos actuales. No introducir todavía pose estimada de ORB-SLAM3: esa sustitución pertenece a la Fase 5.

## Funciones, clases y nodos que hay que localizar

```text
labels_from_uint8_2d
MultiArrayPlotter
MultiArrayPlotter.on_numeric
MultiArrayPlotter.on_labels
MultiArrayPlotter.poll_publishers
MultiArrayPlotter.start_run
MultiArrayPlotter.stop_run
MultiArrayPlotter.mpl_update
Clase_Subscriber
graficar_GT
graficar_tray
graficar_GTvsTray
make_labels_2d
pose2yaw
pose_callback
vel_callback
acc_callback
feedback_callback
enviar_graficas
TrayAction::Feedback
gen_tray::array_to_msg
```

Si el nombre real cambia, el planificador debe localizarlo por búsqueda estática antes de implementar y actualizar la documentación del paquete.

## Cambios requeridos

1. **Conservar la separación adaptador/plotter.** Los nodos ROS 2 preparan datos y etiquetas; `graficar.py` dibuja sin intervenir en control.
2. **Crear y cargar `graficas.yaml`.** Todos los topics, modos, ventanas, buffers, periodos y opciones de backend deben quedar configurados y validados.
3. **Eliminar topics globales hardcodeados.** Usar topics relativos y namespace/remappings del launch.
4. **Corregir las etiquetas del feedback.** El índice `1` debe etiquetarse `vx/vy/vz/vyaw`; el índice `0`, `x/y/z/yaw`; y así sucesivamente.
5. **Permitir seleccionar la magnitud de trayectoria.** Posición, velocidad, aceleración, jerk y ratio deben poder representarse sin recompilar ni comentar/descomentar líneas.
6. **Activar las fuentes GT necesarias.** Posición es obligatoria; velocidad también para la comparación de perfiles; aceleración puede representarse cuando el plugin la publique con significado documentado.
7. **Añadir flags de disponibilidad.** No calcular ni publicar comparación/error hasta disponer de una referencia y una muestra real compatibles.
8. **Rechazar muestras obsoletas.** No comparar feedback nuevo con GT cuya antigüedad supere `edad_maxima_gt_ms`; registrar el descarte sin bloquear.
9. **Normalizar yaw.** El error angular debe calcularse en un intervalo coherente, normalmente `[-pi, pi]`, para evitar saltos artificiales de aproximadamente `2*pi`.
10. **Separar comparación y error.** Debe poder verse referencia/real superpuestas y, en otra vista o modo, su diferencia.
11. **Usar tiempo de simulación.** El eje X debe reflejar tiempo ROS desde el comienzo de la adquisición o `t_act` cuando el modo lo requiera; la elección debe estar documentada.
12. **Reiniciar entre ejecuciones.** Un goal nuevo o una reaparición del publisher no puede mezclar sin aviso datos de una trayectoria anterior.
13. **Soportar varios drones sin mezcla.** Seleccionar explícitamente un namespace o lanzar instancias aisladas por dron.
14. **Añadir modo headless.** Las pruebas automáticas deben poder generar el artefacto gráfico sin pantalla, mientras que el modo interactivo se mantiene para desarrollo.
15. **Declarar dependencias de ejecución.** `package.xml` debe incluir la dependencia ROS/Ubuntu correspondiente a Matplotlib y cualquier dependencia Python realmente utilizada.
16. **Integrar mediante launch opcional.** Añadir un launch específico o argumentos `enable_graphs`, `graph_drone` y `graph_mode`; las gráficas no deben abrirse por defecto en simulaciones no interactivas.
17. **Emitir marcadores de diagnóstico.** Registrar inicio/parada, fuente elegida, namespace, magnitud, número de series, descarte por antigüedad y ruta de exportación cuando aplique.
18. **Mantener bajo coste.** No copiar históricos ilimitados, no recalcular todo el buffer en callbacks ROS y no degradar de forma material la frecuencia del control.

## Cambios prohibidos

- No usar las gráficas como entrada del controlador.
- No modificar valores de `hardware.yaml` o `tray_dron.yaml` para que una curva parezca mejor.
- No ocultar errores recortando el eje Y o filtrando datos sin indicarlo.
- No comparar grados con radianes ni posición con velocidad.
- No etiquetar como posición un índice de velocidad.
- No calcular error antes de recibir ambas señales.
- No restar yaw sin tratar la discontinuidad angular.
- No mezclar datos de varios drones sobre topics globales.
- No imponer umbrales de rendimiento como si procedieran de hardware real; todavía no existe ese hardware.
- No concluir que el control es bueno únicamente porque la curva “parece cercana”.
- No eliminar GT en esta subfase; su retirada funcional se realizará en Fase 5.
- No abrir Matplotlib por defecto en una simulación headless o de regresión.

## Paquetes a compilar

```bash
./codex/herramientas/build_selected_packages.sh lib_tray dron_individual simulacion_dron
```

Si solo se modifica `simulacion_dron` y sus dependencias ya están instaladas, puede realizarse primero un build reducido de ese paquete. La decisión y el resultado se registrarán en historial.

## Pruebas de componente requeridas

### Prueba 1 — Decodificación y etiquetas

Publicar arrays conocidos y comprobar:

- decodificación correcta del `UInt8MultiArray` 2D;
- mismo número de etiquetas y series;
- rechazo o reinicio controlado si cambia el tamaño;
- etiquetas correctas para los índices `0..4`.

### Prueba 2 — Error conocido

Inyectar referencia y GT sintéticos con diferencias conocidas:

```text
referencia = [1.0, 2.0, 3.0, 0.2]
real       = [0.5, 2.5, 2.0, 0.1]
error      = [0.5, -0.5, 1.0, 0.1]
```

La salida numérica y las curvas deben coincidir con el resultado esperado.

### Prueba 3 — Normalización de yaw

Usar una referencia próxima a `+pi` y una medida próxima a `-pi`. El error debe ser el ángulo corto y no un salto cercano a `2*pi`.

### Prueba 4 — Datos ausentes u obsoletos

- feedback sin GT;
- GT sin feedback;
- GT más antiguo que `edad_maxima_gt_ms`;
- publisher que desaparece y reaparece.

El sistema debe pausar/reiniciar de forma controlada sin excepción ni acceso fuera de rango.

## Pruebas Gazebo requeridas

### Prueba 1 — Perfiles de una trayectoria

YAML de escenario:

```text
codex/archivos_auxiliares/trayectorias/tray_prueba_fase1_1I_perfiles.yaml
```

Secuencia:

1. configurar un cuadricóptero de cuatro motores;
2. activar las gráficas para `dron_1`;
3. enviar una trayectoria polinómica corta;
4. observar posición, velocidad, aceleración y jerk deseados;
5. comprobar continuidad y etiquetas/unidades;
6. repetir con perfil trapezoidal y elipse.

### Prueba 2 — Referencia frente a GT y error

1. ejecutar una traslación segura con control activo;
2. representar `x/y/z/yaw` deseados y reales;
3. representar `e_x/e_y/e_z/e_yaw`;
4. verificar que el signo del error coincide con `referencia - real`;
5. comprobar que no se publican errores antes de recibir ambas fuentes.

### Prueba 3 — Velocidad

1. activar `sensor/GT/vel`;
2. representar `vx/vy/vz/vyaw` deseados y reales;
3. representar sus errores;
4. confirmar unidades y ausencia de mezcla con posición.

### Prueba 4 — Dos drones

1. configurar dos drones;
2. abrir gráficas de `dron_1`;
3. enviar goals distintos a `dron_1` y `dron_2`;
4. comprobar que las curvas corresponden solo al namespace seleccionado;
5. cambiar o lanzar una segunda instancia para `dron_2` sin colisiones de topics.

### Prueba 5 — Modo headless

Ejecutar una trayectoria con backend no interactivo y guardar al menos un PNG. La prueba debe terminar sin depender de un servidor gráfico y dejar registrada la ruta del artefacto.

## Comando base

```bash
./codex/herramientas/run_simulation.sh \
  --prueba fase1_1I_graficas \
  --launch "ros2 launch simulacion_dron multi_dron.launch.py enable_graphs:=true graph_drone:=dron_1 graph_mode:=error_posicion" \
  --post-scenario-wait-sec 20
```

Los nombres exactos de argumentos se adaptarán al launch implementado, manteniendo el mismo contrato funcional.

## Observación visual requerida

En modo interactivo se debe comprobar y registrar:

- ventana abierta y actualizándose;
- título con dron y magnitud;
- eje X en segundos;
- eje Y con unidad correcta;
- leyenda completa;
- curvas no mezcladas entre drones;
- reinicio claro al comenzar una nueva ejecución;
- ausencia de congelación de Gazebo/control mientras se dibuja.

Una captura visual demuestra la representación, pero no sustituye la prueba numérica de índices, unidades y error conocido.

## Patrones de reducción de logs

```text
GRAPH-RUN-START|GRAPH-RUN-STOP|GRAPH-SOURCE|GRAPH-SERIES|GRAPH-RESET|GRAPH-SAMPLE-DROPPED|GRAPH-EXPORT|graficar_GT|graficar_tray|graficar_GTvsTray|numeric_array|labels_array|AccionTrayectoria|sensor/GT/(pose|vel|acc)|publisher|Tamaño del array cambió|Traceback|IndexError|ERROR|FATAL|Segmentation fault|Killed
```

Los marcadores nuevos deben incluir como mínimo:

```text
namespace=...
mode=...
series=...
reason=...
```

## Criterio de éxito

La subfase se considera `CONSEGUIDA` solo si:

1. `lib_tray`, `dron_individual` y `simulacion_dron` compilan.
2. Los tests de índice, etiquetas, error conocido y yaw pasan.
3. El YAML de gráficas se instala, carga y valida.
4. Las vistas obligatorias de posición, velocidad, aceleración, jerk, comparación y error pueden generarse.
5. Las curvas usan la semántica `[pos, vel, acc, jerk, ratio]` de `TrayAction` sin etiquetas incorrectas.
6. No se calcula error con datos ausentes u obsoletos.
7. La comparación de yaw usa diferencia angular normalizada.
8. Un dron no contamina las gráficas de otro.
9. El modo interactivo funciona y el modo headless genera un artefacto.
10. La instrumentación no bloquea ni altera el control.
11. GT queda identificado como dependencia temporal de Fase 1 y pendiente de sustitución funcional en Fase 5.
12. La documentación del paquete y el historial de la ejecución quedan actualizados.

No se exige que el error esté por debajo de un umbral físico definitivo, porque los parámetros actuales son arbitrarios y no existe un dron real. Sí se exige que el error se calcule correctamente, sea finito y quede expresado con unidades.

## Criterio de fallo o parcial

La subfase debe marcarse como `NO CONSEGUIDA` si:

- no compila;
- los índices del feedback se interpretan de forma incorrecta;
- las gráficas mezclan posición y velocidad o grados y radianes;
- se produce una excepción por datos no recibidos;
- los topics globales mezclan drones;
- el error de yaw presenta saltos artificiales de `2*pi`;
- la instrumentación bloquea o modifica el control;
- no puede representarse ninguna comparación real/referencia.

Puede marcarse `PARCIAL` si las gráficas básicas funcionan para un dron, pero falta alguno de estos bloques:

- velocidad o aceleración GT;
- aislamiento multi-dron;
- modo headless/exportación;
- reinicio entre goals;
- validación de datos obsoletos.

Debe marcarse `BLOQUEADA` solo si falta una dependencia externa de visualización o una interfaz imprescindible que no pueda resolverse con la estructura actual.

## Documentación a actualizar

Tras cualquier cambio o prueba real, actualizar:

```text
codex/contexto/paquetes/simulacion_dron/00_summary.md
codex/contexto/paquetes/simulacion_dron/<documento_del_subsistema_graficar>.md
codex/contexto/paquetes/dron_individual/<documento_de_TrayAction_o_gen_tray>.md
codex/pipeline/fase_1_control_dron/historial/por_subfase/
codex/pipeline/fase_1_control_dron/pipeline_fase_1_RESUMEN.md
```

La documentación debe explicar:

- arquitectura adaptadores C++ -> arrays -> plotter Python;
- semántica de los cinco índices;
- topics y namespaces;
- parámetros de `graficas.yaml`;
- unidades y tratamiento de yaw;
- modos interactivo y headless;
- limitación temporal de GT y sustitución prevista en Fase 5;
- resultados reales solo cuando existan pruebas ejecutadas.

No escribir resultados de build, simulación o capturas en este contrato. Esos datos pertenecen al historial.
