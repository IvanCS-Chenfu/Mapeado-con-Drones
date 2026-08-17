# Subfase 5G — Velocidad estimada, aceleración de debug y recuperación tras pérdida total

## Estado

```text
sin hacer
```

## Objetivo técnico

Obtener sin Ground Truth la velocidad lineal y angular que necesita el control del dron, medir su calidad frente a GT como métrica externa y completar el mecanismo de recuperación cuando se pierden simultáneamente la pose global y el tracking local de ORB-SLAM3.

La velocidad funcional debe proceder de la evolución temporal de la pose local de ORB-SLAM3, no de diferenciar directamente una pose global que puede sufrir saltos por nuevas correcciones u optimizaciones.

El controlador actual necesita como estado:

```text
posición/orientación
velocidad lineal
velocidad angular
```

La aceleración actual medida no es una entrada funcional del `control_calcular_fuerzas` baseline; la aceleración que aparece en el feedback es la aceleración deseada de trayectoria. Aun así, esta subfase debe permitir calcular una aceleración estimada para diagnóstico mediante un parámetro:

```yaml
debug_acc_est: false
```

Con `false`, el cálculo de aceleración estimada debe quedar desactivado y no consumir recursos innecesarios. Con `true`, debe publicarse/registrarse para comparar contra `sensor/GT/acc` exclusivamente como debug.

La subfase también debe probar una recuperación ciega breve cuando ORB entra en `LOST` y no existe ninguna otra fuente de estado. No se pretende navegar a ciegas: el objetivo es invertir de forma acotada el movimiento reciente para volver a poner en cámara la zona con landmarks que se acaba de abandonar.

## Contexto obligatorio a leer

```text
AGENTS.md
codex/contexto/00_CONTEXTO_COMPACTACION.md
codex/contexto/CONTEXTO_MINIMO_ACTUAL.md
codex/contexto/08_POLITICA_TOKENS_DOCUMENTACION.md
codex/contexto/01_ESTADO_ACTUAL_RESUMEN.md
codex/contexto/01_ESTADO_ACTUAL.md
codex/pipeline/PIPELINE_MAESTRO.md
codex/pipeline/fase_5_poses_drones_sin_gt/pipeline_fase_5_RESUMEN.md
codex/pipeline/fase_5_poses_drones_sin_gt/pipeline_fase_5.md
codex/pipeline/fase_5_poses_drones_sin_gt/subfases/subfase_5A.md
codex/pipeline/fase_5_poses_drones_sin_gt/subfases/subfase_5B.md
codex/pipeline/fase_5_poses_drones_sin_gt/subfases/subfase_5E.md
codex/pipeline/fase_5_poses_drones_sin_gt/subfases/subfase_5F.md
```

Leer historiales reales de 5B/5E/5F y la decisión final de suavizado tomada por el usuario en 5F.

Documentación:

```text
codex/contexto/paquetes/dron_individual/control.md
codex/contexto/paquetes/dron_individual/trayectorias.md
codex/contexto/paquetes/orbslam3_ros2/stereo_slam_node.md
codex/contexto/paquetes/simulacion_dron/graficas_y_gui.md
codex/contexto/paquetes/simulacion_dron/modelos.md
```

5A debe haber fijado el topic de pose local/body, el estado de tracking y el punto exacto donde integrar el estimador de velocidad.

## Diagnóstico de partida

En el baseline entregado:

- `control_calcular_fuerzas` se suscribe a `sensor/GT/vel` como `geometry_msgs/TwistStamped`;
- usa `twist.linear.x/y/z` como velocidad lineal en world;
- transforma `twist.angular` desde world a body mediante `R_act.transpose()`;
- `gen_tray` usa velocidad lineal inicial y `twist.angular.z` como velocidad inicial de yaw;
- `control_calcular_fuerzas` declara una variable `x_ddot`, pero la aceleración actual no participa en el cálculo funcional;
- `sensor/GT/acc` existe en simulación y el código de gráficas contiene callbacks de aceleración, por lo que puede usarse como referencia externa;
- no existe IMU ni otra fuente de pose/velocidad independiente de ORB-SLAM3 para recuperación cuando tracking está perdido.

5E debe haber proporcionado una pose local de cuerpo coherente y 5F debe haber fijado si la pose global que se usa finalmente es raw o smoothed.

Un error de diseño a evitar es:

```text
v = (pose_global(t) - pose_global(t-1)) / dt
```

si `pose_global` puede cambiar por optimización. Un salto del mapa no es movimiento físico y no debe convertirse en una velocidad falsa.

## Archivos permitidos a modificar

5A debe fijar paths. Baseline probable:

```text
src/dron_individual/src/<estimador_pose_velocidad>*
src/dron_individual/include/<estimador_pose_velocidad>*
src/dron_individual/config/*
src/dron_individual/launch/*

src/dron_individual/src/control_tray/*recovery*      # solo si 5A decidió un componente separado
src/dron_individual/src/control_tray/gen_tray.cpp   # integración de cancelación/recovery ya acordada en 5B
src/dron_individual/src/control_tray/control_calcular_fuerzas.cpp  # solo hook de recovery si imprescindible; sustitución normal final en 5H

src/simulacion_dron/src/graficar/graficar.py
src/simulacion_dron/src/graficar/*vel*
src/simulacion_dron/src/graficar/*acc*
src/simulacion_dron/src/control_tray/scenario_runner_node.cpp
src/simulacion_dron/config/*
src/simulacion_dron/launch/*
codex/archivos_auxiliares/trayectorias/*

codex/contexto/paquetes/dron_individual/
codex/contexto/paquetes/simulacion_dron/
codex/pipeline/fase_5_poses_drones_sin_gt/
```

Si el estimador de pose de 5E vive en otro paquete Dron, implementar velocidad junto a él o en un componente claramente dependiente, evitando duplicar timestamps/frames.

## Archivos prohibidos

```text
ORB_SLAM3/**
src/orbslam3_multi/**
src/orbslam3_server/**
orbslam3_msgs/**                    # salvo interfaz de estado ya prevista por 5A
build/**
install/**
log/**
```

No usar GT como entrada de velocidad, aceleración o recuperación.

## Funciones, clases o nodos que hay que localizar

5A/5E deben haber fijado nombres definitivos. Baseline:

```text
<nodo estimador de pose de 5E>
callback de pose local/body
callback de tracking state

Clase_Publisher::callback_vel          # control_calcular_fuerzas baseline
Clase_Publisher::callback_pose
Clase_Publisher::enviar_fuerzas

Clase_Servicio_Accion::execute         # gen_tray baseline

scenario_runner_node
graficar.py
graficar_GT.cpp
graficar_GTvsTray.cpp
plugin_sensor_groundtrurh
```

Topics a fijar:

```text
<local_body_pose_topic>
<estimated_velocity_topic>
<estimated_acceleration_debug_topic>
<localization_status_topic>
sensor/GT/vel        # Simulación, métrica únicamente
sensor/GT/acc        # Simulación, métrica únicamente
```

## Cambios requeridos

### A. Velocidad lineal

1. Mantener al menos las dos últimas muestras válidas de pose local con timestamps originales:

```text
L_T_body(t0)
L_T_body(t1)
```

2. Calcular el incremento de traslación con `dt` real del mensaje, rechazando `dt<=0`, timestamps repetidos, gaps absurdos o muestras no finitas.

3. Estimar velocidad local mediante diferencia temporal o un método causal equivalente:

```text
v_L ~= (p_L(t1) - p_L(t0)) / dt
```

4. Evitar usar posición global corregida como origen de la derivada. Una nueva `C_KF` puede cambiar la posición global sin que el dron se haya movido.

5. Expresar la velocidad en el frame que espera el controlador. Si el controlador necesita `v_W`, usar la rotación válida local→world:

```text
v_W = R_WL * v_L
```

   Una traslación global no afecta a la velocidad. Si el sistema está `LOCAL_ONLY`, conservar una velocidad coherente en `map_local` y etiquetar el frame.

### B. Velocidad angular

6. No estimar solo yaw si el controlador consume `wx/wy/wz`. A partir de dos orientaciones locales válidas:

```text
R0
R1
Delta_R = R0^T * R1
```

   obtener el incremento angular mediante una representación robusta —quaternion delta, log SO(3) o equivalente— y dividir por `dt`.

7. Definir claramente el frame de `omega`. Si el output es world, mantener la convención que espera el controlador antes de su transformación a body. No mezclar body/world entre componentes.

8. Tratar correctamente wrap de yaw y cuaterniones `q`/`-q`; no generar spikes por representación equivalente.

### C. Filtrado causal y delay

9. Aplicar un filtro causal pequeño porque diferenciar amplifica ruido. El filtro debe ser configurable y medido; no introducir una ventana grande que retrase el control.

10. Empezar con la estrategia fijada por 5A o una de primer orden/ventana corta acordada. Registrar parámetros en YAML y no hardcodearlos.

11. Publicar también una versión raw de velocidad para diagnóstico si el coste es despreciable, de forma análoga a 5F, al menos durante desarrollo.

12. Medir delay del filtro y error contra GT. Si el filtro reduce ruido pero introduce un retraso claramente perjudicial, volver a ajustar y presentar resultados.

### D. Aceleración opcional de debug

13. Añadir parámetro:

```yaml
debug_acc_est: false
```

14. Con `false`:
   - no derivar velocidad para obtener aceleración adicional;
   - no publicar topic de aceleración debug, salvo un status estático si se requiere;
   - no reservar buffers costosos innecesarios.

15. Con `true`, calcular:

```text
a ~= (v(t1) - v(t0)) / dt
```

   con filtrado/validación apropiados y publicar solo con propósito diagnóstico.

16. Si se calcula aceleración angular debug, distinguirla de la aceleración lineal y no asumir que el controlador la consume.

### E. Gráficas y métricas

17. Crear gráfica por dron de:

```text
vx_est vs vx_GT
vy_est vs vy_GT
vz_est vs vz_GT
wx_est vs wx_GT
wy_est vs wy_GT
wz_est vs wz_GT
```

18. Calcular:

```text
RMSE/MAE/máximo por componente
norma de error lineal
norma de error angular
número de muestras
frecuencia
processing/filter delay
```

19. Con `debug_acc_est=true`, crear gráfica:

```text
ax_est vs ax_GT
ay_est vs ay_GT
az_est vs az_GT
```

   y métricas equivalentes. No utilizar esa comparación para realimentar el estimador online.

20. Correlacionar por timestamp como en 5F. No comparar muestras GT/estimadas desalineadas solo por llegada.

### F. Recuperación ciega cuando ORB está LOST

21. Mantener continuamente un buffer corto y acotado de estados válidos inmediatamente anteriores a la pérdida, incluyendo cuando sea posible:

```text
timestamp
pose local
velocidad lineal estimada
velocidad angular estimada
modo/frame
```

22. Cuando 5B declare `LOCALIZATION_LOST`:
   - cancelar/interrumpir el goal normal;
   - congelar el buffer de pre-pérdida;
   - marcar la pose/velocidad posteriores como no observadas/no válidas;
   - no seguir publicando una extrapolación como estado fiable.

23. Estimar la dirección reciente de movimiento usando varias muestras, no un único sample si hay suficiente historial. El objetivo es identificar de dónde venía el dron justo antes de dejar de ver landmarks.

24. Probar al menos dos estrategias de recuperación:

```text
Estrategia A: inversión directa/acotada
  v_recovery ~= -k * v_recent
  omega_recovery ~= -k_omega * omega_recent

Estrategia B: frenado breve + inversión
  desacelerar movimiento reciente
  -> aplicar movimiento opuesto acotado
```

   `k`, `k_omega`, duración y límites no se fijan por intuición: se parametrizan y se comparan en simulación.

25. No invertir ciegamente todos los ejes de la misma forma. Tratar por separado:
   - movimiento horizontal;
   - vertical Z;
   - yaw/rotación;
   y decidir mediante pruebas qué componentes conviene revertir en cada escenario.

26. Si la velocidad reciente es aproximadamente cero, usar el historial de desplazamiento/orientación previo para intentar volver hacia la última zona visual útil. Si tampoco existe historial útil, no inventar una dirección y no iniciar una traslación ciega arbitraria.

27. La ejecución física de recovery debe usar el mecanismo mínimo que 5A haya demostrado viable con el controlador actual. Si se necesita una breve propagación/modelo open-loop para aplicar el comando sin pose medida, debe quedar etiquetada como predicción no válida y limitada estrictamente en tiempo.

28. No considerar que el dron “ha vuelto a la pose anterior”. La maniobra solo busca volver a entrar en una zona con features.

29. Mientras recovery está activo, seguir procesando imágenes ORB continuamente.

30. En el primer instante en que ORB recupere tracking válido:
   - abortar inmediatamente el comando ciego;
   - descartar la predicción open-loop como autoridad;
   - volver al estado local observado;
   - no reanudar automáticamente la trayectoria cancelada;
   - dejar que Fase 3/4 recupere global si puede reconocer el submapa/anchor.

31. Añadir límites de seguridad configurables:

```text
max_recovery_time
max_recovery_command
max_reverse_speed
max_reverse_yaw_rate
max_attempts
```

   Los nombres/unidades exactos deben fijarse en 5A/5G. No permitir recuperación ciega indefinida.

32. Si se agotan límites sin recuperar ORB, mantener `LOCALIZATION_LOST` y cesar exploración normal. No seguir recorriendo el entorno a ciegas.

33. Añadir markers equivalentes a:

```text
[F5G-VEL] frame=... raw=... filtered=...
[F5G-VEL-METRICS] rmse=... p95_delay=...
[F5G-ACC-DEBUG] enabled=true ...
[F5G-RECOVERY-START] strategy=... recent_v=... recent_w=...
[F5G-RECOVERY-CMD] ...
[F5G-RECOVERY-ORB-RECOVERED] elapsed=...
[F5G-RECOVERY-STOP] reason=tracking_recovered|timeout|no_history|limit
```

34. Después de las pruebas, presentar al usuario resultados de las estrategias A/B de recuperación. Si una variante no es claramente mejor, no ocultar la ambigüedad; mantener la decisión explícita en historial/configuración.

## Cambios prohibidos

- No usar `sensor/GT/vel` como velocidad funcional.
- No usar `sensor/GT/acc` para recovery ni para filtrar online.
- No derivar velocidad funcional de saltos de pose global.
- No suponer `dt` constante si los mensajes tienen timestamp.
- No ignorar wrap de yaw/cuaterniones.
- No activar `debug_acc_est` por defecto si no es necesario.
- No hacer que el cálculo de aceleración debug afecte la frecuencia de pose/velocidad cuando está desactivado.
- No publicar durante `LOST` una pose extrapolada como si fuese observada/fiable.
- No intentar recorrer una trayectoria larga a ciegas.
- No reproducir varios segundos de trayectoria al revés sin límites.
- No invertir Z automáticamente sin probar su efecto.
- No seguir recovery después de que ORB recupere tracking.
- No reanudar automáticamente el goal cancelado.
- No cambiar ganancias del controlador para ocultar una mala estimación de velocidad.

## Paquetes a compilar

Baseline si el estimador está en `dron_individual`:

```bash
./codex/herramientas/build_selected_packages.sh dron_individual simulacion_dron
```

Añadir el paquete Dron real del estimador si es distinto.

No compilar servidor salvo una corrección mecánica de contrato ya autorizada.

## Pruebas Gazebo requeridas

### Prueba 1 — Velocidad lineal en movimiento simple

Ejecutar movimientos con dirección conocida:

```text
X
Y
Z
combinado XY
```

Comparar `v_est` contra GT solo en nodo de métricas. Incluir aceleración/desaceleración, no solo velocidad constante.

### Prueba 2 — Velocidad angular

Ejecutar:

```text
yaw positivo
yaw negativo
combinación traslación + yaw
```

Comprobar que no hay spike artificial al cruzar ±π y que `wx/wy/wz` mantienen la convención acordada.

### Prueba 3 — Corrección global durante movimiento

Provocar una actualización/optimización de pose global mientras el dron se mueve.

Éxito específico: la velocidad estimada no presenta un pico proporcional al salto de traslación global, porque se deriva de la pose local.

### Prueba 4 — Comparación de filtro

Comparar raw vs filtered en una misma ejecución o ejecuciones reproducibles.

Medir:

```text
RMSE
ruido
pico máximo
latencia/lag
```

No fijar un filtro final sin evidencia.

### Prueba 5 — Aceleración debug OFF/ON

**OFF:**

```yaml
debug_acc_est: false
```

Comprobar que no se ejecuta/publica el cálculo adicional.

**ON:**

```yaml
debug_acc_est: true
```

Comparar aceleración lineal estimada con `sensor/GT/acc` y guardar métricas.

### Prueba 6 — Pérdida ORB durante traslación

Ejecutar primero la estrategia A y después la B con el mismo escenario de pérdida visual.

Para cada una medir externamente con GT:

```text
tasa de recuperación ORB
tiempo hasta recuperar
desplazamiento durante LOST
velocidad máxima
aceleración máxima
picos de fuerza/torque si están disponibles
```

GT no participa en la maniobra.

### Prueba 7 — Pérdida ORB durante yaw

Provocar `LOST` mientras el dron gira. Comprobar si invertir yaw reciente ayuda a volver a la zona con landmarks y que no se introducen traslaciones arbitrarias.

### Prueba 8 — Pérdida con velocidad casi cero

1. dejar al dron casi parado;
2. perder features por orientación/escena;
3. comprobar que usa historial previo si existe;
4. si no existe historial útil, comprobar `reason=no_history` y ausencia de traslación ciega inventada.

### Prueba 9 — Pérdida prolongada no recuperable

Mantener una escena sin landmarks más allá de los límites.

Comprobar:

```text
recovery termina por timeout/limit
LOCALIZATION_LOST permanece
no se ejecutan nuevos goals normales
no se siguen enviando comandos ciegos indefinidamente
```

### Prueba 10 — Prueba larga con pérdidas/relocalización

Integrar varias maniobras en un recorrido largo y provocar al menos una pérdida recuperable. Registrar velocity metrics y recovery events sin alterar Fase 3/4.

## Patrones de reducción de logs

### Velocidad/aceleración

```text
F5G-VEL|F5G-VEL-METRICS|F5G-ACC-DEBUG|debug_acc_est|GT|map_epoch|correction|ERROR|FATAL|Segmentation fault|Killed
```

### Recuperación

```text
F5G-RECOVERY-START|F5G-RECOVERY-CMD|F5G-RECOVERY-ORB-RECOVERED|F5G-RECOVERY-STOP|TRACKING|LOST|RELOCAL|GOAL|RESULT|ERROR|FATAL|Segmentation fault|Killed
```

Reducir por estrategia A/B y por prueba; no mezclar conclusiones de ejecuciones distintas.

## Criterio de éxito

La subfase se considera `CONSEGUIDA` solo si:

1. velocidad lineal y angular se calculan sin GT desde poses locales/timestamps;
2. los frames de `TwistStamped` son coherentes con lo que espera el controlador;
3. una corrección global no se convierte en un pico falso de velocidad;
4. el filtro elegido tiene error/delay medidos y no degrada materialmente la frecuencia;
5. existen gráficas y métricas de velocidad vs GT;
6. `debug_acc_est=false` evita el cálculo adicional;
7. `debug_acc_est=true` produce aceleración diagnóstica comparable con GT;
8. la pérdida total cancela navegación normal y no publica pose stale como válida;
9. la recuperación usa solo estado previo no-GT, está limitada y se aborta en cuanto vuelve tracking;
10. se comparan al menos inversión directa y frenado+inversión en condiciones reproducibles;
11. existe comportamiento definido para `v≈0` y para ausencia de historial útil;
12. un timeout detiene la recuperación ciega prolongada;
13. GT solo se usa para medir resultados externos;
14. build y todas las pruebas obligatorias pasan o, si la calidad no es suficiente, se marca `PARCIAL` y se itera;
15. historial y documentación quedan actualizados.

## Criterio de fallo o parcial

- `NO CONSEGUIDA`: velocidad funcional usa GT, los saltos de mapa generan velocidades falsas, recovery usa GT/pose stale como medida actual, o la maniobra ciega puede continuar sin límites.
- `PARCIAL`: cálculo de velocidad correcto pero error/delay aún alto; aceleración debug no es fiable; o recovery funciona solo en algunos escenarios y requiere ajuste antes de 5H/5I.
- `BLOQUEADA`: la pose local no tiene timestamps/frames utilizables o el controlador no ofrece ningún mecanismo viable para aplicar una maniobra ciega corta sin una nueva decisión funcional.

No pasar a 5H hasta que el usuario haya visto las métricas de 5F/5G y considere que pose y velocidad son suficientemente buenas para sustituir GT.

## Documentación a actualizar

```text
codex/contexto/00_CONTEXTO_COMPACTACION.md
codex/contexto/paquetes/dron_individual/
codex/contexto/paquetes/simulacion_dron/
codex/pipeline/fase_5_poses_drones_sin_gt/pipeline_fase_5_RESUMEN.md
codex/pipeline/fase_5_poses_drones_sin_gt/historial/INDEX.md
codex/pipeline/fase_5_poses_drones_sin_gt/historial/por_subfase/historial_5G.md
codex/pipeline/fase_5_poses_drones_sin_gt/historial/por_subfase/historial_5G_RESUMEN.md
```

Registrar por separado cada estrategia de recovery y sus métricas. No borrar una estrategia fallida cuando otra funcione mejor.
