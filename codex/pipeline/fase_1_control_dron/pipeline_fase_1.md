# Pipeline Fase 1 — Control del dron en simulación

Resumen de entrada:

```text
codex/pipeline/fase_1_control_dron/pipeline_fase_1_RESUMEN.md
```

## Estado

```text
realizado
```

La fase se documenta con estado `realizado` porque el proyecto de referencia contiene implementaciones para sus nueve bloques. Este documento no sustituye la evidencia de build o simulación y no añade resultados al historial vacío.

## Objetivo general

Construir de forma incremental el entorno de simulación y la cadena de control de un dron:

1. arrancar Gazebo desde un launch;
2. definir el cuerpo físico mediante Xacro/URDF y YAML;
3. insertar una o varias instancias con namespaces independientes;
4. aplicar fuerza y torque a cada motor mediante un plugin Gazebo-ROS 2;
5. publicar Ground Truth e imágenes de cámara;
6. generar referencias temporales con `lib_tray`;
7. calcular fuerza/torque y repartirlos entre cuatro motores;
8. enviar objetivos a cada dron desde una GUI y observar el movimiento en Gazebo;
9. representar perfiles, estado real y errores de seguimiento mediante gráficas.

## Fuentes de referencia

Orden de autoridad para esta fase:

1. código, launch, Xacro, YAML, interfaces y CMake de `simulacion_dron`, `dron_individual` y `lib_tray`;
2. documentación vigente de paquetes en `codex/contexto/paquetes/`;
3. wiki del proyecto como explicación histórica, no como prueba de funcionamiento;
4. este pipeline y los contratos de subfase.

La wiki de consulta es:

```text
https://github.com/IvanCS-Chenfu/TFG/wiki/Explicaci%C3%B3n
```

## Arquitectura final de la fase

```text
sim_dron.yaml
  -> launch Gazebo + mundo + factory
  -> multi_dron.launch.py
      -> namespace dron_i
      -> generador_URDF
          -> hardware.yaml
          -> dron_plugins.xacro
          -> /spawn_entity
      -> generar_dron.launch.py
          -> gen_tray /AccionTrayectoria
          -> control_calcular_fuerzas
          -> aplicar_fuerzas_dron

simulacion_dron/src/graficar
  -> adaptadores GT / TrayAction / GT-vs-Tray
  -> numeric_array + labels_array
  -> graficar.py / Matplotlib

Gazebo model
  -> plugin_sensor_groundtrurh
      -> sensor/GT/pose
      -> sensor/GT/vel
      -> sensor/GT/acc
  -> libgazebo_ros_camera.so
      -> sensor/camara_*/image_raw
      -> sensor/camara_*/camera_info
  -> plugin_actuar_motores
      <- motor/*
```

## Política de configuración física

Los valores del repositorio son un baseline arbitrario de simulación. No existe todavía un dron físico del que procedan las masas, inercias, dimensiones o constantes.

Antes de probar cada subfase se debe revisar el YAML correspondiente. No está permitido:

- copiar valores sin unidades;
- presentar los valores de ejemplo como datos reales;
- mantener constantes incompatibles entre Xacro y control;
- usar vectores con tamaños incorrectos;
- ocultar en código valores que deberían configurarse;
- suponer que seis u ocho motores están controlados porque el Xacro los dibuja.

La masa total y la inercia usadas por el controlador deben ser coherentes con el modelo físico. Cuando exista hardware real, esos valores se sustituirán por medidas, fichas técnicas o cálculos documentados.

## Subfases

| Subfase | Nombre | Paquetes | Puerta de salida |
|---|---|---|---|
| `1A` | Entorno base de Gazebo | `simulacion_dron` | Gazebo y el mundo arrancan desde launch; factory y reloj disponibles. |
| `1B` | Modelo físico Xacro/URDF | `simulacion_dron`, configuración de `dron_individual` | URDF válido y coherente para el cuadricóptero de cuatro motores. |
| `1C` | Generación e inserción multi-dron | `simulacion_dron`, `dron_individual` | Una o varias entidades únicas aparecen bajo namespaces independientes. |
| `1D` | Plugin de fuerzas y torques | `simulacion_dron` | Comandos manuales a `motor/*` mueven el cuadricóptero. |
| `1E` | Ground Truth y cámaras | `simulacion_dron` | GT se publica y las imágenes mono/estéreo se visualizan en RViz2. |
| `1F` | Librería `lib_tray` | `lib_tray` | Pol3, veltrap y elipse tienen tests deterministas automáticos. |
| `1G` | Acción y control del dron | `dron_individual`, `lib_tray`, `simulacion_dron` | El cuadricóptero sigue trayectorias usando GT como estado temporal. |
| `1H` | GUI de simulación multi-dron | `simulacion_dron`, `dron_individual` | La GUI selecciona un dron, envía un goal y se observa su movimiento. |
| `1I` | Gráficas de trayectorias, estado y error | `simulacion_dron`, `dron_individual`, `lib_tray` | Se visualizan perfiles, GT, referencia y errores sin mezclar drones ni magnitudes. |

## Dependencias

```text
1A -> 1B -> 1C -> 1D -> 1E -> 1F -> 1G -> 1H -> 1I
```

- `1B` puede validar Xacro sin Gazebo, pero la integración visual depende de `1A`.
- `1D` necesita que el modelo ya se inserte.
- `1G` necesita GT de `1E`, `lib_tray` de `1F` y actuación de `1D`.
- `1H` no implementa una GUI operacional del sistema global; esa función pertenece a la Fase 7.
- `1I` usa GT como referencia de simulación y depende del feedback de `1G`; en Fase 5 la entrada funcional de pose/velocidad deberá dejar de ser GT.

## Alcance de cuatro, seis y ocho motores

El modelo físico y `plugin_actuar_motores` pueden declarar 4, 6 u 8 links/topics. El requisito obligatorio de esta fase es:

```text
fisico.brazos.numero: 4
```

El mixer actual `aplicar_fuerzas_dron` publica solo:

```text
motor/arr_iz
motor/ab_iz
motor/ab_der
motor/arr_der
```

Por ello:

- cuatro motores deben funcionar de extremo a extremo;
- seis y ocho pueden existir como variantes de modelo o actuación manual;
- no se exige mixer, estabilidad ni trayectoria controlada para seis u ocho;
- ninguna prueba de seis u ocho puede utilizarse para declarar control completo.

## Uso de Ground Truth y transición futura

Fase 1 admite GT para construir y validar el control simulado. Los puntos de entrada vigentes son:

```text
gen_tray                 <- sensor/GT/pose + sensor/GT/vel
control_calcular_fuerzas <- sensor/GT/pose + sensor/GT/vel
```

En Fase 5 se deberán sustituir estas entradas por pose y velocidad estimadas sin GT. No corresponde realizar esa sustitución en esta fase.

## Reglas de prueba

1. Compilar únicamente paquetes afectados con `build_selected_packages.sh`.
2. Ejecutar tests de componente antes de simulaciones largas.
3. Usar launch y YAML exactos en cada prueba.
4. Guardar logs completos como artefactos, pero leer solo reducidos.
5. Registrar observaciones visuales cuando el criterio dependa de Gazebo o RViz2.
6. No escribir una ejecución en este contrato; la evidencia futura va a `historial/por_subfase/`.
7. No declarar compatibilidad con hardware real mientras los parámetros sean arbitrarios.

## Resultado de la fase

La salida funcional es una línea base de simulación en la que un cuadricóptero de cuatro motores puede recibir una trayectoria, generar referencias, calcular esfuerzos, repartir consignas y moverse en Gazebo. También dispone de instrumentación para representar perfiles deseados, estado GT temporal, comparaciones y errores de seguimiento. La salida documental localiza todos los YAML, topics, actions, frames y dependencias de GT necesarios para reproducirla y sustituirla en fases posteriores.
