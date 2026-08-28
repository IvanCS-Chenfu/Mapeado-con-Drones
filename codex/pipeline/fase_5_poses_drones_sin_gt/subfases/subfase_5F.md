# Subfase 5F — Validación cuantitativa de pose

## Estado

```text
PARCIAL; instrumentacion validada, analisis por revision pendiente y puerta humana pendiente
```

## Objetivo

Validar experimentalmente el estimador de 5E antes de conectarlo al control,
demostrando la separación:

```text
O = control continuo
W = global corregible
```

GT se usa solo como referencia externa de métricas. Esta subfase no obliga a
implementar smoothing.

## Comparación correcta

`O_T_B` se compara con GT transformado/alineado al mismo frame; como `O` puede
tener origen arbitrario, no se usa directamente error world.

`W_T_B` se compara con GT world/body, aplicando la extrínseca correcta.

Métricas mínimas:

```text
posición: RMSE, MAE, p95, máximo
orientación: yaw y error angular útil
temporal: frecuencia, inter-arrival, jitter y latencias desde image_stamp
eventos: reference switch, revision, loop, fiducial, Local BA, map_epoch
```

La prueba fundamental observa una corrección global significativa:

```text
W_T_B cambia
O_T_B no sufre salto artificial
```

## Instrumentación

Reutilizar las gráficas de `simulacion_dron`, namespaced por dron y
correlacionadas por timestamp, no por índice. Registrar al menos `drone_id`,
`pose_source`, epoch, reference KF, revisión, poses O/W/GT y tracking.

La entrega acordada contiene ambas capas:

1. informe numérico automático por dron y agregado;
2. gráficas temporales O/W/GT, errores, revisiones, cambios de reference KF y
   validez global.

El emparejamiento temporal usa timestamps y reporta skew, muestras emparejadas
y descartadas. Para comparar O, cada epoch obtiene una alineación GT->O fija;
esa alineación no se recalcula en cada frame ni oculta deriva o saltos.

Ámbitos:

```text
simulacion/simulacion_dron/src/graficar/**
simulacion/simulacion_dron/config/**
simulacion/simulacion_dron/launch/**
instrumentación mínima del estimador
```

No realimentar GT, tocar control normal ni cambiar el optimizador para mejorar
una gráfica.

## Validacion visual RViz2 acordada

Un visualizador independiente consume el `NavigationState` namespaced de cada
dron y publica sus poses globales estimadas como `MarkerArray`:

```text
/dron_1/orbslam/navigation_state
/dron_2/orbslam/navigation_state
                 -> /global_drone_poses
```

RViz2 la representa como un sistema de referencia situado en `W_T_B`, con la
convencion habitual X rojo, Y verde y Z azul, y una etiqueta pequena con el
nombre `drone_1` o `drone_2`. El `frame_id` es el frame global W usado por los
KFs, para observar directamente si la pose sigue al grafo y se actualiza al
cambiar `pose_revision` tras una optimizacion.

La visualizacion solo esta vigente con `global_valid=true`. Al perder autoridad
global se ocultan ejes y etiqueta en vez de congelar una pose obsoleta. No usa
GT, no publica un TF `world -> body`, no conserva un trail historico y no
introduce smoothing ni cambios de control. GT puede permanecer disponible para
las metricas externas separadas, pero no forma parte de esta comprobacion
visual.

La prueba visual usa exactamente
`prueba_tipica_rodeo_edificio_dos_fiduciales.yaml`. Para que el recorrido no
dependa de que exista un anchor antes del primer movimiento, activa únicamente
en esa simulación `use_legacy_gt_goal_policy_for_simulation=true`: todos los
goals se generan y siguen con GT legacy, mientras `W_T_B` se calcula y muestra
de forma independiente. El parámetro conserva default `false` y no adelanta la
navegación global de 5H.

## Suavizado

No implementarlo por defecto. Solo abrir un experimento separado si las
métricas revelan una necesidad concreta de un consumidor global. Nunca usarlo
para corregir `O` o tapar mezcla de frames.

## Pruebas acordables

1. recorrido corto: calidad, frecuencia y latencia;
2. recorrido largo: cambios de KF, Local BA y deriva;
3. fiducial/loop/optimización: corrección global frente a continuidad local;
4. dos drones cuando la instrumentación namespaced sea estable.

Ejecución integrada acordada: una trayectoria larga de dos drones con control
GT legacy, anchors y loops reales, varios reference KF y revisiones globales.
Si no aparece de forma natural una corrección global significativa, se permite
una prueba dirigida adicional que fuerce una revisión por la API/backend, sin
usar GT como entrada funcional del estimador.

Patrones iniciales:

```text
F5F|POSE_METRIC|CONTROL_JUMP|GLOBAL_JUMP|REFERENCE_SWITCH|REVISION|LATENCY|JITTER|RMSE|MAE|GT|ERROR|FATAL
```

## Puerta y criterio

Codex presenta las métricas y se detiene. No se avanza a integración hasta que
el usuario acepte la calidad.

Éxito requiere métricas completas, ausencia de saltos artificiales en `O`,
global coherente con GT, latencia/frecuencia medidas y aceptación explícita.
No se fijan umbrales numéricos sin evidencia y acuerdo.

La ampliacion RViz2 debe demostrar ademas que, mientras hay autoridad global,
cada sistema XYZ coincide con `W_T_B`, queda identificado por dron y responde a
una revision global sin modificar `O_T_B`. Su implementacion y prueba concreta
requieren una preparacion y autorizacion funcional posteriores.

El bloque 5C+5D+5E+5F termina aquí. La aceptación humana posterior no autoriza
automáticamente 5G ni 5H; esas subfases conservan su preparación propia.
