# Subfase 5F — Validación cuantitativa de pose

## Estado

```text
sin hacer; puerta humana obligatoria antes de 5G/5H
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

Ámbitos:

```text
simulacion/simulacion_dron/src/graficar/**
simulacion/simulacion_dron/config/**
simulacion/simulacion_dron/launch/**
instrumentación mínima del estimador
```

No realimentar GT, tocar control normal ni cambiar el optimizador para mejorar
una gráfica.

## Suavizado

No implementarlo por defecto. Solo abrir un experimento separado si las
métricas revelan una necesidad concreta de un consumidor global. Nunca usarlo
para corregir `O` o tapar mezcla de frames.

## Pruebas acordables

1. recorrido corto: calidad, frecuencia y latencia;
2. recorrido largo: cambios de KF, Local BA y deriva;
3. fiducial/loop/optimización: corrección global frente a continuidad local;
4. dos drones cuando la instrumentación namespaced sea estable.

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
