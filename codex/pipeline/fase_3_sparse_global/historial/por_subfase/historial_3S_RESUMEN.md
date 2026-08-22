# Historial 3S - resumen

## Estado

```text
CONSEGUIDA
```

`simulacion_dron/config/fase3_debug.yaml` controla RViz2, grafo web, apertura
del navegador y diagnosticos `[F3*]`. Los cuatro valores quedan en `false` y
pueden sobrescribirse mediante argumentos launch homonimos. Los errores reales
permanecen visibles mediante nivel ROS `ERROR`.

## Evidencia

- build 3/3;
- CTest 9/9 + 10/10 + 8/8;
- contratos configuracion/web 15/15;
- launch instalado muestra los cuatro defaults false;
- prueba 196 `success=true`, cinco pasos y cuatro goals correctos;
- servidor global activo y cierre limpio;
- cero `[F3*]`, cero procesos RViz2/bridge/navegador;
- RSS RViz/web 0.0 MiB y recursos estables.

El exit 255 de Gazebo pertenece al cleanup posterior a `SIM-DONE` y no invalida
la prueba.
