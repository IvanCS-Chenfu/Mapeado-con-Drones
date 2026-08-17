# `Optimizer`

## Rol

`Optimizer` agrupa las optimizaciones g2o utilizadas por tracking, mapping,
inicialización, loops y merge.

Archivos:

```text
ORB_SLAM3/include/Optimizer.h
ORB_SLAM3/src/Optimizer.cc
```

## Familias principales

### Pose del frame actual

```cpp
PoseOptimization(...)
```

Ajusta el frame de tracking actual. Debe mantenerse para que ORB-SLAM3 funcione
como frontend visual.

### Mapa local

```cpp
LocalBundleAdjustment(...)
LocalInertialBA(...)
```

Ajustan una ventana local de KeyFrames y MapPoints. Los invoca `LocalMapping`.

### Corrección de loop

```cpp
OptimizeEssentialGraph(...)
OptimizeEssentialGraph4DoF(...)
```

Propagan la restricción de loop por el grafo y modifican poses históricas.
Los invoca `LoopClosing::CorrectLoop()`.

### Ajuste global

```cpp
GlobalBundleAdjustemnt(...)
FullInertialBA(...)
```

Pueden ajustar gran parte o todo el mapa. `LoopClosing` puede lanzarlos en un
hilo después de corregir un loop.

### Merge

Las rutas de merge usan optimización local de welding, essential graph y, en
algunos casos, Global BA. También pueden modificar grandes regiones del Atlas.

## Autoridad recomendada

Para el modo frontend del proyecto:

| Operación | Política |
|---|---|
| `PoseOptimization` | mantener |
| Local BA | mantener inicialmente |
| Essential graph por loop | desactivar |
| Global BA posterior a loop | desactivar |
| Merge interno de mapas | desactivar |

La detección y la aplicación son separables: se puede ejecutar place
recognition y evitar las llamadas de corrección global.

## Advertencia

Desactivar todas las funciones de `Optimizer` no deja ORB-SLAM3 funcionando
igual. En particular, eliminar `PoseOptimization` rompe la estimación visual y
el servidor no puede reemplazarla con su pose graph global.
