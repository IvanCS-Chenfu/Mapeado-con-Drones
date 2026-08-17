# `LocalMapping`

## Rol

`LocalMapping` mantiene el mapa local alrededor de los KeyFrames recientes.

Archivos:

```text
ORB_SLAM3/include/LocalMapping.h
ORB_SLAM3/src/LocalMapping.cc
```

## Flujo principal

Para cada nuevo KeyFrame:

1. procesa e inserta el KeyFrame;
2. elimina MapPoints locales de baja calidad;
3. triangula nuevos MapPoints;
4. busca y fusiona duplicados en vecinos;
5. ejecuta Local Bundle Adjustment cuando corresponde;
6. realiza culling de KeyFrames;
7. entrega el KeyFrame a `LoopClosing`.

## Local Bundle Adjustment

En mapas visuales no inerciales llama:

```cpp
Optimizer::LocalBundleAdjustment(...)
```

En mapas inerciales llama:

```cpp
Optimizer::LocalInertialBA(...)
```

Estas optimizaciones pueden modificar poses y MapPoints de una ventana local,
pero no deberían producir la corrección global de toda una trayectoria propia
de un loop.

Actualmente no existe una clave de configuración para desactivar Local BA de
forma independiente. Hacerlo requiere una condición explícita alrededor de
estas llamadas.

## Relación con loop closing

`CorrectLoop()` solicita detener temporalmente `LocalMapping` antes de corregir
el grafo y lo libera al finalizar. En logs se observa como:

```text
Local Mapping STOP
Local Mapping RELEASE
```

Con loop closing realmente desactivado, esa parada no debería ocurrir por un
loop.

## Uso como frontend del servidor

La primera política recomendable es:

```text
mantener LocalMapping y Local BA
desactivar corrección de loops y Global BA
```

Local BA mejora la triangulación y la coherencia reproyectiva que el pose graph
actual del servidor no sustituye.

Un experimento posterior puede desactivar Local BA, pero debe medir:

- estabilidad de tracking;
- calidad y cantidad de MapPoints;
- ruido de poses relativas;
- frecuencia de pérdida y creación de nuevos epochs.
