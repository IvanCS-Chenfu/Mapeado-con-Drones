# Subfase 1F — Librería de trayectorias `lib_tray` y tests

## Estado

```text
realizado
```

## Dependencias

No necesita Gazebo para sus tests matemáticos. En la secuencia de fase se ejecuta después de `1E` para preparar el control de `1G`.

## Objetivo técnico

Crear una librería C++ independiente de ROS 2 runtime que genere y evalúe referencias temporales para cuatro ejes conceptuales (`x`, `y`, `z`, `yaw`) mediante:

```text
GenTrayPol3
GenTrayVelTrap
GenTrayElipse
```

Cada evaluación devuelve por eje:

```text
[posición, velocidad, aceleración, jerk, ratio]
```

La librería debe tener tests automáticos deterministas, no solo ejecutables que imprimen valores.

## Contexto obligatorio a leer

```text
AGENTS.md
codex/pipeline/fase_1_control_dron/pipeline_fase_1_RESUMEN.md
codex/contexto/paquetes/lib_tray/00_summary.md
src/lib_tray/include/lib_tray/*.hpp
src/lib_tray/src/generacion/*.cpp
src/lib_tray/test/*.cpp
```

La wiki se consulta solo para la explicación matemática histórica. El código es la autoridad de API y semántica actual.

## Diagnóstico de partida

La implementación de referencia contiene tres librerías estáticas/compartidas de Ament y dos ejecutables de prueba:

```text
test_gen_tray_pol3
test_gen_tray_veltrap
```

Los ejecutables actuales reciben un tiempo y muestran datos por consola, pero no forman una batería `colcon test` con assertions. No hay test equivalente de `GenTrayElipse`.

## API obligatoria

### `GenTrayPol3`

```cpp
GenTrayPol3(std::size_t N_ejes);
calcular_trayectoria(posiciones_iniciales,
                     posiciones_finales,
                     velocidades_iniciales,
                     velocidades_finales,
                     tiempos_finales,
                     finales_absolutas);
evaluar(double t);
```

- `tf > 0` por eje.
- Soporta posición final absoluta o relativa.
- Clampa evaluación a `[0, tf]`.
- Al finalizar deja velocidad, aceleración y jerk según la semántica implementada/testeada.

### `GenTrayVelTrap`

```cpp
GenTrayVelTrap(std::size_t N_ejes);
calcular_trayectoria(... velocidades_maximas, double t_a, finales_absolutas);
evaluar(double t);
```

- `t_a > 0`.
- Eje de distancia cero: `estacionario=true` y `tf=0`.
- Perfil triangular si no se alcanza `v_max`.
- Perfil trapezoidal en caso contrario.
- Debe funcionar para desplazamiento positivo y negativo.

### `GenTrayElipse`

```cpp
GenTrayElipse(4);
calcular_trayectoria(posiciones_iniciales,
                     centro_elipse,
                     parametros_elipse,
                     yaw_absoluto,
                     t_a);
evaluar(double t);
```

Semántica de vectores:

```text
posiciones_iniciales = [x0, y0, z0, yaw0]
centro_elipse        = [xc, yc, zc, yaw_ref]
parametros_elipse    = [rx, ry, alpha, tf]
```

- requiere exactamente 4 ejes;
- `rx > 0`, `ry > 0`, `tf > 0`, `t_a > 0`;
- recorre una vuelta usando tabla de longitud de arco;
- usa perfil triangular o trapezoidal sobre el arco;
- yaw absoluto: mantiene `yaw_ref`;
- yaw relativo: mira al centro con offset.

### Semántica vertical que debe documentarse

El código de referencia calcula:

```text
z_final = z0 + 2 * (zc - z0)
```

Por tanto `zc` actúa como punto medio vertical, no simplemente como altura final. Los tests deben fijar explícitamente esta semántica o, si se decide cambiarla, hacerlo con autorización funcional y actualizar `TrayAction`, GUI y documentación. No asumir silenciosamente que `zc` es el final.

## Archivos permitidos a modificar

```text
src/lib_tray/include/lib_tray/gen_tray_pol3.hpp
src/lib_tray/include/lib_tray/gen_tray_veltrap.hpp
src/lib_tray/include/lib_tray/gen_tray_elipse.hpp
src/lib_tray/src/generacion/gen_tray_pol3.cpp
src/lib_tray/src/generacion/gen_tray_veltrap.cpp
src/lib_tray/src/generacion/gen_tray_elipse.cpp
src/lib_tray/test/
src/lib_tray/CMakeLists.txt
src/lib_tray/package.xml
codex/contexto/paquetes/lib_tray/
```

## Archivos prohibidos

```text
src/dron_individual/src/control_tray/
src/simulacion_dron/
src/orbslam3_multi/
src/orbslam3_server/
```

No adaptar la librería a Gazebo ni leer YAML desde ella.

## Clases y funciones a localizar

```text
GenTrayPol3::calcular_trayectoria
GenTrayPol3::calcular_coeficientes
GenTrayPol3::evaluar_eje
GenTrayVelTrap::calcular_trayectoria
GenTrayVelTrap::calcular_coeficientes
GenTrayVelTrap::evaluar_eje
GenTrayElipse::calcular_trayectoria
GenTrayElipse::calcular_coeficientes
GenTrayElipse::evaluar_trayectoria
GenTrayElipse::evaluar_perfil_arco
theta_desde_tiempo
posicion_yaw
comprobar_tamano_vector
```

## Cambios requeridos

1. Mantener las tres clases como librerías reutilizables.
2. Validar tamaños de vectores y parámetros no permitidos.
3. Garantizar salidas finitas para entradas válidas.
4. Añadir tests automáticos con `ament_cmake_gtest`, Catch2 u otra infraestructura aprobada.
5. Mantener los ejecutables manuales solo si aportan diagnóstico, sin sustituir a los tests.
6. Añadir test específico de elipse.
7. Probar continuidad y valores en puntos de cambio, no solo un tiempo arbitrario.
8. Documentar unidades: posición m/rad, velocidad m/s o rad/s, aceleración, jerk, tiempo s y ratio %.
9. No depender de ROS, Gazebo, GT o reloj simulado.

## Tests automáticos obligatorios

### Constructor y validación

- `N_ejes=0` falla en todas las clases.
- `GenTrayElipse(N!=4)` falla.
- tamaños de vector incorrectos fallan.
- `tf<=0`, `t_a<=0`, radios no positivos y `vmax<=0` para eje móvil fallan.

### Polinomio cúbico

- valores exactos en `t=0` y `t=tf`;
- posición absoluta y relativa;
- velocidades inicial/final;
- evaluación antes de 0 y después de `tf`;
- continuidad de posición, velocidad y aceleración;
- ratio 0–100.

### Perfil trapezoidal

- caso estacionario;
- caso triangular;
- caso trapezoidal;
- desplazamientos positivos y negativos;
- continuidad en `t1` y `t1+tc`;
- valor final y derivadas finales;
- `get_coeficientes()` coherente.

### Elipse

- una vuelta completa vuelve a `x/y` iniciales;
- radios distintos;
- elipse rotada por `alpha`;
- perfil triangular y trapezoidal;
- yaw absoluto;
- yaw hacia el centro;
- semántica vertical actual;
- derivadas finitas cerca de extremos;
- ratio 0–100.

### Rendimiento

Medir evaluación repetida sin convertir un umbral arbitrario en requisito de éxito. Registrar media/máximo solo como información de regresión.

## Paquete a compilar y probar

```bash
./codex/herramientas/build_selected_packages.sh lib_tray
colcon test --packages-select lib_tray
colcon test-result --verbose
```

## Patrones de reducción de logs

```text
lib_tray|GenTrayPol3|GenTrayVelTrap|GenTrayElipse|FAILED|Failure|ERROR|Exception|Segmentation fault|Killed
```

## Criterio de éxito

1. `lib_tray` compila e instala headers y targets.
2. Las tres clases tienen tests automáticos.
3. Todos los casos obligatorios pasan.
4. No aparecen NaN/Inf para entradas válidas.
5. Las excepciones esperadas están cubiertas.
6. La semántica vertical de la elipse queda fijada y documentada.
7. `gen_tray` puede enlazar las tres librerías en `1G`.

## Criterio de fallo o parcial

- `NO CONSEGUIDA`: no compila, falla un test obligatorio o la API no es estable.
- `PARCIAL`: pol3/veltrap pasan, pero elipse o tests automáticos quedan incompletos.
- `BLOQUEADA`: falta una decisión funcional sobre la semántica que impide escribir un criterio determinista.

## Riesgos

- confundir salida impresa con test;
- discontinuidades en cambios de tramo;
- división por cero;
- valores no finitos no validados;
- yaw discontinuo en ±π;
- usar `zc` con significado distinto entre biblioteca, action y GUI.

## Documentación a actualizar al ejecutar

```text
codex/contexto/paquetes/lib_tray/
codex/pipeline/fase_1_control_dron/historial/por_subfase/historial_1F.md
codex/pipeline/fase_1_control_dron/historial/por_subfase/historial_1F_RESUMEN.md
```
