# Subfase 1G — Build, pruebas y criterios

Este archivo complementa `subfase_1G.md`.

## Estado de la subfase

```text
realizado
```

## Paquetes a compilar

```bash
./codex/herramientas/build_selected_packages.sh lib_tray dron_individual simulacion_dron
colcon test --packages-select lib_tray dron_individual simulacion_dron
colcon test-result --verbose
```

## Preparación de YAML

Antes de ejecutar:

1. fijar `fisico.brazos.numero: 4`;
2. calcular masa total a partir de `hardware.yaml` y reflejarla en `tray_dron.yaml`;
3. igualar longitud, ángulo y `fuerza2torque` entre ambos YAML;
4. registrar gains, gravedad, velocidades máximas y `t_a`;
5. comprobar que todos los valores son finitos.

No ajustar gains durante una prueba sin crear una ejecución nueva en historial.

## Test de componente 1 — Action server aislado

Con topics GT sintéticos controlados:

- goal pol3 válido;
- goal veltrap válido;
- goal elipse válido;
- tipo `>2` rechazado;
- tiempos/radios inválidos rechazados o abortados con error;
- cancelación;
- goal nuevo que sustituye al anterior;
- ausencia de GT y timeout/política acordada;
- feedback con arrays de cinco elementos y ratio válido.

## Test de componente 2 — Mixer de cuatro motores

Inyectar combinaciones conocidas de fuerza/torque y comprobar:

- thrust puro produce cuatro valores coherentes;
- torque x/y produce pares opuestos;
- torque z respeta alternancia;
- matriz singular/casi singular usa la ruta esperada;
- no se publican topics de 6/8;
- ausencia de NaN/Inf.

Si no existe API pura para el mixer, se permite extraer la matemática sin cambiar comportamiento, con autorización acordada.

## Pruebas Gazebo obligatorias

Usar `scenario_runner_node` cuando el escenario pueda automatizarse. Cada prueba debe registrar goal, resultado y observación de pose.

### Prueba 1 — Hover/altura

- un dron;
- objetivo vertical seguro;
- yaw constante;
- comprobar ascenso y estabilización aproximada;
- registrar error de posición y ausencia de divergencia.

### Prueba 2 — Traslación pol3

Enviar objetivo en `x/y/z` con tiempos positivos. Verificar que el dron se desplaza hacia el objetivo y la acción termina con `success=true`.

### Prueba 3 — Velocidad trapezoidal

Enviar desplazamiento con `tipo_trayectoria=1`. Comprobar aceleración, tramo central cuando exista y desaceleración, sin saltos graves.

### Prueba 4 — Yaw

Mantener posición y cambiar yaw. Comprobar giro sin pérdida vertical grave.

### Prueba 5 — Elipse

Enviar `tipo_trayectoria=2` con radios, ángulo y tiempo de vuelta válidos. Confirmar recorrido orbital y semántica vertical documentada.

### Prueba 6 — Cancelación y reemplazo

- iniciar una trayectoria larga;
- cancelar y comprobar resultado cancelado;
- iniciar otra y reemplazarla con un goal nuevo;
- el goal anterior no debe terminar como success.

### Prueba 7 — Estado no disponible

Arrancar `gen_tray` sin GT o detener el topic. Verificar la política acordada: timeout/abort/espera explícita. No aceptar bloqueo silencioso indefinido como éxito.

### Prueba 8 — Dos drones aislados

Enviar goals distintos a `/dron_1/AccionTrayectoria` y `/dron_2/AccionTrayectoria`. Cada uno debe recibir su feedback y controlar solo sus motores.

## Comando de simulación base

```bash
./codex/herramientas/run_simulation.sh \
  --prueba fase1_1G_control \
  --launch "ros2 launch simulacion_dron multi_dron.launch.py" \
  --post-scenario-wait-sec 20
```

Los YAML de trayectoria concretos deben guardarse en `codex/archivos_auxiliares/trayectorias/` cuando se ejecute la subfase; no se inventan en este ZIP.

## Observaciones y métricas

Como mínimo registrar:

```text
error inicial/final de posición [m]
error de yaw [rad o deg]
tiempo de acción [s]
frecuencia de feedback [Hz]
frecuencia de control/mixer [Hz]
consignas mínima/máxima por motor [N]
NaN/Inf: sí/no
cancelación y reemplazo: resultado
```

Los umbrales numéricos de seguimiento se deben acordar antes de ejecutar. No inventarlos en el contrato si no hay requisito del usuario.

## Patrones de reducción de logs

```text
SCENARIO-RUNNER|GOAL|RESULT|success|Generando trayectoria|tipo_trayectoria|pose_actualizada|vel_actualizada|Acción CANCELADA|Acción finalizada OK|Error calculando trayectoria|A casi singular|control/tray|motor/|NaN|Inf|ERROR|FATAL|Segmentation fault|Killed
```

## Criterio de éxito

1. Build y tests devuelven éxito.
2. `lib_tray` pasa todos sus tests.
3. Los ocho casos obligatorios se ejecutan.
4. Goals válidos terminan con `success=true`, salvo los cancelados/reemplazados.
5. No hay control cruzado entre namespaces.
6. No aparecen NaN/Inf o singularidades no tratadas.
7. El cuadricóptero de cuatro motores muestra hover, traslación, yaw y elipse coherentes.
8. Los YAML físico y de control son coherentes y se adjuntan a la evidencia.
9. GT se usa solo como estado de Fase 1 y queda señalado para sustitución en Fase 5.
10. No se afirma control 6/8.

## Criterio de fallo o parcial

- `NO CONSEGUIDA`: build falla, una trayectoria obligatoria diverge, goals se gestionan mal, hay NaN/Inf o el mixer controla links incorrectos.
- `PARCIAL`: action y control básico funcionan, pero falta elipse, cancelación, aislamiento o una prueba obligatoria.
- `BLOQUEADA`: falta una dependencia externa o una decisión de seguridad imprescindible que no puede cerrarse durante la ejecución.

## Evidencia a guardar

```text
YAML exactos
comandos y escenario
resultado de colcon test
logs reducidos
métricas
capturas/vídeo Gazebo cuando aplique
conclusión por prueba y agregada
```

Nunca abrir ni pegar el log completo.
