# Subfase 1H — GUI de simulación multi-dron y cierre integral

## Estado

```text
realizado
```

## Dependencias

```text
1C — namespaces y número de drones
1G — action server y control funcional de cuatro motores
```

## Objetivo técnico

Crear una GUI de simulación que lea el número y namespace base de `sim_dron.yaml`, permita seleccionar un dron, construir un goal `TrayAction` y enviarlo al action server namespaced. Probar en Gazebo que se mueve exclusivamente el dron seleccionado.

Esta GUI es una herramienta de prueba de la Fase 1. No es la GUI operacional de mapa, tareas y misiones prevista para la Fase 7.

## Comportamiento esperado

- La GUI descubre `dron.numero` y `dron.namespace_base`.
- Selecciona `dron_1 ... dron_N`.
- Construye `/namespace_i/AccionTrayectoria`.
- Permite elegir pol3, veltrap o elipse.
- Permite introducir posición/yaw, parámetros temporales o elípticos y flags absolutos.
- Envía el goal sin bloquear la interfaz.
- Gazebo muestra el movimiento del dron elegido.
- Los demás drones no se mueven por ese goal.

## Contexto obligatorio a leer

```text
AGENTS.md
codex/pipeline/fase_1_control_dron/pipeline_fase_1_RESUMEN.md
codex/pipeline/fase_1_control_dron/subfases/subfase_1C.md
codex/pipeline/fase_1_control_dron/subfases/subfase_1G.md
codex/contexto/paquetes/simulacion_dron/00_summary.md
codex/contexto/paquetes/dron_individual/00_summary.md
```

## Diagnóstico de partida

La implementación de referencia se encuentra en:

```text
src/simulacion_dron/src/control_tray/gui_tray_multi.py
src/simulacion_dron/launch/multi_dron.launch.py
src/simulacion_dron/config/sim_dron.yaml
```

La GUI actual:

- usa Tkinter y `rclpy`;
- lee el YAML directamente desde el share del paquete;
- crea un `ActionClient` por nombre completo;
- permite seleccionar tipos `0..2`;
- convierte yaw en grados a quaternion Z;
- envía el goal en un thread;
- no muestra actualmente feedback/resultados ni ofrece cancelación explícita;
- usa etiquetas `tx/ty/tz/tyaw` como segundos incluso cuando elipse los interpreta como radios/ángulo/tiempo.

Las carencias de feedback/cancelación pueden tratarse como mejoras opcionales si el objetivo obligatorio de movimiento visual ya se cumple; no deben afirmarse como implementadas sin código y prueba.

## YAML obligatorio

```text
src/simulacion_dron/config/sim_dron.yaml
```

```yaml
dron.numero: 2
dron.namespace_base: "dron"
dron.spawn_box: [x1, x2, y1, y2]
```

La GUI debe utilizar exactamente el mismo esquema de nombre que el launch:

```text
/<namespace_base>_<i>/AccionTrayectoria
```

No debe mantener una lista separada hardcodeada.

## Contrato de interfaz

### Selección

```text
índice válido: 1..N
nombre visible del action completo
```

### Goal común

```text
tipo_trayectoria
x, y, z [m]
yaw [deg en GUI, rad/quaternion en mensaje]
absoluto_x/y/z/yaw
```

### Parámetros por tipo

Pol3:

```text
tx,ty,tz,tyaw = tiempos [s]
```

Veltrap:

```text
los tiempos vienen del YAML de control;
la GUI debe indicar que tx.. no gobiernan el perfil o deshabilitarlos.
```

Elipse:

```text
tx=rx [m]
ty=ry [m]
tz=alpha [rad o deg con conversión explícita]
tyaw=tf [s]
```

Las etiquetas deben adaptarse al tipo o mostrar una ayuda inequívoca.

## Archivos permitidos a modificar

```text
src/simulacion_dron/src/control_tray/gui_tray_multi.py
src/simulacion_dron/launch/multi_dron.launch.py
src/simulacion_dron/config/sim_dron.yaml
src/simulacion_dron/CMakeLists.txt
src/simulacion_dron/package.xml
src/dron_individual/action/TrayAction.action       # solo si existe acuerdo funcional
codex/contexto/paquetes/simulacion_dron/
codex/contexto/paquetes/dron_individual/
```

## Archivos prohibidos

```text
src/orbslam3_multi/
src/orbslam3_server/
ORB_SLAM3/
orbslam3_ros2/
```

No añadir mapa sparse, tareas, reservas, nube densa o comandos de operación general.

## Funciones y clases a localizar

```text
load_sim_cfg
TrayActionGUI
_ns_for_i
_action_full_name
_get_client
_build_gui
_on_drone_change
_on_tipo_trayectoria_change
_on_send
ActionClient
MultiThreadedExecutor
```

## Cambios requeridos

1. Instalar `gui_tray_multi.py` como ejecutable del paquete.
2. Leer `sim_dron.yaml` desde el share instalado.
3. Validar `dron.numero` y `namespace_base`.
4. Crear/cachear `ActionClient` por action completo sin pisar atributos internos de `Node`.
5. Convertir yaw correctamente.
6. Enviar goals en background para no congelar Tkinter.
7. Mostrar el destino namespaced antes de enviar.
8. Adaptar etiquetas y rangos a pol3/veltrap/elipse.
9. Emitir log del goal con tipo, target, flags y action.
10. Gestionar claramente servidor no disponible.
11. Opcional: mostrar aceptación, feedback, resultado y cancelación; si se implementa, probarlo.
12. Integrar la GUI en el launch final sin iniciar una segunda instancia ROS/Gazebo.

## Cambios prohibidos

- No numerar drones desde 0 si el launch usa 1..N.
- No enviar siempre a un action global sin namespace.
- No bloquear el hilo de GUI esperando el servidor.
- No usar rangos arbitrarios como límites físicos del dron sin indicarlo.
- No etiquetar radios como segundos en el modo elipse.
- No declarar “goal completado” sin callback de resultado.
- No convertir esta GUI en la Fase 7.

## Paquetes a compilar

```bash
./codex/herramientas/build_selected_packages.sh lib_tray dron_individual simulacion_dron
```

## Pruebas requeridas

### Prueba 1 — Un dron

1. configurar `dron.numero: 1`;
2. abrir el launch final;
3. seleccionar `dron_1`;
4. enviar una trayectoria corta y segura;
5. observar movimiento en Gazebo;
6. comprobar log del action correcto.

### Prueba 2 — Dos drones, selección individual

1. configurar `dron.numero: 2`;
2. enviar un goal solo a `dron_1`;
3. confirmar que `dron_2` no recibe el goal;
4. enviar un goal distinto a `dron_2`;
5. observar ambos resultados.

### Prueba 3 — Tipos de trayectoria

Enviar desde GUI:

```text
pol3
veltrap
elipse
```

Verificar que las etiquetas/valores enviados coinciden con la semántica de `TrayAction`.

### Prueba 4 — Servidor ausente

Seleccionar un índice/action no disponible mediante un entorno controlado. La GUI debe registrar error y seguir respondiendo.

### Prueba 5 — Cierre integral

Arrancar la cadena completa de Fase 1 con:

```text
Gazebo
modelo de cuatro motores
plugin de actuación
GT
cámaras
lib_tray
control
GUI
```

Mover al menos dos drones de forma independiente y comprobar una imagen en RViz2 durante la ejecución.

## Comando base

```bash
./codex/herramientas/run_simulation.sh \
  --prueba fase1_1H_gui \
  --launch "ros2 launch simulacion_dron multi_dron.launch.py" \
  --post-scenario-wait-sec 20
```

La interacción manual con GUI y la observación visual deben registrarse en historial; el log por sí solo no demuestra que la ventana funcionó.

## Patrones de reducción de logs

```text
GUI lista|Enviando goal|No hay servidor de acción|AccionTrayectoria|tipo_trayectoria|dron_[0-9]+|Generando trayectoria|Acción finalizada OK|RESULT|success|camera|GT|ERROR|FATAL|Traceback|Segmentation fault|Killed
```

## Criterio de éxito

1. Los paquetes compilan.
2. La GUI abre y permanece interactiva.
3. Lee N y namespace del YAML.
4. Envía al action correcto.
5. Un dron se mueve desde la GUI.
6. Dos drones se controlan de forma independiente.
7. Pol3, veltrap y elipse se representan sin ambigüedad grave.
8. El caso de servidor ausente no bloquea ni cierra la GUI.
9. La prueba integral incluye Gazebo, GT, cámaras, control y RViz2.
10. No se afirma ninguna función de la futura GUI de Fase 7.

## Criterio de fallo o parcial

- `NO CONSEGUIDA`: GUI no abre, envía al dron equivocado, bloquea o no produce movimiento.
- `PARCIAL`: funciona con un dron, pero falla selección múltiple, modo elipse o diagnóstico de servidor ausente.
- `BLOQUEADA`: no existe entorno gráfico/Tkinter o la sesión no permite validar GUI/Gazebo visualmente.

## Riesgos

- Tkinter y executor ROS en hilos distintos;
- acceso a widgets desde callbacks ROS;
- sliders float para índices enteros;
- semántica sobrecargada de `tx..tyaw`;
- ausencia de callback de resultado;
- cierre de GUI que apague `rclpy` mientras existen futures;
- confundir esta herramienta con la GUI operacional futura.

## Documentación a actualizar al ejecutar

```text
codex/contexto/paquetes/simulacion_dron/
codex/contexto/paquetes/dron_individual/
codex/pipeline/fase_1_control_dron/pipeline_fase_1_RESUMEN.md
codex/pipeline/fase_1_control_dron/historial/por_subfase/historial_1H.md
codex/pipeline/fase_1_control_dron/historial/por_subfase/historial_1H_RESUMEN.md
```
