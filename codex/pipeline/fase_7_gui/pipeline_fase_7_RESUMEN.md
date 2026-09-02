# Resumen — Fase 7: GUI 3D de operación multi-dron

## Estado

```text
en curso: bloques 1 y 2 conseguidos
Preparación documental: actualizada para ciclo iterativo con Fase 6
Bloque 1 (7A-7D): conseguido el 2026-09-02
Bloque 2 (7E, 7F, 7H): conseguido el 2026-09-02
Historial: disponible en historial/por_subfase/
```

## Objetivo

Crear una GUI de escritorio propia en `src/servidor/`, implementada con C++ + Qt + OpenGL + ROS 2, sin página web y sin RViz2 embebido. El centro de la aplicación es un viewport 3D en `world` con grid, cámara orbit/pan/zoom y capas para sparse, drones, KFs, fiduciales, trayectorias y futura nube densa.

## Decisiones cerradas

- GUI propia desde cero; Qt/OpenGL son infraestructura gráfica, no un visualizador externo.
- Dos paquetes en Servidor: `multidron_gui_lib` para lógica/modelos/render/widgets
  testeables y `multidron_gui` para orquestación/ejecutable/launch ROS 2 + Qt.
- Proceso/nodo GUI independiente del servidor de mapa; cerrar la GUI no afecta al pipeline.
- Tema oscuro/moderno, subordinado a legibilidad y rendimiento.
- Callbacks ROS → caches `GuiDataModel` → snapshots → thread Qt/OpenGL.
- Render fluido; no reconstruir buffers pesados en cada frame si el dato no cambió.
- Panel derecho con tarjeta por dron y scroll vertical cuando no caben.
- La barra de progreso puede ser discreta por hitos/zonas; la GUI no inventa progreso.
- Si `GO_TO` puede reportar porcentaje real con bajo coste, lo publica el productor y la GUI lo dibuja.
- Botones superiores para activar/desactivar layers.
- Umbral de score solo visual.
- Modo de color de MapPoints por gradiente rojo→amarillo→verde según score; compatible con el filtro.
- Picking genérico. Para MapPoint se muestran al menos `x,y,z` en `world`; clicar no crea una tarea.
- Mostrar únicamente la trayectoria actual futura de cada dron, no todo el historial de propuestas.
- Tareas GUI baseline: `GO_TO`, `CAPTURE_SPARSE`, `CAPTURE_DENSE`; todas las poses son `(x,y,z,yaw)` absolutas en `world`.
- No hay botones pause/resume/cancel en esta fase.
- `GO_TO` entra por `task_server` y se ejecuta mediante `task_manager`, planner
  y reservas de Fase 6; nunca mediante `TrayAction` directo desde la GUI.
- `CAPTURE_SPARSE`: llegar a pose y esperar creación natural de KF; no forzar ORB-SLAM3; si no aparece, tarea fallida.
- `CAPTURE_DENSE`: contrato/UI preparados; captura/reconstrucción real en Fase 8.
- `DenseMapLayer` se valida en Fase 7 con datos sintéticos/replay, no con una reconstrucción inventada.
- Dron perdido/stale: conservar última pose válida, mostrar tag `PERDIDO` y
  ejes más transparentes; no borrar ni propagar pose falsa.

## Ciclo con Fase 6

Primera estrategia acordada: ejecutar Fase 7 hasta donde permitan Fases 1–5 y
la telemetría real existente; cuando una subfase dependa de contratos de tareas,
progreso, trayectoria vigente o voxeles de Fase 6, volver a Fase 6 hasta
desbloquearla y repetir. Tramo inicial probable: `7A`–`7F`, `7H` y partes
testeables/sintéticas de `7L`; `7G`, `7I`, `7J`, `7K` y `7M` definitiva quedan
condicionadas por Fase 6 real.

## Puerta de validación hacia atrás

La GUI también sirve para corroborar visualmente Fases 3–6. Si el mensaje recibido ya contiene un dato incorrecto, no se corrige en la GUI: se vuelve a la fase propietaria, se arregla allí y se repite la prueba de Fase 7. Si el mensaje es correcto pero se dibuja mal, el problema sí pertenece a Fase 7.

Ejemplos: sparse/KFs → Fase 3; fiduciales → Fase 4; pose/tracking → Fase 5; tareas/progreso/trayectoria → Fase 6.

Ante una duda funcional nueva no cubierta por este contrato, Codex debe parar y preguntarle al usuario antes de decidir.

## Subfases

```text
7A  arquitectura, dependencias y contratos ROS 2
7B  ventana y layout
7C  GuiDataModel + bridge ROS asíncrono
7D  motor 3D, grid y cámara
7E  sparse + score
7F  drones + KFs + fiduciales
7G  trayectoria actual
7H  picking + inspector
7I  tarjetas + progreso
7J  GO_TO desde GUI
7K  CAPTURE_SPARSE + preparación CAPTURE_DENSE
7L  DenseMapLayer + rendimiento
7M  integración, validación visual y cierre
```

Estado ejecutado: `7A`-`7F` y `7H` conseguidas. `7G` y `7I`-`7M` permanecen
pendientes. El siguiente bloqueo real es Fase 6: trayectoria vigente, estado y
progreso de tareas y acciones operativas para continuar la GUI.

## Prueba final

Arrancar N drones, servidor y GUI propia. Visualizar datos reales, mover la cámara, probar capas/score/picking, observar trayectorias y progreso, enviar `GO_TO`, ejecutar `CAPTURE_SPARSE`, comprobar que dense permanece preparada pero no fingida antes de Fase 8, cerrar/reabrir la GUI y confirmar que el pipeline nunca dependió de ella.
