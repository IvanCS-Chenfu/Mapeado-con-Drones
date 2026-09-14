# Subfase 7I - Tarjetas, tareas y coverage de fachada

## Estado

PARCIAL. Tarjetas, lifecycle, seleccion de subROI, progreso y trayectoria ACTIVE
estan conseguidos. La migracion anade la linea autoritativa de coverage 1D.

## Objetivo

Mostrar el estado real de cada tarea sin reconstruirlo en GUI. Al seleccionar
la tarea de una tarjeta se resaltan simultaneamente su subROI y los intervalos
de fachada recorridos; al seleccionar otra cosa se retiran ambas selecciones.

## Contrato visual

- La tarjeta consume `TaskState` y muestra `progress`, incluido `TO_FINISH`.
- Coverage procede del servidor como union de intervalos world; la GUI no lo
  calcula, suaviza ni conserva como maximo historico.
- Cada intervalo se dibuja como una linea del color estable del subROI, sobre
  la fachada y a la altura media del nivel.
- Varios intervalos disjuntos se representan como varios segmentos de la misma
  tarea.
- La linea de coverage solo aparece al seleccionar la tarea.
- La trayectoria `ACTIVE` es otra capa y permanece visible con independencia de
  la seleccion.
- `FREE`, `OCCUPIED` y `RESERVED` conservan sus toggles y prioridades actuales.
- No se dibujan rayos depth, nube densa ni ramas aplazadas.

## Cambios requeridos

1. Replicar los intervalos de `TaskState` en `GuiDataModel`.
2. Propagar la seleccion desde la tarjeta a `Scene3DWidget`.
3. Crear una capa de lineas estable, sin tarjetas anidadas ni nuevos paneles.
4. Traducir `TO_FINISH` y conservar el ancho inicial ya validado del dock.
5. Actualizar tests de modelo, seleccion, desaparicion de tarea y renderer.

## Pruebas

- Modelo: intervalos ordenados, reemplazo por revision y limpieza terminal.
- GUI F7: seleccionar tarjeta durante un barrido y observar subROI + linea;
  deseleccionar; mantener trayectoria ACTIVE; mostrar 50 % centro-extremo.
- Layout con dos drones y tareas concurrentes.

## Criterio de exito

La GUI representa exactamente el coverage publicado y su lifecycle, no mezcla
la linea con la trayectoria ni calcula geometria de mision por su cuenta.
