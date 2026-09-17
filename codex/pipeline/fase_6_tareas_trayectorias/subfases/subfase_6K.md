# Subfase 6K - Fiduciales oportunistas durante una tarea

## Estado

`PENDIENTE DE MIGRACION`.

## Objetivo

Interrumpir de forma controlada la inspeccion de fachada cuando aparezca una
oportunidad de anclar un fiducial aun no visto por el submapa actual.

## Contrato funcional

La seleccion de punto consulta los fiduciales dentro de un radio configurable.
Solo considera pendiente uno que no haya sido visto por el mismo
`(drone_id, map_epoch)`. Si existe, emite `LOOK_FIDUCIAL` en lugar de una
orden de fachada.

Si la mirada no lo detecta, se busca una nueva pose de observacion segura que
reduzca la distancia al fiducial. El dron se aproxima mediante el flujo D*
normal y vuelve a mirar. Si no hay pose FREE que aporte progreso, libera esa
subtarea y continua la eleccion normal: no mantiene un bucle estacionario.

Una deteccion se entrega al pipeline existente de Fase 4/5. Esta subfase no
altera su optimizacion ni declara cobertura del subROI por el mero hecho de
mirar un fiducial.

## Cambios requeridos

- Registrar vistas por submapa, no solo por drone global.
- Representar `LOOK_FIDUCIAL` y su continuacion como workflow correlacionado.
- Reutilizar los mecanismos externos vigentes de fiducial hasta que estos se
  migren de manera independiente.

## Exclusiones

- No modifica la logica de optimizacion por fiducial de Fase 3/4/5.
- No fuerza una ruta no FREE ni reserva un corredor sin planificacion.

## Validacion y exito

Un fiducial ya visto no interrumpe otra vez el mismo submapa. Uno pendiente
prioriza una mirada; si no se detecta, la aproximacion mejora la observacion o
se libera sin bloqueo. La deteccion conserva el contrato anterior de anclaje.
