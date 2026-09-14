# Subfase 6I - Trayectorias multi-waypoint reproducibles

## Estado

PARCIAL, con la cadena fisica ya validada. Esta migracion conserva la cadena y
cambia unicamente el productor de objetivos: de metas `UNKNOWN` a tramos de
fachada confirmados `FREE`.

## Objetivo

Convertir una polilinea XYZ de D* en una trayectoria fisica reproducible en
servidor, dron y GUI mediante la misma `Pol3Waypoints`, sin controladores ni
sistemas de referencia paralelos.

## Contrato estable

```text
candidato de fachada -> D* -> polilinea XYZ -> Pol3Waypoints
                     -> validacion -> reserva -> ACTIVE
                     -> action del dron -> terminal normal
```

- `TrajectoryPlan` se expresa en W y transporta waypoints y tiempos acumulados.
- El dron convierte W->O una vez al aceptar el plan y congela esa ejecucion.
- Servidor y dron usan la misma version de `lib_tray`.
- Los waypoints cercanos se simplifican, conservando siempre el destino final.
- Cada tramo respeta `trajectory_min_segment_duration_sec=8.0`; el empalme C1
  usa `trajectory_waypoint_blend_sec=3.0`.
- Las rutas normales conservan el yaw/pitch de observacion recibido. D* solo
  planifica XYZ.
- La GUI muestra exclusivamente el `TrajectoryPlan ACTIVE`.
- `gen_tray` publica `control/trajectory_active` como fuente comun de
  ocupacion fisica, tambien para goals externos al servidor. Ninguna nueva
  inspeccion ni subtarea ordinaria puede sustituir una trayectoria activa;
  solo los protocolos STOP autorizados conservan esa capacidad.

## STOP

STOP es una orden local, no una ruta D* degenerada. El dron captura su pose
canonica, ejecuta una trayectoria normal al mismo punto durante
`stop_duration_sec=5.0` y comunica el terminal ordinario de la action. No existe
`stop_completed` ni una segunda maquina de estados.

Al iniciar STOP se retira la polilinea visible y la reserva movil; queda solo la
reserva HOLD del volumen fisico del dron. Tras el terminal, el servidor vuelve a
encolarlo y genera el siguiente movimiento.

## Integracion con fachada

- Solo se ejecuta el prefijo cuyo swept volume completo ha sido confirmado
  `FREE` por 6N.
- `OCCUPIED`/inflacion o una reserva ajena que afecten cualquier parte de la
  ruta activa pueden ordenar STOP y replan.
- Cambios `UNKNOWN -> FREE` fuera de esa condicion no cancelan la trayectoria.
- La orientacion de inspeccion es constante durante el tramo; una correccion
  visual local no acumula coverage y debe restaurarla antes de continuar.

## Cambios requeridos

1. Eliminar referencias y parametros de metas `UNKNOWN` y fallback volumetrico.
2. Aceptar el candidato/prefijo de fachada sin alterar `Pol3Waypoints`.
3. Conservar la secuencia atomica plan-validacion-reserva-despacho.
4. Mantener telemetria causal de cada STOP y terminal por `trajectory_id`.
5. Aplazar `InspectFacade` mientras `control/trajectory_active=true`, sin
   contarlo como fallo de depth ni reasignar la tarea.

## Pruebas

Unitarias de simplificacion, tiempos y empalme; integracion con un prefijo
`FREE`; STOP por ocupacion real del corredor; terminal normal y reencolado; GUI
sin adelantar la siguiente polilinea.

## Criterio de exito

La trayectoria ejecutada coincide con la publicada y reservada, no presenta
movimientos bruscos, y toda retirada fisica pasa por STOP antes de replanificar.
