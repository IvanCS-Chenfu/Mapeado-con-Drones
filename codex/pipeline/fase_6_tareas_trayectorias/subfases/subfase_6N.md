# Subfase 6N - Inspeccion depth bajo demanda

## Estado

MIGRACION AUTORIZADA. Sustituye completamente el depth automatico por KeyFrame.
La nube densa global sigue perteneciendo a Fase 8.

## Objetivo

Antes de cada tramo de fachada, confirmar espacio `FREE`, estimar la orientacion
de la fachada y devolver al servidor un unico resultado compacto. Depth
persistente aporta solo evidencia `FREE`; los MapPoints ORB cualificados son la
unica fuente `OCCUPIED`.

## Servicio de inspeccion

El servidor solicita al dron inspeccionar un objetivo W. Dentro de una sola
operacion entre servidor y dron:

1. Capturar depth mirando la fachada actual y estimar su normal.
2. Girar hacia el punto solicitado por el servidor.
3. Esperar el terminal de esa trayectoria local y capturar el segundo depth.
4. Volver a la orientacion de fachada obtenida en la primera captura.
5. Devolver ambos productos, sus referencias KF/camara, yaw/pitch, calidad,
   motivo de fallo y si el segundo frame fue disparado por `TRACKING_RISK`.

El movimiento posterior mantiene yaw/pitch de la primera captura. En una
esquina con dos normales pronunciadas se orientan al mismo semiespacio y se usa
su media ponderada por soporte valido del plano y confianza del ajuste.

La operacion no puede comenzar mientras `gen_tray` informe una trayectoria
fisica activa, aunque esa trayectoria proceda de un cliente externo al
servidor. En ese caso responde `drone_busy`; el servidor conserva tarea,
runtime y contador de fallos, y vuelve a intentarlo tras el terminal ordinario.

## Captura exacta ante TRACKING_RISK

El wrapper conserva un buffer pequeno de pares estereo rectificados indexado
por `frame_id`. Si el riesgo persiste durante la mirada al objetivo, la segunda
captura usa exclusivamente el par exacto del frame que dispara el evento. No se
calcula ni transmite depth de frames anteriores. Cero inliers ORB no es FREE;
solo disparidad valida aporta evidencia.

## Muestreo y voxelizacion

- Muestrear uniformemente dentro de la mascara proyectada del corredor; nunca
  cortar un recorrido row-major al alcanzar `max_points`.
- Mantener la banda fiable depth acordada de 1--5 m.
- Recorrer cada rayo mediante voxel supercover/DDA para no dejar huecos.
- Rellenar huecos entre rayos vecinos solo cuando sus profundidades sean
  coherentes y no exista discontinuidad.
- Los filtros de textura/discontinuidad siguen siendo parametrizables, pero no
  deben impedir confirmar un corredor visible.
- El servidor conserva la evidencia relativa al KF y la retira/reintegra al
  cambiar `W_T_KF`.
- FREE depth nunca degrada `OCCUPIED` respaldado por sparse cualificado.

## Fallos

Si no se obtiene depth/normal fiable, se reintenta hasta tres veces. Tras el
tercer fallo, la tarea pasa a `TO_FINISH`, se desasigna y el dron vuelve a la
cola general. No se inventa FREE ni se bloquea el dron.

`drone_busy` no es un fallo de inspeccion y no consume ninguno de esos tres
intentos.

El resultado conserva por separado los fallos de giro al objetivo, captura
objetivo y restauracion de fachada. Una restauracion correcta no puede
sobrescribir el error anterior ni producir `success=false, reason=ok`.

Antes de anclarse, una captura depth solo puede provocar STOP local por peligro;
no publica evidencia global persistente.

## Fiduciales

La inspeccion puede detectar un fiducial, pero no duplica Fase 3/4. Si ese ID no
ha sido visto por el mismo `(drone_id,map_epoch)`, activa la interrupcion de 6O
y el pipeline fiducial normal. Si ya fue visto por ese submapa, se ignora.

## Cambios requeridos

1. Retirar el enqueue automatico por KF y sus parametros runtime.
2. Anadir captura exacta bajo demanda y el servicio compuesto de inspeccion.
3. Calcular normales/soporte/confianza localmente.
4. Integrar solo FREE reversible en `VoxelMapWorker` con precedencia sparse.
5. Anadir tests de muestreo uniforme, DDA, huecos y normal ponderada.

## Pruebas

Prueba normal de barrido con GUI F7 y Gazebo; prueba dirigida a textura pobre
para forzar `TRACKING_RISK`; retirada/reintegracion por revision de KF; ausencia
de falsos `OCCUPIED` depth; confirmacion del prefijo FREE completo.

## Criterio de exito

La inspeccion produce evidencia FREE sin huecos relevantes, orientacion estable
y una respuesta correlacionada, sin depth por KF ni trafico continuo.
