# Historial 6J

## Prueba 682 - 2026-09-12

`SIM-DONE success=true`, ambos drones GT llegaron a fiducial 2. D1 confirmó
`reservation_dstar_1_1` y D2 quedó en `waiting_reservation`. Un STOP válido de
D1 conservó HOLD; al mantener FIFO puro, D2 permaneció delante y el dueño del
HOLD no pudo reemplazarlo. Resultado: evidencia de conflicto correcta, pero
interbloqueo de scheduling a corregir en 6K.

## Prueba 683 - 2026-09-12

Tras reencolar al dueño de HOLD con prioridad local, D1 obtuvo commits
posteriores, liberó `reservation_dstar_1_8` y D2 fue reactivado con
`reservation_dstar_2_9`. La GUI F7 mostró solo rutas ACTIVE y ambas acciones
físicas llegaron a coexistir. `SIM-DONE success=true`; sin guarda de recursos.

## Prueba 686 - 2026-09-12

`SIM-DONE success=true` tras 200 s de Gazebo+GUI F7 con dos drones GT en
fiducial 2. Los perfiles registraron `obstacle_inflation=(4,4,3)` y
`body=(2,2,1)`: el margen estatico queda separado de la reserva fisica. D2
confirmo primero 280 celdas; el primer commit de D1 fue rechazado, se intento
la alternativa `dstar_1_3` y, al seguir sin ruta, paso a espera segura. Mas
tarde D1 y D2 alcanzaron `ACTIVE` en paralelo (`dstar_1_13`/`dstar_2_12` y
`dstar_1_18`/`dstar_2_17`). Dos STOP reales de D2 produjeron exactamente una
causa cada uno, ambas por `unknown -> occupied` dentro de la inflacion estatica;
cada STOP sustituyo MOVING por HOLD local de 75 celdas. Resultado: geometria
fisica, replan previo a espera, HOLD local y telemetria causal deduplicada
validados; el muestreo de la curva ejecutable sigue pendiente.

## Prueba 687 - 2026-09-12

`SIM-DONE success=true` tras 179 s con Gazebo y GUI F7. La nueva proyeccion
`VoxelMap.reserved_voxels` no modifico el snapshot raw: al commit de D1 la GUI
registro `reserved=310`; tras `F6J-RESERVATION-RELEASE` de la misma trayectoria
recibio `reserved=0`. Los STOP conservaron HOLD local de 75 celdas y luego los
reemplazos publicaron el siguiente volumen MOVING. Hubo reservas simultaneas
de ambos drones (`reserved=557` y despues `587`). Resultado: lifecycle visual
de reserva, restitucion al release y coexistencia de reservas validados.
