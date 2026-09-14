# Resumen histórico 6J

## Estado vigente

`PARCIAL`. `ReservationOverlay` separa `RESERVED` de la evidencia raw y aplica
las reservas ajenas al snapshot del planner. Se validaron commit, conflicto,
replace/release y la invisibilidad de reservas como voxeles raw.

## Evidencia

- CTest `task_lib`: 10/10, incluidas regresiones de owner/conflicto/replace.
- Prueba 682: D1 confirmó 1615 celdas y D2 entró en `waiting_reservation`; el
  HOLD tras STOP reveló el interbloqueo de scheduling documentado en 6K.
- Prueba 683: D1 reemplazó HOLD y liberó `reservation_dstar_1_8`; D2 recibió
  después `reservation_dstar_2_9` y ambos alcanzaron rutas `ACTIVE` en paralelo.
- Prueba 686: reserva fisica `body=(2,2,1)` separada de inflacion estatica,
  replan antes de espera, HOLD local de 75 celdas y dos causas STOP unicas.
- Prueba 687: GUI F7 recibio `reserved=310` al commit y `reserved=0` tras
  release; HOLD local y reservas simultaneas tambien quedaron publicados.

## Pendiente

El corredor actual samplea la polilínea D* con paso configurable. Falta usar el
sampler de la curva ejecutable compartida de `lib_tray` antes de reclamar un
swept volume definitivo.

La revision manual prolongada de 688 confirma buen funcionamiento general de
RESERVED y paralelo posterior, pero observó contencion inicial y algun STOP
aislado potencialmente evitable. Ambos quedan diferidos: requieren diagnostico
al cerrar una prueba y no cambian la politica segura vigente.
