# Resumen - 7G

Estado: PARCIAL. La GUI F7 muestra/reemplaza `Plan previsto` de 6G y ya consume
el lifecycle 6I: representa `ACTIVE` y retira estados terminales, sin usar el
rastro de poses. Build correcto y CTest 9/9. En 638 recibio la actualizacion
`ACTIVE` de D1, pero la accion legacy preemptada fallo por doble finalizacion y
el helper cerro todo el launch. Falta corregir ese lifecycle y despues una
simulacion persistente que confirme visualmente una trayectoria ejecutada y su
reemplazo real.

Actualización 679: la capa principal ignora `PLANNED`, representa solo
`ACTIVE` y limpia por `trajectory_id`. Build y CTest 9/9 correctos. La
simulación GT con D1 confirmó que los candidatos pendientes no sustituyen la
polilínea activa y que los terminales atrasados no la borran. Este alcance de
lifecycle visual queda CONSEGUIDO; 7G agregado sigue PARCIAL.
