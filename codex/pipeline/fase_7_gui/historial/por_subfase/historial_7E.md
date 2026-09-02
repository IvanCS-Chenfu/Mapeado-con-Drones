# Historial 7E

## 2026-09-02 - Implementacion y pruebas

Se formalizo el gradiente fijo por `score`, el filtro exclusivamente visual y
el control slider/campo sincronizado. `/global_sparse_cloud` conserva score e
identidad `(drone_id,map_epoch,local_mp_id)` y retiro el field `rgb` temporal.

El contrato sparse paso 1/1, `orbslam3_server` paso 13/13 CTests y
`multidron_gui_lib` 9/9. El smoke sintetico mostro puntos verdes/amarillos y
oculto los rojos bajo 0.35 sin solapes. En 378R la GUI recibio hasta 5544
puntos con score 0.0206..1.0; el usuario valido controles y visualizacion.
378R expiro despues en la puerta manual. 378RR repitio la ruta y termino con
`SIM-DONE success=true`, 2935 puntos finales y shutdown limpio.

Conclusion: `CONSEGUIDA`. El timeout de 378R pertenece al harness tras la
revision humana y se conserva como intento no exitoso; 378RR aporta el cierre.
