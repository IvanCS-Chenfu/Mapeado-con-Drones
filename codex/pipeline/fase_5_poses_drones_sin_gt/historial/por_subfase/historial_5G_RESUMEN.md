# Historial 5G - Resumen

Estado agregado: `PARCIAL`; la prueba visual 246 exige ampliar el bloqueo de
fuente desde cada goal individual a la mision YAML completa.

- `GT_FALLBACK` es la unica entrada operacional de GT y queda visible mediante
  `pose_source`; no alimenta mapa, anchors ni pose global.
- La cualificacion ORB usa exclusivamente tracking `OK`, anchor del epoch y 20
  muestras consecutivas. El error GT-estimada no interviene por la deriva.
- La implementacion vigente congela por goal, pero esto es insuficiente porque
  el controlador conserva la ultima consigna entre goals. La politica correcta
  congela por mision: `GT -> ORB` solo tras finalizar todo el YAML; `ORB -> GT`
  sigue siendo inmediato y GT queda retenido hasta terminar la mision.
- Builds de `dron_individual` correctos y GTest de politica/mux 2/2.
- Prueba 243: mision completa, 22/22 goals, 44/44 handshakes y cero entradas
  ORB dentro de goals bloqueados en GT.

No repetir 242 sin la correccion de espera de `velocity_valid`: su primer
intento de frontera era correcto, pero entregaba el siguiente goal sobre la
primera muestra ORB sin velocidad valida.
