# Subfase 3R - Pruebas

## Regresiones locales

1. Base ORB determinista, acotada y actualizada solo para IDs afectados.
2. Un raw no anclado conserva factores neutros.
3. Con baseline `0.06 m`, distancias 1, 2 y 5 m tienen factor `1`; 0.5 m tiene
   factor `0.25`; por encima de 5 m se aplica la caida cuadratica acotada.
4. Cambiar el baseline desplaza solo el limite lejano; el cercano sigue en 1 m,
   y mover el punto a la banda valida recupera el score.
5. Un punto maduro aislado baja y recupera score al aparecer vecinos.
6. Evidencia inlier `+0.04` es idempotente y se aplica despues de los factores.
7. Fused score coincide con `clamp(media(miembros) + 0.04 * N, 0, 1)` y no
   depende del orden.
8. Un raw cercano penalizado conserva su score propio, pero un raw bueno puede
   diluirlo mediante la media fused; no se aplica cap permanente.
9. Cambiar un raw recalcula solo los fused tracks que lo contienen.
10. La visibilidad sparse genera diagnostico pero cero penalizaciones numericas.
11. Fusion rejected/stale no deja score; merge/removal mantiene IDs exactos.
12. `GlobalMapBuilder` publica la misma geometria con score alto o bajo.

## Build y tests

```text
./codex/herramientas/build_selected_packages.sh \
  orbslam3_multi orbslam3_server simulacion_dron
```

Ejecutar la bateria funcional de los paquetes afectados. De aparecer deuda de
lint historica, aislar tests funcionales y registrarla sin modificar codigo
ajeno.

## Simulacion acordada

```text
YAML: codex/archivos_auxiliares/trayectorias/prueba_tipica_rodeo_edificio_dos_fiduciales.yaml
launch: ros2 launch simulacion_dron multi_dron.launch.py
```

Verificar mediante logs reducidos:

- recorrido y cierre del escenario;
- commits raw y fused `F3R-*` sin NaN, crash ni stale visible;
- puntos anclados con penalizaciones geometricas y recuperaciones;
- banda neutra 1-5 m con el baseline actual, mas conteos near/far coherentes;
- fusiones con bonus `0.04 * N` y propagacion por cambios raw;
- diagnosticos de visibilidad sin deltas negativos;
- nube publicada sin filtrado y recursos estables.

La validacion visual comprueba el vertice de score en el grafo web y RViz2 con
gradiente verde-amarillo-rojo, prestando especial atencion al descenso de score
de puntos a menos de 1 m, a que paredes habituales hasta 5 m no pierdan score
por distancia y a que las revisitas sigan reforzando zonas coherentes. Sin
confirmacion visual del usuario, el cierre maximo es `PARCIAL`.
