# Subfase 3R - Absorbida: KFs llegados durante/despues de optimizacion

## Estado

```text
CANCELADA COMO SUBFASE INDEPENDIENTE; responsabilidad absorbida.
```

La antigua propuesta creaba una cola post-optimizacion para reprocesar KFs
llegados durante una optimizacion. El nuevo flujo hace innecesaria esa ruta:

- `3C/3G` producen un `ChangeSet` por cada KF nuevo o modificado;
- `3D` incorpora inmediatamente el KF a `GlobalPoseStore` si el submapa esta
  anclado;
- `3K` reconcilia en su commit los KFs tardios que pertenecen a la ventana y
  actualiza el control desde el que se extienden los posteriores;
- `3N` recibe cada KF materialmente nuevo mediante la admision normal de una
  `LoopTask` canonica;
- `3Q` valida revisiones y nunca necesita reinyectar el KF por una ruta especial.

No se debe implementar `PostOptimizationKeyFrameQueue`, estados `REBASED`
especiales ni un segundo pipeline de loop. Corregir una pose tampoco debe
simular que el KF acaba de llegar: solo una revision material relevante puede
crear/coalescer otra `LoopTask`.

## Regresiones conservadas

Las pruebas futuras deben cubrir:

1. KF nuevo posterior al target durante solver;
2. KF insertado tarde dentro de la ventana capturada;
3. cambio de epoch durante una tarea;
4. commit stale sin retry inmediato;
5. ningun KF procesado dos veces por una cola post-optimizacion.

Este archivo conserva la decision para evitar que la subfase o su clase sean
recreadas. El detalle funcional esta en `subfase_3D.md`, `subfase_3K.md`,
`subfase_3K_worker_secundario.md` y `subfase_3Q_implementacion.md`.

## Incremento Visual

No añade vertices ni aristas. `3R` esta absorbida; recrear una cola o etapa
visual propia ocultaria el ownership real fijado por 3D/3K/3Q.
