# Vision experimental retirada de `dron_individual`

## Estado

La carpeta `src/vision/` fue retirada en 1K. Contenia copias de prototipos de
nube, profundidad, ICP, planos, TSDF y control que no se compilaban,
instalaban, importaban ni lanzaban desde el pipeline vigente.

## Decision

No se rescata su implementacion: varios scripts usaban matrices camara-cuerpo
fijas incompatibles con el pitch dinamico de 1J. Las copias que puedan
permanecer en paquetes legacy son solo referencia historica y no deben
reactivarse desde `dron_individual`.

Las futuras Fases 6/7 deben implementar su procesamiento sobre los paquetes
vigentes y consultar la TF body-camara al stamp del dato.
