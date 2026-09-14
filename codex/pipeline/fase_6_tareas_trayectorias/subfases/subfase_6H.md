# Subfase 6H - Barrido de fachada y coverage lineal

## Estado

MIGRACION AUTORIZADA. Sustituye la exploracion volumetrica hacia voxeles
`UNKNOWN`. No se mantienen ambos modos en produccion.

## Objetivo

Mapear el exterior de cada `BaseSubRoi` mediante un barrido lateral de su
fachada. El servidor elige un tramo pendiente, confirma mediante depth que el
corredor ejecutable es `FREE`, lo recorre mirando la fachada y mide coverage
como longitud realmente recorrida.

## Geometria

- En `AB`, la fachada observada es la cara inferior; las caras izquierda y
  derecha delimitan los extremos del barrido.
- `BC`, `CD` y `DA` aplican la misma construccion rotada.
- El punto nominal esta a `preferred_wall_distance_m=2.5` de la fachada, a
  `preferred_displacement_m=2.0` del dron y en la altura media del nivel.
- Los tres objetivos son minimos suaves y bilaterales, con pesos iniciales
  iguales. Alejarse por cualquiera de sus lados aumenta el coste.
- `OCCUPIED` y `RESERVED` son vetos duros de D*, no terminos de ese coste.

## Ciclo de una subtarea

1. Elegir el intervalo no cubierto mas cercano y un candidato de avance.
2. Solicitar la inspeccion depth bajo demanda definida en 6N.
3. Exigir que todo el swept volume fisico hasta el candidato sea `FREE`.
4. Si solo existe un prefijo `FREE`, recortarlo al extremo conectado mas lejano.
   Se ejecuta unicamente si mide al menos `facade_min_free_prefix_m=1.0`.
5. Planificar XYZ con D*, generar la trayectoria, validarla, reservarla,
   publicarla y ejecutarla mediante 6I-6K.
6. Mantener durante el tramo el yaw/pitch de observacion de la primera captura.
7. Al terminal normal, incorporar el intervalo fisicamente recorrido si la
   orientacion estuvo dentro de la tolerancia y volver a encolar el dron.

No se despacha otro movimiento de la misma subtarea antes del terminal normal.
Un STOP retira la ruta/reserva activa y sigue el lifecycle comun ya implantado.

## Coverage

Coverage es la union 1D de intervalos de fachada recorridos fisicamente mirando
la fachada. No es un enum voxel, no depende del numero total de voxeles y no
crece durante una correccion `TRACKING_RISK`.

- Se fusionan intervalos solapados y se publica la union autoritativa.
- Del centro a un extremo lateral representa aproximadamente 50 %.
- Completion usa `facade_completion_ratio=0.99`.
- Si se alcanza un extremo con zona pendiente, la tarea pasa a `TO_FINISH`, se
  desasigna y el dron vuelve a la cola general.
- La asignacion posterior minimiza distancia al intervalo aun no cubierto, no a
  la AABB completa.
- Tres inspecciones consecutivas sin depth/normal fiable tambien producen
  `TO_FINISH`, nunca un bloqueo permanente del dron.

## Ramas

Las ramas, portales y coverage volumetrico futuro quedan documentados y
aplazados. No se conserva runtime parcial de `BLOCKED_BRANCH` en este barrido.
La casa actual no permite validarlos y no debe generalizarse como unico entorno.

## Cambios requeridos

1. Sustituir `SurfaceCoverageAnalyzer` y `PlanCoverageCandidate` por geometria,
   costes e intervalos de fachada testeables en `task_lib`.
2. Añadir `TO_FINISH` y los intervalos recorridos al contrato de tarea.
3. Eliminar parametros y ramas de seleccion de meta `UNKNOWN`, fallback cercano,
   porcentaje volumetrico y analisis periodico de inaccesibilidad.
4. Integrar el ciclo secuencial con inspeccion, D*, trayectoria, reserva,
   ejecucion y terminal normal.
5. Mantener colas/runtimes independientes por dron.

## Pruebas

- Unitarias: geometria rotada, coste bilateral, union de intervalos, recorte de
  prefijo `FREE`, 50 % centro-extremo, 99 % completion y `TO_FINISH`.
- Integracion: D1 con GT llega al fiducial 2, recibe `MAP_SECTION`, inspecciona,
  recorre hacia una cara lateral y actualiza la linea de coverage en GUI F7.
- Verificar D*, reservas, STOP y terminal ordinario sin destinos `UNKNOWN`.

## Criterio de exito

El dron progresa lateralmente por la fachada con corredores confirmados `FREE`,
el porcentaje coincide con longitud recorrida y una tarea parcial puede volver
a asignarse por proximidad a su intervalo pendiente.
