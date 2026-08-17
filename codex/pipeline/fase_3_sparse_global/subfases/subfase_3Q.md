# Subfase 3Q - Optimizacion covisible comun por loop y fiducial

## Estado

```text
PREPARACION CERRADA; IMPLEMENTACION PENDIENTE DE AUTORIZACION
```

3Q convierte `OptimizationEvidence` en una rama real de la misma `LoopTask`
BAJA y generaliza la optimizacion fiducial para que ambas rutas consuman un
grafo SE(3) comun con constraints temporales, covisibles, fiduciales y de loop.

```text
LoopTask -> 3N -> 3O -> error alto
         -> stop_drones=true
         -> grafo covisible -> solver -> validacion
         -> commit poses/constraints -> fusion 3P directa opcional
         -> dirty sets -> task end -> stop_drones=false
```

La prioridad de la tarea no cambia y no existe preemption: una tarea fiducial
MAX que llegue durante 3Q espera a que termine la `LoopTask` activa y sera la
siguiente en empezar. El flujo principal continua incorporando datos mientras
el mission gate no envia un nuevo goal.

## Acuerdos principales

- loops inter/intra dron o submapa siguen la misma decision geometrica;
- una optimizacion por loop requiere dos queries independientes coherentes;
- la ventana es el subgrafo minimo conectado hasta autoridades hard;
- una dependencia soft se sigue incluso a otro submapa y puede incorporar un
  tramo completo delimitado por dos fiduciales;
- fusiones anteriores empiezan como constraints relativas blandas medibles;
- fiduciales hard son los unicos puntos world absolutamente inmoviles;
- covisibilidad confirmada entra tanto en optimizacion loop como fiducial;
- el 30 % es densidad base de controles, ampliable por constraints fuertes;
- se reutilizan builder, solver, validator, commit y continuidad existentes;
- inicialmente solo se comprometen accepts completos;
- inliers RANSAC viven en la tarea y se reutilizan para fusion posterior;
- una fusion posterior omitida no invalida una optimizacion correcta;
- stale/rollback termina el intento y encola una BAJA fresca deduplicada;
- raw nunca se modifica y el worker secundario nunca publica.

## Documentos

- `subfase_3Q_especificacion.md`: topologia, ventanas, autoridad e invariantes.
- `subfase_3Q_implementacion.md`: tipos, componentes, solver, commit y eventos.
- `subfase_3Q_testing.md`: tests, matriz Gazebo, logs y revision visual.
- `subfase_3Q_criterios.md`: exito, parcial, fallo y parametros provisionales.

El acuerdo principal es ejecutable, pero puede ajustarse si las pruebas reales
revelan sobrerigidez, constraints insuficientes, mala seleccion de ventana o
coste excesivo. Cada cambio funcional requerira conversacion y quedara
registrado sin ocultar los intentos anteriores.
