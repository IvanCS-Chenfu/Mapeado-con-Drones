# ADR 0004 — Pipeline total del proyecto

## Estado

Aceptada.

## Contexto

El proyecto no termina al conseguir un mapa sparse global.

El objetivo final es obtener una nube densa global de un entorno definido por YAML usando varios drones, sin depender de ground truth para la pose final ni para la construcción del mapa.

La nube sparse global es una fase necesaria, pero no es el objetivo final.

## Decisión

El pipeline maestro del proyecto se divide en nueve fases principales:

1. Validar control del dron en simulación.
2. Separar paquetes entre servidor, dron y simulación.
3. Conseguir un mapa sparse global multi-dron.
4. Sustituir el fiducial simulado por fiducial visual real.
5. Obtener la pose global de cada dron sin ground truth.
6. Generar tareas y trayectorias de mapeo.
7. Crear una GUI 3D propia distinta de RViz2.
8. Construir una nube densa global multi-dron.
9. Añadir mejoras opcionales y pruebas avanzadas.

La Fase 3 es la fase actual porque hereda el trabajo iniciado cuando el mapa
sparse global era la antigua Fase 1. Al cerrar Fase 3, el siguiente bloque
previsto es ejecutar la Fase 2 de separación física de paquetes. La numeración
nueva expresa ownership documental y dependencias, no obliga a ejecutar Fase 2
antes de cerrar el trabajo sparse ya abierto.

## Fase 1 — Control del dron en simulación

Objetivo:

Reconstruir y validar la cadena base de Gazebo Classic, Xacro/URDF, YAML,
sensores, generación de trayectorias, control, mixer de cuatro motores, GUI de
simulación y gráficas.

Resultado esperado:

- uno o varios drones pueden crearse en Gazebo;
- el modelo físico y la configuración quedan definidos por YAML;
- el control funcional obligatorio cubre cuatro motores;
- la dependencia de GT en control queda localizada para sustituirla en Fase 5;
- trayectorias, perfiles, referencias y errores pueden graficarse sin mezclar
  magnitudes o drones.

## Fase 2 — Separación de paquetes

Objetivo:

Separar los paquetes de `src/` en tres grupos físicos:

1. paquetes que irían en cada dron real;
2. paquetes del ordenador servidor;
3. paquetes exclusivos de simulación.

Resultado esperado:

```text
dron/
servidor/
simulacion/
```

Esta fase debe facilitar pasar de simulación a despliegue real, con builds por
grupo, YAML con ownership claro y guardas contra divergencias.

## Fase 3 — Mapa sparse global multi-dron

Objetivo:

Construir una nube sparse global coherente usando varios drones y la información de ORB-SLAM3.

Debe evitar:

- dobles paredes;
- fusión incorrecta de submapas;
- pérdida de submapas históricos;
- uso indebido de ground truth;
- correcciones destructivas.

Resultado esperado:

- mapa sparse fused en frame `world`;
- submapas por `(drone_id, map_epoch)`;
- landmarks globales con score;
- loops validados;
- fiduciales tratados como observaciones absolutas;
- poses corregidas suficientemente fiables para avanzar a separación,
  fiducial real, pose sin GT, tareas, GUI y dense.

## Fase 4 — Fiducial real

Objetivo:

Reemplazar el fiducial simulado basado en ground truth por detección visual de
tags ligada al KeyFrame exacto que vio cada marca.

Resultado esperado:

- ORB-SLAM3 solo notifica KFs; no detecta fiduciales;
- el wrapper detecta tags en la imagen izquierda exacta del KF;
- el servidor interpreta tags/cubos y aplica anchors/revisitas;
- GT queda fuera del camino funcional y solo sirve para métricas/debug externo.

## Fase 5 — Pose global de cada dron sin ground truth

Objetivo:

Usar ORB-SLAM3 local, correcciones globales de KFs y un estimador embarcado para
estimar pose y velocidad sin ground truth funcional.

Esto puede incluir:

- relocalización contra el mapa global;
- matching de features contra landmarks globales;
- PnP/RANSAC;
- refinamiento local;
- fusión con odometría local;
- validación contra la nube global.

No se permite usar GT como entrada del algoritmo.

## Fase 6 — Tareas y trayectorias

Objetivo:

Generar y coordinar misiones de mapeo sparse desde YAML, ROI, tareas por nivel,
trayectorias cortas, percepción local de obstáculos y reservas espaciales entre
drones.

Resultado esperado:

- el servidor reparte tareas de cobertura;
- cada dron replanifica localmente sin perder tracking;
- las reservas evitan conflictos dron-dron;
- regiones inaccesibles no bloquean eternamente la misión.

## Fase 7 — GUI propia distinta de RViz2

Objetivo:

Crear una interfaz gráfica 3D específica del proyecto para visualizar y controlar el sistema.

La GUI debería mostrar:

- puntos sparse nuevos;
- fused landmarks;
- poses de drones;
- trayectoria de cada dron;
- estado de submapas;
- estado de fiduciales;
- nube densa cuando exista;
- métricas de calidad;
- logs resumidos;
- estado de fase/prueba.

RViz2 puede seguir usándose para debug, pero no debe ser la GUI final del proyecto.

El visualizador web diagnóstico de Fase 3 no sustituye esta GUI funcional.

## Fase 8 — Nube densa global

Objetivo:

Construir nubes densas usando:

- poses globales estimadas;
- cámaras de los drones;
- mapa sparse global;
- OpenCV C++;
- calibración estéreo;
- filtros geométricos;
- integración en frame `world`.

La nube densa debe colocar cada punto en su posición global correcta.

No se debe usar ground truth para colocar la nube densa.

## Fase 9 — Mejoras opcionales

Objetivo:

Mejorar realismo y robustez.

Ejemplos:

- usar nuevos mapas de Gazebo;
- probar reflejos;
- probar cambios de iluminación;
- detectar objetos dinámicos;
- evitar mapear puntos dinámicos;
- filtrar superficies problemáticas;
- añadir escenarios más complejos;
- comparar rendimiento con varios números de drones.

La Fase 9 queda como placeholder futuro; sus subfases se definirán cuando el
proyecto avance hasta esa fase.

## Consecuencias

El archivo `PIPELINE_MAESTRO.md` debe reflejar estas nueve fases.

Cada fase puede tener subfases.

La Fase 3 activa se planifica ahora como una secuencia `3A`-`3X`.

Las subfases antiguas `12R-*`, `13`, `14` y `15` se conservan como legacy/solo
referencia. No deben ejecutarse como planificación activa.

No debe confundirse el pipeline total con el pipeline interno de la Fase 3.

## Reglas para Codex

Codex debe consultar `PIPELINE_MAESTRO.md` antes de ejecutar una fase.

Codex no debe avanzar a fases posteriores si la fase actual no cumple sus criterios de éxito.

Codex debe documentar cada intento en el historial de fase correspondiente.

Codex no debe convertir una mejora opcional de Fase 9 en requisito de fases anteriores.

## Cuándo revisar esta decisión

Solo debe revisarse si cambia el objetivo final del proyecto.
