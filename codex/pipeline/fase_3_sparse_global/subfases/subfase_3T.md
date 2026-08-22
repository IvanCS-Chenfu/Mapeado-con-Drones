# Subfase 3T - Limpieza, configuracion y handoff de Fase 3

## Estado

```text
CONSEGUIDA
```

Checkpoint anterior a la limpieza:

```text
1b96a7a checkpoint: guardar estado de fase 3 antes de limpieza 3X
```

`3T` no redisenha los algoritmos de Fase 3. Audita la implementacion vigente,
elimina rutas sustituidas, ordena la configuracion y deja un handoff
reproducible para Fase 2.

La ejecucion original de la limpieza conservo 3Q `A REVISAR` por la deformacion
de la prueba 194. Tras la revision visual correcta de 195, el usuario acepto
3Q para el cierre de Fase 3 y dejo la politica adaptativa de evidencia como
mejora futura, no como bloqueo. La incidencia historica no se elimina.

## Objetivos

1. Conservar una unica ruta runtime por responsabilidad.
2. Retirar el legacy de Fase 3 ya sustituido.
3. Absorber las subfases que no representan trabajo independiente.
4. Diagnosticar duplicacion, codigo muerto y configuracion sin consumidor.
5. Documentar el codigo activo y sus invariantes.
6. Organizar los parametros en YAML por dominio y despliegue.
7. Actualizar launches, manifiestos, documentacion y resultado final.
8. Demostrar con build, tests y Gazebo que no aparecen regresiones.

La arquitectura que debe permanecer es:

```text
principal: ingesta -> raw -> ChangeSet -> derivados -> publicacion
secundario: cola priorizada -> un worker -> tarea -> commit/rechazo
observabilidad: eventos -> RViz2/web, sin gobernar el pipeline
```

## Diagnostico y retirada

Antes de borrar se deben localizar y clasificar:

- implementaciones repetidas, helpers equivalentes y clases pasarela;
- codigo, parametros, metricas y topics sin consumidor;
- rutas duplicadas de scheduling, optimizacion o publicacion;
- launches o tests de replay sin cobertura vigente;
- archivos instalados innecesariamente por CMake;
- defaults repetidos entre C++, launch y YAML;
- referencias activas hacia archivos legacy.

Cada candidato se retira solo si no participa en build, instalacion, tests o
runtime, o si su sustituto activo esta probado. El diagnostico se registra en
el historial 3T y se resume en el resultado final. No se divide codigo grande
por estetica ni se adelanta la separacion de paquetes de Fase 2.

Eliminar de la version actual los directorios `legacy/`, `legacy2/`, snapshots,
codigo antiguo y documentacion duplicada de Fase 3. No se reescribe Git. Se
conservan historiales, conclusiones, trayectorias y evidencia reproducible. El
resultado final enumera lo retirado y remite al checkpoint para recuperarlo;
no se crea un indice que presente el legacy como contenido disponible.

## Renumeracion final

La limpieza se ejecuto inicialmente con el identificador provisional `3X`.
Antes del cierre definitivo se fijo la secuencia activa:

```text
3Q optimizacion -> 3R scoring -> 3S debug -> 3T limpieza/handoff
```

Por ello este contrato y su historial pasan de 3X a 3T. La subfase de scoring
pasa de 3S a 3R y la nueva 3S controla la observabilidad del launch. Los
marcadores actuales de scoring son `[F3R-*]`; los logs 192-195 conservan
`[F3S-*]` porque reflejan la numeracion existente cuando se ejecutaron.

## Subfases absorbidas

Eliminar como contratos independientes:

- responsabilidad transversal anterior a la 3R final;
- auditoria de arquitectura que usaba provisionalmente 3T;
- auditoria del visualizador que usaba provisionalmente 3U;
- regresion integral que usaba provisionalmente 3V;
- rendimiento y robustez que usaban provisionalmente 3W.

Sus invariantes y conclusiones utiles se integran en `3T`, la documentacion
vigente y `RESULTADO_FINAL_FASE_3.md`. Los historiales se conservan bajo
`historial/absorbidas/` con nombres descriptivos para no colisionar con la
numeracion activa.

## Comentarios del codigo

El codigo activo de `orbslam3_multi`, `orbslam3_server` y la observabilidad de
`simulacion_dron` debe explicar:

- responsabilidad y ownership de clases;
- contratos de funciones publicas e internas complejas;
- locks, revisiones, atomicidad y orden de commit;
- invariantes geometricos y de identidad;
- decisiones algoritmicas o de rendimiento no evidentes.

No se comentan instrucciones obvias. Los comentarios explican contratos y
motivos, no traducen el codigo linea por linea.

## ADR y dominios

Crear:

```text
codex/contexto/decisiones/ADR_0009_configuracion_por_dominio_y_despliegue.md
```

El ADR prepara los tres grupos de Fase 2:

- **dron**: masa, inercia, controladores, calibracion y propiedades fisicas;
- **servidor**: fusion, scoring, loops, optimizacion, colas y snapshots;
- **simulacion**: mundo, modelos, ruido, GUI y comportamiento de Gazebo.

La propiedad depende de la responsabilidad semantica, no solo del consumidor
actual. Por ejemplo, `body_T_camera` pertenece conceptualmente al dron aunque
el servidor lo consuma hoy; Fase 2 decidira su interfaz definitiva.

El ADR distingue propiedad, consumidor, seleccion y distribucion. Cargar un
YAML en el servidor no configura automaticamente nodos ROS 2 remotos. `3T` no
introduce un protocolo de distribucion.

## YAML

Organizar todos los parametros activos controlables por el servidor:

```text
orbslam3_server/config/global_map/
simulacion_dron/config/global_map/
  runtime.yaml
  fiducials.yaml
  optimization.yaml
  loop_fusion.yaml
  scoring.yaml
  replay_debug.yaml
```

Cada paquete contiene sus propias copias. El launch standalone carga solo las
de `orbslam3_server`; Gazebo carga solo las de `simulacion_dron`. Ahora los
parametros compartidos y sus valores deben ser iguales.

Reglas:

- un test detecta claves desconocidas, omisiones y divergencias;
- `replay_debug.yaml` no se carga normalmente;
- los argumentos launch quedan para valores dinamicos como numero de drones,
  namespace o seleccion de configuracion;
- los defaults C++ son fallback seguro, no perfil operativo oculto;
- CMake instala todos los YAML;
- parametros intrinsecos de dron o exclusivos de simulacion no se copian al
  servidor; su movimiento definitivo corresponde a Fase 2.

## Launches y manifiestos

Actualizar launches y `launches.md` para describir los YAML de cada modo, los
argumentos dinamicos y los visualizadores opcionales, sin afirmar que siguen
pendientes componentes ya implementados.

Actualizar `package.xml` de `orbslam3_multi`, `orbslam3_server` y
`simulacion_dron`:

```text
version: 0.1.0
license: GPL-3.0-only
maintainer: ivancalvosantos2003@uma.es
```

Sustituir tambien descripciones `TODO`. No usar ni modificar
`fase45_sandbox`.

## Resultado final

Crear `RESULTADO_FINAL_FASE_3.md` con:

- arquitectura, flujos y responsabilidades vigentes;
- topics, launches y YAML activos;
- pruebas finales y evidencia relevante;
- codigo y contratos absorbidos o retirados;
- recuperacion mediante el checkpoint;
- la incidencia 3Q de la prueba 194 y su mejora futura documentada;
- punto de entrada y decisiones pendientes para Fase 2.

Sincronizar pipeline maestro, resumen de Fase 3, estado actual, ultima sesion,
catalogo de pruebas y documentacion de paquetes tocados.

## Exclusiones

- No corregir ni cerrar 3Q dentro de 3T.
- No cambiar algoritmos para mejorar resultados visuales.
- No modificar `ORB_SLAM3`, `orbslam3_ros2`, `orbslam3_msgs` ni
  `fase45_sandbox`.
- No borrar historiales o evidencia de intentos fallidos.
- No reescribir Git.
- No implementar distribucion remota ni separar paquetes de Fase 2.
- No introducir workers, colas, publishers o autoridades nuevas.

## Verificacion

### Revision estatica

- ausencia de legacy y enlaces rotos por los contratos retirados;
- ausencia de scheduling, publicacion o autoridad duplicada;
- parametros YAML declarados, consumidos y sincronizados;
- perfiles debug fuera del arranque normal;
- YAML y launches instalados por CMake;
- visualizador local, opcional y ajeno a decisiones ROS;
- `git diff --check` correcto.

### Build y tests

```bash
./codex/herramientas/build_selected_packages.sh \
  orbslam3_multi orbslam3_server simulacion_dron
```

Ejecutar regresiones funcionales y tests nuevos de YAML, sincronizacion,
parametros declarados, launches, enlaces, colas, commits, fusion, optimizacion,
publicacion y visualizador web.

### Prueba integrada

Ejecutar con RViz2 y el visualizador web:

```text
codex/archivos_auxiliares/trayectorias/
  prueba_tipica_rodeo_edificio_dos_fiduciales.yaml
```

Debe conservar ingesta/publicacion, un solo secundario activo, prioridad,
backpressure, revisiones coherentes, scoring, color RViz2, grafo web, drenaje y
recursos sin degradacion material. El objetivo no es mejorar 3Q. Una incidencia
3Q se trata como limitacion conocida salvo evidencia de que la limpieza la
introdujo o agravo. La validacion visual final corresponde al usuario.

## Criterio de exito

`3T` queda `CONSEGUIDA` si:

1. legacy y contratos absorbidos desaparecen sin referencias rotas;
2. no queda duplicacion activa injustificada;
3. el codigo vigente tiene comentarios utiles;
4. ADR, YAML, launches y manifiestos coinciden con el runtime;
5. build, tests y Gazebo no muestran regresiones de limpieza;
6. el resultado final mantiene visible la incidencia 3Q y su mejora futura.

Queda `PARCIAL` si permanece limpieza, configuracion, documentacion o evidencia
pendiente. Queda `NO CONSEGUIDA` si la retirada rompe comportamiento, elimina
evidencia necesaria o deja responsabilidades duplicadas.

Concluir `3T` no cambio por si solo el estado de 3Q. El cierre posterior de 3Q
y de toda la Fase 3 procede de la revision visual y decision expresa del
usuario, no de la limpieza.

## Resultado

El contrato se completo el 2026-08-22. La evidencia de build, tests, limpieza
y Gazebo vive en `historial/por_subfase/historial_3T_RESUMEN.md`; el handoff
operativo completo esta en `RESULTADO_FINAL_FASE_3.md`. La inspeccion visual
humana de la prueba 195 fue confirmada posteriormente como correcta por el
usuario. La Fase 3 queda conseguida y 3Q conserva una mejora futura sin aplicar.

## Riesgos aceptados

- referencias no detectadas al borrar legacy;
- drift entre YAML, mitigado mediante tests;
- cambios de tipos o precedencia al mover defaults;
- ruido documental por exceso de comentarios;
- reproduccion de la incidencia conocida de 3Q en la prueba final.
