# ADR 0009 - Configuracion por dominio y modo de despliegue

<!-- ACUERDOS_CIERRE_F2_2026_08_24_START -->
## Extensión: ownership, authority y deployment profile

> **Vigencia:** acuerdo cerrado el 2026-08-24. Este bloque prevalece sobre cualquier
> frase anterior incompatible del mismo documento. No borra ni reescribe evidencia
> histórica; distingue siempre entre estado actual, deuda conocida y arquitectura objetivo.

La decisión se amplía con tres conceptos independientes:
1. **semantic ownership**: quién posee conceptualmente el dato;
2. **authority/control**: quién decide su valor;
3. **deployment source/profile**: desde qué copia local lo carga un despliegue.

Dron es caja negra. Un YAML ROS no distribuye parámetros remotamente.

Réplicas:
- accidental/semánticamente duplicada: prohibida;
- parcial declarada: permitida con claves y regla de igualdad;
- completa declarada: permitida solo para un deployment profile justificado y guardado.
`global_map` Servidor↔Simulación es una réplica completa deliberada y se mantiene.

Contratos futuros:
- valor controlado por Servidor y consumido en Dron: cliente Dron al arrancar → servicio
  de configuración Servidor → valor local Dron;
- intrínseco Dron requerido por Servidor (`body_T_camera`): TF o contrato de calibración
  Dron→Servidor.
No se implementan estos servicios en Fase 2 si no existe necesidad actual.

Reloj:
- standalone Dron/Servidor false;
- Simulación hace override true.
Defaults C++ son fallback/tipo, no perfiles operacionales ocultos.
<!-- ACUERDOS_CIERRE_F2_2026_08_24_END -->

## Estado

Aceptada para preparar la separacion de Fase 2.

## Contexto

El proyecto debe funcionar tanto con un servidor y drones reales como dentro
de una simulacion que arranca todos los procesos. Hasta 3T, muchos defaults del
servidor estaban repartidos entre C++, el launch de `orbslam3_server` y el
launch de `simulacion_dron`, sin una configuracion operacional completa.

Un archivo de parametros ROS 2 configura al nodo que lo carga; no distribuye
automaticamente esos valores a nodos ejecutados en otras maquinas.

## Decision

La configuracion se clasifica por responsabilidad semantica:

- **dron**: masa, inercia, controladores, calibracion y propiedades fisicas;
- **servidor**: fusion, scoring, loops, optimizacion, colas, snapshots y
  politica global;
- **simulacion**: mundo, modelos, ruido, GUI, escenarios y comportamiento
  exclusivo de Gazebo.

Los parametros controlables por el servidor tienen dos copias deliberadas:

- `orbslam3_server/config/global_map/` para despliegue standalone o real;
- `simulacion_dron/config/global_map/` para el despliegue Gazebo.

Cada launch carga solo la copia de su despliegue. Mientras ambas representen la
misma configuracion validada, un test exige igualdad exacta de claves y valores.
Una divergencia futura debera ser intencional, documentada y cubierta por un
perfil o criterio explicito.

Los parametros se separan en `runtime`, `fiducials`, `optimization`,
`loop_fusion`, `scoring` y `replay_debug`. Este ultimo no se carga en ejecucion
normal. Los launch arguments se reservan para identidad, seleccion de perfil y
valores operativos que cambian por ejecucion.

Los defaults C++ permanecen como fallback defensivo y fuente de tipos, no como
perfil operacional oculto.

## Consecuencias

- el servidor real y la simulacion son desplegables de forma independiente;
- existe duplicacion de datos controlada por una prueba de sincronizacion;
- CMake debe instalar los YAML de cada paquete;
- Fase 2 podra mover fisicamente paquetes sin volver a decidir ownership;
- un parametro puede pertenecer al dron aunque hoy lo consuma el servidor.

`body_T_camera` es el caso principal: representa calibracion del dron. El
servidor lo consume temporalmente para el fiducial simulado, pero Fase 2 debera
definir su transporte mediante TF o un contrato de calibracion.

## Fuera de alcance

- distribuir parametros remotamente mediante services o actions;
- cambiar algoritmos o valores validados;
- mover ya los paquetes a los tres directorios definitivos;
- tratar los parametros exclusivos de Gazebo como configuracion de servidor.
