# ADR 0009 - Configuracion por dominio y modo de despliegue

## Estado

Aceptada y aplicada en Fase 2 (2026-08-24).

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
- Fase 2 movio fisicamente los paquetes sin volver a decidir ownership;
- un parametro puede pertenecer al dron aunque hoy lo consuma el servidor.

`body_T_camera` representa calibracion del dron. Cada despliegue conserva una
replica parcial local verificada; el transporte remoto por TF o por un contrato
de calibracion permanece como evolucion futura.

## Fuera de alcance

- distribuir parametros remotamente mediante services o actions;
- cambiar algoritmos o valores validados;
- redisenar de nuevo los tres directorios definitivos;
- tratar los parametros exclusivos de Gazebo como configuracion de servidor.
