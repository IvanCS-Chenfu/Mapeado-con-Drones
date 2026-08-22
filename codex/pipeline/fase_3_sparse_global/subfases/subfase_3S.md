# Subfase 3S - Perfil de debug y observabilidad opcional

## Estado

```text
CONSEGUIDA
```

## Objetivo

Permitir que el launch normal de simulacion ejecute el mapa sparse global sin
abrir herramientas visuales ni inundar la terminal con diagnosticos de Fase 3.
La configuracion debe residir en un YAML legible y no alterar el procesamiento,
las colas ni las publicaciones funcionales.

## Configuracion

Crear `simulacion_dron/config/fase3_debug.yaml` con cuatro booleanos:

```text
fase3_rviz2
fase3_grafo_web
fase3_abrir_navegador_web
fase3_logs_terminal
```

Todos quedan inicialmente en `false`. El launch usa esos valores como defaults
y permite sobrescribirlos mediante argumentos homonimos.

- `fase3_rviz2=false`: no crea el proceso RViz2 sparse;
- `fase3_grafo_web=false`: no crea el bridge/servidor web;
- `fase3_abrir_navegador_web=false`: no crea el helper del navegador;
- `fase3_logs_terminal=false`: ejecuta `global_map_server` con nivel ROS
  `ERROR`, suprimiendo diagnosticos `[F3*]` pero conservando errores reales.

El helper del navegador solo puede arrancar cuando tanto el grafo como su
propio booleano estan activos.

## Alcance

- leer el YAML al construir `multi_dron.launch.py` y fallar pronto si falta una
  clave o no es booleana;
- añadir un argumento `log_level` al launch standalone del servidor;
- mantener el perfil como configuracion exclusiva del despliegue simulado;
- instalar el YAML mediante el `config/` ya exportado por CMake;
- proteger claves, defaults y consumidores mediante tests.

## Exclusiones

- no desactivar `/global_sparse_cloud`, `/global_keyframes`, backpressure ni
  eventos internos;
- no cambiar algoritmos, prioridades, score, fusion u optimizacion;
- no ocultar errores `ERROR`/`FATAL`;
- no modificar los launches replay, que conservan sus controles explicitos;
- no corregir 3Q.

## Prueba acordada

Ejecutar `prueba_debug_fase3_silencioso.yaml`: dos drones arrancan tracking,
llegan al fiducial 2, avanzan unos metros y esperan un drenaje corto.

Comprobar mediante log reducido y procesos del launch:

1. escenario y acciones terminan con `success=true`;
2. no arrancan `sparse_global_rviz`, `pipeline_flow_bridge` ni
   `pipeline_flow_browser`;
3. no aparece ningun marcador `[F3*]` si no existe un error real;
4. `global_map_server` permanece vivo hasta el cierre;
5. no aparecen errores graves ni fallos de acciones.

## Criterio de exito

3S queda `CONSEGUIDA` si YAML, launch y tests coinciden y la prueba corta
cumple los cinco criterios. Queda `PARCIAL` si el mapa funciona pero alguna
herramienta o diagnostico sigue activo. Queda `NO CONSEGUIDA` si el perfil
impide arrancar el servidor, oculta errores reales o rompe el escenario.

## Riesgos aceptados

- un nivel `ERROR` reduce la evidencia diagnóstica disponible durante una
  ejecución normal;
- una combinación manual incoherente de grafo apagado y navegador encendido se
  resuelve no iniciando el navegador;
- la ausencia de marcadores obliga a usar éxito del escenario, lifecycle de
  procesos y ausencia de errores como evidencia de esta prueba.

## Resultado

Build 3/3, CTest 9/9 + 10/10 + 8/8 y prueba 196 `success=true`. El servidor
permanece operativo, no se crean procesos visuales y no aparece ningun marcador
`[F3*]`. Evidencia resumida en `historial_3S_RESUMEN.md`.
