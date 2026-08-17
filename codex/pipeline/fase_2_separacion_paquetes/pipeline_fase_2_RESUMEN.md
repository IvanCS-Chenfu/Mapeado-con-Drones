# Pipeline Fase 2 — Resumen

Usar este archivo antes de abrir `pipeline_fase_2.md` o los contratos de las
subfases.

## Estado

```text
Fase 2: SIN HACER
Preparación documental: CERRADA
Acuerdo funcional: CERRADO
Ejecución: NO AUTORIZADA POR ESTE DOCUMENTO
Dudas abiertas de arquitectura: ninguna
```

## Objetivo

Reorganizar el workspace en tres grupos físicos dentro de `src/`:

```text
src/dron/
src/servidor/
src/simulacion/
```

Cada grupo debe representar un despliegue distinto:

- `dron`: software que se instalaría en cada dron;
- `servidor`: mapa global y coordinación central;
- `simulacion`: Gazebo, escenarios, adaptadores, herramientas visuales y launch
  de integración.

`simulacion` puede y debe depender de `dron` y `servidor`. En cambio, `dron` y
`servidor` deben poder compilarse de forma aislada, sin encontrar paquetes del
otro grupo.

## Distribución acordada

```text
src/
├── dron/
│   ├── dron_individual/
│   ├── lib_tray/
│   ├── ORB_SLAM3/
│   ├── orbslam3_ros2/
│   └── orbslam3_msgs/
├── servidor/
│   ├── orbslam3_multi/
│   ├── orbslam3_server/
│   └── orbslam3_msgs/
├── simulacion/
│   └── simulacion_dron/
└── codex/
```

`orbslam3_msgs` se copia completo en `dron` y `servidor`, conservando el mismo
nombre ROS 2. La copia canónica es la de `servidor`; una guarda automática debe
comprobar que la copia de `dron` no diverge.

## Estrategia principal de build

Tras borrar por completo `build/`, `install/` y `log/`, la idea principal es
mantener espacios separados por grupo:

```text
build/{dron,servidor,simulacion}/
install/{dron,servidor,simulacion}/
log/{dron,servidor,simulacion}/
```

Cada grupo se descubre con `--base-paths` y se compila paquete a paquete en
orden topológico. `simulacion` se construye después de cargar los prefijos de
`dron` y `servidor`.

Si esta estrategia produce un bloqueo real no resoluble de forma razonable, las
alternativas, en este orden, son:

1. usar workspaces de build temporales por grupo manteniendo el código bajo las
   tres carpetas acordadas;
2. reutilizar un único `build/install/log`, limpiándolo antes de cambiar de
   grupo;
3. compilar con selección/exclusión explícita de paquetes y documentar la
   limitación.

No se cambia de estrategia por comodidad: primero debe registrarse el fallo
concreto y comprobarse que no es una ruta, dependencia o script corregible.

## Política de YAML

- Un paquete solo carga YAML instalados dentro de su mismo grupo.
- Dentro de un grupo, un launch puede cargar YAML de cualquier paquete del
  grupo mediante el índice de paquetes instalado.
- Si un grupo necesita un dato conceptual de otro, crea un YAML local parcial
  con sufijo `_<grupo_origen>`, por ejemplo `physical_dron.yaml` en
  `simulacion`.
- No se copia el YAML completo: solo las claves realmente consumidas.
- Las claves replicadas se documentan y se comparan automáticamente.
- No se repite un mismo dato dentro de un grupo.
- `mass_total` e `inertia_total` viven en el YAML físico del grupo `dron` y son
  consumidos por trayectoria/control desde launch.
- Las masas e inercias por enlace del modelo simulado son independientes; la
  Fase 2 no recalcula la inercia total ni rediseña profundamente el URDF.
- Cada función de debug dispone de su propio flag y todos quedan `false` por
  defecto.

## Subfases

| ID | Nombre | Salida principal |
|---|---|---|
| 2A | Crear grupos y mover paquetes | Árbol `src/dron`, `src/servidor`, `src/simulacion` |
| 2B | Limpiar y compilar por grupos | Builds aislados y corrección de dependencias/rutas |
| 2C | Reorganizar YAML y launch | Configuración con ownership, sin duplicados internos |
| 2D | Comprobar el funcionamiento | Dos drones rodean el edificio con el flujo completo |
| 2E | Actualizar contexto de Codex | MD, índices, herramientas y rutas coherentes |
| 2F | Crear diagrama del sistema | Vista estática y actividad en vivo, separada de `pipeline_flow` |
| 2G | Guardas y cierre | Verificaciones automáticas y regresión final |

## Prueba final

La prueba funcional oficial es la vuelta completa al edificio con dos drones,
usando el escenario típico existente y comunicación ROS 2 directa mediante
topics, services y actions.

La prueba base se ejecuta con todos los flags de debug en `false`. Después se
realizan comprobaciones específicas de los dos visualizadores, activando solo
los flags necesarios y devolviéndolos a `false` al cerrar.

## Exclusiones

- No implementar Bluetooth, serialización propia ni bridges de transporte.
- No cambiar algoritmos de control, SLAM, fusión u optimización salvo una
  corrección mínima imprescindible para conservar el comportamiento tras la
  reorganización.
- No rediseñar las interfaces de `orbslam3_msgs`.
- No recalcular automáticamente la inercia completa desde el URDF.
- No fusionar el visualizador arquitectónico con el visualizador interno
  `pipeline_flow`.
- No crear historiales antes de ejecutar las subfases.
