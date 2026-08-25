# Subfase 4A - Contrato geometrico y configuracion de objetos fiduciales visuales

## Estado

```text
CONSEGUIDA el 2026-08-24
Preparacion: cerrada junto con 4B
Acuerdo: confirmado y ejecutado
Autorizacion funcional: consumida para el bloque 4A+4B
Prueba: contratos, builds y Gazebo 201/202 correctos
```

Evidencia: `historial/por_subfase/historial_4A_RESUMEN.md`.

## Detalle largo

```text
subfases/detalle/subfase_4A_DETALLE.md
```

Este archivo es el contrato ejecutable corto. El detalle conserva el diseño
importado desde `Fase_4_completa_4A_4I_muy_detallada.zip`.

## Objetivo

Definir el contrato geometrico de Fase 4: objetos `box`, caras soportadas,
`tag_id`, `object_id`, dimensiones, `size_m`, convenciones de frame,
`object_T_tag`, ownership YAML y validadores necesarios antes de crear modelos
o detectar tags.

## Decisiones activas

- baseline: tres objetos fiduciales en Gazebo con IDs 101-105, 201-205 y 301-305;
- poses baseline: `(0,+8.5,1)`, `(0,-8.5,1)` y `(+8.5,0,1)`;
- zona fiducial configurable por el usuario, inicialmente `[1,5] m`;
- Servidor es autoridad semantica y Simulacion aporta el perfil de deployment;
- el scoring 3R conserva su configuracion actual y se revisara en el futuro si
  conviene compartir o separar ambos rangos;
- `size_m` es el lado fisico del cuadrado AprilTag, no el tamano de la cara;
- `tag_id` identifica caras y puede ser `0` si el diccionario lo admite;
- el wrapper no conoce `object_id`, caras, `object_T_tag` ni `world_T_object`;
- Servidor interpreta semantica y Simulacion conserva solo lo especifico de Gazebo;
- cualquier cambio de interfaces/configuracion debe reflejarse en `system_architecture`.

## Archivos probables al ejecutar

- configuracion fiducial de Servidor y Simulacion;
- replicas YAML permitidas por ADR 0009;
- validadores/guardas de consistencia;
- metadata de `system_architecture` si nace una relacion/configuracion nueva.

## Prohibido

- usar GT como entrada funcional;
- hacer que Dron lea YAML de Servidor o Simulacion;
- implementar deteccion visual o spawn Gazebo en 4A;
- modificar ORB-SLAM3.

## Pruebas requeridas

Validar baseline de tres objetos, prisma rectangular, `size_m` mayor que cara,
IDs duplicados, IDs fuera de diccionario, ID cero, tamano invalido,
configuracion vacia de despliegue real y ownership/replicas con
`system_architecture`. Validar tambien `min_distance_m=1.0`,
`max_distance_m=5.0` y el rechazo de rangos no finitos o desordenados.

## Paquetes

Compilar solo los paquetes afectados por la configuracion/validadores que se
modifiquen en 4A, previsiblemente `simulacion_dron` y/o paquetes de Servidor
segun el punto exacto acordado durante la preparacion.

## Criterio de exito

YAML/configuracion validos, IDs unicos, geometria inequivoca, ownership conforme
ADR 0009, guardas verdes y documentacion sincronizada. No se marca conseguida
sin build, pruebas/logs y cierre documental real.
