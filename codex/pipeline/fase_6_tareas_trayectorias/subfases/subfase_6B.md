# Subfase 6B - Geometria y semantica de MAP_SECTION

## Estado

```text
sin hacer
```

## Dependencia

6A.

## Objetivo tecnico

Dividir el ROI en niveles y cuatro responsabilidades regionales solapadas por
nivel, y formalizar ownership base/ramas sin convertirlas en rutas.

## Comportamiento esperado

El resto vertical se suma al ultimo nivel. En XY se fija
`A=(xmin,ymin)`, `B=(xmax,ymin)`, `C=(xmax,ymax)` y `D=(xmin,ymax)`. Cada
subROI conserva la longitud completa de su lado y llega exactamente hasta el
centro del slice:

```text
AB = [xmin,xmax] x [ymin,ymid]
BC = [xmid,xmax] x [ymin,ymax]
CD = [xmin,xmax] x [ymid,ymax]
DA = [xmin,xmid] x [ymin,ymax]
```

Cada una ocupa el 50 % del slice. Dos adyacentes solapan en un cuarto del
slice, equivalente a la mitad de cada subROI; las opuestas solo comparten la
frontera central. No existe ratio de solape configurable. No son 4/8 puntos,
un recorrido A-B-C ni un sentido horario obligatorio.

Una `MAP_SECTION` es responsabilidad inicial. Puede navegar por todo
`hard_flight_volume`. Una expansion confirmada por frontier crea ownership 3D,
puede cruzar nivel/subROI y no obliga a mapear fuera de `mapping_roi`.

Si dos accesos conectan la misma region, se fusiona la identidad: primer owner
para coverage detallada y segunda pasada ligera para loops/covisibilidad. Una
region sin conectividad conocida no provoca busqueda infinita.

Desde este bloque `task_server` publica un snapshot real de geometria
`reliable+transient_local`. Antes de 6E contiene regiones de mision sin
asignacion, no tareas ficticias. `mission_flow` muestra una vista 2D del nivel
seleccionado y la GUI prepara una `MissionRegionLayer` con el prisma 3D.

## Cambios requeridos

1. Definir `MappingLevel`, cuatro `BaseSubRoi` estables y las formulas exactas
   de mitad/solape anteriores.
2. Cubrir ROI menor que `level_height`, multiplo exacto y resto final.
3. Definir `task_id`, `base_owner`, `branch_owner_task_id` y transferencia.
4. Separar ownership geometrico base de `frontier lineage` dinamico de 6H.
5. Formalizar merge de dos entradas y pasada secundaria ligera.
6. Emitir resumen `MISSION_GEOMETRY_READY` y snapshot de geometria sin puntos,
   tareas ni assignments ficticios.
7. Añadir en el panel derecho `Regiones de mision`; clicar una resalta el
   prisma con relleno translucido, contorno y etiqueta `Nivel N - lado`.
8. Mantener seleccion unica: otra region/entidad la sustituye y clicar vacio la
   limpia. En 6E/7I la misma identidad se enlazara con la tarea real del dron.

## Limites

No detectar puertas/habitaciones semanticamente, no generar waypoints ni
frontiers, no limitar una rama a su slice y no usar el ROI como pared fisica.
No asignar regiones a drones ni declarar 7I conseguida antes de 6E.

## Pruebas

- Geometria determinista de uno/varios niveles y cuatro mitades exactas;
  verificar areas, intersecciones adyacentes y frontera de opuestas.
- Casos conceptuales: abierto, puerta en un lado, rama cruzando niveles, puerta
  cerrada, pasillo L, dos entradas y reapertura posterior.
- Visualizacion de geometria real no asignada en `mission_flow` y GUI con
  Gazebo abierto; probar cambio de region, seleccion de otra entidad y click
  vacio. No se exige movimiento.

## Criterio de exito

Cuatro responsabilidades del 50 % por nivel, intersecciones demostradas,
snapshot real y seleccion GUI coherente, sin `tasks_per_level`, puntos A-B-C,
rutas rigidas ni assignments prematuros.
