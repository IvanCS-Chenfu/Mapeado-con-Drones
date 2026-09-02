# Subfase 6H - Exploracion, frontier lineage y coverage

## Estado

```text
sin hacer
```

## Dependencia

6B, 6D, 6E y 6G.

## Objetivo tecnico

Elegir que objetivo XYZ observar dentro de una `MAP_SECTION`, gestionar ramas
3D y decidir completion sin perseguir UNKNOWN detras de paredes.

## Comportamiento esperado

Los frontiers separan FREE conocido de UNKNOWN potencialmente alcanzable y se
agrupan en clusters. Una rama no se reclama por verla: nace al cruzar/explorar
un frontier y confirmar nuevo FREE. Los nuevos voxeles/frontiers heredan
`lineage_id` y `owner_task_id`.

Si otro linaje conecta el mismo FREE, se fusiona la region, se conserva el
primer owner detallado y el segundo dron puede hacer pasada simple para
loop/covisibilidad. La responsabilidad puede cruzar subROIs y niveles.

El score inicial combina sin pesos inventados: ganancia de coverage, superficie,
distancia preferida, continuidad, coste de viaje, riesgo, UNKNOWN y clearance.
MP score/depth endpoints aportan coverage de superficie; rayos FREE no.

## Cambios requeridos

1. Detectar/agrupar frontiers alcanzables desde FREE conectado.
2. Implementar lineage, claim tras expansion confirmada, herencia y merge.
3. Seleccionar candidatos; pasar objetivo XYZ a D*, no una ruta prehecha.
4. Mantener coverage separada de occupancy y ownership base/branch.
5. Finalizar cuando no haya frontiers utiles alcanzables, coverage sea suficiente y ramas esten cerradas/no alcanzables.
6. Reabrir region si aparece conectividad nueva durante la mision.
7. Evitar intentos infinitos y registrar causas/lineage en grafo.

## Limites

No detector semantico obligatorio de habitaciones, no patron A-B-C y no exigir
que todo UNKNOWN desaparezca. No duplicar coverage detallada por segunda puerta.

## Pruebas

ROI abierto, puerta unica, rama cruzando niveles, puerta cerrada, pasillo L,
dos entradas/merge, reapertura y region inaccesible. Verificar lineage,
completion y ausencia de loops infinitos con GUI+Gazebo+grafo.

## Criterio de exito

La tarea cubre regiones accesibles y ramas propias, fusiona accesos duplicados
y termina justificadamente sin confundir desconocido con pendiente alcanzable.
