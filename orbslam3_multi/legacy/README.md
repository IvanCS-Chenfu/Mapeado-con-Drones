# Legacy `orbslam3_multi`

Estos archivos son copias congeladas del diseño anterior de `orbslam3_multi`.

Reglas:

- No forman parte del build.
- No deben recibir funcionalidad nueva.
- No deben ser incluidos desde codigo activo.
- Solo sirven como referencia durante la migracion de Fase 1.

La implementación activa validada tras `1C` ya no conserva estos módulos en
`src/` ni en `include/`. Solo mantiene:

```text
orbslam3_multi/include/orbslam3_multi/raw_map_types.hpp
orbslam3_multi/include/orbslam3_multi/raw_map_database.hpp
orbslam3_multi/src/raw_map_database.cpp
```

Las clases nuevas reales de la planificación empiezan en `1C` con
`RawMapDatabase`. Si se necesita consultar el diseño anterior, usar estos
archivos `_antiguo`; no reintroducirlos en el build activo sin una subfase
explícita.
