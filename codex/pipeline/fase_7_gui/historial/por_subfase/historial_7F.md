# Historial 7F

## 2026-09-02 - Implementacion y pruebas

Se consolidaron toggles y geometrias de drones, KFs y fiduciales, labels
permanentes `D<n>`/`F<n>` y logs agregados. La pose visual usa exclusivamente
`NavigationState` estimado; stale conserva la ultima pose y baja opacidad.

378R mostro dos drones, sparse global, hasta 53 KFs y tres objetos fiduciales;
el usuario valido la GUI. 378RR cerro con 46 KFs, tres fiduciales cargados,
RViz ausente y shutdown limpio. El inspector de KF no muestra dron/submapa:
`KeyframeVisual` y `/global_keyframes` no contienen esa metadata, y por acuerdo
no se ampliaron topics ni productores en este bloque.

Conclusion: `CONSEGUIDA` dentro del contrato real disponible.
