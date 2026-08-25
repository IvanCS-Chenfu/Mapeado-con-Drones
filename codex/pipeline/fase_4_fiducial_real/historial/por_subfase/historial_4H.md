# Historial 4H

## 2026-08-25 - Handoff visual, retirada GT y regresion integral

- objetivo: alimentar `FiducialAnchorManager` con primary visual y eliminar GT fiducial;
- cambios: servidor, launch/config/CMake, grafos/bridge/contratos y tests backend;
- builds: `orbslam3_multi`, `orbslam3_server` y `simulacion_dron`, correctos;
- CTest final: Servidor 150 y Simulacion 85 tests sin fallos;
- prueba 214, GT compilado pero OFF: 80/80 primary, tres objetos, cinco anchors;
- prueba 215: 73/73 primary y cuatro anchors. Derivas reales activan el manager;
  tareas existentes `loop_submap_window_too_small` no bloquean;
- retirada GT: subscription/callback/buffer, conversion body-camera,
  parametros/radios, header/test y arista web eliminados; replay solo visual;
- prueba 216 sin codigo GT: 52 batches/52 primary, objetos 1/2/3, ambos drones,
  52 PUB/SHOW, cero unknown/no-primary y ninguna referencia GT runtime;
- smoke 217: ambos bridges/navegadores READY/live y cierre limpio;
- trafico 214: 14464 bytes/80 batches/94 tags, 180.8 bytes/batch, maximo 4 Hz;
- conclusion: CONSEGUIDA;
- pendiente externo: revisar `loop_submap_window_too_small` en el backend correspondiente.
