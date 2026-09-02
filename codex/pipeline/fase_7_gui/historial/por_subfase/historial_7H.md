# Historial 7H

## 2026-09-02 - Implementacion y pruebas

La seleccion conserva `EntityKey`, elige por distancia en pantalla y desempata
por profundidad. Soporta MapPoint, dron, KF, fiducial, trayectoria y voxel;
limpia de forma segura entidades desaparecidas y dibuja highlight/inspector.

El test determinista cubre superposicion en profundidad y 100k candidatos en
menos de 500 ms. Durante 378R el usuario genero picks reales de MapPoints, KFs
y fiduciales, ademas de limpiezas al clicar vacio. No se enviaron tareas.
378RR verifico cierre limpio del proceso GUI.

Conclusion: `CONSEGUIDA`.
