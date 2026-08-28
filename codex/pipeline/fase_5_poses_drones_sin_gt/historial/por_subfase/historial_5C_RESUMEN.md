# Historial 5C - resumen

## Estado

```text
Conclusion: CONSEGUIDA
Prueba integrada: 230
```

`SparseGlobalBackend::QueryGlobalPose()` reutiliza `GlobalPoseStore` y devuelve
`AVAILABLE/PENDING/UNKNOWN/INVALID_EPOCH`. Build `orbslam3_multi` correcto y
CTest 9/9. Gazebo 230 observa consultas PENDING y AVAILABLE para ambos drones.
