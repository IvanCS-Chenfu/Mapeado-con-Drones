# Historial 4A - resumen

```text
Estado agregado: CONSEGUIDA
Ultimo estado bueno: prueba 202
Pendiente: ninguno propio de 4A
```

Se implemento el contrato canonico de Servidor y su replica exacta de
Simulacion para tres objetos `box` a ±8.5 m, 15 IDs unicos y rango configurable
inicial `[1,5] m`. Los validadores aceptan `tag_id=0` y tags mayores que la
cara, pero rechazan geometria, IDs y rangos invalidos.

Evidencia vigente: builds aislados correctos de `simulacion_dron` y
`orbslam3_server`, CTest 10/10 en ambos, guarda completa 15/15 y pruebas 201/202
con 15 texturas verificadas, 3 SDF y 3 objetos creados.

No repetir: el primer CTest bajo sandbox no pudo escribir `LastTest.log`; el
primer pase real detecto solo estilo. Un pytest posterior tampoco ejecuto por
tres rutas mal nombradas; la repeticion correcta paso 38/38.
