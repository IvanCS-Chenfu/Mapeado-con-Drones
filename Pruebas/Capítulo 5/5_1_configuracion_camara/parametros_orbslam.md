# Configuración de cámara en ORB-SLAM3

## YAML efectivo

El launch `dron/dron_individual/launch/orbslam_use.launch.py` toma por defecto
`dron/dron_individual/config/orbslam/orbslam_stereo.yaml`. Sus parámetros de
calibración son:

| Parámetro | Valor |
|---|---:|
| `Camera.type` | `PinHole` |
| `Camera.fx` / `Camera.fy` | `286.02185016085167` |
| `Camera.cx` / `Camera.cy` | `240.5` / `180.5` |
| `Camera.width` / `Camera.height` | `480` / `360` |
| `Camera.fps` | `20.0` |
| `Camera.bf` | `16.303245459168547` |
| `Camera.k1`, `k2`, `p1`, `p2` | `0.0` |
| `Camera.bFishEye` | `0` |
| `ORBextractor.nFeatures` | `900` |
| `ORBextractor.scaleFactor` | `1.2` |
| `ORBextractor.nLevels` | `8` |
| `loopClosing` | `0` |

La relación entre `bf`, `fx` y la línea base es coherente:

```text
bf / fx = 16.303245459168547 / 286.02185016085167 = 0.057 m
```

Además, con el campo de visión horizontal del `.xacro`:

```text
fx = 480 / (2 * tan(1.3962634 / 2)) = 286.02185016085167
```

Por tanto, las intrínsecas publicadas por `CameraInfo` y las utilizadas por
ORB-SLAM3 son coherentes entre sí.

## YAML alternativo localizado en simulación

También existe `simulacion/simulacion_dron/config/orbslam/orbslam_stereo.yaml`,
pero no es el valor por defecto del launch de `dron_individual`. Ese archivo
contiene una calibración distinta (`640x480`, `fx=435.2046959714599`,
`bf=26.1122817582876` y `1200` features). Debe tratarse como configuración
alternativa o histórica y no mezclarse con la configuración efectiva anterior.

## Distorsión y rectificación

La prueba real devuelve `distortion_model: plumb_bob` con los cinco valores de
distorsión a cero. Las matrices `R` y `P` observadas son identidad y proyección
sin desplazamiento horizontal, respectivamente. El YAML de ORB-SLAM3 refleja
la misma ausencia de distorsión mediante `k1`, `k2`, `p1` y `p2` iguales a cero.
