# Pipeline Fase 1 — resumen

Usar este archivo antes de abrir `pipeline_fase_1.md` o los contratos completos de subfase.

## Estado vigente

```text
Fase 1: 1A-1K realizadas; cierre tecnico conseguido
Subfases: 1A, 1B, 1C, 1D, 1E, 1F, 1G, 1H, 1I, 1J y 1K
Historial: pitch fisico 1J y limpieza/flag de logs 1K validados
```

El estado `realizado` refleja que los bloques descritos existen en el código de referencia y que el contrato documental de la fase está creado. La evidencia concreta de futuras reconstrucciones, builds y simulaciones se escribirá únicamente cuando se vuelvan a ejecutar las pruebas.

## Objetivo

Reconstruir, entender y validar la cadena completa que permite crear uno o varios drones en Gazebo Classic, dotarlos de actuación y sensores, generar trayectorias, controlarlos en lazo cerrado y enviarles objetivos desde una GUI de simulación y analizar perfiles, estado y errores mediante gráficas.

## Secuencia

```text
1A Gazebo
  -> 1B modelo físico Xacro/URDF
  -> 1C inserción de uno o varios drones
  -> 1D fuerzas y torques por motor
  -> 1E Ground Truth y cámaras
  -> 1F lib_tray
  -> 1G acción, control y mixer de cuatro motores
  -> 1H GUI multi-dron y prueba integral
  -> 1I gráficas de perfiles, GT, referencia y error
  -> 1J pitch fisico del rig stereo
  -> 1K limpieza y cierre de Fase 1
```

El selector previo al joint 1J quedo validado con
`phase5_navigation_source=gt|orb`: permite ejecutar primero las pruebas bajo GT
forzado, con ORB en sombra, y repetirlas despues bajo autoridad ORB estricta.
1J incorpora joint, servo y extrinseca dinamica. 1K retira prototipos sin uso
y añade `debug_fase_1=false` como puerta maestra de telemetria informativa.

## Paquetes principales

```text
simulacion_dron
dron_individual
lib_tray
```

La wiki pública del proyecto se usa como explicación histórica y complemento documental:

```text
https://github.com/IvanCS-Chenfu/TFG/wiki/Explicaci%C3%B3n
```

El código, los launch, los Xacro y los YAML del workspace son la referencia del comportamiento implementado.

## Regla transversal de YAML

Cada paso que dependa de características físicas o de configuración debe usar YAML. Los valores actuales son datos arbitrarios de prueba: no representan un dron real ni deben presentarse como mediciones.

Los contratos deben mantener coherencia entre:

```text
dron_individual/config/hardware.yaml
simulacion_dron/config/sim_dron.yaml
dron_individual/config/tray_dron.yaml
simulacion_dron/urdf/*.xacro
plugins Gazebo
nodos de trayectoria y control
simulacion_dron/config/graficas.yaml
```

Como mínimo se documentan unidades, tamaños de vectores, valores permitidos, límites, valores por defecto y relaciones entre masa, inercia, geometría, sensores, actuadores y control.

## Alcance de motores

- El cierre obligatorio de la fase es un cuadricóptero de **cuatro motores**.
- El Xacro y el plugin de actuación pueden contemplar seis u ocho motores.
- La aparición de modelos de seis u ocho motores no implica que su control funcione.
- `aplicar_fuerzas_dron` y el criterio de control de esta fase se limitan a cuatro motores.

## Ground Truth

En esta fase se acepta que `gen_tray` y `control_calcular_fuerzas` usen:

```text
sensor/GT/pose
sensor/GT/vel
```

El plugin también publica `sensor/GT/acc`. Esta dependencia debe quedar localizada y documentada. En la **Fase 5** se sustituirá como entrada funcional del control por la estimación local-global sin GT.

## Puerta de cierre

La fase queda descrita cuando otro chat de Codex puede reconstruir cada bloque, compilar los paquetes afectados, ejecutar las pruebas indicadas y distinguir con precisión:

- qué viene del YAML;
- qué pertenece a Gazebo;
- qué se ejecutaría en un dron;
- dónde entra GT;
- por qué el control obligatorio solo cubre cuatro motores;
- cómo representar perfiles, referencia, GT y errores sin mezclar magnitudes o drones;
- qué resultados deben guardarse posteriormente en `historial/por_subfase/`.

## Siguiente lectura

```text
codex/pipeline/fase_1_control_dron/pipeline_fase_1.md
codex/pipeline/fase_1_control_dron/subfases/subfase_1A.md
```
