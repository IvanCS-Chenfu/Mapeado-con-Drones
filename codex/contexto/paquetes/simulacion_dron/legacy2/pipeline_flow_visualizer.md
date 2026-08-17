# Visualizador `pipeline_flow`

## Componentes

```text
src/visualizer/pipeline_flow_bridge.py
web/pipeline_flow/index.html
web/pipeline_flow/styles.css
web/pipeline_flow/graph_definition.js
web/pipeline_flow/app.js
web/pipeline_flow/vendor/cytoscape.min.js
```

## Bridge

`PipelineFlowBridge : rclpy.Node` subscribe un topic `std_msgs/String` con JSON
y guarda los ultimos `512` payloads en `EventStore`. Un
`ThreadingHTTPServer` enlazado a `127.0.0.1` sirve los assets y `/events` como
Server-Sent Events.

Cada handler SSE inicializa actualmente su secuencia local a `0`; no usa
`Last-Event-ID`. Al conectar o reconectar entrega todos los eventos que aun
queden en el `deque`, hasta `512`, aunque sean anteriores a la conexion.

Parametros:

```text
topic=/global_mapping/flow_events
port=8765
web_root=<share/simulacion_dron/web/pipeline_flow>
```

El shutdown captura `ExternalShutdownException`, cierra HTTP y une el thread.
No existe canal navegador -> ROS.

## Topologia

`graph_definition.js` contiene nodos y aristas estables. Cada nodo incluye
resumen/ownership; cada arista incluye payload y categoria. Hay aristas
paralelas separadas para:

- wrapper delta y snapshot;
- commits raw delta y snapshot;
- pose de KF nuevo y reconciliacion snapshot;
- nube y KFs hacia RViz2.

La paleta semantica distingue ingesta, pose, secundario, estado derivado y
publicacion.

## Actividad

`app.js` mantiene una cola visual maxima de `400` eventos y consume uno cada
`110 ms`. Los pulsos principales duran aproximadamente `520 ms`. Para tareas
secundarias, el mapa `task_id -> etapa` apaga la etapa previa antes de encender
la siguiente; el inicio de una nueva tarea limpia la anterior, coherente con un
solo worker.

Los contadores de referencia evitan apagar un nodo compartido mientras otra
arista sigue activa. La barra lateral muestra los eventos recientes y el
tooltip presenta el contrato completo de nodo/arista.

## Limitaciones live diagnosticadas

- El consumo fijo equivale a `9.09` eventos/s. Con `400` pendientes, el evento
  visible puede estar unos `44 s` por detras.
- Como cada pulso dura `520 ms`, alrededor de cinco eventos no-task pueden
  permanecer activos simultaneamente y encender varios nodos.
- Una reconexion vuelve a introducir hasta `512` eventos antiguos y produce un
  replay adicional.
- Cuando la cola supera `400`, se borran los eventos mas antiguos sin indicar
  gap ni saltar directamente al ultimo estado.
- Los eventos incluyen `stamp_ns`, pero la UI no calcula ni muestra latencia
  backend->SSE->render; los logs solo prueban `READY`, no frescura visual.

## Launch

`simulacion_dron/launch/multi_dron.launch.py` inicia el bridge y opcionalmente
abre `http://127.0.0.1:<port>` con `xdg-open`. Ambos procesos son observabilidad
y no forman parte del criterio funcional de ROS.

## Validacion

- Captura desktop: grafo Cytoscape no vacio, leyenda y topologia completas.
- Evento sintetico `wrapper_server_delta`: recibido por ROS y expuesto en SSE.
- `prueba_75/76`: marcador `[F1U-FLOW-WEB-READY]` y cierre limpio.
- La observacion humana posterior detecto retraso, solapamiento y aristas/nodos
  aparentemente fuera de tiempo; el comportamiento concuerda con la cola y la
  semantica de reconexion descritas arriba.
