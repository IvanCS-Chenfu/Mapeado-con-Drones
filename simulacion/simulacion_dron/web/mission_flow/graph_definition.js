window.FLOW_GRAPH = {
  phase: "6A-6P",
  categories: {
    drone: {label: "Dron", color: "#287c8e", active: "#00a8c6"},
    transport: {label: "Contrato ROS", color: "#a06418", active: "#e58b00"},
    worker: {label: "Worker", color: "#6f5591", active: "#a45cce"},
    state: {label: "Estado", color: "#39734f", active: "#00a35e"}
  },
  nodes: [
    {id: "task_manager", label: "task_manager", description: "Handshake local versionado.", category: "drone", position: {x: 120, y: 210}},
    {id: "registration", label: "RegisterDrone", description: "Servicio idempotente de registro.", category: "transport", position: {x: 390, y: 210}},
    {id: "task_worker", label: "TaskWorker", description: "Asigna MAP_SECTION y gobierna intervalos de fachada.", category: "worker", position: {x: 680, y: 210}},
    {id: "voxel_worker", label: "VoxelMapWorker", description: "OCCUPIED sparse y FREE atravesado/depth reversible.", category: "worker", position: {x: 680, y: 410}},
    {id: "inspection_worker", label: "InspectionWorker", description: "Dos capturas depth bajo demanda y orientación de fachada.", category: "worker", position: {x: 680, y: 610}},
    {id: "planning_worker", label: "PlanningWorker", description: "D* Lite 3D sobre corredor de fachada confirmado FREE.", category: "worker", position: {x: 980, y: 410}},
    {id: "reservation_worker", label: "ReservationWorker", description: "Reserva el volumen físico de cada trayectoria activa.", category: "worker", position: {x: 1280, y: 410}},
    {id: "mission_geometry", label: "MissionGeometry", description: "Snapshot durable de niveles y subROI sin asignar.", category: "state", position: {x: 980, y: 120}},
    {id: "drone_registry", label: "DroneRegistry", description: "Snapshot durable de drones aceptados.", category: "state", position: {x: 980, y: 290}},
    {id: "task_states", label: "TaskStateArray", description: "Lifecycle MAP_SECTION, TO_FINISH e intervalos recorridos.", category: "state", position: {x: 980, y: 470}},
    {id: "voxel_map", label: "VoxelMap", description: "Snapshot reversible OCCUPIED/FREE para GUI y cambios incrementales hacia D*.", category: "state", position: {x: 980, y: 610}},
    {id: "planned_routes", label: "PlannedRoute", description: "Ruta ejecutable y lifecycle ACTIVE/terminal compartido con GUI.", category: "state", position: {x: 1280, y: 610}}
  ],
  edges: [
    {id: "manager_to_registration", source: "task_manager", target: "registration", label: "request", category: "transport"},
    {id: "registration_to_task_worker", source: "registration", target: "task_worker", label: "validated", category: "transport"},
    {id: "task_worker_geometry", source: "task_worker", target: "mission_geometry", label: "snapshot", category: "state"},
    {id: "task_worker_registry", source: "task_worker", target: "drone_registry", label: "snapshot", category: "state"},
    {id: "task_to_voxel", source: "task_worker", target: "voxel_worker", label: "actualización", category: "worker"},
    {id: "task_to_inspection", source: "task_worker", target: "inspection_worker", label: "objetivo fachada", category: "worker"},
    {id: "inspection_to_voxel", source: "inspection_worker", target: "voxel_worker", label: "FREE depth", category: "worker"},
    {id: "voxel_to_task_states", source: "voxel_worker", target: "task_states", label: "map revision", category: "state"},
    {id: "task_worker_states", source: "task_worker", target: "task_states", label: "assign", category: "state"},
    {id: "voxel_worker_map", source: "voxel_worker", target: "voxel_map", label: "snapshot", category: "state"},
    {id: "voxel_to_planning", source: "voxel_worker", target: "planning_worker", label: "MapChangeEvent", category: "worker"},
    {id: "task_worker_to_planning", source: "task_worker", target: "planning_worker", label: "objetivo XYZ", category: "worker"},
    {id: "planning_to_route", source: "planning_worker", target: "planned_routes", label: "D* plan/replan", category: "state"},
    {id: "planning_to_reservation", source: "planning_worker", target: "reservation_worker", label: "reserva", category: "worker"}
  ]
};
