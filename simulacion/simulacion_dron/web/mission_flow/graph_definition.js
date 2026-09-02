window.FLOW_GRAPH = {
  phase: "6A-6C",
  categories: {
    drone: {label: "Dron", color: "#287c8e", active: "#00a8c6"},
    transport: {label: "Contrato ROS", color: "#a06418", active: "#e58b00"},
    worker: {label: "Worker", color: "#6f5591", active: "#a45cce"},
    state: {label: "Estado", color: "#39734f", active: "#00a35e"}
  },
  nodes: [
    {id: "task_manager", label: "task_manager", description: "Handshake local versionado.", category: "drone", position: {x: 120, y: 210}},
    {id: "registration", label: "RegisterDrone", description: "Servicio idempotente de registro.", category: "transport", position: {x: 390, y: 210}},
    {id: "task_worker", label: "TaskWorker", description: "Writer de misión, geometría y lifecycle.", category: "worker", position: {x: 680, y: 210}},
    {id: "voxel_worker", label: "VoxelMapWorker", description: "Reservado para 6D; inactivo en este bloque.", category: "worker", position: {x: 680, y: 410}},
    {id: "planning_worker", label: "PlanningWorker", description: "Reservado para 6G-6I; inactivo.", category: "worker", position: {x: 980, y: 410}},
    {id: "reservation_worker", label: "ReservationWorker", description: "Reservado para 6J; inactivo.", category: "worker", position: {x: 1280, y: 410}},
    {id: "mission_geometry", label: "MissionGeometry", description: "Snapshot durable de niveles y subROI sin asignar.", category: "state", position: {x: 980, y: 120}},
    {id: "drone_registry", label: "DroneRegistry", description: "Snapshot durable de drones aceptados.", category: "state", position: {x: 980, y: 290}}
  ],
  edges: [
    {id: "manager_to_registration", source: "task_manager", target: "registration", label: "request", category: "transport"},
    {id: "registration_to_task_worker", source: "registration", target: "task_worker", label: "validated", category: "transport"},
    {id: "task_worker_geometry", source: "task_worker", target: "mission_geometry", label: "snapshot", category: "state"},
    {id: "task_worker_registry", source: "task_worker", target: "drone_registry", label: "snapshot", category: "state"},
    {id: "task_to_voxel", source: "task_worker", target: "voxel_worker", label: "6D", category: "worker"},
    {id: "voxel_to_planning", source: "voxel_worker", target: "planning_worker", label: "6G", category: "worker"},
    {id: "planning_to_reservation", source: "planning_worker", target: "reservation_worker", label: "6J", category: "worker"}
  ]
};
