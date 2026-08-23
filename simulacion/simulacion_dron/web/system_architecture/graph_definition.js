window.SYSTEM_ARCHITECTURE = {
  "version": 1,
  "groups": ["dron", "servidor", "simulacion"],
  "nodes": [
    {"data": {"id": "group_dron", "label": "Dron", "kind": "group", "group": "dron", "role": "Software desplegado en cada dron"}},
    {"data": {"id": "group_servidor", "label": "Servidor", "kind": "group", "group": "servidor", "role": "Mapa sparse global y coordinacion"}},
    {"data": {"id": "group_simulacion", "label": "Simulacion", "kind": "group", "group": "simulacion", "role": "Gazebo, escenarios y observabilidad"}},
    {"data": {"id": "dron_individual", "label": "dron_individual", "kind": "package", "group": "dron", "parent": "group_dron", "role": "Trayectoria y control por dron"}, "position": {"x": 160, "y": 180}},
    {"data": {"id": "lib_tray", "label": "lib_tray", "kind": "package", "group": "dron", "parent": "group_dron", "role": "Generacion de trayectorias"}, "position": {"x": 160, "y": 80}},
    {"data": {"id": "ORB_SLAM3", "label": "ORB_SLAM3", "kind": "package", "group": "dron", "parent": "group_dron", "role": "Frontend visual local"}, "position": {"x": 160, "y": 380}},
    {"data": {"id": "orbslam3", "label": "orbslam3", "kind": "package", "group": "dron", "parent": "group_dron", "role": "Wrapper ROS 2 de ORB-SLAM3"}, "position": {"x": 160, "y": 280}},
    {"data": {"id": "orbslam3_msgs_dron", "label": "orbslam3_msgs", "kind": "package", "group": "dron", "parent": "group_dron", "role": "Replica exacta del contrato ROS 2"}, "position": {"x": 160, "y": 480}},
    {"data": {"id": "orbslam3_server", "label": "orbslam3_server", "kind": "package", "group": "servidor", "parent": "group_servidor", "role": "I/O ROS 2, colas y publicacion"}, "position": {"x": 560, "y": 180}},
    {"data": {"id": "orbslam3_multi", "label": "orbslam3_multi", "kind": "package", "group": "servidor", "parent": "group_servidor", "role": "Backend sparse global"}, "position": {"x": 560, "y": 300}},
    {"data": {"id": "orbslam3_msgs_server", "label": "orbslam3_msgs", "kind": "package", "group": "servidor", "parent": "group_servidor", "role": "Copia canonica del contrato ROS 2"}, "position": {"x": 560, "y": 420}},
    {"data": {"id": "simulacion_dron", "label": "simulacion_dron", "kind": "package", "group": "simulacion", "parent": "group_simulacion", "role": "Gazebo, launch, escenarios y visualizadores"}, "position": {"x": 960, "y": 280}}
  ],
  "edges": [
    {"data": {"id": "lib_to_dron", "source": "lib_tray", "target": "dron_individual", "layer": "dependency", "label": "C++ API", "interface": "lib_tray"}},
    {"data": {"id": "orbslam3_core_wrapper", "source": "ORB_SLAM3", "target": "orbslam3", "layer": "map", "label": "frontend API", "interface": "System / Atlas"}},
    {"data": {"id": "msgs_dron_wrapper", "source": "orbslam3_msgs_dron", "target": "orbslam3", "layer": "dependency", "label": "messages", "interface": "orbslam3_msgs"}},
    {"data": {"id": "server_msgs_contract", "source": "orbslam3_msgs_server", "target": "orbslam3_server", "layer": "dependency", "label": "messages", "interface": "orbslam3_msgs"}},
    {"data": {"id": "server_backend_internal", "source": "orbslam3_server", "target": "orbslam3_multi", "layer": "map", "label": "backend API", "interface": "SparseGlobalBackend"}},
    {"data": {"id": "dron_to_server_map", "source": "orbslam3", "target": "orbslam3_server", "layer": "map", "label": "map delta / snapshot", "interface": "/dron_X/orbslam/orb_map_delta + GetOrbMap"}},
    {"data": {"id": "sim_to_dron_sensors", "source": "simulacion_dron", "target": "dron_individual", "layer": "sensor", "label": "sensores", "interface": "/dron_X/sensor/*"}},
    {"data": {"id": "sim_to_server_fiducial", "source": "simulacion_dron", "target": "orbslam3_server", "layer": "sensor", "label": "fiducial simulado", "interface": "/dron_X/sensor/GT/pose"}},
    {"data": {"id": "dron_to_sim_control", "source": "dron_individual", "target": "simulacion_dron", "layer": "control", "label": "fuerza / torque", "interface": "/dron_X/control/* + /motor/*"}},
    {"data": {"id": "orbslam3_to_dron", "source": "orbslam3", "target": "dron_individual", "layer": "map", "label": "pose local", "interface": "/dron_X/orbslam/pose_local"}},
    {"data": {"id": "server_to_sim_observability", "source": "orbslam3_server", "target": "simulacion_dron", "layer": "observability", "label": "flow events", "interface": "/global_mapping/flow_events"}},
    {"data": {"id": "server_to_sim_backpressure", "source": "orbslam3_server", "target": "simulacion_dron", "layer": "control", "label": "mission gate", "interface": "/global_mapping/backpressure_active"}},
    {"data": {"id": "server_to_sim_cloud", "source": "orbslam3_server", "target": "simulacion_dron", "layer": "observability", "label": "mapa global", "interface": "/global_sparse_cloud + /global_keyframes"}},
    {"data": {"id": "sim_launch_dron", "source": "simulacion_dron", "target": "dron_individual", "layer": "dependency", "label": "launch overlay", "interface": "FindPackageShare(dron_individual)"}},
    {"data": {"id": "sim_launch_server", "source": "simulacion_dron", "target": "orbslam3_server", "layer": "dependency", "label": "launch overlay", "interface": "FindPackageShare(orbslam3_server)"}}
  ]
};
