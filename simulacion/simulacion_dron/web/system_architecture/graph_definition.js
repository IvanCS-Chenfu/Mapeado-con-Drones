window.SYSTEM_ARCHITECTURE = {
  "version": 2,
  "groups": [
    "dron",
    "servidor",
    "simulacion"
  ],
  "layers": [
    "runtime",
    "build",
    "config",
    "deployment"
  ],
  "nodes": [
    {
      "data": {
        "id": "group_dron",
        "label": "Dron",
        "kind": "group",
        "group": "dron",
        "role": "Software desplegado en cada dron"
      }
    },
    {
      "data": {
        "id": "group_servidor",
        "label": "Servidor",
        "kind": "group",
        "group": "servidor",
        "role": "Mapa sparse global y coordinacion"
      }
    },
    {
      "data": {
        "id": "group_simulacion",
        "label": "Simulacion",
        "kind": "group",
        "group": "simulacion",
        "role": "Gazebo, escenarios y observabilidad"
      }
    },
    {
      "data": {
        "id": "dron_individual",
        "label": "dron_individual",
        "kind": "package",
        "group": "dron",
        "parent": "group_dron",
        "role": "Trayectoria, control y actuadores"
      },
      "position": {
        "x": 160,
        "y": 180
      }
    },
    {
      "data": {
        "id": "lib_tray",
        "label": "lib_tray",
        "kind": "package",
        "group": "dron",
        "parent": "group_dron",
        "role": "Generacion de trayectorias"
      },
      "position": {
        "x": 160,
        "y": 80
      }
    },
    {
      "data": {
        "id": "ORB_SLAM3",
        "label": "ORB_SLAM3",
        "kind": "package",
        "group": "dron",
        "parent": "group_dron",
        "role": "Frontend visual local"
      },
      "position": {
        "x": 160,
        "y": 380
      }
    },
    {
      "data": {
        "id": "orbslam3",
        "label": "orbslam3",
        "kind": "package",
        "group": "dron",
        "parent": "group_dron",
        "role": "Wrapper ROS 2 de ORB-SLAM3"
      },
      "position": {
        "x": 160,
        "y": 280
      }
    },
    {
      "data": {
        "id": "orbslam3_msgs_dron",
        "label": "orbslam3_msgs",
        "kind": "package",
        "group": "dron",
        "parent": "group_dron",
        "role": "Replica exacta del contrato ROS 2"
      },
      "position": {
        "x": 160,
        "y": 480
      }
    },
    {
      "data": {
        "id": "orbslam3_server",
        "label": "orbslam3_server",
        "kind": "package",
        "group": "servidor",
        "parent": "group_servidor",
        "role": "I/O ROS 2, colas y publicacion"
      },
      "position": {
        "x": 560,
        "y": 180
      }
    },
    {
      "data": {
        "id": "orbslam3_multi",
        "label": "orbslam3_multi",
        "kind": "package",
        "group": "servidor",
        "parent": "group_servidor",
        "role": "Backend sparse global"
      },
      "position": {
        "x": 560,
        "y": 300
      }
    },
    {
      "data": {
        "id": "orbslam3_msgs_server",
        "label": "orbslam3_msgs",
        "kind": "package",
        "group": "servidor",
        "parent": "group_servidor",
        "role": "Copia canonica del contrato ROS 2"
      },
      "position": {
        "x": 560,
        "y": 420
      }
    },
    {
      "data": {
        "id": "multidron_gui",
        "label": "multidron_gui",
        "kind": "package",
        "group": "servidor",
        "parent": "group_servidor",
        "role": "Proceso Qt de operacion y visualizacion"
      },
      "position": {
        "x": 760,
        "y": 180
      }
    },
    {
      "data": {
        "id": "multidron_gui_lib",
        "label": "multidron_gui_lib",
        "kind": "package",
        "group": "servidor",
        "parent": "group_servidor",
        "role": "Modelo ROS/Qt y renderer OpenGL"
      },
      "position": {
        "x": 760,
        "y": 300
      }
    },
    {
      "data": {
        "id": "simulacion_dron",
        "label": "simulacion_dron",
        "kind": "package",
        "group": "simulacion",
        "parent": "group_simulacion",
        "role": "Gazebo, launch, escenarios y visualizadores"
      },
      "position": {
        "x": 960,
        "y": 280
      }
    },
    {
      "data": {
        "id": "mission_msgs_dron",
        "label": "mission_msgs",
        "kind": "package",
        "group": "dron",
        "parent": "group_dron",
        "role": "Replica exacta de contratos de mision"
      },
      "position": {"x": 360, "y": 760}
    },
    {
      "data": {
        "id": "task_manager_lib",
        "label": "task_manager_lib",
        "kind": "package",
        "group": "dron",
        "parent": "group_dron",
        "role": "Logica pura del agente de mision"
      },
      "position": {"x": 160, "y": 760}
    },
    {
      "data": {
        "id": "task_manager",
        "label": "task_manager",
        "kind": "package",
        "group": "dron",
        "parent": "group_dron",
        "role": "Registro y lifecycle local"
      },
      "position": {"x": 160, "y": 870}
    },
    {
      "data": {
        "id": "mission_msgs_server",
        "label": "mission_msgs",
        "kind": "package",
        "group": "servidor",
        "parent": "group_servidor",
        "role": "Contrato canonico de mision"
      },
      "position": {"x": 760, "y": 760}
    },
    {
      "data": {
        "id": "task_lib",
        "label": "task_lib",
        "kind": "package",
        "group": "servidor",
        "parent": "group_servidor",
        "role": "Configuracion y geometria puras"
      },
      "position": {"x": 560, "y": 760}
    },
    {
      "data": {
        "id": "task_server",
        "label": "task_server",
        "kind": "package",
        "group": "servidor",
        "parent": "group_servidor",
        "role": "Coordinador de mision multi-dron"
      },
      "position": {"x": 560, "y": 870}
    }
  ],
  "edges": [
    {
      "data": {"id": "task_manager_lib_to_manager", "source": "task_manager_lib", "target": "task_manager", "layer": "build", "label": "API local", "interface": "task_manager_lib", "activity_mode": "none", "interface_kind": "api"}
    },
    {
      "data": {"id": "mission_msgs_dron_to_manager", "source": "mission_msgs_dron", "target": "task_manager", "layer": "build", "label": "contratos", "interface": "mission_msgs", "activity_mode": "none", "interface_kind": "api"}
    },
    {
      "data": {"id": "task_lib_to_server", "source": "task_lib", "target": "task_server", "layer": "build", "label": "geometria", "interface": "task_lib", "activity_mode": "none", "interface_kind": "api"}
    },
    {
      "data": {"id": "mission_msgs_server_to_task_server", "source": "mission_msgs_server", "target": "task_server", "layer": "build", "label": "contratos", "interface": "mission_msgs", "activity_mode": "none", "interface_kind": "api"}
    },
    {
      "data": {"id": "mission_msgs_server_to_gui", "source": "mission_msgs_server", "target": "multidron_gui_lib", "layer": "build", "label": "geometria", "interface": "mission_msgs", "activity_mode": "none", "interface_kind": "api"}
    },
    {
      "data": {"id": "mission_msgs_replica", "source": "mission_msgs_server", "target": "mission_msgs_dron", "layer": "deployment", "label": "replica exacta", "interface": "tree byte a byte", "activity_mode": "none", "interface_kind": "replica"}
    },
    {
      "data": {"id": "sim_deploys_task_server", "source": "simulacion_dron", "target": "task_server", "layer": "deployment", "label": "launch", "interface": "launch_phase6", "activity_mode": "none", "interface_kind": "launch"}
    },
    {
      "data": {"id": "sim_deploys_task_manager", "source": "simulacion_dron", "target": "task_manager", "layer": "deployment", "label": "launch por dron", "interface": "launch_phase6", "activity_mode": "none", "interface_kind": "launch"}
    },
    {
      "data": {"id": "task_manager_to_task_server", "source": "task_manager", "target": "task_server", "layer": "runtime", "label": "registro", "interface": "/mission/register_drone", "activity_mode": "direct", "interface_kind": "service", "producer": "task_manager", "consumer": "task_server", "ttl_ms": 1800}
    },
    {
      "data": {"id": "task_server_to_gui_geometry", "source": "task_server", "target": "multidron_gui_lib", "layer": "runtime", "label": "geometria", "interface": "/mission/geometry", "activity_mode": "direct", "interface_kind": "topic_publish", "producer": "task_server", "consumer": "RosDataBridge", "ttl_ms": 1800}
    },
    {
      "data": {"id": "task_server_to_sim_mission_flow", "source": "task_server", "target": "simulacion_dron", "layer": "runtime", "label": "mission debug", "interface": "/mission/flow_events", "activity_mode": "direct", "interface_kind": "debug_topic", "producer": "task_server", "consumer": "mission_flow_bridge", "status": "debug_optional", "ttl_ms": 900}
    },
    {
      "data": {
        "id": "gui_lib_to_gui",
        "source": "multidron_gui_lib",
        "target": "multidron_gui",
        "layer": "build",
        "label": "Qt/ROS API",
        "interface": "multidron_gui_lib",
        "activity_mode": "none",
        "interface_kind": "api"
      }
    },
    {
      "data": {
        "id": "sim_deploys_gui",
        "source": "simulacion_dron",
        "target": "multidron_gui",
        "layer": "deployment",
        "label": "launch GUI",
        "interface": "launch_multidron_gui",
        "activity_mode": "none",
        "interface_kind": "launch"
      }
    },
    {
      "data": {
        "id": "lib_to_dron",
        "source": "lib_tray",
        "target": "dron_individual",
        "layer": "build",
        "label": "C++ API",
        "interface": "lib_tray",
        "activity_mode": "none",
        "interface_kind": "api"
      }
    },
    {
      "data": {
        "id": "orbslam_core_wrapper",
        "source": "ORB_SLAM3",
        "target": "orbslam3",
        "layer": "build",
        "label": "frontend API",
        "interface": "System / Atlas",
        "activity_mode": "none",
        "interface_kind": "api"
      }
    },
    {
      "data": {
        "id": "msgs_dron_wrapper",
        "source": "orbslam3_msgs_dron",
        "target": "orbslam3",
        "layer": "build",
        "label": "messages",
        "interface": "orbslam3_msgs",
        "activity_mode": "none",
        "interface_kind": "dependency"
      }
    },
    {
      "data": {
        "id": "server_msgs_contract",
        "source": "orbslam3_msgs_server",
        "target": "orbslam3_server",
        "layer": "build",
        "label": "messages",
        "interface": "orbslam3_msgs",
        "activity_mode": "none",
        "interface_kind": "dependency"
      }
    },
    {
      "data": {
        "id": "server_backend_internal",
        "source": "orbslam3_server",
        "target": "orbslam3_multi",
        "layer": "build",
        "label": "backend API",
        "interface": "SparseGlobalBackend",
        "activity_mode": "none",
        "interface_kind": "api"
      }
    },
    {
      "data": {
        "id": "msgs_contract_replica",
        "source": "orbslam3_msgs_server",
        "target": "orbslam3_msgs_dron",
        "layer": "config",
        "label": "contrato canonico",
        "interface": "replica exacta orbslam3_msgs",
        "activity_mode": "none",
        "interface_kind": "replica"
      }
    },
    {
      "data": {
        "id": "calibration_to_server",
        "source": "dron_individual",
        "target": "orbslam3_server",
        "layer": "config",
        "label": "calibracion Dron",
        "interface": "calibration.yaml -> calibration_dron.yaml",
        "activity_mode": "none",
        "interface_kind": "replica",
        "status": "transitional"
      }
    },
    {
      "data": {
        "id": "calibration_to_sim",
        "source": "dron_individual",
        "target": "simulacion_dron",
        "layer": "config",
        "label": "calibracion Dron",
        "interface": "calibration.yaml -> calibration_dron.yaml",
        "activity_mode": "none",
        "interface_kind": "replica"
      }
    },
    {
      "data": {
        "id": "actuators_to_sim",
        "source": "dron_individual",
        "target": "simulacion_dron",
        "layer": "config",
        "label": "replica actuadores",
        "interface": "actuators.yaml -> actuators_dron.yaml",
        "activity_mode": "none",
        "interface_kind": "partial_replica"
      }
    },
    {
      "data": {
        "id": "globalmap_profile_to_sim",
        "source": "orbslam3_server",
        "target": "simulacion_dron",
        "layer": "config",
        "label": "deployment profile",
        "interface": "config/global_map + config/fiducial_objects.yaml exact mirror",
        "activity_mode": "none",
        "interface_kind": "full_replica"
      }
    },
    {
      "data": {
        "id": "sim_launch_dron",
        "source": "simulacion_dron",
        "target": "dron_individual",
        "layer": "deployment",
        "label": "launch",
        "interface": "generar_dron.launch.py",
        "activity_mode": "none",
        "interface_kind": "launch"
      }
    },
    {
      "data": {
        "id": "sim_launch_server",
        "source": "simulacion_dron",
        "target": "orbslam3_server",
        "layer": "deployment",
        "label": "launch",
        "interface": "global_orb_map_server.launch.py",
        "activity_mode": "none",
        "interface_kind": "launch"
      }
    },
    {
      "data": {
        "id": "sim_to_orbslam_stereo",
        "source": "simulacion_dron",
        "target": "orbslam3",
        "layer": "runtime",
        "label": "stereo",
        "interface": "/dron_X/sensor/camara_{izq,der}/image_raw",
        "activity_mode": "direct",
        "interface_kind": "topic",
        "producer": "Gazebo camera plugins",
        "consumer": "orbslam3 stereo",
        "ttl_ms": 650
      }
    },
    {
      "data": {
        "id": "sim_to_dron_gt",
        "source": "simulacion_dron",
        "target": "dron_individual",
        "layer": "runtime",
        "label": "GT provisional",
        "interface": "/dron_X/sensor/GT/{pose,vel}",
        "activity_mode": "direct",
        "interface_kind": "topic",
        "producer": "plugin_sensor_groundtrurh",
        "consumer": "navigation_state_mux",
        "status": "provisional_phase5",
        "ttl_ms": 650
      }
    },
    {
      "data": {
        "id": "sim_to_dron_action",
        "source": "simulacion_dron",
        "target": "dron_individual",
        "layer": "runtime",
        "label": "trayectoria",
        "interface": "/dron_X/AccionTrayectoria",
        "activity_mode": "direct",
        "interface_kind": "action",
        "producer": "GUI/scenario_runner",
        "consumer": "gen_tray",
        "ttl_ms": 1800
      }
    },
    {
      "data": {
        "id": "dron_to_sim_motors",
        "source": "dron_individual",
        "target": "simulacion_dron",
        "layer": "runtime",
        "label": "motores",
        "interface": "/dron_X/motor/{arr_iz,ab_iz,ab_der,arr_der}",
        "activity_mode": "direct",
        "interface_kind": "topic",
        "producer": "aplicar_fuerzas_dron",
        "consumer": "plugin_actuar_motores",
        "ttl_ms": 650
      }
    },
    {
      "data": {
        "id": "orbslam_to_server_delta",
        "source": "orbslam3",
        "target": "orbslam3_server",
        "layer": "runtime",
        "label": "OrbMap delta",
        "interface": "/dron_X/orbslam/orb_map_delta",
        "activity_mode": "direct",
        "interface_kind": "topic",
        "producer": "orbslam3 stereo",
        "consumer": "GlobalMapServer::OnDelta",
        "ttl_ms": 1200
      }
    },
    {
      "data": {
        "id": "server_to_orbslam_snapshot_request",
        "source": "orbslam3_server",
        "target": "orbslam3",
        "layer": "runtime",
        "label": "snapshot request",
        "interface": "/dron_X/orbslam/get_full_map request",
        "activity_mode": "direct",
        "interface_kind": "service_request",
        "producer": "GlobalMapServer",
        "consumer": "orbslam3",
        "ttl_ms": 2200
      }
    },
    {
      "data": {
        "id": "orbslam_to_server_snapshot_response",
        "source": "orbslam3",
        "target": "orbslam3_server",
        "layer": "runtime",
        "label": "snapshot response",
        "interface": "/dron_X/orbslam/get_full_map response",
        "activity_mode": "direct",
        "interface_kind": "service_response",
        "producer": "orbslam3",
        "consumer": "GlobalMapServer",
        "ttl_ms": 2200
      }
    },
    {
      "data": {
        "id": "server_to_sim_backpressure",
        "source": "orbslam3_server",
        "target": "simulacion_dron",
        "layer": "runtime",
        "label": "mission gate",
        "interface": "/global_mapping/backpressure_active",
        "activity_mode": "direct",
        "interface_kind": "topic",
        "producer": "GlobalMapServer",
        "consumer": "scenario_runner",
        "ttl_ms": 1800
      }
    },
    {
      "data": {
        "id": "server_to_sim_sparse_map",
        "source": "orbslam3_server",
        "target": "simulacion_dron",
        "layer": "runtime",
        "label": "mapa global",
        "interface": "/global_sparse_cloud + /global_keyframes",
        "activity_mode": "direct",
        "interface_kind": "topic_publish",
        "producer": "GlobalMapServer",
        "consumer": "RViz launched by simulacion_dron",
        "ttl_ms": 1800
      }
    },
    {
      "data": {
        "id": "server_to_sim_pipeline_flow",
        "source": "orbslam3_server",
        "target": "simulacion_dron",
        "layer": "runtime",
        "label": "pipeline debug",
        "interface": "/global_mapping/flow_events",
        "activity_mode": "direct",
        "interface_kind": "debug_topic",
        "producer": "GlobalMapServer",
        "consumer": "pipeline_flow_bridge",
        "status": "debug_optional",
        "ttl_ms": 900
      }
    },
    {
      "data": {
        "id": "wrapper_to_server_fiducial_observations",
        "source": "orbslam3",
        "target": "orbslam3_server",
        "layer": "runtime",
        "label": "fiducial batches",
        "interface": "/dron_X/orbslam/fiducial_keyframe_observations",
        "activity_mode": "direct",
        "interface_kind": "topic_publish",
        "producer": "orbslam3 stereo FiducialWorker",
        "consumer": "GlobalMapServer / RawMapDatabase",
        "ttl_ms": 1800
      }
    },
    {
      "data": {
        "id": "fiducial_config_server_to_wrapper",
        "source": "orbslam3_server",
        "target": "orbslam3",
        "layer": "runtime",
        "label": "fiducial config",
        "interface": "/global_mapping/get_fiducial_config",
        "activity_mode": "direct",
        "interface_kind": "service",
        "producer": "fiducial_config_server.py",
        "consumer": "orbslam3 stereo",
        "ttl_ms": 2200
      }
    }
  ]
};
