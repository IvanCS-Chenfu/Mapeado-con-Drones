window.SYSTEM_ARCHITECTURE_METADATA = {
  "nodes": {
    "dron_individual": {
      "path": "dron/dron_individual",
      "ros_name": "dron_individual",
      "executables": [
        "gen_tray",
        "control_calcular_fuerzas",
        "aplicar_fuerzas_dron"
      ],
      "owned_yaml": [
        "physical.yaml",
        "control.yaml",
        "trajectory.yaml",
        "actuators.yaml",
        "vision.yaml",
        "calibration.yaml"
      ],
      "dependencies": [
        "lib_tray"
      ],
      "cross_group": [
        "simulacion_dron (runtime/deployment)"
      ],
      "status": "active",
      "docs": "codex/contexto/paquetes/dron_individual/00_summary.md"
    },
    "lib_tray": {
      "path": "dron/lib_tray",
      "ros_name": "lib_tray",
      "executables": [
        "lib_tray shared library"
      ],
      "owned_yaml": [],
      "dependencies": [],
      "cross_group": [],
      "status": "active",
      "docs": "codex/contexto/paquetes/lib_tray/00_summary.md"
    },
    "ORB_SLAM3": {
      "path": "dron/ORB_SLAM3",
      "ros_name": "ORB_SLAM3",
      "executables": [
        "libORB_SLAM3"
      ],
      "owned_yaml": [],
      "dependencies": [],
      "cross_group": [],
      "status": "external",
      "docs": "codex/contexto/paquetes/ORB_SLAM3/00_summary.md"
    },
    "orbslam3": {
      "path": "dron/orbslam3_ros2",
      "ros_name": "orbslam3",
      "executables": [
        "stereo"
      ],
      "owned_yaml": [],
      "dependencies": [
        "ORB_SLAM3",
        "orbslam3_msgs"
      ],
      "cross_group": [
        "orbslam3_server (runtime)"
      ],
      "status": "integration",
      "docs": "codex/contexto/paquetes/orbslam3_ros2/00_summary.md"
    },
    "orbslam3_msgs_dron": {
      "path": "dron/orbslam3_msgs",
      "ros_name": "orbslam3_msgs",
      "executables": [],
      "owned_yaml": [],
      "dependencies": [],
      "cross_group": [
        "replica exacta de Servidor"
      ],
      "status": "verified replica",
      "docs": "codex/contexto/paquetes/orbslam3_msgs/00_summary.md"
    },
    "orbslam3_server": {
      "path": "servidor/orbslam3_server",
      "ros_name": "orbslam3_server",
      "executables": [
        "global_map_server",
        "fiducial_config_server.py"
      ],
      "owned_yaml": [
        "config/global_map/*",
        "config/fiducial_objects.yaml"
      ],
      "dependencies": [
        "orbslam3_multi",
        "orbslam3_msgs"
      ],
      "cross_group": [
        "orbslam3",
        "simulacion_dron"
      ],
      "status": "active",
      "docs": "codex/contexto/paquetes/orbslam3_server/00_summary.md"
    },
    "orbslam3_multi": {
      "path": "servidor/orbslam3_multi",
      "ros_name": "orbslam3_multi",
      "executables": [
        "SparseGlobalBackend library"
      ],
      "owned_yaml": [],
      "dependencies": [],
      "cross_group": [],
      "status": "active",
      "docs": "codex/contexto/paquetes/orbslam3_multi/00_summary.md"
    },
    "orbslam3_msgs_server": {
      "path": "servidor/orbslam3_msgs",
      "ros_name": "orbslam3_msgs",
      "executables": [],
      "owned_yaml": [],
      "dependencies": [],
      "cross_group": [
        "contrato canonico para Dron"
      ],
      "status": "canonical",
      "docs": "codex/contexto/paquetes/orbslam3_msgs/00_summary.md"
    },
    "simulacion_dron": {
      "path": "simulacion/simulacion_dron",
      "ros_name": "simulacion_dron",
      "executables": [
        "generador_URDF",
        "fiducial_spawner.py",
        "scenario_runner_node",
        "visualizer bridges"
      ],
      "owned_yaml": [
        "sim_dron.yaml",
        "physical_dron.yaml",
        "simulated_sensors.yaml",
        "debug.yaml",
        "fiducial_rendering.yaml",
        "fiducial_objects.yaml (deployment replica)"
      ],
      "dependencies": [],
      "cross_group": [
        "launch Dron/Servidor",
        "replicas declaradas"
      ],
      "status": "active",
      "docs": "codex/contexto/paquetes/simulacion_dron/00_summary.md"
    }
  },
  "edges": {
    "sim_to_orbslam_stereo": {
      "message_type": "sensor_msgs/msg/Image (x2)",
      "namespace": "/dron_X/sensor/camara_{izq,der}/image_raw",
      "qos": "sensor best effort",
      "data_transferred": "imagenes estereo rectificables"
    },
    "sim_to_dron_gt": {
      "message_type": "geometry_msgs/msg/PoseStamped + TwistStamped",
      "namespace": "/dron_X/sensor/GT/{pose,vel}",
      "qos": "keep last 10",
      "data_transferred": "pose y velocidad provisionales para control"
    },
    "sim_to_dron_action": {
      "message_type": "dron_individual/action/TrayAction",
      "namespace": "/dron_X/AccionTrayectoria",
      "qos": "action ROS 2",
      "data_transferred": "tipo, destino y duracion de trayectoria"
    },
    "dron_to_sim_motors": {
      "message_type": "std_msgs/msg/Float64",
      "namespace": "/dron_X/motor/*",
      "qos": "keep last 10",
      "data_transferred": "fuerza por motor"
    },
    "orbslam_to_server_delta": {
      "message_type": "orbslam3_msgs/msg/OrbMap",
      "namespace": "/dron_X/orbslam/orb_map_delta",
      "qos": "reliable keep last 10",
      "data_transferred": "deltas de keyframes y map points"
    },
    "server_to_orbslam_snapshot_request": {
      "message_type": "orbslam3_msgs/srv/GetOrbMap request",
      "namespace": "/dron_X/orbslam/get_full_map",
      "qos": "service ROS 2",
      "data_transferred": "peticion de snapshot completo"
    },
    "orbslam_to_server_snapshot_response": {
      "message_type": "orbslam3_msgs/srv/GetOrbMap response",
      "namespace": "/dron_X/orbslam/get_full_map",
      "qos": "service ROS 2",
      "data_transferred": "snapshot completo del mapa local"
    },
    "server_to_sim_backpressure": {
      "message_type": "std_msgs/msg/Bool",
      "namespace": "/global_mapping/backpressure_active",
      "qos": "reliable transient local",
      "data_transferred": "estado del mission gate"
    },
    "server_to_sim_sparse_map": {
      "message_type": "sensor_msgs/msg/PointCloud2 + visualization_msgs/msg/MarkerArray",
      "namespace": "/global_sparse_cloud + /global_keyframes",
      "qos": "reliable transient local",
      "data_transferred": "nube sparse y frustums globales"
    },
    "server_to_sim_pipeline_flow": {
      "message_type": "std_msgs/msg/String",
      "namespace": "/global_mapping/flow_events",
      "qos": "best effort keep last 256",
      "data_transferred": "eventos JSON de observabilidad interna"
    },
    "fiducial_config_server_to_wrapper": {
      "message_type": "orbslam3_msgs/srv/GetFiducialConfig",
      "namespace": "/global_mapping/get_fiducial_config",
      "qos": "service ROS 2",
      "data_transferred": "familia, politica, umbral y tamanos por tag_id"
    },
    "wrapper_to_server_fiducial_observations": {
      "message_type": "orbslam3_msgs/msg/FiducialKeyFrameObservations",
      "namespace": "/dron_X/orbslam/fiducial_keyframe_observations",
      "qos": "reliable volatile keep last 32",
      "data_transferred": "identidad exacta de KF y observaciones camera_T_tag validas"
    }
  }
};
