#!/usr/bin/env python3
import math
import threading
import tkinter as tk
from tkinter import ttk

import os
import yaml
from ament_index_python.packages import get_package_share_directory

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import PoseStamped
from dron_individual.action import TrayAction


ACTION_NAME = "AccionTrayectoria"   # nombre del servidor de acción (relativo al namespace del dron)


def load_sim_cfg():
    """
    Lee mi_tfg/config/sim_dron.yaml (formato ros__parameters) y devuelve:
      - n_drones (int)
      - namespace_base (str)
      - spawn_box (lista [x1,x2,y1,y2] o None)
    """
    cfg_path = os.path.join(
        get_package_share_directory('simulacion_dron'),
        'config',
        'sim_dron.yaml'
    )

    with open(cfg_path, 'r') as f:
        cfg = yaml.safe_load(f) or {}

    ros_params = cfg.get('/**', {}).get('ros__parameters', {})
    n_drones = int(ros_params.get('dron.numero', 1))
    namespace_base = str(ros_params.get('dron.namespace_base', 'dron'))
    spawn_box = ros_params.get('dron.spawn_box', None)

    return n_drones, namespace_base, spawn_box


class TrayActionGUI(Node):
    def __init__(self):
        super().__init__("tray_action_gui")

        # Leer config de sim_dron.yaml
        self.n_drones, self.namespace_base, self.spawn_box = load_sim_cfg()
        if self.n_drones < 1:
            self.n_drones = 1

        # Cache de ActionClients por action_name completo
        # No usar self._clients porque es un atributo interno de rclpy.node.Node
        self._action_clients = {}

        # Defaults
        self.defaults = {
            "drone_i": 1,
            "tipo_trayectoria": 0,
            "x": 0.0, "y": 0.0, "z": 1.0, "yaw_deg": 0.0,
            "tx": 20.0, "ty": 20.0, "tz": 20.0, "tyaw": 20.0,
            "absoluto_x": True,
            "absoluto_y": True,
            "absoluto_z": True,
            "absoluto_yaw": True,
        }

        self._build_gui()
        self.get_logger().info(
            f"GUI lista. n_drones={self.n_drones}, namespace_base='{self.namespace_base}', action='{ACTION_NAME}'"
        )

    # ---------- helpers para action ----------
    def _ns_for_i(self, i: int) -> str:
        return f"{self.namespace_base}_{i}"

    def _action_full_name(self, i: int) -> str:
        # Resultado: "/dron_0/AccionTrayectoria"
        return f"/{self._ns_for_i(i)}/{ACTION_NAME}"

    def _tipo_trayectoria_txt(self, tipo: int) -> str:
        nombres = {
            0: "0 - pol3",
            1: "1 - veltrap",
            2: "2 - elipse",
        }
        return nombres.get(tipo, str(tipo))

    def _get_client(self, i: int) -> ActionClient:
        full_name = self._action_full_name(i)
        if full_name not in self._action_clients:
            self._action_clients[full_name] = ActionClient(self, TrayAction, full_name)
        return self._action_clients[full_name]

    # ---------------- GUI ----------------
    def _build_gui(self):
        self.root = tk.Tk()
        self.root.title("TrayAction Launcher (multi-drone)")

        frm = ttk.Frame(self.root, padding=12)
        frm.grid(row=0, column=0, sticky="nsew")
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(0, weight=1)
        frm.columnconfigure(1, weight=1)

        # --- selector de dron ---
        self.var_drone = tk.IntVar(value=self.defaults["drone_i"])

        ttk.Label(frm, text="Drone i", width=14).grid(row=0, column=0, sticky="w", pady=4)
        self.lbl_drone = ttk.Label(frm, text=str(self.var_drone.get()), width=8)
        self.lbl_drone.grid(row=0, column=2, sticky="e")

        # --- selector de tipo de trayectoria ---
        self.var_tipo_trayectoria = tk.IntVar(value=self.defaults["tipo_trayectoria"])

        ttk.Label(frm, text="Trayectoria", width=14).grid(row=2, column=0, sticky="w", pady=4)

        self.lbl_tipo_trayectoria = ttk.Label(
            frm,
            text=self._tipo_trayectoria_txt(self.var_tipo_trayectoria.get()),
            width=14
        )
        self.lbl_tipo_trayectoria.grid(row=2, column=2, sticky="e")

        self.sld_tipo_trayectoria = ttk.Scale(
            frm,
            from_=0,
            to=2,
            orient="horizontal",
            variable=self.var_tipo_trayectoria,
            command=lambda _evt: self._on_tipo_trayectoria_change()
        )
        self.sld_tipo_trayectoria.grid(row=2, column=1, sticky="we", padx=8)

        self.sld_drone = ttk.Scale(
            frm, from_=0, to=max(1, self.n_drones), orient="horizontal",
            variable=self.var_drone, command=lambda _evt: self._on_drone_change()
        )
        self.sld_drone.grid(row=0, column=1, sticky="we", padx=8)

        self.lbl_target = ttk.Label(frm, text=self._action_full_name(self.var_drone.get()))
        self.lbl_target.grid(row=1, column=0, columnspan=4, sticky="w", pady=(0, 10))

        # --- sliders originales ---
        self.var_x    = tk.DoubleVar(value=self.defaults["x"])
        self.var_y    = tk.DoubleVar(value=self.defaults["y"])
        self.var_z    = tk.DoubleVar(value=self.defaults["z"])
        self.var_yaw  = tk.DoubleVar(value=self.defaults["yaw_deg"])
        self.var_tx   = tk.DoubleVar(value=self.defaults["tx"])
        self.var_ty   = tk.DoubleVar(value=self.defaults["ty"])
        self.var_tz   = tk.DoubleVar(value=self.defaults["tz"])
        self.var_tyaw = tk.DoubleVar(value=self.defaults["tyaw"])

        # --- checkboxes absoluto ---
        self.var_absoluto_x = tk.BooleanVar(value=self.defaults["absoluto_x"])
        self.var_absoluto_y = tk.BooleanVar(value=self.defaults["absoluto_y"])
        self.var_absoluto_z = tk.BooleanVar(value=self.defaults["absoluto_z"])
        self.var_absoluto_yaw = tk.BooleanVar(value=self.defaults["absoluto_yaw"])

        def add_slider(row, text, var, frm_range, fmt="{:.2f}", absoluto_var=None):
            ttk.Label(frm, text=text, width=14).grid(row=row, column=0, sticky="w", pady=4)

            val_lbl = ttk.Label(frm, text=fmt.format(var.get()), width=8)
            val_lbl.grid(row=row, column=2, sticky="e")

            sld = ttk.Scale(
                frm,
                from_=frm_range[0],
                to=frm_range[1],
                orient="horizontal",
                variable=var,
                command=lambda _evt: val_lbl.config(text=fmt.format(var.get()))
            )
            sld.grid(row=row, column=1, sticky="we", padx=8)

            if absoluto_var is not None:
                ttk.Checkbutton(frm, text="absoluto", variable=absoluto_var).grid(
                    row=row,
                    column=3,
                    sticky="w",
                    padx=(8, 0)
                )

        r = 3
        add_slider(r, "x (m)", self.var_x, (-10.0, 10.0), absoluto_var=self.var_absoluto_x); r += 1
        add_slider(r, "y (m)", self.var_y, (-10.0, 10.0), absoluto_var=self.var_absoluto_y); r += 1
        add_slider(r, "z (m)", self.var_z, (0.0, 10.0), absoluto_var=self.var_absoluto_z); r += 1
        add_slider(r, "yaw (deg)", self.var_yaw, (-180.0, 180.0), fmt="{:.0f}", absoluto_var=self.var_absoluto_yaw); r += 1

        ttk.Separator(frm, orient="horizontal").grid(row=r, column=0, columnspan=4, sticky="we", pady=(6, 6)); r += 1

        add_slider(r, "tx (s)", self.var_tx, (1.0, 60.0), fmt="{:.0f}"); r += 1
        add_slider(r, "ty (s)", self.var_ty, (1.0, 60.0), fmt="{:.0f}"); r += 1
        add_slider(r, "tz (s)", self.var_tz, (1.0, 60.0), fmt="{:.0f}"); r += 1
        add_slider(r, "tyaw (s)", self.var_tyaw, (1.0, 60.0), fmt="{:.0f}"); r += 1

        ttk.Separator(frm, orient="horizontal").grid(row=r, column=0, columnspan=4, sticky="we", pady=(10, 10)); r += 1

        ttk.Button(frm, text="Enviar goal", command=self._on_send).grid(
            row=r,
            column=0,
            columnspan=4,
            sticky="we"
        )

        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

    def _on_drone_change(self):
        # ttk.Scale usa float internamente; redondeamos
        i = int(round(float(self.var_drone.get())))
        i = max(1, min(i, self.n_drones))
        self.var_drone.set(i)
        self.lbl_drone.config(text=str(i))
        self.lbl_target.config(text=self._action_full_name(i))

    def _on_tipo_trayectoria_change(self):
        tipo = int(round(float(self.var_tipo_trayectoria.get())))
        tipo = max(0, min(tipo, 2))
        self.var_tipo_trayectoria.set(tipo)
        self.lbl_tipo_trayectoria.config(text=self._tipo_trayectoria_txt(tipo))
    
    # ------------- Acción -------------
    def _on_send(self):
        i = int(self.var_drone.get())
        full_action_name = self._action_full_name(i)
        client = self._get_client(i)
        
        tipo_trayectoria = int(round(float(self.var_tipo_trayectoria.get())))
        tipo_trayectoria = max(0, min(tipo_trayectoria, 2))

        # Leer sliders
        x = float(self.var_x.get())
        y = float(self.var_y.get())
        z = float(self.var_z.get())
        yaw_deg = float(self.var_yaw.get())
        tx = float(self.var_tx.get())
        ty = float(self.var_ty.get())
        tz = float(self.var_tz.get())
        tyaw = float(self.var_tyaw.get())

        # Leer checkboxes absoluto
        absoluto_x = bool(self.var_absoluto_x.get())
        absoluto_y = bool(self.var_absoluto_y.get())
        absoluto_z = bool(self.var_absoluto_z.get())
        absoluto_yaw = bool(self.var_absoluto_yaw.get())

        # yaw deg -> quaternion (Z)
        yaw_rad = math.radians(yaw_deg)
        qz = math.sin(yaw_rad / 2.0)
        qw = math.cos(yaw_rad / 2.0)

        # Construir goal
        goal = TrayAction.Goal()
        goal.tipo_trayectoria = tipo_trayectoria
        goal.target_pose = PoseStamped()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = self.get_clock().now().to_msg()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.position.z = z
        goal.target_pose.pose.orientation.x = 0.0
        goal.target_pose.pose.orientation.y = 0.0
        goal.target_pose.pose.orientation.z = qz
        goal.target_pose.pose.orientation.w = qw
        goal.tx = tx
        goal.ty = ty
        goal.tz = tz
        goal.tyaw = tyaw
        goal.absoluto_x = absoluto_x
        goal.absoluto_y = absoluto_y
        goal.absoluto_z = absoluto_z
        goal.absoluto_yaw = absoluto_yaw

        def send_goal_thread():
            if not client.wait_for_server(timeout_sec=2.0):
                self.get_logger().error(f"No hay servidor de acción en {full_action_name}")
                return

            self.get_logger().info(
                f"Enviando goal a {full_action_name}: "
                f"tipo_trayectoria={self._tipo_trayectoria_txt(tipo_trayectoria)}, "
                f"pos=({x:.2f},{y:.2f},{z:.2f}), yaw={yaw_deg:.1f}deg, "
                f"tx={tx:.1f}, ty={ty:.1f}, tz={tz:.1f}, tyaw={tyaw:.1f}, "
                f"absoluto_x={absoluto_x}, absoluto_y={absoluto_y}, "
                f"absoluto_z={absoluto_z}, absoluto_yaw={absoluto_yaw}"
            )

            client.send_goal_async(goal)

        threading.Thread(target=send_goal_thread, daemon=True).start()

    def _on_close(self):
        try:
            self.root.destroy()
        except tk.TclError:
            pass
        rclpy.shutdown()


def main():
    rclpy.init()
    node = TrayActionGUI()

    executor = MultiThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    try:
        node.root.mainloop()
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()