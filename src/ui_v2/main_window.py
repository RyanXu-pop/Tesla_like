import os
import sys
import logging
import asyncio
import math
import time
from typing import Optional
from PySide6.QtWidgets import QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QPushButton, QMessageBox
from PySide6.QtCore import Qt, Slot, QTimer

# 核心与网络
from src.core.constants import PATHS_CONFIG, PARAMS_CONFIG, CONFIG
from src.core.models import RobotPose
from src.core.utils import apply_pose_transform
from src.network.mqtt_agent import MqttAgent
from src.network.async_ssh_manager import AsyncSSHManager
from src.controllers.map_manager import MapManager
from src.controllers.workflow_controller import WorkflowController
from src.controllers.navigation_controller import NavigationController
from src.controllers.teleop_controller import TeleopController
from src.controllers.pose_recorder import PoseRecorder
from src.ui.system_setting import SystemSetting

# V2 引入
from .theme import apply_theme
from .robot_state_hub import RobotStateHub
from .main_layout import MainLayoutWidget

class MyMainWindow(QMainWindow):
    """
    UI V2 主窗口
    采用单向数据流 (MVVM)：
      MqttAgent -> Store -> Panels/MapView
    PC 端只负责远程控制与显示。建图算法运行在机器人端，界面只渲染
    机器人端发布的 /map，并将当前帧 /scan 作为动态覆盖层显示。
    """
    def __init__(self, mqtt_agent=None):
        super().__init__()
        # 全局应用暗色主题
        from PySide6.QtWidgets import QApplication
        apply_theme(QApplication.instance())
        
        self.setWindowTitle("ROS2 Control Panel V2")
        self.setMinimumSize(1024, 768)

        # 1. 核心状态池 (Single Source of Truth)
        self.store = RobotStateHub(self)
        
        # 2. 网络设施初始化
        self.async_ssh = AsyncSSHManager()
        self.mqtt_agent = mqtt_agent if mqtt_agent is not None else MqttAgent()
        
        # 3. 控制器初始化
        self.map_mgr = MapManager(map_bounds=PARAMS_CONFIG['map_bounds'])
        self.workflow_ctrl = WorkflowController(self.async_ssh, self.map_mgr, self)
        self.nav_ctrl = NavigationController(mqtt_agent=self.mqtt_agent)
        self.teleop_ctrl = TeleopController(mqtt_agent=self.mqtt_agent, parent=self)
        record_xlsx_path = PATHS_CONFIG.get('record_xlsx', 'pose_records.xlsx')
        self.pose_recorder = PoseRecorder(record_xlsx_path, parent=self)
        self._map_to_odom = None
        self._frame_transforms = {}
        self._latest_odom_pose: Optional[RobotPose] = None
        self._mapping_pose_log_counter = 0
        self._show_mapping_scan_overlay = bool(PARAMS_CONFIG.get("show_mapping_scan_overlay", True))
        self._map_source = "local"
        self._live_map_received = False
        self._cleared_for_live_map = False
        self._last_live_map_received_at: Optional[float] = None
        self._last_scan_received_at: Optional[float] = None
        self._mapping_map_log_counter = 0
        self._mapping_scan_log_counter = 0
        self._last_mapping_scan_status = ""
        self._mapping_scan_stale_timeout_s = 1.5
        
        # 4. 界面构建
        self.ui = MainLayoutWidget(self.store, self)
        self.setCentralWidget(self.ui)
        
        # 5. 信号绑定 (数据总线)
        self._bind_signals()
        
        # 6. 加载地初始数据
        self._load_initial_data()
        
        # 7. 启动 MQTT
        try:
            self.mqtt_agent.connect_broker()
            logging.info("[V2] MqttAgent started.")
        except Exception as e:
            logging.error(f"[V2] MqttAgent connect failed: {e}")



    def _bind_signals(self):
        """核心数据流组装"""
        # --- UI Top Bar Actions ---
        self.ui.btn_simulation.clicked.connect(self._toggle_simulation)
        self.ui.btn_fullscreen.clicked.connect(self._toggle_fullscreen)
        self.ui.btn_settings.clicked.connect(self._show_system_settings)
        self.ui.btn_locate_me.clicked.connect(self._center_map_on_robot)
        
        # ================== MQTT Data -> Store ==================
        self.mqtt_agent.pose_updated.connect(self._on_pose_data)
        self.mqtt_agent.voltage_updated.connect(self.store.update_voltage)
        self.mqtt_agent.chassis_status_updated.connect(self.store.update_chassis_status)
        self.mqtt_agent.scan_updated.connect(self._on_scan_data)
        self.mqtt_agent.path_updated.connect(self._on_path_data)
        self.mqtt_agent.map_updated.connect(self._on_live_map_data)
        if hasattr(self.mqtt_agent, 'transform_updated'):
            self.mqtt_agent.transform_updated.connect(self._on_tf_data)
        
        # MQTT 连接状态 → Store + TelemetryPanel
        self.mqtt_agent.connection_status.connect(self._on_mqtt_connection_status)
        
        # Odom 数据 → 建图模式下更新位姿
        # 这里只更新显示用的机器人位姿，不在 PC 端做任何 map 级二次修正。
        if hasattr(self.mqtt_agent, 'odom_updated'):
            self.mqtt_agent.odom_updated.connect(self._on_odom_data)

        # ================== Store -> MapView ==================
        self.store.map_data_changed.connect(self.ui.map_view.update_map)
        self.store.global_path_changed.connect(self._on_store_path_changed)
        
        self.store.robot_pose_changed.connect(self._on_store_robot_pose_changed)
        
        def safely_update_scan(scan_dict):
            self._on_store_scan_changed(scan_dict)
            return
            pose = self.store.current_pose
            if pose is not None and scan_dict:
                # 当前帧 /scan 只作为动态显示层，不在 PC 端累计成地图。
                scan_x = pose.x
                scan_y = pose.y
                scan_yaw = pose.yaw
                sensor_transform = self._find_sensor_transform(scan_dict.get("frame_id", ""))
                if sensor_transform:
                    scan_x, scan_y, scan_yaw = apply_pose_transform(
                        transform_x=pose.x,
                        transform_y=pose.y,
                        transform_yaw=pose.yaw,
                        pose_x=float(sensor_transform.get("x", 0.0)),
                        pose_y=float(sensor_transform.get("y", 0.0)),
                        pose_yaw=float(sensor_transform.get("yaw", 0.0)),
                    )
                self.ui.map_view.update_scan(scan_dict, scan_x, scan_y, scan_yaw)
        self.store.laser_scan_changed.connect(safely_update_scan)
        self.store.mapping_state_changed.connect(self._on_mapping_state_changed)

        # ================== Pose Recorder / Navigation ==================
        self.ui.pose_panel.sig_start_trace.connect(self.pose_recorder.start)
        self.ui.pose_panel.sig_start_trace.connect(lambda: self.ui.pose_panel.set_trace_active(True))
        
        def on_stop_trace():
            ok = self.pose_recorder.stop()
            self.ui.pose_panel.set_trace_active(False)
            if ok:
                QMessageBox.information(self, "记录完成", "连续轨迹已保存至 pose_records.xlsx")
        self.ui.pose_panel.sig_stop_trace.connect(on_stop_trace)
        
        def on_record_point():
            pose = self.store.current_pose
            if pose:
                self.ui.pose_panel.add_point(pose.x, pose.y, pose.angle)
                logging.info(f"[PoseRecord] 手动打卡位置: {pose.x}, {pose.y}, {pose.angle}")
        self.ui.pose_panel.sig_record_point.connect(on_record_point)
        
        import numpy as np
        self.ui.pose_panel.sig_go_to_selected.connect(
            lambda x, y, yaw: self.nav_ctrl.set_goal_pose(x, y, yaw, np.eye(3))
        )
        self.pose_recorder.status_message.connect(lambda msg: logging.info(f"[PoseRecorder] {msg}"))
        
        # ================== UI Intents -> Controllers ==================
        # 接收并显示系统级全局通知
        def show_popup(msg):
            # Only pop up for final results, not intermediate "正在..." steps
            if "成功" in msg or "失败" in msg or "完成" in msg:
                QMessageBox.information(self, "系统提示", msg)
            else:
                self.statusBar().showMessage(msg, 3000)
                
        self.store.workflow_message.connect(show_popup)
        
        # 建图开关
        self.ui.control_panel.sig_start_mapping.connect(self._do_start_mapping)
        self.ui.control_panel.sig_stop_mapping.connect(self._do_stop_mapping)
        self.ui.control_panel.sig_save_map.connect(self._do_save_map)
        
        # 导航开关
        self.ui.control_panel.sig_start_navigation.connect(self._do_start_nav)
        self.ui.control_panel.sig_stop_navigation.connect(self._do_stop_nav)
        
        # 交互设定
        self.ui.control_panel.sig_set_initial_pose.connect(lambda: self.ui.map_view.set_interaction_mode("initial_pose"))
        self.ui.control_panel.sig_set_goal_pose.connect(lambda: self.ui.map_view.set_interaction_mode("goal"))
        
        # 处理手动坐标输入
        import numpy as np
        identity_inv = np.eye(3)
        self.ui.control_panel.sig_manual_initial_pose.connect(
            lambda x, y, yaw: self.nav_ctrl.set_initial_pose(x, y, yaw, identity_inv)
        )
        self.ui.control_panel.sig_manual_goal.connect(
            lambda x, y, yaw: self.nav_ctrl.set_goal_pose(x, y, yaw, identity_inv)
        )
        # 地图回传意图处理器
        self.ui.map_view.interaction_triggered.connect(self._on_map_interaction)

        # 工作流消息
        self.workflow_ctrl.status_message.connect(lambda msg: self.statusBar().showMessage(msg, 5000))
        self.workflow_ctrl.map_synced.connect(self._load_initial_data)
        self.workflow_ctrl.workflow_finished.connect(self._on_workflow_finished)
        
        # SSH 系统操作
        self.ui.control_panel.sig_start_chassis.connect(self._do_start_chassis)
        self.ui.control_panel.sig_start_mqtt_node.connect(self._do_start_mqtt_node)
        
        # 地图下载/上传
        self.ui.control_panel.sig_download_map.connect(self._do_download_map)
        self.ui.control_panel.sig_upload_map.connect(self._do_upload_map)
        
        # 初始位姿保存/恢复
        self.ui.control_panel.sig_save_initial_pose.connect(self._do_save_initial_pose)
        self.ui.control_panel.sig_recall_initial_pose.connect(self._do_recall_initial_pose)

    def _load_initial_data(self):
        """首次加载静态地图"""
        if self.store.mapping_running:
            return
        ok = self.map_mgr.load(PATHS_CONFIG['map_yaml'])
        if ok and self.map_mgr.map_data:
            from src.core.models import MapMetadata
            
            map_data = self.map_mgr.map_data
            map_raster = map_data.get('data', map_data['image'])
            
            meta = MapMetadata(
                resolution=map_data.get('resolution', 0.05),
                origin_x=map_data.get('origin', [0, 0, 0])[0],
                origin_y=map_data.get('origin', [0, 0, 0])[1],
                origin_yaw=map_data.get('origin', [0, 0, 0])[2],
                width=map_raster.shape[1],
                height=map_raster.shape[0],
                data=map_raster,
                encoding=map_data.get('encoding', 'image')
            )
            self._apply_local_map(meta)

    # ---------------- 业务编排 ----------------
    
    def _apply_local_map(self, map_meta):
        self._map_source = "local"
        self._live_map_received = False
        self._cleared_for_live_map = False
        self._last_live_map_received_at = None
        self.store.update_map(map_meta)
        logging.info("[V2][MappingMap] source=LOCAL cleared_for_live_map=%s", self._cleared_for_live_map)

    def _on_live_map_data(self, map_meta):
        self._last_live_map_received_at = time.monotonic()
        if not (self.store.mapping_running or self._map_source == "live_mqtt" or self._cleared_for_live_map):
            return

        first_live_frame = not self._live_map_received or self._map_source != "live_mqtt"
        self._map_source = "live_mqtt"
        self._live_map_received = True
        self.store.update_map(map_meta)

        should_log = first_live_frame
        if self.store.mapping_running:
            self._mapping_map_log_counter += 1
            should_log = should_log or self._mapping_map_log_counter % 20 == 0
        if should_log:
            logging.info("[V2][MappingMap] source=LIVE cleared_for_live_map=%s", self._cleared_for_live_map)
        self._cleared_for_live_map = False

    def _on_scan_data(self, scan_dict):
        self._last_scan_received_at = time.monotonic()
        self.store.update_scan(scan_dict or {})

    def _on_path_data(self, path_points):
        self.store.update_path(path_points or [])

    @Slot()
    def _do_start_mapping(self):
        self.store.set_mapping_running(True)
        import asyncio
        asyncio.create_task(self.workflow_ctrl.execute_mapping_workflow())
        
    @Slot()
    def _do_stop_mapping(self):
        self.store.set_mapping_running(False)
        import asyncio
        asyncio.create_task(self.workflow_ctrl.execute_stop_mapping_workflow())

    @Slot()
    def _do_save_map(self):
        import asyncio
        map_name = self.ui.control_panel.get_map_name().strip() or "my_map"
        asyncio.create_task(self.workflow_ctrl.execute_save_map_workflow(map_name))
        QMessageBox.information(self, "已下发", "地图保存指令已下发，请稍候...")

    @Slot()
    def _do_start_nav(self):
        import asyncio
        if not self.store.chassis_running:
            QMessageBox.warning(self, "依赖提示", "请先开启「启动底盘」，否则无法进行导航定位！")
            return
        self.store.set_navigation_busy(True, "starting")
        map_name = self.ui.control_panel.get_map_name().strip()
        asyncio.create_task(self.workflow_ctrl.execute_navigation_workflow(map_name))

    @Slot()
    def _do_stop_nav(self):
        import asyncio
        self.store.set_navigation_busy(True, "stopping")
        asyncio.create_task(self.workflow_ctrl.execute_stop_navigation_workflow())



    @Slot(float, float, float, str)
    def _on_map_interaction(self, x: float, y: float, yaw: float, mode: str):
        import numpy as np
        # V2 地图直接工作在物理世界坐标系，所以逆变换矩阵为单位阵
        identity_inv = np.eye(3)
        if mode == 'initial_pose':
            self.nav_ctrl.set_initial_pose(x, y, yaw, identity_inv)
            logging.info(f"[_on_map_interaction] Sent initial pose: {x}, {y}, {yaw}")
        elif mode == 'goal':
            self.nav_ctrl.set_goal_pose(x, y, yaw, identity_inv)
            logging.info(f"[_on_map_interaction] Sent Nav goal: {x}, {y}, {yaw}")
            
    @Slot(float, float, float)
    def _on_manual_initial_pose(self, x: float, y: float, yaw: float):
        import numpy as np
        identity_inv = np.eye(3)
        self.nav_ctrl.set_initial_pose(x, y, yaw, identity_inv)
        logging.info(f"[Manual] Sent initial pose: x={x}, y={y}, yaw={yaw}")

    @Slot(float, float, float)
    def _on_manual_goal(self, x: float, y: float, yaw: float):
        import numpy as np
        identity_inv = np.eye(3)
        self.nav_ctrl.set_goal_pose(x, y, yaw, identity_inv)
        logging.info(f"[Manual] Sent nav goal: x={x}, y={y}, yaw={yaw}")

    def _center_map_on_robot(self):
        """将视角中心平滑移动到当前小车所在位置"""
        pose = self.store.current_pose
        if pose is not None and self.ui.map_view:
            self.ui.map_view.centerOn(pose.x, pose.y)

    # ---------------- 杂项设置与仿真 ----------------
    
    @Slot()
    def _toggle_fullscreen(self):
        if self.isFullScreen():
            self.showNormal()
        else:
            self.showFullScreen()
            
    @Slot()
    def _show_system_settings(self):
        dialog = SystemSetting(current_config=CONFIG, parent=self)
        if dialog.exec():
            # Apply changes (naive reconnect for MQTT if needed)
            settings = dialog.get_settings()
            new_host = settings.get("ip", self.mqtt_agent.host)
            new_port = self.mqtt_agent.port
            try:
                new_port = int(settings.get("port", self.mqtt_agent.port))
            except: pass
            
            if new_host != self.mqtt_agent.host or new_port != self.mqtt_agent.port:
                self.mqtt_agent.update_connection(new_host, new_port)

    # 仿真子进程列表
    _sim_processes = []
    
    @Slot(bool)
    def _toggle_simulation(self, checked: bool):
        self.async_ssh.mock_mode = checked
        if checked:
            self.ui.btn_simulation.setText("🔴 停止仿真")
            import subprocess
            scripts_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', 'scripts')
            for name in ['mock_robot.py', 'mock_lidar.py']:
                p = os.path.join(scripts_dir, name)
                if os.path.isfile(p):
                    proc = subprocess.Popen([sys.executable, p], stdout=None, stderr=None)
                    self._sim_processes.append(proc)
        else:
            self.ui.btn_simulation.setText("🟢 启动仿真")
            for p in self._sim_processes:
                p.terminate()
            self._sim_processes.clear()

    # ---------------- SSH 系统操作 ----------------
    @Slot()
    def _do_start_chassis(self):
        import asyncio
        asyncio.create_task(self.workflow_ctrl.execute_chassis_workflow())
        logging.info("[V2] Chassis bringup requested.")

    @Slot()
    def _do_start_mqtt_node(self):
        import asyncio
        asyncio.create_task(self.workflow_ctrl.execute_mqtt_workflow())
        logging.info("[V2] MQTT node start requested.")

    # ---------------- 地图下载/上传 ----------------
    @Slot()
    def _do_download_map(self):
        from PySide6.QtWidgets import QFileDialog
        save_dir = QFileDialog.getExistingDirectory(self, "选择保存目录")
        if save_dir:
            import asyncio
            map_name = self.ui.control_panel.get_map_name().strip() or "my_map"
            asyncio.create_task(self.workflow_ctrl.download_map(map_name, save_dir))
            QMessageBox.information(self, "已下发", f"地图将下载到: {save_dir}")

    @Slot()
    def _do_upload_map(self):
        from PySide6.QtWidgets import QFileDialog
        file_path, _ = QFileDialog.getOpenFileName(self, "选择地图文件", "", "地图 (*.yaml *.pgm);;All (*)")
        if file_path:
            import asyncio
            asyncio.create_task(self.workflow_ctrl.upload_map(file_path))

    # ---------------- 初始位姿保存/恢复 ----------------
    @Slot()
    def _do_save_initial_pose(self):
        import json
        pose = self.store.current_pose
        if pose:
            data = {"x": pose.x, "y": pose.y, "yaw": pose.yaw, "angle": pose.angle}
            path = PATHS_CONFIG.get('initial_pose_json', 'initial_pose.json')
            with open(path, 'w') as f:
                json.dump(data, f)
            logging.info(f"[V2] Saved initial pose to {path}: {data}")
            QMessageBox.information(self, "已保存", f"初始位姿已保存至 {path}")
        else:
            QMessageBox.warning(self, "无数据", "当前没有有效的机器人位姿")

    @Slot()
    def _do_recall_initial_pose(self):
        import json, numpy as np
        path = PATHS_CONFIG.get('initial_pose_json', 'initial_pose.json')
        try:
            with open(path, 'r') as f:
                data = json.load(f)
            pose = RobotPose.from_dict(data)
            x, y, yaw = pose.x, pose.y, pose.angle
            self.nav_ctrl.set_initial_pose(pose.x, pose.y, pose.yaw, np.eye(3))
            logging.info(f"[V2] Recalled initial pose from {path}: {data}")
            QMessageBox.information(self, "已恢复", f"初始位姿已恢复: X={x:.2f} Y={y:.2f} Yaw={yaw:.2f}")
        except FileNotFoundError:
            QMessageBox.warning(self, "文件不存在", f"未找到保存文件: {path}")
        except Exception as e:
            QMessageBox.warning(self, "错误", f"恢复失败: {e}")

    # ---------------- MQTT 连接状态 ----------------
    def _on_tf_data(self, transform: dict):
        parent = str(transform.get("parent", "")).lstrip("/")
        child = str(transform.get("child", "")).lstrip("/")
        if parent and child:
            self._frame_transforms[(parent, child)] = transform
        if parent == "map" and child == "odom":
            self._map_to_odom = transform
        self._refresh_mapping_pose()

    @staticmethod
    def _invert_transform(transform: dict) -> dict:
        yaw = float(transform.get("yaw", 0.0))
        tx = float(transform.get("x", 0.0))
        ty = float(transform.get("y", 0.0))
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        inv_x = -(cos_yaw * tx + sin_yaw * ty)
        inv_y = sin_yaw * tx - cos_yaw * ty
        return {"x": inv_x, "y": inv_y, "yaw": -yaw}

    @staticmethod
    def _is_finite_values(*values) -> bool:
        try:
            return all(math.isfinite(float(value)) for value in values)
        except (TypeError, ValueError):
            return False

    def _lookup_transform(self, parent: str, child: str):
        parent_norm = str(parent or "").lstrip("/")
        child_norm = str(child or "").lstrip("/")
        if not parent_norm or not child_norm:
            return None
        direct = self._frame_transforms.get((parent_norm, child_norm))
        if direct:
            return direct
        reverse = self._frame_transforms.get((child_norm, parent_norm))
        if reverse:
            return self._invert_transform(reverse)
        return None

    def _resolve_mapping_pose_from_tf(self) -> Optional[RobotPose]:
        for base_frame in ("base_footprint", "base_link"):
            direct = self._lookup_transform("map", base_frame)
            if direct:
                x = float(direct.get("x", 0.0))
                y = float(direct.get("y", 0.0))
                yaw = float(direct.get("yaw", 0.0))
                if not self._is_finite_values(x, y, yaw):
                    continue
                return RobotPose(
                    x=x,
                    y=y,
                    z=0.0,
                    yaw=yaw,
                    angle=math.degrees(yaw),
                    source="tf_map_base",
                )

        map_to_odom = self._lookup_transform("map", "odom")
        if not map_to_odom:
            return None

        for base_frame in ("base_footprint", "base_link"):
            odom_to_base = self._lookup_transform("odom", base_frame)
            if not odom_to_base:
                continue
            map_x = float(map_to_odom.get("x", 0.0))
            map_y = float(map_to_odom.get("y", 0.0))
            map_yaw = float(map_to_odom.get("yaw", 0.0))
            base_x = float(odom_to_base.get("x", 0.0))
            base_y = float(odom_to_base.get("y", 0.0))
            base_yaw = float(odom_to_base.get("yaw", 0.0))
            if not self._is_finite_values(map_x, map_y, map_yaw, base_x, base_y, base_yaw):
                continue
            x, y, yaw = apply_pose_transform(
                transform_x=map_x,
                transform_y=map_y,
                transform_yaw=map_yaw,
                pose_x=base_x,
                pose_y=base_y,
                pose_yaw=base_yaw,
            )
            if not self._is_finite_values(x, y, yaw):
                continue
            return RobotPose(
                x=x,
                y=y,
                z=0.0,
                yaw=yaw,
                angle=math.degrees(yaw),
                source="tf_map_odom_base",
            )
        return None

    def _refresh_mapping_pose(self):
        if not self.store.mapping_running:
            return

        pose = self._resolve_mapping_pose_from_tf()
        source = "TF"
        if pose is None and self._latest_odom_pose is not None:
            source = "ODOM_FALLBACK"
            pose = RobotPose(
                x=self._latest_odom_pose.x,
                y=self._latest_odom_pose.y,
                z=self._latest_odom_pose.z,
                yaw=self._latest_odom_pose.yaw,
                angle=self._latest_odom_pose.angle,
                source="odom_fallback",
            )
        if pose is None:
            return
        if not self._is_finite_values(pose.x, pose.y, pose.z, pose.yaw, pose.angle):
            return

        self.store.update_robot_pose(pose)
        self._mapping_pose_log_counter += 1
        if self._mapping_pose_log_counter % 50 == 0:
            logging.info(
                "[V2][MappingPose] source=%s x=%.3f y=%.3f yaw=%.1f°",
                source,
                pose.x,
                pose.y,
                math.degrees(pose.yaw),
            )

    def _find_sensor_transform(self, frame_id: str):
        child = str(frame_id or "").lstrip("/")
        if not child or child in {"base_link", "base_footprint"}:
            return None

        preferred_parents = ("base_link", "base_footprint", "base_scan")
        for parent in preferred_parents:
            transform = self._frame_transforms.get((parent, child))
            if transform:
                return transform

        for (parent, candidate_child), transform in self._frame_transforms.items():
            if candidate_child == child and parent not in {"map", "odom"}:
                return transform

        return None

    def _on_store_robot_pose_changed(self, pose):
        if not pose:
            return
        if not self._is_finite_values(pose.x, pose.y, pose.z, pose.yaw, pose.angle):
            return
        self.ui.map_view.update_robot_pose(pose.x, pose.y, pose.yaw)
        if self.pose_recorder.recording:
            self.pose_recorder.append(pose.x, pose.y, 0.0, pose.angle)

    def _on_store_path_changed(self, path_points):
        if self.store.mapping_running:
            self.ui.map_view.clear_path()
            return
        self.ui.map_view.update_path(path_points or [])

    def _log_mapping_scan(self, status: str):
        self._mapping_scan_log_counter += 1
        if status != self._last_mapping_scan_status or self._mapping_scan_log_counter % 20 == 0:
            logging.info("[V2][MappingScan] %s", status)
        self._last_mapping_scan_status = status

    def _on_store_scan_changed(self, scan_dict):
        if not scan_dict:
            self.ui.map_view.clear_scan()
            return

        pose = self.store.current_pose
        if pose is None:
            if self.store.mapping_running:
                self._log_mapping_scan("skipped:no_pose")
            self.ui.map_view.clear_scan()
            return

        scan_x = pose.x
        scan_y = pose.y
        scan_yaw = pose.yaw
        frame_id = str(scan_dict.get("frame_id", "")).lstrip("/")
        sensor_transform = self._find_sensor_transform(frame_id)

        if self.store.mapping_running:
            if not self._show_mapping_scan_overlay:
                self._log_mapping_scan("skipped:disabled")
                self.ui.map_view.clear_scan()
                return
            if not str(pose.source or "").startswith("tf_"):
                self._log_mapping_scan("skipped:no_tf_pose")
                self.ui.map_view.clear_scan()
                return
            if self._last_scan_received_at is None or time.monotonic() - self._last_scan_received_at > self._mapping_scan_stale_timeout_s:
                self._log_mapping_scan("skipped:stale_scan")
                self.ui.map_view.clear_scan()
                return
            if frame_id not in {"", "base_link", "base_footprint"} and sensor_transform is None:
                self._log_mapping_scan("skipped:no_sensor_tf")
                self.ui.map_view.clear_scan()
                return

        if sensor_transform:
            scan_x, scan_y, scan_yaw = apply_pose_transform(
                transform_x=pose.x,
                transform_y=pose.y,
                transform_yaw=pose.yaw,
                pose_x=float(sensor_transform.get("x", 0.0)),
                pose_y=float(sensor_transform.get("y", 0.0)),
                pose_yaw=float(sensor_transform.get("yaw", 0.0)),
            )

        self.ui.map_view.update_scan(scan_dict, scan_x, scan_y, scan_yaw)
        if self.store.mapping_running:
            self._log_mapping_scan("rendered")

    def _on_mapping_state_changed(self, running: bool):
        self.ui.map_view.clear_path()
        self.ui.map_view.clear_scan()
        self.store.update_path([])
        self.store.update_scan({})
        self._mapping_scan_log_counter = 0
        self._last_mapping_scan_status = ""

        if running:
            self._live_map_received = False
            self._cleared_for_live_map = True
            self._mapping_map_log_counter = 0
            self.ui.map_view.clear_map()
            logging.info("[V2][MappingMap] source=%s cleared_for_live_map=%s", self._map_source.upper(), True)
            return

        logging.info("[V2][MappingMap] source=%s cleared_for_live_map=%s", self._map_source.upper(), self._cleared_for_live_map)

    def _on_mqtt_connection_status(self, connected: bool, message: str):
        logging.info(f"MQTT Status: {connected} - {message}")
        # 更新遥测面板的连接指示器
        if connected:
            self.ui.telemetry_panel.indicator_circle.setStyleSheet("color: #3fb950; font-size: 16px;")
            self.ui.telemetry_panel.status_label.setText("MQTT 已连接")
        else:
            self.ui.telemetry_panel.indicator_circle.setStyleSheet("color: #f14c4c; font-size: 16px;")
            self.ui.telemetry_panel.status_label.setText("MQTT 连接断开")

    # ---------------- Odom (建图模式) ----------------
    def _on_odom_data(self, pose):
        if pose and self._is_finite_values(pose.x, pose.y, pose.z, pose.yaw, pose.angle):
            self._latest_odom_pose = pose
        self._refresh_mapping_pose()

    def _on_pose_data(self, pose):
        if not pose:
            return
        if not self._is_finite_values(pose.x, pose.y, pose.z, pose.yaw, pose.angle):
            return
        # Mapping mode pose should come from TF-resolved map coordinates.
        # Ignore AMCL pose here to avoid overriding mapping pose with stale nav data.
        if self.store.mapping_running:
            return
        self.store.update_robot_pose(pose)

    # ---------------- 键盘遥控 (WASD) ----------------
    def keyPressEvent(self, event):
        if hasattr(self, 'teleop_ctrl'):
            self.teleop_ctrl.handle_key_press(event)
        super().keyPressEvent(event)

    def keyReleaseEvent(self, event):
        if hasattr(self, 'teleop_ctrl'):
            self.teleop_ctrl.handle_key_release(event)
        super().keyReleaseEvent(event)

    # ---------------- 窗口关闭 ----------------
    def _clear_runtime_layers(self):
        self.ui.map_view.clear_path()
        self.ui.map_view.clear_scan()
        self.store.update_path([])
        self.store.update_scan({})

    def _apply_local_map(self, map_meta):
        self._map_source = "local"
        self._live_map_received = False
        self._cleared_for_live_map = False
        self._last_live_map_received_at = None
        self.store.update_map(map_meta)

    def _on_live_map_data(self, map_meta):
        self._last_live_map_received_at = time.monotonic()
        if not (self.store.mapping_running or self._map_source == "live_mqtt" or self._cleared_for_live_map):
            return

        self._map_source = "live_mqtt"
        self._live_map_received = True
        self.store.update_map(map_meta)
        self._cleared_for_live_map = False

    @Slot()
    def _do_start_mapping(self):
        if not self.store.chassis_running:
            QMessageBox.warning(self, "依赖提示", "请先开启「启动底盘」，否则无法获取雷达和里程计数据！")
            return
        import asyncio
        asyncio.create_task(self.workflow_ctrl.execute_mapping_workflow())

    @Slot()
    def _do_stop_mapping(self):
        asyncio.create_task(self.workflow_ctrl.execute_stop_mapping_workflow())

    @Slot()
    def _do_start_chassis(self):
        if self.store.chassis_running:
            asyncio.create_task(self.workflow_ctrl.execute_stop_chassis_workflow())
        else:
            asyncio.create_task(self.workflow_ctrl.execute_chassis_workflow())

    @Slot()
    def _do_start_mqtt_node(self):
        if self.store.mqtt_running:
            asyncio.create_task(self.workflow_ctrl.execute_stop_mqtt_workflow())
        else:
            asyncio.create_task(self.workflow_ctrl.execute_mqtt_workflow())

    @Slot(str, bool, str)
    def _on_workflow_finished(self, workflow_name: str, success: bool, message: str):
        logging.info("[WorkflowFinished] %s success=%s msg=%s", workflow_name, success, message)
        if workflow_name not in {"save_map", "download_map", "upload_map"}:
            self.statusBar().showMessage(message, 6000)

        # 重点优化：当核心启动型服务失败时，给用户一个明确的弹窗，而不是像以前直接吞掉
        if not success and not workflow_name.startswith("stop_"):
            titles = {
                "mqtt": "连接 MQTT 桥接失败",
                "chassis": "启动底盘失败",
                "gmapping": "启动建图失败",
                "navigation": "启动导航失败"
            }
            if workflow_name in titles:
                QMessageBox.warning(self, titles[workflow_name], message)

        if workflow_name == "chassis":
            if success:
                self.store.set_chassis_running(True)
            return

        if workflow_name == "stop_chassis":
            if success:
                self.store.set_chassis_running(False)
                self.store.set_mapping_running(False)
                self.store.set_navigation_running(False)
                self._clear_runtime_layers()
            return

        if workflow_name == "mqtt":
            if success:
                self.store.set_mqtt_running(True)
            return

        if workflow_name == "stop_mqtt":
            if success:
                self.store.set_mqtt_running(False)
            return

        if workflow_name == "gmapping":
            if success:
                self.store.set_mapping_running(True)
            else:
                self.store.set_mapping_running(False)
            return

        if workflow_name == "stop_mapping":
            if success:
                self.store.set_mapping_running(False)
                self._clear_runtime_layers()
            return

        if workflow_name == "navigation":
            if success:
                self.store.set_navigation_running(True)
            else:
                self.store.set_navigation_running(False)
            self.store.set_navigation_busy(False)
            return

        if workflow_name == "stop_navigation":
            if success:
                self.store.set_navigation_running(False)
                self._clear_runtime_layers()
            self.store.set_navigation_busy(False)
            return

        if workflow_name == "save_map":
            if success:
                QMessageBox.information(self, "保存地图", message)
            else:
                QMessageBox.warning(self, "保存地图失败", message)
            return

        if workflow_name == "download_map":
            if success:
                QMessageBox.information(self, "下载地图", message)
            else:
                QMessageBox.warning(self, "下载地图失败", message)
            return

        if workflow_name == "upload_map":
            if success:
                QMessageBox.information(self, "上传地图", message)
            else:
                QMessageBox.warning(self, "上传地图失败", message)
            return

    def _refresh_mapping_pose(self):
        if not self.store.mapping_running:
            return

        pose = self._resolve_mapping_pose_from_tf()
        if pose is None and self._latest_odom_pose is not None:
            pose = RobotPose(
                x=self._latest_odom_pose.x,
                y=self._latest_odom_pose.y,
                z=self._latest_odom_pose.z,
                yaw=self._latest_odom_pose.yaw,
                angle=self._latest_odom_pose.angle,
                source="odom_fallback",
            )
        if pose is None:
            return
        if not self._is_finite_values(pose.x, pose.y, pose.z, pose.yaw, pose.angle):
            return

        self.store.update_robot_pose(pose)

    def _log_mapping_scan(self, status: str):
        self._last_mapping_scan_status = status

    def _on_mapping_state_changed(self, running: bool):
        self._clear_runtime_layers()
        self._mapping_scan_log_counter = 0
        self._last_mapping_scan_status = ""

        if running:
            self._live_map_received = False
            self._cleared_for_live_map = True
            self._mapping_map_log_counter = 0
            self.ui.map_view.clear_map()

    def _on_mqtt_connection_status(self, connected: bool, message: str):
        if message:
            self.statusBar().showMessage(message, 4000)

    def closeEvent(self, event):
        for p in self._sim_processes:
            p.terminate()
        try:
            loop = asyncio.get_event_loop()
            asyncio.ensure_future(self.async_ssh.close_async())
        except:
            pass
        event.accept()
