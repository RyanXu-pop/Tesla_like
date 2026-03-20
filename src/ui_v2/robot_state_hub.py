import logging
import math
from typing import Optional

from PySide6.QtCore import QObject, QTimer, Signal

from src.core.models import MapMetadata, RobotPose


class RobotStateHub(QObject):
    """
    Single source of truth for V2 UI state.
    """

    voltage_changed = Signal(float, float)
    chassis_alive_changed = Signal(bool)
    robot_pose_changed = Signal(RobotPose)
    laser_scan_changed = Signal(dict)
    global_path_changed = Signal(list)
    map_data_changed = Signal(MapMetadata)

    mapping_state_changed = Signal(bool)
    navigation_state_changed = Signal(bool)
    chassis_service_changed = Signal(bool)
    mqtt_service_changed = Signal(bool)
    navigation_busy_changed = Signal(bool, str)
    workflow_message = Signal(str)

    def __init__(self, parent=None):
        super().__init__(parent)
        self._state = {
            "chassis_alive": False,
            "voltage": 0.0,
            "chassis_running": False,
            "mqtt_running": False,
            "mapping_running": False,
            "navigation_running": False,
            "navigation_busy": False,
            "navigation_busy_reason": "",
            "robot_pose": None,
            "target_pose": None,
            "initial_pose": None,
            "map_metadata": None,
            "laser_scan": None,
            "global_path": [],
        }

        # Watchdog 已被移除，依靠明确的状态事件
        pass

    def _ping_watchdog(self):
        pass

    def _on_watchdog_timeout(self):
        pass

    def update_voltage(self, voltage: float):
        self._ping_watchdog()
        self._state["voltage"] = voltage
        percent = min(max((voltage - 20.0) / (24.0 - 20.0), 0), 1) * 100.0
        self.voltage_changed.emit(voltage, percent)

    def update_chassis_status(self, is_alive: bool):
        self._ping_watchdog()
        if self._state["chassis_alive"] != is_alive:
            self._state["chassis_alive"] = is_alive
            self.chassis_alive_changed.emit(is_alive)

    def update_robot_pose(self, pose: RobotPose):
        numeric_values = (pose.x, pose.y, pose.z, pose.yaw, pose.angle)
        if not all(math.isfinite(value) for value in numeric_values):
            logging.warning("[Store] Ignored non-finite robot pose: %s", pose)
            return
        self._ping_watchdog()
        self._state["robot_pose"] = pose
        self.robot_pose_changed.emit(pose)

    def update_scan(self, scan_data: dict):
        self._ping_watchdog()
        self._state["laser_scan"] = scan_data
        self.laser_scan_changed.emit(scan_data)

    def update_path(self, path: list):
        self._ping_watchdog()
        self._state["global_path"] = path
        self.global_path_changed.emit(path)

    def update_map(self, map_meta: MapMetadata):
        self._ping_watchdog()
        self._state["map_metadata"] = map_meta
        self.map_data_changed.emit(map_meta)

    def set_mapping_running(self, running: bool):
        self._state["mapping_running"] = running
        self.mapping_state_changed.emit(running)
        if running:
            self.set_navigation_running(False)

    def set_navigation_running(self, running: bool):
        self._state["navigation_running"] = running
        self.navigation_state_changed.emit(running)
        if running:
            self.set_mapping_running(False)

    def set_chassis_running(self, running: bool):
        if self._state["chassis_running"] == running:
            return
        self._state["chassis_running"] = running
        self.chassis_service_changed.emit(running)
        if not running and self._state["chassis_alive"]:
            self._state["chassis_alive"] = False
            self.chassis_alive_changed.emit(False)

    def set_mqtt_running(self, running: bool):
        if self._state["mqtt_running"] == running:
            return
        self._state["mqtt_running"] = running
        self.mqtt_service_changed.emit(running)

    def set_navigation_busy(self, busy: bool, reason: str = ""):
        self._state["navigation_busy"] = busy
        self._state["navigation_busy_reason"] = reason if busy else ""
        self.navigation_busy_changed.emit(busy, self._state["navigation_busy_reason"])

    def broadcast_message(self, msg: str):
        self.workflow_message.emit(msg)

    @property
    def mapping_running(self) -> bool:
        return self._state["mapping_running"]

    @property
    def navigation_running(self) -> bool:
        return self._state["navigation_running"]

    @property
    def navigation_busy(self) -> bool:
        return self._state["navigation_busy"]

    @property
    def navigation_busy_reason(self) -> str:
        return self._state["navigation_busy_reason"]

    @property
    def current_pose(self) -> Optional[RobotPose]:
        return self._state["robot_pose"]

    @property
    def chassis_running(self) -> bool:
        return self._state["chassis_running"]

    @property
    def mqtt_running(self) -> bool:
        return self._state["mqtt_running"]
