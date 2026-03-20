import os
import re

file_path = r"c:/Users/ruiru/OneDrive/Desktop/操作/robotPannel-main-等待5s停止功能，日志整洁/src/ui_v2/main_window.py"
with open(file_path, "r", encoding="utf-8") as f:
    content = f.read()

# 1. Update imports
content = content.replace(
    "from .theme import apply_theme\nfrom .robot_state_hub import RobotStateHub\nfrom .map.map_view import MapGraphicsView\nfrom .panels.telemetry_panel import TelemetryPanel\nfrom .panels.control_panel import ControlPanel\nfrom .panels.teleop_panel import TeleopPanel\nfrom .panels.pose_panel import PoseRecordPanel\nfrom .panels.unified_drawer import UnifiedDrawer",
    "from .theme import apply_theme\nfrom .robot_state_hub import RobotStateHub\nfrom .main_layout import MainLayoutWidget"
)

# 2. Update __init__ setup
content = content.replace(
    "        # 4. 界面构建\n        self._setup_ui()",
    "        # 4. 界面构建\n        self.ui = MainLayoutWidget(self.store, self)\n        self.setCentralWidget(self.ui)"
)

# 3. Delete _setup_ui and eventFilter
setup_ui_start = content.find("    def _setup_ui(self):")
event_filter_end = content.find("        return super().eventFilter(obj, event)") + len("        return super().eventFilter(obj, event)\n")
if setup_ui_start != -1 and event_filter_end != -1:
    content = content[:setup_ui_start] + "\n" + content[event_filter_end:]

# 4. Delete _reposition_drawer logic
drawer_logic = """        # ================== Unified Drawer Animations ==================
        def _reposition_drawer(h):
            if not self.map_view: return
            my_h = self.map_view.height()
            margin = 20
            self.unified_drawer.move(margin, my_h - h - margin)
        self.unified_drawer.height_changed.connect(_reposition_drawer)
        
"""
content = content.replace(drawer_logic, "")

# 5. Mass rename panel references
renames = [
    ("self.map_view", "self.ui.map_view"),
    ("self.telemetry_panel", "self.ui.telemetry_panel"),
    ("self.control_panel", "self.ui.control_panel"),
    ("self.teleop_panel", "self.ui.teleop_panel"),
    ("self.pose_panel", "self.ui.pose_panel"),
    ("self.unified_drawer", "self.ui.unified_drawer"),
    ("self.top_bar", "self.ui.top_bar"),
    ("self.btn_simulation", "self.ui.btn_simulation"),
    ("self.btn_fullscreen", "self.ui.btn_fullscreen"),
    ("self.btn_settings", "self.ui.btn_settings"),
    ("self.btn_locate_me", "self.ui.btn_locate_me")
]

for old, new in renames:
    content = content.replace(old, new)

# Undo duplicate `self.ui.ui...`
content = content.replace("self.ui.ui", "self.ui")

# 6. Add button connections inside _bind_signals
bind_start = content.find("    def _bind_signals(self):")
if bind_start != -1:
    idx = content.find("        # ================== MQTT Data -> Store ==================", bind_start)
    insert_str = """        # --- UI Top Bar Actions ---
        self.ui.btn_simulation.clicked.connect(self._toggle_simulation)
        self.ui.btn_fullscreen.clicked.connect(self._toggle_fullscreen)
        self.ui.btn_settings.clicked.connect(self._show_system_settings)
        self.ui.btn_locate_me.clicked.connect(self._center_map_on_robot)
        
"""
    content = content[:idx] + insert_str + content[idx:]

with open(file_path, "w", encoding="utf-8") as f:
    f.write(content)

print("done")
