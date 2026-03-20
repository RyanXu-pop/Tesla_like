import os
import sys
from PySide6.QtWidgets import QWidget, QVBoxLayout, QHBoxLayout, QPushButton
from PySide6.QtCore import Qt

from src.ui_v2.map.map_view import MapGraphicsView
from src.ui_v2.panels.telemetry_panel import TelemetryPanel
from src.ui_v2.panels.control_panel import ControlPanel
from src.ui_v2.panels.teleop_panel import TeleopPanel
from src.ui_v2.panels.pose_panel import PoseRecordPanel
from src.ui_v2.panels.unified_drawer import UnifiedDrawer

class MainLayoutWidget(QWidget):
    """
    负责 UI V2 的所有主界面布局元件的分发和相对定位。
    充当纯粹的视图层 (View Layer)。
    """
    def __init__(self, store, parent=None):
        super().__init__(parent)
        self.store = store
        self._setup_ui()

    def _setup_ui(self):
        base_layout = QVBoxLayout(self)
        base_layout.setContentsMargins(0, 0, 0, 0)
        
        # 核心地图视图
        self.map_view = MapGraphicsView()
        base_layout.addWidget(self.map_view)
        
        # --- 悬浮操作区设定 (Absolute Positioning Overlay) ---
        # 右上角：遥测展板 (实际上由抽屉管理)
        self.telemetry_panel = TelemetryPanel(self.store)
        self.control_panel = ControlPanel(self.store, self.map_view)
        
        self.teleop_panel = TeleopPanel()
        self.teleop_panel.toggle_drawer() # 强行展开内部逻辑
        self.teleop_panel.header_btn.hide()
        
        self.pose_panel = PoseRecordPanel()
        self.pose_panel.toggle_drawer() # 强行展开内部逻辑
        self.pose_panel.header_btn.hide()

        # 左侧定位统一抽屉
        self.unified_drawer = UnifiedDrawer(self.map_view)
        self.unified_drawer.add_panel(self.telemetry_panel)
        self.unified_drawer.add_panel(self.control_panel)
        self.unified_drawer.add_panel(self.teleop_panel)
        self.unified_drawer.add_panel(self.pose_panel)
        
        # 抽屉高度动画时保持左下角吸底对齐
        def _reposition_drawer(h):
            my_h = self.map_view.height()
            margin = 20
            self.unified_drawer.move(margin, my_h - h - margin)
        self.unified_drawer.height_changed.connect(_reposition_drawer)
        
        # 顶部：系统设置与全屏按钮
        self.top_bar = QWidget(parent=self.map_view)
        top_layout = QHBoxLayout(self.top_bar)
        top_layout.setContentsMargins(0, 0, 0, 0)
        top_layout.setAlignment(Qt.AlignRight)
        
        self.btn_simulation = QPushButton("🟢 仿真")
        self.btn_simulation.setCheckable(True)
        self.btn_simulation.setStyleSheet("background: rgba(40,40,40,200); color: white; border-radius: 4px; padding: 6px 12px;")
        top_layout.addWidget(self.btn_simulation)
        
        self.btn_fullscreen = QPushButton("⛶ 全屏")
        self.btn_fullscreen.setStyleSheet("background: rgba(40,40,40,200); color: white; border-radius: 4px; padding: 6px 12px;")
        top_layout.addWidget(self.btn_fullscreen)
        
        self.btn_settings = QPushButton("⚙ 设置")
        self.btn_settings.setStyleSheet("background: #007acc; color: white; border-radius: 4px; padding: 6px 12px;")
        top_layout.addWidget(self.btn_settings)

        # 悬浮定位小车按钮 (Apple Maps Locate Me 效果)
        self.btn_locate_me = QPushButton("📍")
        self.btn_locate_me.setParent(self.map_view)
        self.btn_locate_me.setCursor(Qt.PointingHandCursor)
        self.btn_locate_me.setStyleSheet("""
            QPushButton {
                background: rgba(45, 45, 48, 230);
                color: #007acc;
                border: 1px solid #3e3e42;
                border-radius: 20px;
                font-size: 20px;
                padding-bottom: 2px;
            }
            QPushButton:hover {
                background: #3e3e42;
                color: #0098ff;
            }
        """)
        self.btn_locate_me.setFixedSize(40, 40)

    def resizeEvent(self, event):
        super().resizeEvent(event)
        
        w = self.width()
        h = self.height()
        if w == 0 or h == 0: return

        # 定位顶部工具栏 (靠右上角)
        self.top_bar.setGeometry(w - 250, 20, 230, 40)
        
        # 统一抽屉定位于左下角
        margin = 20
        drawer_h = int(h * 0.85) # 抽屉最大占屏幕高度 85%
        self.unified_drawer.set_max_height(drawer_h)
        self.unified_drawer.move(margin, h - self.unified_drawer.height() - margin)
        
        # 定位 Locate Me 按钮 (右下角)
        self.btn_locate_me.move(w - 60, h - 60)
