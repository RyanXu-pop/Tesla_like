import os
import sys

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from src.ui_v2.panels.control_panel import ControlPanel
from src.ui_v2.robot_state_hub import RobotStateHub


def test_navigation_busy_starting_updates_button(qapp):
    store = RobotStateHub()
    panel = ControlPanel(store)

    store.set_navigation_busy(True, "starting")

    assert panel.btn_toggle_navigation.text() == "启动中..."
    assert not panel.btn_toggle_navigation.isEnabled()


def test_navigation_busy_stop_success_restores_idle_state(qapp):
    store = RobotStateHub()
    panel = ControlPanel(store)

    store.set_navigation_running(True)
    store.set_navigation_busy(True, "stopping")
    assert panel.btn_toggle_navigation.text() == "停止中..."

    store.set_navigation_running(False)
    store.set_navigation_busy(False)

    assert panel.btn_toggle_navigation.text() == "启动导航"
    assert panel.btn_toggle_navigation.isEnabled()
