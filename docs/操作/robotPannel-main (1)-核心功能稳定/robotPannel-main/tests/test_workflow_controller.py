import os
import sys
import asyncio
from unittest.mock import AsyncMock, MagicMock

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from src.controllers.workflow_controller import WorkflowController


import pytest


@pytest.fixture
def workflow_ctrl():
    async_ssh = MagicMock()
    map_mgr = MagicMock()
    return WorkflowController(async_ssh, map_mgr)


def test_execute_navigation_workflow_aborts_on_chassis_failure(workflow_ctrl):
    events = []
    workflow_ctrl.workflow_finished.connect(lambda name, ok, msg: events.append((name, ok, msg)))
    workflow_ctrl.start_service_async = AsyncMock(return_value=(False, "boom"))

    asyncio.run(workflow_ctrl.execute_navigation_workflow())

    assert events[-1] == ("navigation", False, "底盘启动失败: boom")


def test_execute_stop_navigation_workflow_emits_result(workflow_ctrl):
    events = []
    workflow_ctrl.workflow_finished.connect(lambda name, ok, msg: events.append((name, ok, msg)))
    workflow_ctrl.stop_service_async = AsyncMock(return_value=(True, "stopped"))

    asyncio.run(workflow_ctrl.execute_stop_navigation_workflow())

    assert events[-1] == ("stop_navigation", True, "stopped")
