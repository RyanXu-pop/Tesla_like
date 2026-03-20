import math
import os
import sys
import time
from types import SimpleNamespace
from unittest.mock import MagicMock

import pytest

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from src.core.models import RobotPose
from src.core.utils import apply_pose_transform
from src.ui_v2.main_window import MyMainWindow


def _make_dummy_for_pose_update():
    store = SimpleNamespace(mapping_running=True, update_robot_pose=MagicMock())
    dummy = SimpleNamespace(
        store=store,
        _frame_transforms={},
        _latest_odom_pose=None,
        _mapping_pose_log_counter=0,
    )
    dummy._is_finite_values = MyMainWindow._is_finite_values
    dummy._invert_transform = MyMainWindow._invert_transform
    dummy._lookup_transform = lambda parent, child: MyMainWindow._lookup_transform(dummy, parent, child)
    dummy._resolve_mapping_pose_from_tf = lambda: MyMainWindow._resolve_mapping_pose_from_tf(dummy)
    return dummy


def test_mapping_pose_prefers_direct_map_to_base_tf():
    dummy = _make_dummy_for_pose_update()
    dummy._frame_transforms[("map", "base_footprint")] = {"x": 1.5, "y": -0.4, "yaw": 0.3}
    dummy._refresh_mapping_pose = lambda: MyMainWindow._refresh_mapping_pose(dummy)

    MyMainWindow._on_odom_data(dummy, RobotPose(x=9.0, y=9.0, yaw=1.2, angle=68.75493541569878, source="odom"))

    updated = dummy.store.update_robot_pose.call_args.args[0]
    assert updated.x == 1.5
    assert updated.y == -0.4
    assert updated.yaw == 0.3
    assert updated.source == "tf_map_base"


def test_mapping_pose_composes_map_odom_with_odom_base():
    dummy = _make_dummy_for_pose_update()
    dummy._frame_transforms[("map", "odom")] = {"x": 2.0, "y": 1.0, "yaw": math.pi / 2}
    dummy._frame_transforms[("odom", "base_footprint")] = {"x": 1.0, "y": 0.0, "yaw": 0.1}
    dummy._refresh_mapping_pose = lambda: MyMainWindow._refresh_mapping_pose(dummy)

    MyMainWindow._on_odom_data(dummy, RobotPose(x=0.0, y=0.0, yaw=0.0, angle=0.0, source="odom"))

    updated = dummy.store.update_robot_pose.call_args.args[0]
    assert updated.x == pytest.approx(2.0)
    assert updated.y == pytest.approx(2.0)
    assert updated.yaw == pytest.approx(math.pi / 2 + 0.1)
    assert updated.source == "tf_map_odom_base"


def test_mapping_pose_falls_back_to_odom_when_tf_missing():
    dummy = _make_dummy_for_pose_update()
    dummy._refresh_mapping_pose = lambda: MyMainWindow._refresh_mapping_pose(dummy)
    odom_pose = RobotPose(x=1.2, y=-0.5, yaw=0.25, angle=14.32394487827058, source="odom")

    MyMainWindow._on_odom_data(dummy, odom_pose)

    updated = dummy.store.update_robot_pose.call_args.args[0]
    assert updated.x == odom_pose.x
    assert updated.y == odom_pose.y
    assert updated.yaw == odom_pose.yaw
    assert updated.source == "odom_fallback"


def test_mapping_scan_skips_when_pose_is_not_tf():
    dummy = SimpleNamespace(
        store=SimpleNamespace(mapping_running=True, current_pose=RobotPose(x=1.0, y=2.0, yaw=0.1, source="odom_fallback")),
        _show_mapping_scan_overlay=True,
        _last_scan_received_at=time.monotonic(),
        _mapping_scan_stale_timeout_s=1.5,
        _mapping_scan_log_counter=0,
        _last_mapping_scan_status="",
        _find_sensor_transform=lambda _frame_id: {"x": 0.1, "y": 0.0, "yaw": 0.0},
        _log_mapping_scan=MagicMock(),
        map_view=SimpleNamespace(update_scan=MagicMock(), clear_scan=MagicMock()),
    )

    MyMainWindow._on_store_scan_changed(dummy, {"frame_id": "laser", "ranges": [1.0]})

    dummy.map_view.clear_scan.assert_called_once()
    dummy.map_view.update_scan.assert_not_called()


def test_mapping_scan_renders_current_frame_with_sensor_transform():
    pose = RobotPose(x=1.0, y=2.0, yaw=0.2, angle=11.459155902616466, source="tf_map_base")
    sensor_transform = {"x": 0.1, "y": -0.05, "yaw": 0.15}
    dummy = SimpleNamespace(
        store=SimpleNamespace(mapping_running=True, current_pose=pose),
        _show_mapping_scan_overlay=True,
        _last_scan_received_at=time.monotonic(),
        _mapping_scan_stale_timeout_s=1.5,
        _mapping_scan_log_counter=0,
        _last_mapping_scan_status="",
        _find_sensor_transform=lambda _frame_id: sensor_transform,
        _log_mapping_scan=MagicMock(),
        map_view=SimpleNamespace(update_scan=MagicMock(), clear_scan=MagicMock()),
    )
    scan_dict = {"frame_id": "laser", "angle_min": 0.0, "angle_increment": 0.1, "ranges": [1.0]}

    MyMainWindow._on_store_scan_changed(dummy, scan_dict)

    expected_x, expected_y, expected_yaw = apply_pose_transform(
        transform_x=pose.x,
        transform_y=pose.y,
        transform_yaw=pose.yaw,
        pose_x=sensor_transform["x"],
        pose_y=sensor_transform["y"],
        pose_yaw=sensor_transform["yaw"],
    )
    args = dummy.map_view.update_scan.call_args.args
    assert args[0] == scan_dict
    assert args[1] == pytest.approx(expected_x)
    assert args[2] == pytest.approx(expected_y)
    assert args[3] == pytest.approx(expected_yaw)
    dummy.map_view.clear_scan.assert_not_called()


def test_mapping_state_change_clears_local_map_and_dynamic_layers():
    store = SimpleNamespace(update_path=MagicMock(), update_scan=MagicMock())
    dummy = SimpleNamespace(
        store=store,
        map_view=SimpleNamespace(clear_map=MagicMock(), clear_path=MagicMock(), clear_scan=MagicMock()),
        _map_source="local",
        _live_map_received=True,
        _cleared_for_live_map=False,
        _mapping_map_log_counter=7,
        _mapping_scan_log_counter=9,
        _last_mapping_scan_status="rendered",
    )

    MyMainWindow._on_mapping_state_changed(dummy, True)

    dummy.map_view.clear_map.assert_called_once()
    dummy.map_view.clear_path.assert_called_once()
    dummy.map_view.clear_scan.assert_called_once()
    store.update_path.assert_called_once_with([])
    store.update_scan.assert_called_once_with({})
    assert dummy._live_map_received is False
    assert dummy._cleared_for_live_map is True
    assert dummy._mapping_map_log_counter == 0


def test_live_map_switches_mapping_view_to_live_source():
    map_meta = object()
    store = SimpleNamespace(mapping_running=True, update_map=MagicMock())
    dummy = SimpleNamespace(
        store=store,
        _map_source="local",
        _live_map_received=False,
        _cleared_for_live_map=True,
        _mapping_map_log_counter=0,
        _last_live_map_received_at=None,
    )

    MyMainWindow._on_live_map_data(dummy, map_meta)

    store.update_map.assert_called_once_with(map_meta)
    assert dummy._map_source == "live_mqtt"
    assert dummy._live_map_received is True
    assert dummy._cleared_for_live_map is False


def test_live_map_is_accepted_while_waiting_for_first_live_frame():
    map_meta = object()
    store = SimpleNamespace(mapping_running=False, update_map=MagicMock())
    dummy = SimpleNamespace(
        store=store,
        _map_source="local",
        _live_map_received=False,
        _cleared_for_live_map=True,
        _mapping_map_log_counter=0,
        _last_live_map_received_at=None,
    )

    MyMainWindow._on_live_map_data(dummy, map_meta)

    store.update_map.assert_called_once_with(map_meta)
    assert dummy._map_source == "live_mqtt"
    assert dummy._live_map_received is True
    assert dummy._cleared_for_live_map is False


def test_amcl_pose_is_ignored_in_mapping_mode():
    dummy = SimpleNamespace(
        store=SimpleNamespace(mapping_running=True, update_robot_pose=MagicMock()),
        _is_finite_values=MyMainWindow._is_finite_values,
    )
    pose = RobotPose(x=2.0, y=3.0, yaw=0.2, angle=11.459155902616466, source="amcl")

    MyMainWindow._on_pose_data(dummy, pose)

    dummy.store.update_robot_pose.assert_not_called()


def test_amcl_pose_updates_robot_pose_when_not_mapping():
    dummy = SimpleNamespace(
        store=SimpleNamespace(mapping_running=False, update_robot_pose=MagicMock()),
        _is_finite_values=MyMainWindow._is_finite_values,
    )
    pose = RobotPose(x=2.0, y=3.0, yaw=0.2, angle=11.459155902616466, source="amcl")

    MyMainWindow._on_pose_data(dummy, pose)

    dummy.store.update_robot_pose.assert_called_once_with(pose)
