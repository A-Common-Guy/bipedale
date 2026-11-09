"""Stand-ins for the ROS Python modules on machines without ROS.

The `ros_stubs` fixture lets the launch files and the gamepad nodes be imported
in a plain virtualenv, so their pure logic is tested there too. Where ROS is
installed it replaces nothing.
"""

import importlib
import sys
import types

import pytest

ROS_MODULES = (
    "ament_index_python.packages", "launch.actions", "launch.event_handlers",
    "launch.substitutions", "launch_ros.actions", "rclpy.callback_groups", "rclpy.executors",
    "rclpy.node", "rclpy.qos", "controller_manager_msgs.srv", "diagnostic_msgs.msg",
    "geometry_msgs.msg", "lifecycle_msgs.msg", "rcl_interfaces.srv", "sensor_msgs.msg",
    "std_msgs.msg", "std_srvs.srv",
)


class Stub(Exception):
    """Any class of a missing module; an Exception so `except` clauses accept it."""

    def __init__(self, *args, **kwargs):
        super().__init__(*args)
        self.kwargs = kwargs


class StubModule(types.ModuleType):
    def __getattr__(self, name):
        if name.startswith("__"):
            raise AttributeError(name)
        stub = type(name, (Stub,), {})
        setattr(self, name, stub)
        return stub


@pytest.fixture
def ros_stubs(monkeypatch):
    for dotted in ROS_MODULES:
        try:
            importlib.import_module(dotted)
            continue
        except ImportError:
            pass
        parts = dotted.split(".")
        for depth in range(1, len(parts) + 1):
            name = ".".join(parts[:depth])
            # A namespace package (no __file__) such as this package's launch/
            # directory on sys.path is not the ROS module of that name.
            if getattr(sys.modules.get(name), "__file__", None) is None and not isinstance(
                    sys.modules.get(name), StubModule):
                monkeypatch.setitem(sys.modules, name, StubModule(name))
            if depth > 1:
                monkeypatch.setattr(sys.modules[".".join(parts[:depth - 1])], parts[depth - 1],
                                    sys.modules[name], raising=False)
