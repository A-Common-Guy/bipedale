from __future__ import annotations

import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from operator_input import Ps5Controller, Ps5Snapshot, snapshot_to_drive_command
from operator_input import ps5 as ps5_module


class FakeAbsInfo:
    def __init__(self, minimum: int, maximum: int) -> None:
        self.min = minimum
        self.max = maximum


class FakeDevice:
    """Mimics evdev's capabilities() contract, including the absinfo shape."""

    def __init__(self, path: str, name: str, axes: list[int], keys: list[int]) -> None:
        self.path = path
        self.name = name
        self.closed = False
        self._axes = axes
        self._keys = keys

    def capabilities(self, absinfo: bool = True):
        if not self._axes:
            return {1: list(self._keys)}
        axes = (
            [(code, FakeAbsInfo(0, 255)) for code in self._axes]
            if absinfo
            else list(self._axes)
        )
        return {1: list(self._keys), 3: axes}

    def close(self) -> None:
        self.closed = True


class Ps5DiscoveryTests(unittest.TestCase):
    """The pad reports (code, AbsInfo) pairs by default, so codes need absinfo=False."""

    PAD_AXES = [0, 1, 2, 3, 4, 5, 16, 17]
    PAD_KEYS = [304, 305, 307, 308]

    def setUp(self):
        if ps5_module.ecodes is None:
            self.skipTest("evdev is not installed")
        self.devices: dict[str, FakeDevice] = {}
        self._patch(list(self.devices))

    def _patch(self, paths: list[str]):
        originals = (ps5_module.list_devices, ps5_module.InputDevice)
        ps5_module.list_devices = lambda: paths
        ps5_module.InputDevice = lambda path: self.devices[path]
        self.addCleanup(
            lambda: setattr(ps5_module, "list_devices", originals[0])
        )
        self.addCleanup(
            lambda: setattr(ps5_module, "InputDevice", originals[1])
        )

    def _register(self, path: str, name: str, axes: list[int], keys: list[int]):
        self.devices[path] = FakeDevice(path, name, axes, keys)
        self._patch(sorted(self.devices))

    def test_connected_dualsense_is_discovered(self):
        self._register(
            "/dev/input/event6",
            "DualSense Wireless Controller",
            self.PAD_AXES,
            self.PAD_KEYS,
        )
        self.assertEqual(
            Ps5Controller.discover(),
            [("/dev/input/event6", "DualSense Wireless Controller")],
        )

    def test_motion_and_touchpad_subdevices_are_not_offered(self):
        self._register(
            "/dev/input/event6",
            "DualSense Wireless Controller",
            self.PAD_AXES,
            self.PAD_KEYS,
        )
        self._register(
            "/dev/input/event7",
            "DualSense Wireless Controller Motion Sensors",
            [0, 1, 2, 3, 4, 5],
            [],
        )
        self._register(
            "/dev/input/event8",
            "DualSense Wireless Controller Touchpad",
            [0, 1],
            [272, 330],
        )
        self.assertEqual(
            [path for path, _ in Ps5Controller.discover()], ["/dev/input/event6"]
        )

    def test_unrelated_devices_are_ignored_and_every_device_is_closed(self):
        self._register("/dev/input/event3", "AT Translated Set 2 keyboard", [], [30])
        self._register(
            "/dev/input/event6",
            "DualSense Wireless Controller",
            self.PAD_AXES,
            self.PAD_KEYS,
        )
        self.assertEqual(
            [path for path, _ in Ps5Controller.discover()], ["/dev/input/event6"]
        )
        self.assertTrue(all(device.closed for device in self.devices.values()))

    def test_unreadable_device_is_skipped_rather_than_raising(self):
        self._register(
            "/dev/input/event6",
            "DualSense Wireless Controller",
            self.PAD_AXES,
            self.PAD_KEYS,
        )

        def deny(path: str):
            raise PermissionError(13, "Permission denied")

        ps5_module.InputDevice = deny
        self.assertEqual(Ps5Controller.discover(), [])


class Ps5InputTests(unittest.TestCase):
    def test_disconnected_snapshot_maps_to_stop(self):
        command = snapshot_to_drive_command(
            Ps5Snapshot(left_y=1.0, right_x=1.0),
            max_forward_velocity_m_s=0.5,
            max_yaw_rate_rad_s=1.5,
        )
        self.assertEqual(command.forward_velocity_m_s, 0.0)
        self.assertEqual(command.yaw_rate_rad_s, 0.0)

    def test_sticks_map_to_bounded_si_drive_command(self):
        command = snapshot_to_drive_command(
            Ps5Snapshot(connected=True, left_y=1.0, right_x=-1.0),
            max_forward_velocity_m_s=0.5,
            max_yaw_rate_rad_s=1.5,
        )
        self.assertEqual(command.forward_velocity_m_s, 0.5)
        self.assertEqual(command.yaw_rate_rad_s, -1.5)

    def test_deadzone_is_removed_and_remaining_range_is_rescaled(self):
        command = snapshot_to_drive_command(
            Ps5Snapshot(connected=True, left_y=0.08, right_x=0.54),
            max_forward_velocity_m_s=1.0,
            max_yaw_rate_rad_s=2.0,
            deadzone=0.08,
        )
        self.assertEqual(command.forward_velocity_m_s, 0.0)
        self.assertAlmostEqual(command.yaw_rate_rad_s, 1.0)

    def test_invalid_mapping_configuration_is_rejected(self):
        with self.assertRaises(ValueError):
            snapshot_to_drive_command(
                Ps5Snapshot(connected=True),
                max_forward_velocity_m_s=1.0,
                max_yaw_rate_rad_s=1.0,
                deadzone=1.0,
            )


if __name__ == "__main__":
    unittest.main()
