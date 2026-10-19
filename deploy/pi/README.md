# Running Sandro on the Raspberry Pi

The Pi runs the real-hardware stack (`hardware:=real`) in an arm64 Jazzy container;
the development machine edits code, runs the simulator and RViz. Both are on the same
DDS network (host networking), so `ros2 topic echo` and RViz work from the laptop.

```bash
deploy/pi/pi setup     # once: docker, can0 at boot, performance governor, robot image
deploy/pi/pi launch    # every time: sync + incremental build + launch (hardware:=real)
```

Also available as `make pi-launch`, `make pi-build`, `make pi-shell`, `make pi-can`.

## Why it is fast

| Change                          | Cost on the next `launch`                         |
|---------------------------------|---------------------------------------------------|
| Python, YAML, URDF, launch file | rsync only (~1 s), live through `--symlink-install` |
| One C++ file                    | recompiles that file and relinks (ccache, Release) |
| Header used everywhere          | recompiles its dependents                          |
| Dockerfile / system packages    | `pi setup` again (minutes, rare)                   |

Sources are bind-mounted into the container; `build/`, `install/` and the ccache live on
the Pi, so nothing is rebuilt from scratch. The first `pi build` compiles everything once.

`launch` defaults to `hardware:=real imu_transport:=spi imu_spi_device:=/dev/spidev6.0 rviz:=false`
plus the per-robot `config/sandro_real_overrides.yaml`, and the controller defaults to
`whole_body_controller`; extra arguments are appended, e.g.
`deploy/pi/pi launch arm_hardware:=true` or `deploy/pi/pi launch controller:=lqr_balancer`.

## Gamepad (PS5 DualSense over Bluetooth)

Paired and trusted once; afterwards pressing PS reconnects it and `/dev/input/js0` appears
inside the container (it bind-mounts `/dev`). To pair a new pad, hold PS + Create until the
light bar flashes fast, then on the Pi (BlueZ forgets a scanned device once the scan stops,
so pair while scanning):

```bash
bluetoothctl --timeout 60 scan on &   # wait until the pad shows up in `bluetoothctl devices`
bluetoothctl pair <MAC> && bluetoothctl trust <MAC> && bluetoothctl connect <MAC>
```

If `bluetoothctl show` reports `PowerState: off-blocked`, the radio is rfkill soft-blocked:
`sudo rfkill unblock bluetooth && bluetoothctl power on` (the state persists across reboots).
The button and axis indices in `config/teleop.yaml` are SDL's gamepad layout, which is what
`joy` reports for the DualSense here: L1 = button 9 is the deadman, L2 + R2 (axes 4 and 5)
the software E-stop.
As always, `arm_hardware:=true` is a deliberate extra step; see
`ros2_ws/src/sandro_bringup/docs/HARDWARE_COMMISSIONING.md`.

## Pi hardware wiring (Raspberry Pi 4)

| Device   | Bus          | Pins                                                    |
|----------|--------------|---------------------------------------------------------|
| BNO085   | SPI6, mode 3 | CS GPIO18, MISO 19, MOSI 20, SCLK 21, INT 25, RST 24; P0 and P1 tied to 3V3 |
| MCP2515  | SPI0, mode 0 | CS GPIO8, MISO 9, MOSI 10, SCLK 11, INT 23; 16 MHz crystal |

* The Pi 4's *auxiliary* SPI1 cannot do SPI mode 3, which the BNO085 requires. SPI6 uses
  the same pins (GPIO18-21) and is a full controller.
* With P0 (the WAKE pin) tied high the BNO085 only accepts a host write while INT is
  asserted. The driver therefore resets the chip at activation and writes its
  configuration before it starts draining packets (`bno085_sensor.cpp`).
* An 8 MHz MCP2515 crystal cannot reach 1 Mbit/s with the kernel driver (it needs
  16 MHz, which is why the module carries one).

`/boot/firmware/config.txt` (written by `pi setup`):

```
dtparam=spi=on
dtoverlay=spi6-1cs
dtoverlay=mcp2515-can0,oscillator=16000000,interrupt=23
```

## Tools

* `deploy/pi/tools/bno085_spi_probe.py` is a dependency-light stand-alone BNO085 test
  (run it on the Pi, outside ROS) for wiring and bring-up problems.
* `deploy/pi/pi status` shows container, CAN, SPI and CPU state;
  `deploy/pi/pi can` runs `candump`.

## Latency notes

* The CPU governor is pinned to `performance` (`sandro-perf.service`); `ondemand` adds
  frequency-scaling jitter to a 300 Hz loop.
* The container is privileged with `rtprio=99`, so the controller manager can request
  `SCHED_FIFO`. Check `deploy/pi/pi status` for throttling (`get_throttled` should be 0x0).
