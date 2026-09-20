# CyberBeast Driver User Manual

> **Unofficial English translation** of the vendor document “守护兽驱动用户手册” (CyberBeast Driver User Manual).
> Source: https://cyberbeast.feishu.cn/docx/N3SMd4QyRobzHkx3wP3cT1qXnpf — retrieved and translated 2026-08-12.
> Product: SteadyWin/CyberBeast GIM6010 series motor drive. “Motor Genie” (电机精灵) is the vendor’s Windows GUI tool.
> Large binary attachments (videos, installers, zip archives) were not copied; see the source document for those.


This drive is an extension of the ODrive platform. It retains the original control methods and, through a mapping mechanism, supports compatible control using ODrive commands. This manual explains how to control the motor using the `odrivetool` command, how to operate using the graphical interface "Motor Genie (电机精灵)", and how to use ODrive commands under different protocols.

Please make good use of the document search function (Ctrl+F) when using this manual.

## 1 Preparation

Here we briefly introduce how to connect the motor using Motor Genie and odrivetool, and how to install the software.

### 1.1 Wiring Preparation

![image 800x600](assets_user_manual/img_P1OWb06YgoXoPbxu1f1cXMSLn9e.jpg)

- Type-C USB cable: **The USB cable cannot power the motor drive board**; it is used only for data transmission. Note that some models do not support Type-C and instead use a custom 3-pin data cable — please contact after-sales support for connection instructions.
- XT30 2+2 power cable: We recommend powering the motor with a regulated power supply or a battery. The supply current **must be at least 5A**, and the voltage **must be no lower than 15V and no higher than 60V**. Note: if using a power supply of 24V or below, remember to change the lower voltage limit to 10V or 8V in Motor Genie or odrivetool.

```python
odrv0.config.dc_bus_undervoltage_trip_level = 10
```

![image 311x453](assets_user_manual/img_RihRb7tCuokk8ZxJyFKcxL7fnxf.png)

### 1.2 USB Driver Installation

📎 **Attachment (not downloaded, see original doc):** zadig-2.9.exe (5.1 MB)

Go to the website [ https://zadig.akeo.ie ](https://zadig.akeo.ie) and download the USB driver tool Zadig. Connect the drive to your computer using a Type-C data cable. **Please connect the drive to power beforehand — USB cannot power the drive.** Open Zadig, click "Options", then click "" select the motor via the dropdown menu "CyberBeast Motor Driver Device (Interface 2)":

![image 572x252](assets_user_manual/img_I1X9bOGLkotUbdxKESxcJH3QnQb.png)

![image 415x184](assets_user_manual/img_DsMMbwHoeoV2IzxbpUocTXZYnxg.png)

Use the up and down arrow keys to select between the different USB drivers. For this interface, select the "libusb" driver version, then click "Install Driver" or "Replace Driver" to install the driver for this interface:

![image 415x182](assets_user_manual/img_TJaIbVtn8oIG1qxbZyzck3oKnB6.png)

### 1.3 Software Installation

#### 1.3.1 Motor Genie

1. Install Motor Genie
Click to download the latest version: [Motor Genie installer](https://bl.cyberbeast.cn/actuator/motorwizard.exe)

Download the Motor Genie installer and install it in the following order:

![image 598x464](assets_user_manual/img_PTJ6bCifGo77g8xQGR1crZ81nkf.png)

![image 598x464](assets_user_manual/img_WWLUbgMpgoP68exowhBcqchPnJR.png)

![image 598x464](assets_user_manual/img_VitRbx9Qzoao9fxdbpWcx3MMnsh.png)

#### 1.3.2 odrivetool

##### **Windows**

1. Install python3
📎 **Attachment (not downloaded, see original doc):** 安装python正式.mp4 (Installing Python – official version.mp4) (16.5 MB)

Go to the official Python website [ https://www.python.org ](https://www.python.org), download the latest Python installer, and install it by following the prompts. Do not download Python from third-party websites or from the Microsoft Store. **Install Python with administrator privileges.**

![image 651x401](assets_user_manual/img_GhY2bz13OoDGz8xqYCHcx86Snid.png)

Attached here is a slightly older Python installer, for customers with slow download speeds:

📎 **Attachment (not downloaded, see original doc):** python-3.12.9-amd64.exe (25.7 MB)

2. Install odrivetool and its related libraries
Open a terminal, similar to the page shown below, then run the following commands:

![image 1115x628](assets_user_manual/img_KcOZbdePvo1BxHxiqMocEk86nge.png)

```python
pip install numpy matplotlib odrive==0.6.9.post0 -i https://pypi.tuna.tsinghua.edu.cn/simple
If your drive board version is V8.xx, run the commands below instead, and connect using mtool
pip install matplotlib mwdrive numpy -i https://mirrors.aliyun.com/pypi/simple/
```

This command has already been set up to install from the Tsinghua mirror, so you generally don't need to worry about installation failures caused by slow network speed. numpy and matplotlib are optional and do not need to be installed. If an error occurs partway through, try again. If the error persists after repeated attempts, restart your computer and try again.

Once you have finished the installation steps on Windows, [click here](https://bcnyljrhe70u.feishu.cn/docx/N3SMd4QyRobzHkx3wP3cT1qXnpf#I8K4drWFyooajvxidkycF71TnVd) to jump to the next step.

> 💡 If you see: "'python3' is not recognized as an internal or external command, operable program or batch file", it may be because Python has not been added to your PATH environment variable. Python comes with a small script that can fix this. Navigate to the Python installation folder (typically `C:\Program Files\Python<version>\` or `C:\Users<your username>\AppData\Local\Programs\Python\Python<version>\`), then double-click to run the "win_add2path.py" script.
>

##### Linux

3. Install python3
If `python3 --version` already returns something like `Python 3.8.0` or newer, you can skip this step. The installation command depends on which Linux distribution you are using. For example, on Ubuntu:

```python
sudo apt install python3 python3-pip
```

If this command doesn't work, check the [official Python download website](https://www.python.org/downloads/).

4. Install odrivetool and its related libraries
numpy and matplotlib are optional and do not need to be installed

```python
pip3 install numpy matplotlib odrive==0.6.9.post0 -i https://pypi.tuna.tsinghua.edu.cn/simple
```

5. Install the USB driver

```python
sudo apt install libusb-1.0-0
```

Once you have finished the installation steps on Linux, [click here](https://bcnyljrhe70u.feishu.cn/docx/N3SMd4QyRobzHkx3wP3cT1qXnpf#I8K4drWFyooajvxidkycF71TnVd) to jump to the next step.

##### **WSL (Windows Subsystem for Linux)**

The first three steps are similar to Linux; please refer to the previous section.

- Connect the drive to WSL
Plug the Type-C data cable into Windows. By default, Windows loads the driver for this USB port, but WSL does not. You need to attach this USB port to WSL — please follow the Microsoft documentation ([https://learn.microsoft.com/zh-cn/windows/wsl/connect-usb](https://learn.microsoft.com/zh-cn/windows/wsl/connect-usb)) to do so.

Once you have finished the installation steps on WSL, proceed to the next step: Launching odrivetool.

##### Launching odrivetool

Once you have completed the installation for your system as described above, open a terminal shell (Windows PowerShell or Linux Terminal) and run `odrivetool` (type odrivetool and press Enter). The image below shows a successful connection on Windows:

![image 1112x627](assets_user_manual/img_P9owbPPoQoh2gqxj10DcqaLpned.png)

The command shown in the image above is used to display the main board's supply voltage.

The tool you are looking at is a fully functional Python command prompt, so you can enter any valid Python code.

### 1.4 Frequently Asked Questions (Updating)

Q: Motor Genie cannot connect to the drive board

    A: Install the USB driver following section 1.2 of the user manual

    Q: A motor-related error occurs during calibration

    A: Check whether the three-phase wires have a cold solder joint or are too thin, and whether the power supply is limiting current output

    Q: An encoder-related error occurs during calibration

    A: Manually turn the motor by hand, add the rotor shaft encoder curve in Motor Genie, and observe whether the curve trend matches the rotation. If the encoder curve follows the rotation trend, the encoder's operating environment is poor — adjust its mounting position. If the curve doesn't move at all, check whether the encoder is damaged

    Q: All parameters were written but cannot be saved

    A: After writing the parameters, press ENTER; it's fine once the screen shows a successful write

    Q: The motor behaves abnormally after an upgrade

    A: Erase all parameters and recalibrate the motor

    Q: An undervoltage error occurs and the drive board restarts when the motor moves under load

    A: Check whether the power supply is limiting current output; a limit of around 10A is generally fine. The exact value depends on the motor's parameters

    Q: Overvoltage occurs when the motor moves under load

    A: Check the bus voltage fluctuation. Within the range allowed by the components, raise the upper bus voltage limit; try smoother motion profiles, such as a torque ramp, velocity ramp, or position trapezoidal trajectory.
## 2 Using Motor Genie (电机精灵)

Before proceeding with this section, first complete the USB driver and Motor Genie installation following [ 1.2 ](https://bcnyljrhe70u.feishu.cn/docx/N3SMd4QyRobzHkx3wP3cT1qXnpf#LVn3dl3tEoqyVdxLUxRchQm8nid) and [ 1.3.1 ](https://bcnyljrhe70u.feishu.cn/docx/N3SMd4QyRobzHkx3wP3cT1qXnpf#YsUydUkFpoMOwOx7uinc6txYnKf).

### 2.1 Interface Overview

![image 540x901](assets_user_manual/img_U616bhiVVoWL3SxkNgKcJDA9nVf.png)

![image 540x901](assets_user_manual/img_XcHYbjQwYodiZUxLVSnc4tgXnmc.png)

1. Real-time waveform monitoring
The upper part of the interface displays several real-time data waveform charts, including:

- **Three-phase current waveform** (IA, IB, IC): shows the actual current in the motor's three-phase windings.
- **DQ-axis current** (IQ, ID and their reference values IqRef, IdRef): compares the actual current under current loop control with the target current.
- **Speed curve** (SPD, SPDref): compares real-time speed with the target speed, useful for observing velocity loop performance.
- **Position curve** (EA, MA): shows the actual position versus the target position, useful for tuning the position loop.
2. Central dashboard
- **Motor speed and torque display**: the central dashboard shows the current speed (RPM) and the motor output torque (Nm).
- **Angle indicator**: shows the angle of the motor's current position.
- **One-click start button**: click the central "Start Motor" button to quickly start or stop the motor.
- **Motor running status**: the left side shows the motor's current operating mode (e.g. "Idle" state).
- **Power monitoring**: the right side shows the current motor power and bus voltage.
3. Parameter configuration area
The bottom of the interface provides several buttons, each leading to a different parameter configuration page:

- **Motor Parameters**: set and view basic motor-related parameters.
- **Drive Parameters**: configure the motor drive's control parameters, such as current loop / velocity loop PI parameters.
- **ODrive Parameters**: internal parameter management and debugging for the ODrive architecture.
- **Debug Log**: view detailed log output during debugging, to help pinpoint issues.
Below, I will explain in detail what each of the four middle icons opens up to.

#### 2.1.1 Share, Sync, and Upgrade

    ![image 213x75](assets_user_manual/img_ECZrbvjyBoLfioxB7kdcOL85nZd.png)

    When we open Motor Parameters, we can see three buttons:

    - Share

    ![image 501x465](assets_user_manual/img_HPqdbxDQQotksixg8Q8cg2AWnAe.png)

    After clicking Share, we can import the current motor's parameters or import another motor's parameter json file.

    - Upgrade
    Before upgrading, make sure you have downloaded and installed the DFU driver corresponding to the chip.

        National (国民) DFU driver: [CyberBeast Driver User Manual](CyberBeast_Driver_User_Manual_EN.md)

    ![image 540x900](assets_user_manual/img_EFkqbsBMno2jJfxk7sVciofRnid.png)

    After clicking Upgrade, the motor enters upgrade mode (equivalent to running odrv0.enter_dfu_mode（）in odrivetool). Motor Genie will disconnect from the motor, allowing the motor firmware to be flashed (see [Firmware Upgrade](https://bcnyljrhe70u.feishu.cn/docx/N3SMd4QyRobzHkx3wP3cT1qXnpf#GNdcdWiIsozXIbxLsPWcH0annyb) for details).

    There are three upgrade modes: cloud upgrade, local upgrade, and MCU vendor software upgrade. When upgrading across many versions, it is recommended to check "Erase All Configuration."

    ![image 618x483](assets_user_manual/img_LhSfb4NuJokXoMxH4tDcG8lCnxf.png)

    - Sync
    After each parameter change, you must click Sync while the motor is in the idle state to save the written parameters, so they are not lost on power-off. After syncing, the motor will restart, and you then need to wait a few seconds for the motor to reconnect (`odrv0.save_configuration()`).

    ![image 540x900](assets_user_manual/img_EWUHbfaAfoxdbjxaag8cO2CNnkz.png)

    Press and hold the Sync button to erase all configured parameters and restore the default parameters (`odrv0.erase_configuration()`).

    ![image 458x184](assets_user_manual/img_LbBWbIwLOoMJVYxdwbScDnNGnlh.png)

#### 2.1.2 Motor Parameters

    Clicking the Motor Parameters icon shown above, we can see the following parameters:

> 💡 - After changing any parameters on this page, you must click Sync while the motor is in the idle state to save the parameters.
> - The line below the parameter configuration shows the hardware version / firmware version number. The hardware version refers to the version number of your own drive board, and the firmware version refers to the version number of the drive firmware currently in use (a firmware upgrade may change the firmware version number). In the figure, the hardware version is 3.8.3 and the firmware version is 0.5.15.

    ![image 540x901](assets_user_manual/img_LK7Cb3h94ojZTTxkCrYcazgwnEg.png)

    The description of each parameter and the corresponding odrivetool commands are as follows:

    ***Basic Parameters***

    | **Parameter Name** | **Description** | **Example Value** | `odrivetool` **Field** |
    |---|---|---|---|
    | Pole pairs | Number of magnetic pole pairs of the motor (poles / 2) | 10 | `odrv0.axis0.motor.config.pole_pairs` |
    | Gear ratio | Gear ratio between the motor output shaft and the encoder | 9.67 | `odrv0.axis0.motor.config.gear_ratio` |
    | Bus voltage range | Acceptable bus input voltage range | 15 V ~ 64 V | `odrv0.config.dc_bus_undervoltage_trip_level`<br>`odrv0.config.dc_bus_overvoltage_trip_level` |
    | Bus current | Maximum supply current allowed by the power source | 20.0 A | `odrv0.config.dc_max_positive_current` |
    | Reverse bus current | Maximum regenerative (charge-back) current allowed by the power source (negative value) | -50.0 A | `odrv0.config.dc_max_negative_current` |
    | Motor current | Maximum current allowed for the motor | 60.0 A | `odrv0.axis0.motor.config.current_lim` |
    | Motor speed | Maximum motor speed (RPM) | 186.14 RPM (output shaft) | `odrv0.axis0.controller.config.vel_limit` (the unit in odrivetool is rotor-side turns/s) |
    | Calibration current | Current injected during motor calibration | 2.0 A | `odrv0.axis0.motor.config.calibration_current` |
    | Motor calibration | One-click motor calibration button | - | `odrv0.axis0.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE` |
    | Encoder bandwidth | Encoder filter bandwidth setting | 500 | `odrv0.axis0.encoder.config.bandwidth` |
    | Drive over-temperature protection | Drive over-temperature protection threshold | 25 ~ 80°C | `odrv0.axis0.motor.fet_thermistor.config.enabled = True`<br>`odrv0.axis0.motor.fet_thermistor.config.temp_limit_lower`<br>`odrv0.axis0.motor.fet_thermistor.config.temp_limit_upper` |
    | Motor over-temperature protection | Motor over-temperature protection threshold | 25 ~ 90°C | `odrv0.axis0.motor.motor_thermistor.config.enabled = True`<br>`odrv0.axis0.motor.motor_thermistor.config.temp_limit_lower`<br>`odrv0.axis0.motor.motor_thermistor.config.temp_limit_upper` |
    | Phase resistance | Motor phase resistance | 0.215 Ω | `odrv0.axis0.motor.config.phase_resistance` |
    | Phase inductance | Motor phase inductance | 0.000090 H | `odrv0.axis0.motor.config.phase_inductance` |
    | Torque constant | Current-to-torque conversion coefficient (rotor side) | 0.09700 Nm/A | `odrv0.axis0.motor.config.torque_constant` |

    Communication parameters and how to configure them will be explained in the usage section of each protocol (5.1, 5.2).

#### 2.1.3 Drive Parameters

    ![image 540x901](assets_user_manual/img_GH60bnPHYoOjvwx9584co3FMn4d.png)

    The parameter descriptions and corresponding odrivetool commands are as follows:

    ***Basic Parameters (Control Mode Configuration)***

    | Parameter Name | Description | Example Value | `odrivetool` Field |
    |---|---|---|---|
    | Control mode | Motor control type: position/velocity/current, etc. | Position mode | `odrv0.axis0.controller.config.control_mode`   |
    | Input mode | Input command type: position filter, step/direction, analog, etc. | Position filter | `odrv0.axis0.controller.config.input_mode` |
    | Moment of inertia | Inertia of the motor's rotating part (needed for torque mode) | 0.00 | `odrv0.axis0.controller.config.inertia` |
    | Velocity limit | Whether the speed limit is enabled | Enabled | `odrv0.axis0.controller.config.enable_vel_limit` |
    | Torque mode velocity limit | Whether to limit speed when torque control is enabled | Enabled | `odrv0.axis0.controller.config.enable_torque_mode_vel_limit` |
    | Maximum speed | Maximum operating speed | 186.14 RPM<br>(output shaft) | `odrv0.axis0.controller.config.vel_limit` (the unit in odrivetool is rotor-side turns/s) |

    ***Current Loop***

    | Parameter Name | Description | Example Value | `odrivetool` Field |
    |---|---|---|---|
    | Control bandwidth | Bandwidth of the current loop; higher is faster but may cause oscillation | 1000.0 | `odrv0.axis0.motor.config.current_control_bandwidth` |
    | Q-axis current limit | Maximum Q-axis current of the motor | 60.0 A | `odrv0.axis0.motor.config.current_lim` |
    | Ramp rate | Limit on the rate of current change | 1.000 N·m/s | `odrv0.axis0.controller.config.torque_ramp_rate` |

    ***Velocity Loop***

    | Parameter Name | Description | Example Value | `odrivetool` Field |
    |---|---|---|---|
    | KP | Proportional gain | `0.100` | `odrv0.axis0.controller.config.vel_gain` |
    | KI | Integral gain | `1.000` | `odrv0.axis0.controller.config.vel_integrator_gain` |
    | Ramp rate | **In velocity ramp mode**, the limit on the rate of speed change (acceleration) | `15.000 R/s²` | `odrv0.axis0.controller.config.vel_ramp_rate` |

    ***Position Loop***

    | Parameter Name | Description | Example Value | `odrivetool` Field |
    |---|---|---|---|
    | KP | Proportional gain (for position error) | `20.000` | `odrv0.axis0.controller.config.pos_gain` |
    | Filter bandwidth | Bandwidth for filtering out jitter in the position signal | `10.0` | `odrv0.axis0.controller.config.input_filter_bandwidth` |
    | Trapezoidal velocity limit | **In position trapezoidal trajectory mode**, the maximum target velocity (position input) | `100.0 R/s` | `odrv0.axis0.trap_traj.config.vel_limit` |
    | Trapezoidal acceleration limit | **In position trapezoidal trajectory mode**, the maximum target acceleration | `20.0 R/s²` | `odrv0.axis0.trap_traj.config.accel_limit` |
    | Trapezoidal deceleration limit | **In position trapezoidal trajectory mode**, the maximum target deceleration | `20.0 R/s²` | `odrv0.axis0.trap_traj.config.decel_limit` |

#### 2.1.4 ODrive Parameters

    Motor Genie shows only some of the most basic configurable motor parameters; many other parameters available in odrivetool are not shown in Motor Genie. In the ODrive Parameters function, you can find and enter odrivetool commands directly — any command you can enter in odrivetool can be found here.

    ![image 323x310](assets_user_manual/img_Do8sbzJyqodQamxvaAwc2l3Knlg.png)

    For example, here we can configure the motor to enter closed-loop operating mode on power-up (as shown above).

> 💡 Note: after configuring parameters on this page, if you need the parameters to persist through power loss, you must be in the **idle state** and click Motor Parameters —> Sync to save the parameters. In short, **enter the value, press Enter, then click Sync**.
>

### 2.2 Control in Four Modes

    Text description to be updated; for details on the different modes, see [ ](https://bcnyljrhe70u.feishu.cn/docx/N3SMd4QyRobzHkx3wP3cT1qXnpf#H1aodvyGAokuGWxQeK9cnBOAndg)[3](https://bcnyljrhe70u.feishu.cn/docx/N3SMd4QyRobzHkx3wP3cT1qXnpf#H1aodvyGAokuGWxQeK9cnBOAndg)[.3](https://bcnyljrhe70u.feishu.cn/docx/N3SMd4QyRobzHkx3wP3cT1qXnpf#H1aodvyGAokuGWxQeK9cnBOAndg); for mode tuning, see [ 4.2](https://bcnyljrhe70u.feishu.cn/docx/N3SMd4QyRobzHkx3wP3cT1qXnpf#QZYwd8VetoxIeyxvGZ4ce08Cn2c)

#### 2.2.1 MIT Mode

> 💡 Before using this mode, make sure the motor's torque constant (`odrv0.axis0.motor.config.torque_constant`) is set correctly; otherwise, the output torque may not reach the expected value. You can contact the manufacturer to obtain the correct torque constant.
>

        With the motor already calibrated, switch the dashboard to MIT mode and click Start Motor. Then right-click on the dashboard to see the parameters that can be entered (shown below, from left to right). Next, we will explain how to set the parameters for each usage mode.

        ![image 540x900](assets_user_manual/img_BZ2Obv4lqoqk8vx4LuZc2bdVnJd.png)

        ![image 540x900](assets_user_manual/img_YkJebAC3soS7JGx294fcTUqunEh.png)

        ![image 540x900](assets_user_manual/img_I4t4bNkSSopqACxQP7ZcOqm0nHe.png)

        - Position control mode
        In position control mode, the motor output shaft will move to and hold the set target angle. For example, to hold the motor output shaft steady at 60°, set the following motion parameters: target position 60°, KP 50, KD 1, and set the remaining parameters (velocity feedforward and torque feedforward) to 0:

        ![image 540x900](assets_user_manual/img_UPDXb9chQo5AcwxdpzUcJeoBnEe.png)

        - Velocity control mode
        In velocity control mode, the goal is for the motor output shaft to run at the given desired speed. For example, to run the motor output shaft at 100 RPM, set the following motion parameters: velocity feedforward 100 RPM, KD 1.5, and set the remaining parameters (target position, velocity feedforward, torque feedforward, and KP) to 0:

        ![image 540x900](assets_user_manual/img_FHQwbvVHRo7wObxNij9crEIGnng.png)

        - Torque control mode
        In torque control mode, the goal is for the motor output shaft to run at the given desired torque. For example, to have the motor output 1 Nm of torque, set the following motion parameters: torque feedforward set to 1 Nm, and set the remaining parameters (target position, velocity feedforward, torque feedforward, KP, and KD) to 0:

        ![image 540x900](assets_user_manual/img_FKZ8biwfdop5v3xzCVScMrfzn4c.png)

        - Damping mode
        Damping mode is commonly used to implement motor deceleration, braking, or resistance control. In this mode, the motor outputs an appropriate torque to apply resistance or braking force against external motion, achieving deceleration or suppression of movement. Users can flexibly control the magnitude of the damping or braking force by adjusting the output parameters, to suit different deceleration or braking needs. For example, set the damping coefficient KD to a non-zero value (e.g. KD=2), and set all remaining parameters to 0.

        ![image 540x900](assets_user_manual/img_V7mqbiMphojCouxcDHCcqrHfnwc.png)

        After the parameters are sent, the motor will not turn on its own, but when an external force drives the motor to rotate, the motor will generate an impedance torque opposite to the direction of rotation. The magnitude of this torque is proportional to the speed: the higher the speed, the greater the impedance torque.

        - Zero-torque mode
        Zero-torque mode is designed to keep the motor's output torque at or near zero. In this mode, the motor does not stop completely; instead, it actively outputs torque to counteract its own friction torque, achieving an approximate "zero resistance" effect. To set this up, set all motion parameters to 0. After the parameters are sent, the motor will not turn on its own, but if you try to manually rotate the output shaft, you will feel that its rotational resistance is significantly lower than the mechanical resistance of the motor when unpowered.

        ![image 540x900](assets_user_manual/img_Zu81bNlXtoP0SUx6qdAcBdsnn0b.png)

        - Hybrid force-position control mode
        When applied to certain robots, the upper-level controller sends the joint motor a desired angle, desired velocity, and torque feedforward at the same time. This is hybrid force-position control. For example, set the target position to 180°, velocity to 0, feedforward torque to 0.5 Nm, KP to 15, and KD to 2. This is hybrid force-position control.

        ![image 540x900](assets_user_manual/img_Q9qRbFtE1ovAgLxh31PclGCHn9c.png)

### 2.3 Viewing Error Codes

    When the motor triggers an alarm, it immediately stops running and enters the idle state. As shown below, right-click on the red error to get the error code.

    ![image 540x901](assets_user_manual/img_GCxab9E7Coqdk6xB9n8ceMPDnBe.png)

    ![image 540x901](assets_user_manual/img_CZDHbUHPpo2Ge0xUTSxcT75bnMb.png)

    System Fault: system error, Axis Fault: drive error, Motor Fault: motor error, Encoder Fault: encoder error, Controller Fault: controller error

    You can refer to the [Protocol Manual Error Codes](https://cyberbeast.feishu.cn/docx/BPnQd8reEotLWVxqHFNc9qYZnKh#KKr4dVPH3oQ9r5x5KxocJLmQn4b) to look up the specific error corresponding to each error code. Multiple errors may occur at the same time, among which there will be a root-cause error.

    Taking the figure above as an example, converted to hexadecimal:

    System Fault: 0x00000002

    Axis Fault: 0x00000040

    Motor Fault: 0x01000000

    Encoder Fault: 0

    Controller Fault: 0

    ![image 589x152](assets_user_manual/img_OAtdbL6VuoCvZpxs8ICclGm6nOg.png)

    ![image 580x204](assets_user_manual/img_QS14bvEN2oXWHtxIrbHcSHXRnuc.png)

    ![image 565x313](assets_user_manual/img_QVVobfAdJoTs79xlnagc3FeXndg.png)
## 3 Using Odrivetool

> 💡 Usage tip
>
> - Use the `TAB` key often — it brings up command suggestions, similar to command completion on Linux; you can use the `↑` `↓` `←` `→` keys to select a command;
> - The `↑` `↓` keys display command history;
> - While typing a command, similar historical commands will be suggested; pressing `ctrl + →` will autocomplete it directly

### 3.1 Basic odrivetool Commands

This section covers some commonly used odrivetool commands. If you are already familiar with odrivetool, you can skip this section directly, [click to skip to the next section](https://bcnyljrhe70u.feishu.cn/docx/N3SMd4QyRobzHkx3wP3cT1qXnpf#KySSdxV26oqhyfx5wbWcjGMjnfh).

We will use `<odrv>` as a placeholder for any ODrive object. Each ODrive controller is an ODrive object. In `odrivetool`, it can be accessed as `odrv0`, `odrv1`, and so on. The properties of these ODrive objects are defined by the ODrive API and may change when the ODrive firmware is updated. In addition, we will use `<axis>` as a placeholder for any axis, which is a property of the ODrive object (for example `odrv0.axis0`). An axis represents the location where a motor is connected; our motor currently has only one axis, `axis0`.

This manual assumes the current motor is `odrv0.axis0` by default. If two or more motors are connected, change this accordingly based on your situation.

#### 3.1.1 Motor State Machine

    The current state of an axis is represented by `axis0.current_state`. Users can request a new state by assigning a new value to `axis0.requested_state`. The default state after startup is `AxisState.IDLE`. Descriptions of all states can be found [here](https://docs.odriverobotics.com/v/latest/fibre_types/com_odriverobotics_ODrive.html#ODrive.Axis.AxisState).

#### 3.1.2 Startup Sequence

    By default, ODrive does not perform any action on startup and immediately enters the idle state (IDLE). To change the startup behavior, set the desired startup action to `True` (remember to save the configuration afterward). ODrive will execute all enabled startup actions in the order shown below.

        - `odrv0.axis0.config.startup_motor_calibration = 1` performs three-phase identification on power-on
        - `odrv0.axis0.config.startup_encoder_offset_calibration = 1` performs encoder calibration on power-on
        - `odrv0.axis0.config.startup_closed_loop_control = 1` enters closed-loop control on power-on

#### 3.1.3 Saving and Erasing Configuration

    All variables belonging to a `[...].config` object can be saved to ODrive's non-volatile memory, so they **persist after power is removed**. The relevant commands are as follows:

        - `odrv0.save_configuration()`: saves the configuration to ODrive's persistent memory; this must be executed in the idle state.
        - `odrv0.erase_configuration()`: resets the configuration variables to factory defaults. This operation also reboots the device.

#### 3.1.4 **Viewing Device Error Information**

    When the motor triggers an alarm, it stops immediately and reports an error. You can query the error information with the following command

    ```python
    dump_errors(odrv0)
    ```

    After running this you will see the information shown below; here, the root cause error is undervoltage causing the motor fault.

    ![image 500x240](assets_user_manual/img_GGeebDlv1oXApbxiP67cIy82nnc.png)

    If `dump_errors(odrv0)` cannot find the specific error, we can also determine the exact fault by checking the error code (note that the values returned are all decimal). Use the returned error code to look up the exception code section of the [CyberBeast Driver Protocol Manual](CyberBeast_Driver_Protocol_Manual_EN.md)

    ```python
    odrv0.axis0.motor.error        #Motor fault error code
    odrv0.axis0.encoder.error      #Encoder fault error code
    odrv0.axis0.controller.error   #Controller fault error code
    odrv0.error                    #System fault error code
    ```

#### 3.1.5 Live Plotting Tool

    `start_liveplotter(properties[, layout, ...])`

    Start the live plotting tool (please make sure you have installed the matplotlib library first). For example, to plot the target position and current position:

    ```python
    start_liveplotter(lambda:[odrv0.axis0.encoder.pos_estimate,odrv0.axis0.controller.input_pos],["pos","pos_target"])
    ```

    ![image 640x480](assets_user_manual/img_CI9NbajNtoQGIpxcYVvcSPbUnth.png)

    Stop the currently running live plotting tool.

    ```python
    stop_liveplotter()
    ```

#### 3.1.6 Backing Up and Restoring Parameters

    Open a terminal, **do not enter odrivetool**, and run the following commands in the terminal window

    ![image 1115x628](assets_user_manual/img_GBhebrsi0o9oH7xpZqrcfsNxnNV.png)

    Parameter backup:

    ```text
    odrivetool backup-config "d:/test.json"
    ```

    Here, `d:/test.json` is the save path and file name, which the user can freely change.

    When restoring parameters, **make sure the motor's hardware version is the same**

    The command to restore parameters is:

    ```text
    odrivetool restore-config "d:/test.json"
    ```

#### 3.1.7 Entering Upgrade Mode

    To download firmware, execute the following command. For details on downloading firmware, see [Firmware Upgrade](https://bcnyljrhe70u.feishu.cn/docx/N3SMd4QyRobzHkx3wP3cT1qXnpf#GNdcdWiIsozXIbxLsPWcH0annyb)

    ```text
    odrv0.enter_dfu_mode()
    ```

### 3.2 Motor Calibration

Normally the motor is already calibrated before it leaves the factory. If you have performed a firmware upgrade, disassembled/reassembled the drive board, or erased the configuration, you need to recalibrate the motor.

[Click to skip to the next section.](https://bcnyljrhe70u.feishu.cn/docx/N3SMd4QyRobzHkx3wP3cT1qXnpf#H1aodvyGAokuGWxQeK9cnBOAndg)

The first time a user uses the motor, the motor and encoder need to be calibrated (normally this has already been done before leaving the factory). Before calibration, secure the motor firmly, or hold it tightly by hand, with the output shaft unloaded. The calibration procedure is as follows:

> ⚠️ **Warning!**
>
> Before calibration, unload the motor and secure it firmly!
>

    ![image 415x225](assets_user_manual/img_KdJ7byDPeoGec9xJj6xc49kGnce.png)

    ```text
    odrv0.axis0.requested_state = AXIS_STATE_MOTOR_CALIBRATION
    dump_errors(odrv0)
    odrv0.axis0.requested_state = AXIS_STATE_ENCODER_OFFSET_CALIBRATION
    dump_errors(odrv0)
    odrv0.axis0.motor.config.pre_calibrated = 1
    odrv0.axis0.encoder.config.pre_calibrated = 1
    odrv0.save_configuration()
    ```

    Step-by-step explanation:

    - Step 1: Motor parameter self-identification

    ```text
    odrv0.axis0.motor.config.phase_resistance
    odrv0.axis0.motor.config.phase_inductance
    ```

    This measures the motor's phase resistance and phase inductance; you will hear a sharp "beep". The measured phase resistance and phase inductance can be viewed with the following commands:

    - Step 2: Check the error code

    ```python
    dump_errors(odrv0)
    ```

    Check the system error code after step one. If any red error code appears, restart the motor and try again, or report it to after-sales support.

    - Step 3: Encoder calibration
    This calibrates the encoder, including calibrating the encoder's mounting angle relative to the motor's mechanical angle, as well as calibrating the encoder itself. During this calibration, the motor will slowly rotate forward by a certain angle, then rotate backward by a certain angle. If it only rotates forward and then stops, this indicates an error; check the error code via step 4.

    - Step 4: Check the error code
    After encoder calibration in step 3, check the system's error code. The error that commonly appears is ERROR_CPR_POLEPAIRS_MISMATCH, indicating that the encoder's CPR setting is wrong, or the motor's pole pairs setting is wrong. Check/set these with the following commands:

    ```text
    odrv0.axis0.encoder.config.cpr
    odrv0.axis0.motor.config.pole_pairs
    ```

    - Step 5: Write the motor calibration success flag
    - Step 6: Write the encoder calibration success flag
    - Step 7: Store the calibration results and restart

    ```text
    odrv0.axis0.encoder.config.pre_calibrated = 1
    odrv0.axis0.motor.config.pre_calibrated = 1
    odrv0.save_configuration()
    ```

### 3.3 Four Control Modes

Once motor calibration is complete, you can control the motor.

#### 3.3.1 Position Control

First, change the control mode to position mode

```python
odrv0.axis0.controller.config.control_mode = 3
```

After setting the control mode, we need to set the input mode (input_mode) to determine the algorithm used to control position. Below we describe the three built-in position control algorithms of the motor.

##### Direct Position Control**（Position Control）**

This is a very aggressive control mode — the motor will use full effort to reach the target position, which can easily trigger an alarm, so we do not recommend using it. To use this mode, set the input mode (input_mode) to:

```python
odrv0.axis0.controller.config.input_mode = 1
```

##### Filtered Position Control**（Filtered Position Control）**

Directly asking the ODrive controller to reach the raw setpoint as fast as possible can cause jerky motion. This is because ODrive uses a Zero Order Hold (ZOH) to hold the value between successive commands sent. Even if you are using a continuous trajectory generated by an external source, if that trajectory is sent at a moderate frequency, ODrive may chase each step in a jerky manner. You can use a second-order position filter to smooth the trajectory. To use this mode, set the input mode (input_mode) to:

```python
odrv0.axis0.controller.config.input_mode = 3
```

A good starting point for tuning the filter bandwidth is to set it to the same value as the command sending frequency. For example, if you send position commands at 5Hz, set the filter bandwidth to 5.

```text
odrv0.axis0.controller.config.input_filter_bandwidth = 5
```

> ⚠️ **Warning!**
>
> The position filter mode uses a second-order position filter, which is an "integrating filter". If the motor runs in a single direction for a long time, the accumulated error will keep growing, causing the motor to fail to keep up with the target position, so **this mode is not suitable for long, sustained single-direction operation**.
>

##### **Trapezoidal Trajectory Position Control (Trajectory Control)**

This mode lets the user set acceleration, cruise velocity, and deceleration to control the motor to move smoothly from one position to another. The term "trapezoidal" refers to the fact that the velocity profile looks like a trapezoid, as shown in the figure below, where orange is velocity and blue is position:

![image 257x166](assets_user_manual/img_Beo5bO397obOVnxl7EDc79janvF.png)

To use this mode, set the input mode (input_mode) to:

```python
odrv0.axis0.controller.config.input_mode = 5
```

Adjustable control parameters (these have default values out of the box, and can be configured as needed):

```python
odrv0.axis0.trap_traj.config.vel_limit     #Maximum cruise velocity, in turn/s
odrv0.axis0.trap_traj.config.accel_limit   #Maximum acceleration, in turn/s^2
odrv0.axis0.trap_traj.config.decel_limit   #Maximum deceleration, in turn/s^2
odrv0.axis0.controller.config.inertia      #Inertia, in Nm/(turn/s^2)
```

`vel_limit` is the maximum planned trajectory velocity. This sets your cruise velocity.

`accel_limit` is the maximum acceleration, in turns/second^2

`decel_limit` is the maximum deceleration, in turns/second^2

`controller.config.inertia` is a value related to acceleration (in turns/second^2) and motor torque. The default value is 0. It is optional, but if tuned correctly, it can improve the system's response. Keep in mind that this value needs to be adjusted according to the system's load/mass.

> 💡 All values should be strictly positive (>= 0).
>

It should also be noted that the current limit and velocity limit still apply globally — for example, if the maximum cruise velocity above is set higher than the system-level vel_limit, the global vel_limit takes effect instead.

##### Running the Motor

In all subsequent control operations, to make the motor turn, the motor must enter closed-loop control state. The command in odrivetool is as follows:

```python
odrv0.axis0.requested_state = 8
```

At this point the motor has entered the working state, and a target needs to be entered. **Note that the input unit is turns on the rotor side**

```python
odrv0.axis0.controller.input_pos = 10 #Unit: turns
```

#### 3.3.2 Velocity Control

First, change the control mode to velocity mode

```python
odrv0.axis0.controller.config.control_mode = 2
```

After setting the control mode, we need to set the input mode (input_mode) to determine the algorithm used to control velocity. Below we describe the two built-in velocity control algorithms of the motor.

##### **Direct Velocity Control (Velocity Control)**

This mode is the simplest form of velocity control — the motor will use full effort to reach the target velocity, which can easily trigger an alarm, so we do not recommend using it. Enable it as follows:

```text
odrv0.axis0.controller.config.input_mode = 1
```

##### **Ramped Velocity Control (Ramped Velocity Control)**

Ramped velocity control mode gradually raises the velocity to the target value at a set rate, making it gentler than the direct velocity control above. Enable it as follows:

```text
odrv0.axis0.controller.config.input_mode = 2
```

Acceleration is controlled by adjusting the ramp rate, with acceleration in turns/second^2 on the rotor side:

```text
odrv0.axis0.controller.config.vel_ramp_rate = 0.5 #Ramp rate unit is turn/s^2
```

##### Running the Motor

In all subsequent control operations, to make the motor turn, the motor must enter closed-loop control state. The command in odrivetool is as follows:

```python
odrv0.axis0.requested_state = 8
```

At this point the motor has entered the working state, and a target needs to be entered. **Note that the input unit is turns/s on the rotor side**

```python
odrv0.axis0.controller.input_vel = 10 #Unit: turns/s
```

#### 3.3.3 Torque Control

First, change the control mode to torque mode

```python
odrv0.axis0.controller.config.control_mode = 1
```

After setting the control mode, we need to set the input mode (input_mode) to determine the algorithm used to control torque. Below we describe the two built-in torque control algorithms of the motor.

The unit for torque control is Nm (rotor side), while the current unit in the drive firmware is A, so the torque constant must also be set so the drive can convert Nm into current, thereby driving the motor to output the required torque.

```text
# The torque constant is approximately equal to
odrv0.axis0.motor.config.torque_constant = 8.23/12.3
```

##### **Direct Torque Control (Toruqe Control)**

This is the simplest torque (current) control mode — the motor will use full effort to reach the target torque, which can easily trigger an alarm, so we do not recommend using it. Enable it as follows:

```text
odrv0.axis0.controller.config.input_mode = 1
```

##### **Ramped Torque Control (Ramped Torque Control)**

Ramped torque control is very similar to ramped velocity control. Enable it as follows:

```text
odrv0.axis0.controller.config.input_mode = 6
```

Adjust the ramp rate as follows; the ramp rate unit on the rotor side is Nm/s:

```text
odrv0.axis0.controller.config.torque_ramp_rate = 0.1 #Ramp rate unit is Nm/s
```

##### Running the Motor

In all subsequent control operations, to make the motor turn, the motor must enter closed-loop control state. The command in odrivetool is as follows:

```python
odrv0.axis0.requested_state = 8
```

At this point the motor has entered the working state, and a target needs to be entered. **Note that the input unit is newton-meters on the rotor side**

```python
odrv0.axis0.controller.input_torque = 1.2 #Unit: Nm
```

#### 3.3.4 MIT Control

Motion control mode controls the motor to move to the target position by jointly controlling position, velocity, and torque, expressed by the following formula:

$\begin{array}{c}T_{\text {target }}=K_{p} \times P_{\text {diff }}+K_{d} \times V_{\text {diff }}+T_{f f} \\P_{\text {diff }}=P_{\text {target }}-P_{\text {current }} \\V_{\text {diff }}=V_{\text {target }}-V_{\text {current }}\end{array}$

    Here, $T_{\text {target }}$ is the target torque, $P_{\text {diff }}$ is the position error, $V_{\text {diff }}$ is the velocity error, $K_{p}$ is the position control gain, $K_{d}$ is the velocity control gain (also called the damping coefficient), and $T_{f f}$ is the feedforward torque.

    Enable motion control mode as follows:

    ```python
    odrv0.axis0.controller.config.control_mode = 3
    odrv0.axis0.controller.config.input_mode = 9
    ```

    Adjust the gains:

    ```python
    odrv0.axis0.controller.input_mit_kp = <float> #Position gain, in Nm/rad
    odrv0.axis0.controller.input_mit_kd = <float> #Damping coefficient, in Nm/rad/s
    ```

    Adjust the maximum gain values (this affects parameter precision). **Note that the units here are aligned with CAN MIT.**

    ```python
    odrv0.axis0.controller.config.mit_max_kp      #Default 500, in Nm/rad
    odrv0.axis0.controller.config.mit_max_kd      #Default 5, in Nm/rad/s
    odrv0.axis0.controller.config.mit_max_pos     #Default 12.5, in rad
    odrv0.axis0.controller.config.mit_max_vel     #Default 65, in rad/s
    odrv0.axis0.controller.config.mit_max_torque  #Default 50, in Nm
    ```

    In all subsequent control operations, to make the motor turn, the motor must enter closed-loop control state. The command in odrivetool is as follows:

    ```python
    odrv0.axis0.requested_state = 8
    ```

    Then perform motion control by entering input_pos, input_vel, and input_torque. Note the units:

    ```python
    odrv0.axis0.controller.input_pos = 5     #Unit: turns
    odrv0.axis0.controller.input_vel = 30    #Unit: turn/s
    odrv0.axis0.controller.input_torque = 2  #Unit: Nm
    ```

> 💡 Note that when using **USB control**, the position, velocity, and torque values entered all refer to the **rotor side**, whereas when using **CAN** for MIT control, the position, velocity, and torque in the protocol all refer to the **output shaft side** — this is to remain consistent with the MIT open-source protocol!
>

### 3.4 Extended Features

#### 3.4.1 Brake (Discharge) Resistor

![image 305x315](assets_user_manual/img_TYyWbvQqZohlaDx9qY3cVNYCnvg.png)

Users can connect a brake resistor (also called a discharge resistor) between the brake resistor terminals shown in the figure above (EMG and GND) to shunt and dissipate the instantaneous current when bus current flows back into the power supply.

Users can enable this with the following configuration:

```text
odrv0.config.enable_brake_resistor = True
odrv0.config.brake_resistance = xxx #Set the resistance value of the externally connected brake resistor
odrv0.save_configuration()
```

How can the drive discharge automatically? You can combine the following two methods to have the controller automatically adjust the discharge current:

- By adjusting the maximum regenerative current

```text
odrv0.config.max_regen_current = xxx #Maximum regenerative current, a positive number
```

When the regenerative current exceeds the value set above, the drive shunts the excess current to the discharge resistor.

- By adjusting the back-EMF voltage range

```text
odrv0.config.enable_dc_bus_overvoltage_ramp= True
odrv0.config.dc_bus_overvoltage_ramp_start= xxx #Lower voltage limit for shunting
odrv0.config.dc_bus_overvoltage_ramp_end = xxx #Upper voltage limit for shunting
```

When the voltage generated by the back-EMF exceeds the lower limit above, the drive begins shunting to the discharge resistor. When the voltage is greater than or equal to the upper limit above, the drive applies the full voltage to the discharge resistor in an attempt to shunt it. When the voltage is between the lower and upper limits, the drive applies the supply voltage (back-EMF voltage) to the discharge resistor proportionally.

Note that the two modes above can be enabled at the same time; the drive will attempt to combine both methods for discharging.

#### 3.4.2 Limit Switches

The drive supports two limit switches (LW1 and LW2), where LW1 is the minimum position, which is also the homing position, and LW2 is the maximum position. You can connect a 2-wire switch or a 3-wire NPN switch externally. For a 2-wire switch, connect one end to ground and the other end to LW1 or LW2.

![image 293x300](assets_user_manual/img_Wcbibh8GmoZ7n9xCLChcQnlLnCb.png)

![image 718x210](assets_user_manual/img_HXb9bR18BoK5Y0x8RrrcgaBOn6M.png)

To use two limit switches, use the following configuration:

```text
odrv0.axis0.min_endstop.config.enabled = True
odrv0.axis0.min_endstop.config.gpio_num = 1
odrv0.axis0.max_endstop.config.enabled = True
odrv0.axis0.max_endstop.config.gpio_num = 2
```

When a limit switch is triggered, the system will report a `MIN_ENDSTOP_PRESSED` or `MAX_ENDSTOP_PRESSED` error, and the host computer can perform the relevant action at that point.

> ⚠️ **Warning!**
>
> Hardware version 3.7 does not support the limit switch function. The limit switch function conflicts with the pulse output function.
>

#### 3.4.3 User-Defined Zero Point

By default, the position read from the motor by the user, as well as the input value used for position control, are all referenced to the zero point of the absolute encoder on the drive. However, in the user's application, the encoder's zero point is in most cases not the user's zero point, so the user needs to manually set this zero point offset.

Generally, users can locate this zero point using one of two methods: one is via a limit switch, and the other is by manually setting the zero point offset, i.e. the offset of the user's zero point relative to the encoder's zero point:

```python
#After manually or via position control rotating to the desired user zero position, set the current position as the zero point
odrv0.axis0.encoder.config.index_offset = odrv0.axis0.encoder.pos_estimate + odrv0.axis0.encoder.config.index_offset
```

**For motors with a single encoder, this value is reset to zero on power-up. For motors with dual encoders, this value is retained, and the dual-encoder memory range shifts along with this value.**

#### 3.4.4 Power-Loss Memory Function

Only motors with dual encoders have this function. Please confirm which model of motor you purchased.

    - It can remember the single-turn position, with a memory range of -(gear ratio/2) to (gear ratio/2). For the 6010-8, the memory range is -4 to 4 (unit: turns on the rotor side)
    - If the zero point above is set, for example setting index_offset to 2, the dual-encoder memory range becomes -6 to 2
    - Assuming the memory range is -4 to 4, if power is lost when the motor position is 5, since only the single-turn position range can be remembered, the motor will report a position of -3 when powered on again at its current position; the motor internally handles the turn-count wraparound on its own.
## 4 Parameter Tuning

### 4.1 Firmware Upgrade

#### 4.1.1 Firmware Upgrade Using Motor Genie (电机精灵)

First download and install the latest Motor Genie: [Motor Genie Download](https://bl.cyberbeast.cn/actuator/motorwizard.exe)

Download and install the Nations DFU driver: [CyberBeast Driver User Manual](CyberBeast_Driver_User_Manual_EN.md)

Double-click the "Upgrade" button, then select the firmware to complete the upgrade.

![image 540x951](assets_user_manual/img_JQAob2JcUozgUfxQdkOc5bmxnZg.png)

Select the firmware to be upgraded.

![image 946x527](assets_user_manual/img_KbMzbv4iQoqskRxQHCWcEZNjn5e.png)

Wait for the progress bar to finish to complete the upgrade.

![image 540x950](assets_user_manual/img_EcEsbLSO1orXMwxIOhocZqFznEc.png)

> 💡 If, after upgrading, the motor immediately reports an error as soon as it connects to odrivetool or Motor Genie, perform the erase-configuration procedure. After erasing, the motor must be recalibrated **with no load** (see [Motor Calibration chapter](https://cyberbeast.feishu.cn/docx/N3SMd4QyRobzHkx3wP3cT1qXnpf#KySSdxV26oqhyfx5wbWcjGMjnfh))
>
> - Motor Genie erase configuration: press and hold Sync
> - odrivetool: run odrv0.erase_configuration（）

#### 4.1.2 Firmware Upgrade Using odrivetool

1. USB (DFU) Flashing
📎 **Attachment (not downloaded, see original doc):** odrivetool固件升级.mp4 (odrivetool firmware upgrade, 28.6 MB)

Note that the Nations flashing software flashes firmware via the **Type-C port**. **The Type-C port cannot supply power, so use the power connector to supply power before upgrading.**

Download the USB driver for the Nations flashing tool together with the corresponding flashing software, extract them to any directory, and run them. When installing the driver on Windows 11, simply select the Windows 10 files.

📎 **Attachment (not downloaded, see original doc):** 国民DFU驱动.zip (Nations DFU driver, 14.0 MB)

📎 **Attachment (not downloaded, see original doc):** Nations_MCU_Download_Tool_V1.3.4.zip (19.7 MB)

Then connect the Type-C port, open odrivetool, and run the following command to put the drive into DFU mode:

```text
odrv0.enter_dfu_mode()
```

Finally, click the Nations flashing tool to flash the firmware, as shown below. Note that after flashing completes, click "Common Operations" and then "Reset" to restart the drive; it can then be connected and debugged normally through odrivetool.

![image 415x280](assets_user_manual/img_RG2NbYWpFoZzchxdYhFcvreHnMb.png)

![image 848x660](assets_user_manual/img_VdZabfNadoYAE8xQRTLcCKBNnpc.png)

> 💡 If, after upgrading, the motor immediately reports an error as soon as it connects to odrivetool or Motor Genie, perform the erase-configuration procedure. After erasing, the motor must be recalibrated **with no load** (see [Motor Calibration chapter](https://cyberbeast.feishu.cn/docx/N3SMd4QyRobzHkx3wP3cT1qXnpf#KySSdxV26oqhyfx5wbWcjGMjnfh))
>
> - Motor Genie erase configuration: press and hold Sync
> - odrivetool: run odrv0.erase_configuration（）

2. SWD (JLink or DAP) Flashing
Downloading via SWD is similar to DFU mode, but requires connecting through the SWD debug interface (wiring order shown below) and using JFlash for the download.

![image 1111x803](assets_user_manual/img_MvTQbSgGrocfbHx8sOzciRQkn5c.png)

#### 4.1.3 pyocd

pyocd is a Python version of openOCD that supports common debug tools such as STLink, JLink, and DAP for erasing, flashing, resetting, and other operations. **Note that the drive must be connected via the SWD interface.** See the figure above for the SWD wiring order. **The SWD interface carries a 3.3V supply — be careful not to connect the wires in the wrong order, or you may damage the drive!**

3. Install with `pip install pyocd`
4. Flashing
First, list the connected debug tools:

```text
pyocd list
```

![image 415x74](assets_user_manual/img_U9rdbYjRpob0bxxp2u0ck6YLn3f.png)

Then run the following command to flash the bin file:

```text
pyocd load .\ODrive_N32G455.bin -a 0x8000000
```

![image 415x112](assets_user_manual/img_ELUdbdXiWollpcxxNVfcQGLHn0b.png)

### 4.2 PID and Tuning for Different Control Modes

The video cover serves as an index — jump to the relevant point on the timeline as needed.

📎 **Attachment (not downloaded, see original doc):** 202506061025.mp4 (2.4 GB)

```text
odrv0.axis0.controller.config.pos_gain=20.0
odrv0.axis0.controller.config.vel_gain=0.16
odrv0.axis0.controller.config.vel_integrator_gain=0.32
```

The following procedure can serve as a reference for tuning PID parameters:

5. Set the initial PID values
6. Set vel_integrator_gain to 0
7. odrv0.axis0.controller.config.vel_integrator_gain=0
    Method for tuning vel_gain:

8. Rotate the motor in velocity control mode. If the rotation is unstable, with jitter or vibration, decrease vel_gain until the rotation is stable.
9. Then increase vel_gain by about 30% at a time until noticeable jitter appears.
10. At this point, decrease vel_gain by about 50% to stabilize it.
11. Method for tuning pos_gain:
12. Try rotating the motor in position mode. If the rotation is unstable, with jerking or vibration, decrease pos_gain until the rotation is stable.
13. Then increase pos_gain by about 30% at a time until position control shows noticeable overshoot (i.e., each time the motor overshoots the target position and then oscillates back to it).
14. Then gradually decrease pos_gain until the overshoot disappears.
15. After completing the four steps above, you can set vel_integrator_gain to 0.5*bandwidth*vel_gain, where bandwidth is the system's control bandwidth. What is control bandwidth? For example, if the time from when the user sets a target position to when the motor actually reaches it is 10ms, the control bandwidth is 100Hz, so vel_integrator_gain=0.5*100*vel_gain.
In the tuning process above, it is recommended to use the graphical display in [3](https://bcnyljrhe70u.feishu.cn/docx/N3SMd4QyRobzHkx3wP3cT1qXnpf#KUN8do5syo24zPxNQ41ctwJfnOe)[.1 Real-Time Plotting Tool](https://bcnyljrhe70u.feishu.cn/docx/N3SMd4QyRobzHkx3wP3cT1qXnpf#KUN8do5syo24zPxNQ41ctwJfnOe) to observe the tuning results in real time, avoiding errors from visual judgment. If using Motor Genie, PID tuning effects are "what you see is what you get" — the figure below illustrates the PID parameters:

    ![image 212x284](assets_user_manual/img_HOuUbjHzoojUyQxz14fcJcoxnpg.png)

### 4.3 Key Limits (limits)

#### 4.3.1 Voltage Limits

    If the motor reports an undervoltage or overvoltage alarm, it is usually related to this limit.

    ![image 500x240](assets_user_manual/img_Dja6bc48Mown7Sxv6yJc70zpnkd.png)

    ![image 539x314](assets_user_manual/img_XLB7bt47vooJTox2bhmc1Uelnc4.png)

    - **odrivetool**
    Low-voltage alarm threshold (V), default 15V; overvoltage alarm threshold (V), default 64V.

    ```python
    odrv0.config.dc_bus_undervoltage_trip_level    #Low-voltage alarm threshold (V)
    odrv0.config.dc_bus_overvoltage_trip_level     #Overvoltage alarm threshold (V)
    ```

    - **Motor Genie**

    ![image 315x672](assets_user_manual/img_JT4JbTCkMoR2gDxA1WicdJQ0nxd.png)

    After changing parameters in Motor Genie, remember to click Sync to save the parameters while the motor is idle.

> 💡 Note: when using a 24V or lower power supply, remember to change the lower voltage limit to 10V or 8V on the host computer. Otherwise, undervoltage errors may occur frequently during use.
>

#### 4.3.2 Current Limits

    If the motor reports an overcurrent alarm, it is usually related to this limit.

    ![image 399x229](assets_user_manual/img_BORGbWyc7ojhfNxa0Z1c9BnEnEh.png)

    ![image 537x355](assets_user_manual/img_ML39bD0VDojJOzxk0upcfn8nnjg.png)

    Not yet finalized, to be updated. Set the bus current parameter (`odrv0.config.dc_max_positive_current`) to 200A to prevent unnecessary overcurrent alarms on the motor; the motor current (`odrv0.axis0.motor.config.current_lim`) has a maximum of 80A, with a default of 60A.

    ![image 542x483](assets_user_manual/img_B2OXb9iXqod0MSxndGjcc4hPnwd.png)

#### 4.3.3 Temperature Protection

    Temperature protection on the drive is enabled by default, with a default range of 25℃ to 80℃. When the temperature reaches the lower end of the set range, the motor begins temperature protection and its torque starts to decrease; as the motor temperature keeps rising and approaches the upper protection limit, the torque becomes smaller and smaller until the motor stops running (this process does not generate an error).

    If you don't want protection to start at room temperature, you can try changing the range to 70℃～90℃ (set the drive temperature range and the motor temperature range at the same time).

    The motor will only produce a temperature-related error if its temperature rises sharply.

    - **odrivetool**
    Switches for enabling drive board temperature protection and enabling motor temperature protection: 1 = on, 0 = off.

    ```python
    odrv0.axis0.motor.motor_thermistor.config.enabled = 1  #Enable motor temperature protection
    odrv0.axis0.motor.fet_thermistor.config.enabled = 1    #Enable drive board temperature protection
    ```

    Motor temperature protection thresholds

    ```python
    odrv0.axis0.motor.motor_thermistor.config.temp_limit_lower    #Lower limit
    odrv0.axis0.motor.motor_thermistor.config.temp_limit_upper    #Upper limit
    ```

    Drive board temperature protection thresholds

    ```python
    odrv0.axis0.motor.fet_thermistor.config.temp_limit_lower    #Lower limit
    odrv0.axis0.motor.fet_thermistor.config.temp_limit_upper    #Upper limit
    ```

    Get temperature

    ```python
    # Get temperature
    odrv0.axis0.motor.motor_thermistor.temperature #Motor temperature
    odrv0.axis0.motor.fet_thermistor.temperature #Drive board temperature
    ```

    - **Motor Genie**

    ![image 316x676](assets_user_manual/img_ZwuabdSrToERJzx59h1cOsKrnRb.png)

#### 4.3.4 Velocity Limits

    The velocity limit sets the maximum speed the motor can reach

    - **odrivetool**
    Global velocity limit enable, 1 = on, 0 = off

    ```python
    odrv0.axis0.controller.config.enable_vel_limit = 1
    ```

    Global velocity limit

    ```python
    odrv0.axis0.controller.config.vel_limit = 30
    ```

    The command above limits it to 30 turn/s (revolutions per second). Note that by default this velocity limit has no effect in pure torque mode, but it can be enabled with the switch below:

    ```text
    odrv0.axis0.controller.config.enable_torque_mode_vel_limit = 1
    ```

    - **Motor Genie**
    The values entered below are all global velocity limits. In the figure on the right, the upper switch enables/disables the global velocity limit, and the lower switch determines whether this velocity limit is enabled in pure torque mode.

    ![image 309x682](assets_user_manual/img_IZ1obKcZEohTG2xwtFsc1uorn6g.png)

    ![image 312x512](assets_user_manual/img_EfpQbPFvGop5PjxQzTtcgDuynyh.png)
## 5 Using the Communication Protocol

Communication protocol manual: [CyberBeast Driver Protocol Manual](CyberBeast_Driver_Protocol_Manual_EN.md)

### 5.1 CANSimple

#### 5.1.1 Wiring Instructions

For hardware versions v3.7, 3.9, 3.10, 3.11, wire according to the wire order shown in the first figure; for hardware versions v3.8, 4.1, wire according to the wire order shown in the second figure. Under normal conditions, the CANH dominant voltage should be about 3.5V, and the CANL dominant voltage should be about 1.5V. Measuring CANH and CANL to ground with a multimeter should give about 2.5V.

![image 3289x663](assets_user_manual/img_YrRabL5zeoGTnYxWiLJc3FEmn4f.png)

![image 3289x663](assets_user_manual/img_YdUwbcQtuo7zILxU2NFcHpVUnGd.png)

> ⚠️ **Warning!**
>
> All CAN devices must share a common ground, otherwise CAN may not function properly.
>

#### 5.1.2 120 Ω Impedance Matching

Ensure that the total impedance on the bus is about 60Ω. A standard CAN network requires **120Ω at each end (the control end and the farthest motor end), for 60Ω total**. The board has a built-in software-controlled 120Ω resistor switch, which can be enabled with the following command:

- odrivetool

```python
odrv0.can.config.enable_r120 = True
```

- Motor Genie (电机精灵)

![image 308x221](assets_user_manual/img_CfXPbYNFJo25AbxvvKUcjucEnwe.png)

#### 5.1.3 Switching the Communication Protocol

- odrivetool
The drive's default protocol is CAN Simple. If the user switches to the CANOpen protocol, use the following command to switch back to CAN Simple:

```text
odrv0.can.config.protocol = 1
```

Note that the communication interface also supports RS485, Step/Dir, and PWM. Use the following command to make sure the communication interface is set to CAN:

```text
odrv0.config.comm_intf_mux = 0
```

- Motor Genie
In Motor Genie, you can also switch the physical interface and the CAN protocol version under Motor Parameters -> Communication Parameters.

![image 305x224](assets_user_manual/img_ECJ0bQgIjoTnTCxN3fMc9Edan6d.png)

#### 5.1.4 Changing the Baud Rate

The factory default rate is 500Kbps. For drives that support only CAN, the maximum communication rate is 1Mbps; for drives that support CANFD, the maximum communication rate is 5Mbps. The baud rate can be changed via odrivetool or Motor Genie.

- odrivetool

```python
 odrv0.can.config.baud_rate = 500000
```

- Motor Genie

![image 309x225](assets_user_manual/img_HtMBbyWMpoVCM1xcqm4cb39mnuc.png)

#### 5.1.5 Periodic Messages

Users can configure the motor to periodically send messages to the host computer, instead of the host computer sending request messages to the motor. Periodic messages can be turned on/off via a series of settings under `odrv0.axis0.config.can` (a value of 0 means off; any other value is the period, in ms), as shown in the table below:

| Message | cmd_id | Content | odrivetool Setting | Default |
|---|---|---|---|---|
| Heartbeat | 0x01 | Heartbeat feedback | `odrv0.axis0.config.can.heartbeat_rate_ms` | 100 |
| Get_Encoder_Estimates | 0x09 | Encoder position/velocity feedback | `odrv0.axis0.config.can.encoder_rate_ms` | 10 |
| Get_Motor_Error | 0x03 | Motor error feedback | `odrv0.axis0.config.can.motor_error_rate_ms` | 0 |
| Get_Encoder_Error | 0x03 | Encoder error feedback | `odrv0.axis0.config.can.encoder_error_rate_ms` | 0 |
| Get_Controller_Error | 0x03 | Controller error feedback | `odrv0.axis0.config.can.controller_error_rate_ms` | 0 |
| Get_Iq | 0x14 | Q-axis current feedback | `odrv0.axis0.config.can.iq_rate_ms` | 0 |
| Get_Thermistor_Temperature | 0x15 | Temperature feedback | `odrv0.axis0.motor.motor_thermistor.temperature`<br>`odrv0.axis0.motor.fet_thermistor.temperature` | 0 |
| Get_Bus_Voltage_Current | 0x17 | Bus voltage/current feedback | `odrv0.axis0.config.can.bus_vi_rate_ms` | 0 |

**At the factory, the first two periodic messages are enabled by default**, so when monitoring the CAN bus, users will see these two messages being broadcast at their configured periods. Users can disable them with the following commands:

```text
odrv0.axis0.config.can.heartbeat_rate_ms = 0
odrv0.axis0.config.can.encoder_rate_ms= 0
```

To disable them using Motor Genie, open Motor Genie -> ODrive Parameters, find the commands above, type 0, press Enter, then click Sync to save.

![image 540x901](assets_user_manual/img_FShvbpLvGoYfDoxeOMlcmzSwnsb.png)

![image 540x901](assets_user_manual/img_NL3AbTJB4oIcjwxFyokceiKwnxg.png)

The feedback content of Get_Encoder_Estimates (0x09) only reports the current position and velocity information **after the motor enters closed-loop control**; in the idle state, it only reports the position and velocity from the last time it entered closed-loop control (it reports 0 if closed-loop control has never been entered).

#### 5.1.6 Changing the Motor ID

The drive ID is unique on the bus, in the range 0-63. If the user has multiple motors connected in series on the bus, each motor needs to be set to a different ID. The ID can be modified via USB or over the bus:

- odrivetool

```text
odrv0.axis0.config.can.node_id = xxx
```

- Motor Genie
Open Motor Genie, connect to the motor, click "Motor Parameters", modify the ID under "Communication Parameters", then click "Sync" to store it on the drive.

![image 308x225](assets_user_manual/img_BX1vbxYsiouh3QxetIicAtDCnYb.png)

#### 5.1.7 Getting Detailed Error Information

Search "heartbeat" in the protocol manual to view the detailed description of the CAN heartbeat frame. The heartbeat frame contains the device's current status information (see the figure below), which can be used to quickly determine whether an anomaly exists.

Here, byte5 (`Flags`) identifies the current device status or fault type and can serve as an initial basis for diagnosis; byte0 (`Axis_Error`) records the axis-level error code, which is also helpful for troubleshooting. Note that **these error messages usually only reveal the surface symptoms of a problem — determining the actual root cause requires combining the hints in Flags** with a closer look at the corresponding detailed error description (Get_Error, shown below) for analysis.

![image 762x588](assets_user_manual/img_Va8QboW6Vo7ZSYxwcE3cjsPHnfe.png)

![image 834x338](assets_user_manual/img_XWvEbS0mnoccqNxuXTWcXseYnxg.png)

For example, if Flags is 0x82, which converts to binary 1000 0010, this indicates an encoder anomaly (bit7 is a flag indicating whether the trapezoidal position trajectory has finished executing, and is not itself part of the error). In this case, use function code 0x03 Get_Error to obtain the detailed encoder anomaly, as shown in the manual screenshot above.

After sending 01 to function code 0x03, the detailed encoder anomaly obtained is shown below.

![image 781x30](assets_user_manual/img_Rm8sbxW2aoDKzkxR1rYcYdMQnxh.png)

![image 788x79](assets_user_manual/img_RjN0btollotQwXxExZMcIibjnnh.png)

0x00000002 — looking this up in the [protocol manual's error codes](https://cyberbeast.feishu.cn/docx/BPnQd8reEotLWVxqHFNc9qYZnKh#KKr4dVPH3oQ9r5x5KxocJLmQn4b), it corresponds to an encoder anomaly

![image 591x119](assets_user_manual/img_I1a4bt4j5o1vE5x3vCRcJbczni0.png)

This anomaly is usually caused by a mismatch between the pole pairs configured in the motor drive and the motor's actual pole pairs, and it often occurs during the motor calibration process.

Each time an anomaly occurs, several anomalies may occur at once; you need to check all of the error codes together to determine the root cause of the motor anomaly. This concludes the process of retrieving motor error codes over CAN.

#### 5.1.8 Using RxSdo

![image 604x798](assets_user_manual/img_Fe1ObR938o1qGMxYtCvcglVunrf.png)

CAN RxSdo and TxSdo send data in little-endian format. First, based on your firmware version (for details on firmware versions, see [ ](https://bcnyljrhe70u.feishu.cn/docx/N3SMd4QyRobzHkx3wP3cT1qXnpf#XbxRdrkQMo0Pqux5meqcbF1Jnmb)[2](https://bcnyljrhe70u.feishu.cn/docx/N3SMd4QyRobzHkx3wP3cT1qXnpf#XbxRdrkQMo0Pqux5meqcbF1Jnmb)[.1 Motor Parameters](https://bcnyljrhe70u.feishu.cn/docx/N3SMd4QyRobzHkx3wP3cT1qXnpf#XbxRdrkQMo0Pqux5meqcbF1Jnmb)), confirm the corresponding json file (downloadable from within the manual)

For example, suppose I want to change dc_max_positive_current to 40 (this value is of type float). The first byte of the write data is 01. Looking it up, the id is 98 (STW 0.5.14 json), corresponding to the hexadecimal code 0x0062; the IEEE representation of 40 is shown in the figure: 42 20 00 00. So we need to send: can_id 0x005, with content: 01 62 00 00 00 00 20 42.

![image 311x97](assets_user_manual/img_ZvodbT2qXo6hVlxshRRcJhVsnfg.png)

![image 650x175](assets_user_manual/img_F21ab8R85oheuTxWXRRcRk8InIf.png)

For uint8/bool variables, for example if I want to change the motor's operating state, the first byte of the write data is 01. Looking it up, the id for request_state is 137 (0.5.11), corresponding to hexadecimal 0x0089; writing state 08 gives send content 01 89 00 00 08 00 00 00

![image 1403x586](assets_user_manual/img_VUIGbkTvmoItJzxswCWc379anCf.png)

#### 5.1.9 Communication Timeout Stop

CAN currently has a communication timeout stop feature, which works as follows: starting from the first CAN command sent, if CAN communication is interrupted for longer than the configured time, the motor will report an error, and **communication must be restored and the error cleared** before the motor can run normally again. For example, suppose the setting is 2000 (in ms); if the interval between the motor's first received CAN command and the second received command exceeds 2000ms, the motor will generate an error.

    The parameter defaults to 0 (disabled). The value is set in ms; once set, the command takes effect immediately. The command is as follows:

    - odrivetool
    After setting it, save while in the idle state.

    ```python
    odrv0.can.config.break_timeout = 2000    #Set the timeout to 2000ms
    ```

    After the timeout, the motor will report an error:

    ![image 339x199](assets_user_manual/img_MKGzbBxR7oAoOuxX2FNcHubAnWb.png)

    - Motor Genie
    ODrive Parameters -> the command above -> type it and press Enter, then click Sync while in the idle state.

    ![image 540x901](assets_user_manual/img_GCJJbXxtGobmyvxxXzYcPSOAnec.png)

After the timeout, the motor error code:

![image 408x394](assets_user_manual/img_EDu4bXyUTo0QQ7xnubEcbktwn2f.png)

### 5.2 CANOpen

For [wiring instructions](https://bcnyljrhe70u.feishu.cn/docx/N3SMd4QyRobzHkx3wP3cT1qXnpf#EZFEd6iuSoP56wx1J8EcbntvnGg), [impedance matching](https://bcnyljrhe70u.feishu.cn/docx/N3SMd4QyRobzHkx3wP3cT1qXnpf#EV9jdu00boMITLxNMt2cqSSvnHd), and [baud rate changes](https://bcnyljrhe70u.feishu.cn/docx/N3SMd4QyRobzHkx3wP3cT1qXnpf#TgX5dl5H0o25JOxBQvZcJPlBnyb), these are the same as in the CANSimple section — please refer to that section yourself.

#### 5.2.1 Switching the Communication Protocol

    - odrivetool
    The drive's default protocol is CAN Simple. If the user needs to use CANOpen, choose either one of the following two methods to select the communication mode:

    ```text
    odrv0.can.config.protocol = 2
    ```

    Note that the communication interface also supports RS485, Step/Dir, and PWM. Use the following command to make sure the communication interface is set to CAN:

    ```text
    odrv0.config.comm_intf_mux = 0
    ```

    - Motor Genie
    In Motor Genie, you can also switch the physical interface and the CAN protocol version under Motor Parameters -> Communication Parameters.

    ![image 310x225](assets_user_manual/img_WBuEb6P6AoW20Yx3JuMcGX9CnAd.png)

### 5.3 RS485 (Modbus RTU)

The RS485 interface is an optional interface on some boards. If you need this interface, please confirm with sales.

#### 5.3.1 Wiring Instructions

![image 3289x663](assets_user_manual/img_Cqx8bvqUeoErgQxTJI8ccP7mnve.png)

![image 3289x663](assets_user_manual/img_Fr13bSy7LocGyxx6PSScVFC7nHe.png)

#### 5.3.2 Switching the Communication Protocol

- odrivetool
Note that on hardware versions 3.10 and above, the communication interface also supports RS485, Step/Dir, and PWM. Use the following command to make sure the communication interface is set to CAN:

```python
odrv0.config.comm_intf_mux = 1
```

- Motor Genie
In Motor Genie, you can also switch the physical interface under Communication Parameters.

![image 304x220](assets_user_manual/img_NgNzbGoGJoT47AxYtewclaJOnFd.png)

#### 5.3.3 Changing the Baud Rate

For Modbus RTU over RS485, the maximum communication rate is 115200bps, and the factory default rate is 115200bps. The baud rate can be changed via odrivetool or Motor Genie.

- odrivetool

```python
odrv0.config.uart_c_baudrate = 115200
```

- Motor Genie
Open ODrive Parameters and enter the values as shown in the figure

![image 540x901](assets_user_manual/img_TumIbLYRxoYnsLxQrAfc5UYSnDs.png)

![image 321x273](assets_user_manual/img_DFmmb5Js5ooeQ8xMHzdc8GZenld.png)

#### 5.3.4 Using Random Endpoint Access

To be updated.

#### 5.3.5 Changing the Modbus Station Number

- odrivetool

```python
odrv0.axis0.config.can.node_id = xx
```

### 5.4 PWM Input Control

Confirm that the hardware version is v3.10, 3.11, 4.1, or later before using this feature.

This feature supports controlling various writable parameters, such as position, velocity, or torque, using a standard RC PWM signal (1ms-2ms pulse width, 50Hz). By configuring a GPIO pin as PWM mode and setting up the mapping, the motor can be controlled directly from an RC receiver or another PWM controller.

#### 5.4.1 Wiring Instructions

    Connect the PWM signal wire to the position shown in the figure. For hardware versions 3.10 and 3.11, wire according to the first figure; for hardware version 4.1, wire according to the first figure. This interface supports PWM signals of 5V or below, and the PWM signal must share a common ground with the motor drive.

    ![image 3289x663](assets_user_manual/img_C0KZbykohok7aGxzTazc2yoFn5b.png)

    ![image 3289x663](assets_user_manual/img_M239beNvDoC4Fvxo2ULcr6IFnyd.png)

#### 5.4.2 Software Configuration

    - Switch the communication interface to PWM:

    ```python
    odrv0.config.comm_intf_mux = 3
    ```

    In Motor Genie, you can also switch the physical interface under Communication Parameters; open Motor Parameters to find this option.

    ![image 486x350](assets_user_manual/img_ALl5bqoH7oQ18fxX6F8c75FSnbc.png)

    The subsequent configuration is not supported via Motor Genie. Please first install odrivetool according to [ 1.3.2 ](https://bcnyljrhe70u.feishu.cn/docx/N3SMd4QyRobzHkx3wP3cT1qXnpf#Xrj1d6UwKokjcax0RFvcVJphnZb) before proceeding with the rest of the configuration.

    - Set the PWM mapping control mode
    It can be mapped to velocity/position/torque mode; configure just one of the following three modes as needed.

    Velocity mode

    Set the motor control mode to [ramp velocity mode](https://bcnyljrhe70u.feishu.cn/docx/N3SMd4QyRobzHkx3wP3cT1qXnpf#Mrc6dx39Zo1s4PxXAOgc99Fgnbe) (control_mode = 2, input_mode = 2)

    ```python
    # Set mode to velocity mode
    odrv0.axis0.controller.config.control_mode = 2
    # Ramp velocity mode for velocity mode
    odrv0.axis0.controller.config.input_mode = 2
    # Set PWM input as velocity input
    odrv0.config.gpio1_pwm_mapping.endpoint = odrv0.axis0.controller._input_vel_property
    ```

    Position mode

    Set the motor control mode to [trapezoidal trajectory position mode](https://bcnyljrhe70u.feishu.cn/docx/N3SMd4QyRobzHkx3wP3cT1qXnpf#I21YddeF1ojr7Nx0JdSc5JCznrh) (control_mode = 3, input_mode = 5)

    ```python
    # Set mode to position mode
    odrv0.axis0.controller.config.control_mode = 3
    # Trapezoidal trajectory mode for position mode
    odrv0.axis0.controller.config.input_mode = 5
    # Set PWM input as position input
    odrv0.config.gpio1_pwm_mapping.endpoint = odrv0.axis0.controller._input_pos_property
    ```

    Torque mode

    Set the motor control mode to [ramp torque mode](https://bcnyljrhe70u.feishu.cn/docx/N3SMd4QyRobzHkx3wP3cT1qXnpf#W0JZdpkhxopGx8xA8rIcZVWqnff) (control_mode = 1, input_mode = 6)

    ```python
    # Set mode to torque mode
    odrv0.axis0.controller.config.control_mode = 1
    # Ramp torque mode for velocity mode
    odrv0.axis0.controller.config.input_mode = 6
    # Set PWM input as torque input
    odrv0.config.gpio1_pwm_mapping.endpoint = odrv0.axis0.controller._input_torque_property
    ```

    -  Set the PWM mapping parameters

    ```python
    odrv0.config.gpio1_pwm_mapping.min = -8.0
    odrv0.config.gpio1_pwm_mapping.max = 8.0
    ```

    min and max define the range of control values that the PWM pulse width (1ms-2ms) maps to; internally, the drive linearly maps the pulse width value to a real number within the [min, max] range. min and max can be set to any values as needed.

    -  Save the configuration (optional)

    ```python
    odrv0.save_configuration()
    ```

    - After completing the above configuration, put the motor into closed-loop control to enable PWM control.

    ```python
    odrv0.axis0.requested_state = 8
    ```

### 5.5 Step/Direction Control

Confirm that the hardware version is v3.10, 3.11, 4.1, or later before using this feature.

This feature conflicts with the limit switch feature. It is the simplest way to control the drive, but also the most primitive and fragile. Therefore, unless there is a special reason, do not use this control method.

This feature supports controlling the brushless motor via the traditional Step/Dir (pulse/direction) interface, providing a control method compatible with ordinary stepper motor drivers. In this mode:

- Step (pulse) signal: each rising edge moves the motor by a preset step distance (determined by steps_per_circular_range); the maximum recommended Step frequency is 50kHz
- Dir (direction) signal: the logic level determines the direction of motion (high level = clockwise / low level = counterclockwise)

#### 5.5.1 Wiring Instructions

    Connect the Step (pulse signal) and Dir (direction signal) wires to the positions shown in the figure. This interface supports pulse signals of 5V or below. Make sure the Step and Dir signals share a **common ground (common cathode)** with the motor drive, otherwise the signals may fail to be recognized.

    ![image 3289x663](assets_user_manual/img_VBlHb8pIhodcOqxzPPWcSPWIn2c.png)

    ![image 3289x663](assets_user_manual/img_SSwmbMGB0ooxoNxMQVucfZQBnkd.png)

#### 5.5.2 Software Configuration

    - Switch the communication interface to Step/Dir:

    ```python
    odrv0.config.comm_intf_mux = 2
    ```

    In Motor Genie, you can also switch the physical interface under Communication Parameters; open Motor Parameters to find this option.

    ![image 483x345](assets_user_manual/img_XSOvbaM7Oo3LpZxaQTvcv4v3nwf.png)

    The subsequent configuration is not supported via Motor Genie. Please first install odrivetool according to [ 1.3.2 ](https://bcnyljrhe70u.feishu.cn/docx/N3SMd4QyRobzHkx3wP3cT1qXnpf#Xrj1d6UwKokjcax0RFvcVJphnZb) before proceeding with the rest of the configuration.

    - Enable Step/Dir

    ```python
    odrv0.axis0.config.enable_step_dir = True
    ```

    Note that to exit Step/Direction control, you need to set enable_step_dir to False, then enter the idle state, and then return to closed-loop control.

    - Set the pulse step size
    This determines how many STEP pulses correspond to the circular position range, i.e., how much displacement 1 step represents; the step size is an integer. For best results, set this parameter to a power of 2.

    ```python
     # 1024 steps per revolution on the rotor side
    odrv0.axis0.controller.config.steps_per_circular_range = 1024
    ```

    - After completing the above configuration, set the motor to **position mode**.

    ```python
    # Set mode to position mode
    odrv0.axis0.controller.config.control_mode = 3
    # Trapezoidal trajectory mode for position mode
    odrv0.axis0.controller.config.input_mode = 5
    ```

    - Save the configuration while in the idle state (optional)

    ```python
    odrv0.save_configuration()
    ```

    - After entering closed-loop control, send pulses to control the motor

    ```python
    odrv0.axis0.requested_state = 8
    ```
## 6 SDK and Examples

### 6.1 C/C++ SDK

Based on the CAN Simple protocol, you can download the following C/C++ SDK for integration: [Beijing CyberBeast Technology Co., Ltd./MWMotorSDK](https://gitee.com/cyberbeast/mwmotorsdk)

Quick start based on the SDK: [MWMotor SDK Quick Start](https://cyberbeast.feishu.cn/docx/KRJcdmc3yovKxsxNnXOcNmppnwf)

Please refer to the detailed test examples on the open-source site above for integration testing.

### 6.2 Python SDK

First, follow the steps in Section 1.3.2 to install odrivetool (`pip install ––upgrade odrive`).

Below are three examples:

1. **Hands-on: Power-on Calibration**

```python
import odrive
import time

odrv0 = odrive.find_any()
odrive.utils.dump_errors(odrv0)
odrv0.clear_errors()
odrv0.axis0.requested_state=odrive.utils.AxisState.MOTOR_CALIBRATION
time.sleep(5)
while (odrv0.axis0.current_state!=1):
time.sleep(0.5)
odrive.utils.dump_errors(odrv0)
odrv0.axis0.requested_state=odrive.utils.AxisState.ENCODER_OFFSET_CALIBRATION
time.sleep(6)
while (odrv0.axis0.current_state!=1):
time.sleep(0.5)
odrive.utils.dump_errors(odrv0)
odrv0.axis0.motor.config.pre_calibrated=1
odrv0.axis0.encoder.config.pre_calibrated=1
odrv0.save_configuration() 
```

2. **Hands-on: Velocity Control**

```python
import odrive
import time

odrv0 = odrive.find_any()
odrv0.axis0.controller.config.control_mode=odrive.utils.ControlMode.VELOCITY_CONTROL
odrv0.axis0.controller.config.input_mode=odrive.utils.InputMode.VEL_RAMP
odrv0.axis0.controller.config.vel_ramp_rate=50
odrv0.axis0.requested_state=odrive.utils.AxisState.CLOSED_LOOP_CONTROL
odrv0.axis0.controller.input_vel=15
odrive.utils.dump_errors(odrv0)
time.sleep(5)
odrv0.axis0.controller.input_vel=0
```

3. **Hands-on: Position Control**

```python
import odrive

odrv0 = odrive.find_any()
odrv0.axis0.controller.config.control_mode=odrive.utils.ControlMode.POSITION_CONTROL
odrv0.axis0.controller.config.input_mode=odrive.utils.InputMode.POS_FILTER
odrv0.axis0.requested_state=odrive.utils.AxisState.CLOSED_LOOP_CONTROL
odrv0.axis0.controller.input_pos=10
```

4. **Hands-on: Data Acquisition**
During development and integration, users often need to capture motor operating data, such as voltage/current changes and position/velocity changes. The Python SDK integrates powerful data capture capabilities, allowing simple scripts to capture massive amounts of runtime data, making development and integration much easier.

```python
import odrive
import numpy as np
import time
import math
from odrive.utils import BulkCapture

# Find and connect to the ODrive device
odrv0 = odrive.find_any()
print("Connected successfully")

# Set control mode and input mode
odrv0.axis0.controller.config.control_mode = odrive.utils.ControlMode.POSITION_CONTROL
odrv0.axis0.controller.config.input_mode = odrive.utils.InputMode.POS_FILTER
print("Entering position loop, position filter mode")

# Start the motor
odrv0.axis0.requested_state = odrive.utils.AxisState.CLOSED_LOOP_CONTROL
print("Starting motor")

# Use BulkCapture to capture data (captures Iq current and position information)
cap = BulkCapture(
    lambda :[odrv0.axis0.motor.current_control.Iq_measured, odrv0.axis0.encoder.pos_estimate],  # Function passed for data capture
    data_rate=200,    # 200Hz sampling rate
    duration=15       # Total sampling duration of 15 seconds
)
print("Capturing real-time position and current data")

# Rotate the motor to position 0 and hold for 3 seconds
odrv0.axis0.controller.input_pos = 0
print("Testing position loop, rotating motor to position 0")
while not math.isclose(odrv0.axis0.encoder.pos_estimate, odrv0.axis0.controller.input_pos, rel_tol=0.08, abs_tol=0.08):
    time.sleep(0.01)
time.sleep(3)

# Rotate the motor to position 10 and hold for 3 seconds
odrv0.axis0.controller.input_pos = 10
print("Testing position loop, rotating motor to position 10")
while not math.isclose(odrv0.axis0.encoder.pos_estimate, odrv0.axis0.controller.input_pos, rel_tol=0.08, abs_tol=0.08):
    time.sleep(0.01)
time.sleep(3)

# Rotate the motor to position -10 and hold for 3 seconds
odrv0.axis0.controller.input_pos = -10
print("Testing position loop, rotating motor to position -10")
while not math.isclose(odrv0.axis0.encoder.pos_estimate, odrv0.axis0.controller.input_pos, rel_tol=0.08, abs_tol=0.08):
    time.sleep(0.01)
time.sleep(3)

# Stop the motor
odrv0.axis0.requested_state = odrive.utils.AxisState.IDLE
print("Rotation finished")

# Get the captured data
cap.event.wait()
print(cap.data)
data = cap.data  # Get the captured data
np.savetxt("D:/test.csv", data, delimiter=',')
print("Data saved to test.csv")

```

The code below builds on the previous position control example by adding real-time position and current data capture, and saves the data to a csv file.

In the BulkCapture statement, data_rate represents the sampling frequency in Hz, duration represents the sampling time in seconds, and the lambda expression can include any mathematical operation to facilitate data analysis.

### 6.3 Arduino SDK

    Users can control the motor via the CAN bus using Arduino; the underlying protocol is as described in Section 4.1. Compatible hardware/libraries:

    - Arduino boards with a built-in CAN interface, such as Arduino UNO R4 Minima, Arduino UNO R4 WIFI, etc.
    - Teensy boards with a built-in CAN interface can be accessed using the adapted FlexCAN_T4 library (Teensy 4.0 and Teensy 4.1)
    - Other Arduino-compatible boards can be accessed using an MCP2515-based CAN expansion board
    Below is an example showing how to configure the motor to respond to Arduino position control commands:

    - Configure the motor
    In addition to the basic configuration in Section 3.1.3, set the control bandwidth to 20rad/s (the Arduino Uno's transmission speed is limited, so the control bandwidth doesn't need to be very high; if you use a faster Arduino, you can increase this bandwidth value):

    - Configure CAN
    Configure CAN as follows:

    - Install the ODriveArduino library
    Follow the steps below to install the OdriveArduino library (assuming the Arduino IDE is already installed):

    1. Open the Arduino IDE
    2. Sketch -> Include Library -> Manage Libraries
    3. Enter "ODriveArduino" to search
    4. Click the found ODriveArduino library to install it
    - Arduino source code

    ```cpp
     #include <Arduino.h>
    #include "ODriveCAN.h"
    
    // Documentation for this example can be found here:
    // https://docs.odriverobotics.com/v/latest/guides/arduino-can-guide.html
    
    /* Configuration of example sketch -------------------------------------------*/
    
    // CAN bus baudrate. Make sure this matches for every device on the bus
    #define CAN_BAUDRATE 500000
    
    // ODrive node_id for odrv0
    #define ODRV0_NODE_ID 0
    
    // Uncomment below the line that corresponds to your hardware.
    // See also "Board-specific settings" to adapt the details for your hardware setup.
    
    // #define IS_TEENSY_BUILTIN // Teensy boards with built-in CAN interface (e.g. Teensy 4.1). See below to select which interface to use.
    // #define IS_ARDUINO_BUILTIN // Arduino boards with built-in CAN interface (e.g. Arduino Uno R4 Minima)
    // #define IS_MCP2515 // Any board with external MCP2515 based extension module. See below to configure the module.
    
    /* Board-specific includes ---------------------------------------------------*/
    
    #if defined(IS_TEENSY_BUILTIN) + defined(IS_ARDUINO_BUILTIN) + defined(IS_MCP2515) != 1
    #warning "Select exactly one hardware option at the top of this file."
    
    #if CAN_HOWMANY > 0 || CANFD_HOWMANY > 0
    #define IS_ARDUINO_BUILTIN
    #warning "guessing that this uses HardwareCAN"
    #else
    #error "cannot guess hardware version"
    #endif
    
    #endif
    
    #ifdef IS_ARDUINO_BUILTIN
    // See https://github.com/arduino/ArduinoCore-API/blob/master/api/HardwareCAN.h
    // and https://github.com/arduino/ArduinoCore-renesas/tree/main/libraries/Arduino_CAN
    
    #include <Arduino_CAN.h>
    #include <ODriveHardwareCAN.hpp>
    #endif // IS_ARDUINO_BUILTIN
    
    #ifdef IS_MCP2515
    // See https://github.com/sandeepmistry/arduino-CAN/
    #include "MCP2515.h"
    #include "ODriveMCPCAN.hpp"
    #endif // IS_MCP2515
    
    #ifdef IS_TEENSY_BUILTIN
    // See https://github.com/tonton81/FlexCAN_T4
    // clone https://github.com/tonton81/FlexCAN_T4.git into /src
    #include <FlexCAN_T4.h>
    #include "ODriveFlexCAN.hpp"
    struct ODriveStatus; // hack to prevent teensy compile error
    #endif // IS_TEENSY_BUILTIN
    
    /* Board-specific settings ---------------------------------------------------*/
    
    /* Teensy */
    
    #ifdef IS_TEENSY_BUILTIN
    
    FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can_intf;
    
    bool setupCan() {
      can_intf.begin();
      can_intf.setBaudRate(CAN_BAUDRATE);
      can_intf.setMaxMB(16);
      can_intf.enableFIFO();
      can_intf.enableFIFOInterrupt();
      can_intf.onReceive(onCanMessage);
      return true;
    }
    
    #endif // IS_TEENSY_BUILTIN
    
    /* MCP2515-based extension modules -*/
    
    #ifdef IS_MCP2515
    
    MCP2515Class& can_intf = CAN;
    
    // chip select pin used for the MCP2515
    #define MCP2515_CS 10
    
    // interrupt pin used for the MCP2515
    // NOTE: not all Arduino pins are interruptable, check the documentation for your board!
    #define MCP2515_INT 2
    
    // freqeuncy of the crystal oscillator on the MCP2515 breakout board. 
    // common values are: 16 MHz, 12 MHz, 8 MHz
    #define MCP2515_CLK_HZ 8000000
    
    static inline void receiveCallback(int packet_size) {
      if (packet_size > 8) {
        return; // not supported
      }
      CanMsg msg = {.id = (unsigned int)CAN.packetId(), .len = (uint8_t)packet_size};
      CAN.readBytes(msg.buffer, packet_size);
      onCanMessage(msg);
    }
    
    bool setupCan() {
      // configure and initialize the CAN bus interface
      CAN.setPins(MCP2515_CS, MCP2515_INT);
      CAN.setClockFrequency(MCP2515_CLK_HZ);
      if (!CAN.begin(CAN_BAUDRATE)) {
        return false;
      }
    
      CAN.onReceive(receiveCallback);
      return true;
    }
    
    #endif // IS_MCP2515
    
    /* Arduinos with built-in CAN */
    
    #ifdef IS_ARDUINO_BUILTIN
    
    HardwareCAN& can_intf = CAN;
    
    bool setupCan() {
      return can_intf.begin((CanBitRate)CAN_BAUDRATE);
    }
    
    #endif
    
    /* Example sketch ------------------------------------------------------------*/
    
    // Instantiate ODrive objects
    ODriveCAN odrv0(wrap_can_intf(can_intf), ODRV0_NODE_ID); // Standard CAN message ID
    ODriveCAN* odrives[] = {&odrv0}; // Make sure all ODriveCAN instances are accounted for here
    
    struct ODriveUserData {
      Heartbeat_msg_t last_heartbeat;
      bool received_heartbeat = false;
      Get_Encoder_Estimates_msg_t last_feedback;
      bool received_feedback = false;
    };
    
    // Keep some application-specific user data for every ODrive.
    ODriveUserData odrv0_user_data;
    
    // Called every time a Heartbeat message arrives from the ODrive
    void onHeartbeat(Heartbeat_msg_t& msg, void* user_data) {
      ODriveUserData* odrv_user_data = static_cast<ODriveUserData*>(user_data);
      odrv_user_data->last_heartbeat = msg;
      odrv_user_data->received_heartbeat = true;
    }
    
    // Called every time a feedback message arrives from the ODrive
    void onFeedback(Get_Encoder_Estimates_msg_t& msg, void* user_data) {
      ODriveUserData* odrv_user_data = static_cast<ODriveUserData*>(user_data);
      odrv_user_data->last_feedback = msg;
      odrv_user_data->received_feedback = true;
    }
    
    // Called for every message that arrives on the CAN bus
    void onCanMessage(const CanMsg& msg) {
      for (auto odrive: odrives) {
        onReceive(msg, *odrive);
      }
    }
    
    void setup() {
      Serial.begin(115200);
    
      // Wait for up to 3 seconds for the serial port to be opened on the PC side.
      // If no PC connects, continue anyway.
      for (int i = 0; i < 30 && !Serial; ++i) {
        delay(100);
      }
      delay(200);
    
      Serial.println("Starting ODriveCAN demo");
    
      // Register callbacks for the heartbeat and encoder feedback messages
      odrv0.onFeedback(onFeedback, &odrv0_user_data);
      odrv0.onStatus(onHeartbeat, &odrv0_user_data);
    
      // Configure and initialize the CAN bus interface. This function depends on
      // your hardware and the CAN stack that you're using.
      if (!setupCan()) {
        Serial.println("CAN failed to initialize: reset required");
        while (true); // spin indefinitely
      }
    
      Serial.println("Waiting for ODrive...");
      while (!odrv0_user_data.received_heartbeat) {
        pumpEvents(can_intf);
        delay(100);
      }
    
      Serial.println("found ODrive");
    
      // request bus voltage and current (1sec timeout)
      Serial.println("attempting to read bus voltage and current");
      Get_Bus_Voltage_Current_msg_t vbus;
      if (!odrv0.request(vbus, 1)) {
        Serial.println("vbus request failed!");
        while (true); // spin indefinitely
      }
    
      Serial.print("DC voltage [V]: ");
      Serial.println(vbus.Bus_Voltage);
      Serial.print("DC current [A]: ");
      Serial.println(vbus.Bus_Current);
    
      Serial.println("Enabling closed loop control...");
      while (odrv0_user_data.last_heartbeat.Axis_State != ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL) {
        odrv0.clearErrors();
        delay(1);
        odrv0.setState(ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL);
    
        // Pump events for 150ms. This delay is needed for two reasons;
        // 1. If there is an error condition, such as missing DC power, the ODrive might
        //    briefly attempt to enter CLOSED_LOOP_CONTROL state, so we can't rely
        //    on the first heartbeat response, so we want to receive at least two
        //    heartbeats (100ms default interval).
        // 2. If the bus is congested, the setState command won't get through
        //    immediately but can be delayed.
        for (int i = 0; i < 15; ++i) {
          delay(10);
          pumpEvents(can_intf);
        }
      }
    
      Serial.println("ODrive running!");
    }
    
    void loop() {
      pumpEvents(can_intf); // This is required on some platforms to handle incoming feedback CAN messages
    
      float SINE_PERIOD = 2.0f; // Period of the position command sine wave in seconds
    
      float t = 0.001 * millis();
      
      float phase = t * (TWO_PI / SINE_PERIOD);
    
      odrv0.setPosition(
        sin(phase), // position
        cos(phase) * (TWO_PI / SINE_PERIOD) // velocity feedforward (optional)
      );
    
      // print position and velocity for Serial Plotter
      if (odrv0_user_data.received_feedback) {
        Get_Encoder_Estimates_msg_t feedback = odrv0_user_data.last_feedback;
        odrv0_user_data.received_feedback = false;
        Serial.print("odrv0-pos:");
        Serial.print(feedback.Pos_Estimate);
        Serial.print(",");
        Serial.print("odrv0-vel:");
        Serial.println(feedback.Vel_Estimate);
      }
    }
    ```

### 6.4 ROS SDK

    The following steps have been tested and verified on Ubuntu 23.04 and ROS2 Iron, but are not supported on MAC or Windows platforms, and have not been verified on other ROS2 versions; corrections may be needed before use.

#### 6.4.1 Install the odrive_can Package

        1. Create a new ROS2 workspace (see [ https://docs.ros.org/en/iron/index.html](https://docs.ros.org/en/iron/index.html))
        2. Use git clone [https://github.com/odriverobotics/odrive_can ](https://github.com/odriverobotics/odrive_can) to download the code into the src directory of the workspace above
        3. In the terminal, go to the root directory of the workspace and run:

        ```text
        colcon build --packages-select odrive_can
        ```

        4. Prepare the environment before running:

        ```text
        source ./install/setup.bash
        ```

        5. Run the example node:

        ```text
        ros2 launch odrive_can example_launch.yaml
        ```

#### 6.4.2 Calling Services and Viewing Messages

        Assume the odrive_can_node node above is running in the namespace odrive_axis0 (this can be set in 。/launch/example_launch.yaml). Once the node is running, you can view the published topic messages, for example:

        ```text
        ros2 topic echo /odrive_axis0/controller_status
        ros2 topic echo /odrive_axis0/odrive_status
        ```

        And call the exposed service interfaces; for example, the following call can start motor calibration:

        ```text
        ros2 service call /odrive_axis0/request_axis_state/odrive_can/srv/AxisState "{axis_requested_state:4}"
        ```

## 7 Motor Drive Attachment Download

📎 **Attachment (not downloaded, see original doc):** 电机驱动附件.zip (Motor Drive Accessories.zip) (84.9 MB)
