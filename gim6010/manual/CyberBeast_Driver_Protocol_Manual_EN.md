# CyberBeast Drive Protocol Manual

> **Unofficial English translation** of the vendor document “守护兽驱动协议手册” (CyberBeast Driver Protocol Manual).
> Source: https://cyberbeast.feishu.cn/docx/BPnQd8reEotLWVxqHFNc9qYZnKh — retrieved and translated 2026-08-12.
> Product: SteadyWin/CyberBeast GIM6010 series motor drive. “Motor Genie” (电机精灵) is the vendor’s Windows GUI tool.
> Large binary attachments (videos, installers, zip archives) were not copied; see the source document for those.


> This manual describes the protocol only. For details on how to switch communication protocols, how to use the interfaces, and how to configure communication parameters, see the Communication Protocol Usage section of [CyberBeast Driver User Manual](CyberBeast_Driver_User_Manual_EN.md)
>

## 1 CAN Simple Protocol

### 1.1 Overview

CANSimple is a simplified CAN communication protocol stack designed to make it easier for developers to perform device-to-device communication over a CAN bus. By simplifying the implementation of the low-level protocol, it lets developers implement sending and receiving of CAN data more quickly, making it especially suitable for embedded systems, robot control, and other CAN-bus-based applications.

CANSimple provides a simple, easy-to-use API interface that hides many of the complex details of the CAN protocol. It supports basic data frame sending and receiving, and is suitable for applications that need simple CAN communication functionality.

**Key features**

- **Simplified communication framework**: CANSimple provides a simple interface for CAN bus data transmission, reducing the low-level protocol details developers need to handle.
- **Data frame sending and receiving**: Supports sending and receiving standard CAN data frames, with flexible message management.
- **Compatibility and cross-platform support**: Compatible with multiple hardware platforms and operating systems, and supports communicating over the CAN bus through common hardware interfaces (such as USB-CAN adapters).

### 1.2 Protocol Frame Format

CAN communication uses the standard frame format: data frame, 11-bit ID, 8 bytes of data, as shown in the table below (MSB on the left, LSB on the right):

| Data Field | CAN ID( 11bits)  |   | Data( 8 bytes)  |
|---|---|---|---|
| Segment | Bit10 ~ Bit5 | Bit4 ~ Bit0 | Byte0 ~ Byte7 |
| Description | node_id | cmd_id | Communication data (data) |

- node_id: represents the unique ID of this motor on the bus. It can be read and set in odrivetool using `odrv0.axis0.config.can.node_id`.
- cmd_id: the command code, indicating the message type of the protocol. See the rest of this section for details.
- Communication data: 8 bytes. The parameters carried in each message are encoded as integers or floating-point numbers, with little-endian byte order. Floating-point numbers are encoded per the IEEE 754 standard (you can test the encoding at [ ](https://www.h-schmidt.net/FloatConverter/IEEE754.html)[Online Hex Converter - IEEE754 Floating-Point to Hex Conversion](http://www.speedfly.cn/tools/hexconvert/)).
For example, suppose the three parameters are: Input_Pos=3.14, Vel_FF=1000 (representing 1 rev/s), Torque_FF=5000 (representing 5 Nm), and the CMD ID of the Set_Input_Pos message is 0x00C. Assume the drive's node (node_id) is set to 0x05. Then:

- 11-bit CAN ID = (0x05<<5) + 0x0C = 0xAC
- According to the description of Set_Input_Pos in section 4.1.2, Input_Pos occupies the 4 bytes starting at byte 0, encoded as C3 F5 48 40 (the float 3.14 is encoded per the IEEE 754 standard as the 32-bit value 0x4048f5c3); Vel_FF occupies the 2 bytes starting at byte 4, encoded as E8 03 (1000 = 0x03E8); Torque_FF occupies the 2 bytes starting at byte 6, encoded as 88 13 (5000 = 0x1388). The 8 bytes of communication data are therefore:

| Byte0 | Byte1 | Byte2 | Byte3 | Byte4 | Byte5 | Byte6 | Byte7 |
|---|---|---|---|---|---|---|---|
| C3 | F5 | 48 | 40 | E8 | 03 | 88 | 13 |

### 1.3 Frame Messages

The table below lists all available messages. The blue CMD ID entries below are hyperlinks — click one to view the detailed protocol.

| CMD ID | Name | Description | Direction | Parameters |
|---|---|---|---|---|
| [0x01](https://bcnyljrhe70u.feishu.cn/docx/BPnQd8reEotLWVxqHFNc9qYZnKh#share-TBa6dgSuyoeoErxugoQch6j3n5c) | Heartbeat | Heartbeat message | Motor -> Host | Axis_Error<br>Axis_State<br>Motor_Flag<br>Encoder_Flag<br>Controller_Flag<br>Traj_Done<br>Life |
| [0x02](https://bcnyljrhe70u.feishu.cn/docx/BPnQd8reEotLWVxqHFNc9qYZnKh#share-FAx6dP2kmoHxyaxbn5UcipMKnyd) | Estop | Emergency stop | Host -> Motor |   |
| [0x03](https://bcnyljrhe70u.feishu.cn/docx/BPnQd8reEotLWVxqHFNc9qYZnKh#share-BKS8d5WtSoY4mix2qvccH59qnwg) | Get_Error | Get detailed error information | Motor -> Host | Error_Type |
| [0x04](https://bcnyljrhe70u.feishu.cn/docx/BPnQd8reEotLWVxqHFNc9qYZnKh#share-X6Ftd98iMomItXxUofOcpa4bnQc) | RxSdo | Access accessible parameters | Motor -> Host |   |
| [0x05](https://bcnyljrhe70u.feishu.cn/docx/BPnQd8reEotLWVxqHFNc9qYZnKh#share-WP9EdqVZOo5ZiJx1jJhcpSZnnAg) | TxSdo |   | Motor -> Host |   |
| [0x06](https://bcnyljrhe70u.feishu.cn/docx/BPnQd8reEotLWVxqHFNc9qYZnKh#share-U7eTd4GrcoIy2PxL9vtcAdnOnZO) | Set_Axis_Node_ID | Set Axis node<br>ID | Host -> Motor | Axis_Node_ID |
| [0x07](https://bcnyljrhe70u.feishu.cn/docx/BPnQd8reEotLWVxqHFNc9qYZnKh#share-MVtsdR9btordvyxqzQlcj7fdnFh) | Set_Axis_State | Set Axis requested state | Host -> Motor | Axis_Requested_State |
| [0x08](https://bcnyljrhe70u.feishu.cn/docx/BPnQd8reEotLWVxqHFNc9qYZnKh#share-YeMUdiZbGo7HrDxhwlMct7MbnTh) | Mit_Control | MIT control | Host -> Motor<br>Motor -> Host |   |
| [0x09](https://bcnyljrhe70u.feishu.cn/docx/BPnQd8reEotLWVxqHFNc9qYZnKh#share-INaadnxKAoqbIfxTrObc6rpVnkb) | Get_Encoder_Estimates | Get encoder estimates | Motor -> Host | Pos_Estimate<br>Vel_Estimate |
| [0x0A](https://bcnyljrhe70u.feishu.cn/docx/BPnQd8reEotLWVxqHFNc9qYZnKh#share-CzDGdT2UfobDujxmwJrcNi6knmB) | Get_Encoder_Count | Get encoder count | Motor -> Host | Shadow_Count<br>Count_In_Cpr |
| [0x0B](https://bcnyljrhe70u.feishu.cn/docx/BPnQd8reEotLWVxqHFNc9qYZnKh#share-GFdMdZ1ekoni2ZxijWOckfLRnMd) | Set_Controller_Mode | Set controller mode | Host -> Motor | Control_Mode<br>Input_Mode |
| [0x0C](https://bcnyljrhe70u.feishu.cn/docx/BPnQd8reEotLWVxqHFNc9qYZnKh#share-SGhzdFQmPoVlIOxmlOmctkeqncd) | Set_Input_Pos | Set input position | Host -> Motor | Input_Pos<br>Vel_FF<br>Torque_FF |
| [0x0D](https://bcnyljrhe70u.feishu.cn/docx/BPnQd8reEotLWVxqHFNc9qYZnKh#share-N08GdBzocozD77x4UdYcf0Mbnqb) | Set_Input_Vel | Set input velocity | Host -> Motor | Input_Vel<br>Torque_FF |
| [0x0E](https://bcnyljrhe70u.feishu.cn/docx/BPnQd8reEotLWVxqHFNc9qYZnKh#share-I9DTdSjBgo7IoAx2klnc5CAvnof) | Set_Input_Torque | Set input torque | Host -> Motor | Input_Torque |
| [0x0F](https://bcnyljrhe70u.feishu.cn/docx/BPnQd8reEotLWVxqHFNc9qYZnKh#share-H8dPdn6RloyAhVxLdUPcl9Xgnff) | Set_Limits | Set velocity and current limits | Host -> Motor | Velocity_Limit<br>Current_Limit |
| 0x10 | Start_Anticogging |   | Host -> Motor |   |
| [0x11](https://bcnyljrhe70u.feishu.cn/docx/BPnQd8reEotLWVxqHFNc9qYZnKh#share-ZDKQdDnFxoJkYixJpjocALRZndb) | Set_Traj_Vel_Limit | Set trapezoidal trajectory mode velocity limit | Host -> Motor | Traj_Vel_Limit |
| [0x12](https://bcnyljrhe70u.feishu.cn/docx/BPnQd8reEotLWVxqHFNc9qYZnKh#share-Fj6zdKjRBoRyNOxCh3Fc083ynWe) | Set_Traj_Accel_Limits | Set trapezoidal trajectory mode acceleration/deceleration limits | Host -> Motor | Traj_Accel_Limit<br>Traj_Decel_Limit |
| [0x13](https://bcnyljrhe70u.feishu.cn/docx/BPnQd8reEotLWVxqHFNc9qYZnKh#share-TBPddmrGZotL9uxfz5AcWdlBnmu) | Set_Traj_Inertia | Set trapezoidal trajectory mode inertia | Host -> Motor | Traj_Inertia |
| [0x14](https://bcnyljrhe70u.feishu.cn/docx/BPnQd8reEotLWVxqHFNc9qYZnKh#share-F0o7dtpNmoZBi8xI4q2cxBEGnId) | Get_Iq | Get IQ | Motor -> Host | Iq_Setpoint<br>Iq_Measured |
| [0x16](https://bcnyljrhe70u.feishu.cn/docx/BPnQd8reEotLWVxqHFNc9qYZnKh#share-CxHpd5NDloAP0Bxh9bocOGuBndc) | Reboot | Reboot motor | Host -> Motor |   |
| [0x17](https://bcnyljrhe70u.feishu.cn/docx/BPnQd8reEotLWVxqHFNc9qYZnKh#share-NraTdqRiko3fMCxRNYlcoNmpn9b) | Get_Bus_Voltage_Current | Get Vbus voltage and Ibus current | Motor -> Host | Bus_Voltage<br>Bus_Current |
| [0x18](https://bcnyljrhe70u.feishu.cn/docx/BPnQd8reEotLWVxqHFNc9qYZnKh#share-WCWSdCwPaokfC8xpMCnc4Pq8nUh) | Clear_Errors | Clear errors | Host -> Motor |   |
| [0x19](https://bcnyljrhe70u.feishu.cn/docx/BPnQd8reEotLWVxqHFNc9qYZnKh#share-NVMCdapUyojoLXxIRDDc9rlnnHe) | Set_Move_Incremental | Set incremental move | Host -> Motor | Displacement |
| [0x1A](https://bcnyljrhe70u.feishu.cn/docx/BPnQd8reEotLWVxqHFNc9qYZnKh#share-QCiVdvE1DoCdVXxZ3rHcZ9a4nMU) | Set_Pos_Gain | Set position loop Kp | Host -> Motor | Pos_Gain |
| [0x1B](https://bcnyljrhe70u.feishu.cn/docx/BPnQd8reEotLWVxqHFNc9qYZnKh#share-BwyddusU7obJMwxxfUSck4XAn3f) | Set_Vel_Gains | Set velocity loop Kp and Ki | Host -> Motor | Vel_Gain<br>Vel_Integrator_Gain |
| [0x1C](https://bcnyljrhe70u.feishu.cn/docx/BPnQd8reEotLWVxqHFNc9qYZnKh#share-PCg4djsssoRO9PxWofnclB51nSc) | Get_Torques | Get torque setpoint and current torque value | Motor -> Host | Torque_Setpoint<br>Torque |
| [0x1D](https://bcnyljrhe70u.feishu.cn/docx/BPnQd8reEotLWVxqHFNc9qYZnKh#share-K2lsd9XiDoQP2ax5wshcJCQ8nNb) | Get_Powers | Get electrical power and mechanical power | Motor -> Host | Electrical_Power<br>Mechanical_Power |
| [0x1E](https://bcnyljrhe70u.feishu.cn/docx/BPnQd8reEotLWVxqHFNc9qYZnKh#share-QEBldFE1yoIfjmxado1cEgd0ncf) | Disable_Can | Disable CAN | Host -> Motor |   |
| [0x1F](https://bcnyljrhe70u.feishu.cn/docx/BPnQd8reEotLWVxqHFNc9qYZnKh#share-Vmw9dK0CqoeoOYxk4mkcaNfmn8b) | Save_Configuration | Save and reboot motor | Host -> Motor |   |

Detailed descriptions of all messages are as follows:

#### **0x01 Heartbeat**

Heartbeat message

CMD ID: 0x01

> Output (Motor -> Host)
>

| Start Byte | Name | Type | Description |
|---|---|---|---|
| byte0~3 | Axis_Error | uint32 | Drive error code (`odrv0.axis0.error`) |
| byte4 | Axis_State | uint8 | Drive state (`odrv0.axis0.current_state`) |
| byte5 | Flags | uint8 | bit0: motor error bit (whether `odrv0.axis0.motor.error` is 0)<br>bit1: encoder error bit (whether `odrv0.axis0.encoder.error` is 0)<br>bit2: controller error bit (whether `odrv0.axis0.controller.error` is 0)<br>bit3: system error bit (whether `odrv0.error` is 0)<br>bit7: `odrv0.axis0.controller.trajectory_done`, i.e. whether the position trapezoidal trajectory has finished executing |
| byte6 | Temperature | int8 | The greater of the motor temperature and the drive board temperature (in degrees Celsius). Maximum temperature value is 127 (=0x7F). |
| byte7 | Life | uint8 | The life value of the periodic message, incremented by 1 with each heartbeat message, in the range 0-255. If this life value is not sequential, it indicates that a heartbeat message was lost, i.e. the communication is unstable. |

#### **0x02 Estop**

Emergency stop

CMD ID: 0x02

> Input (Host -> Motor)
>

|   | byte0~7 |   |   |
|---|---|---|---|
| Data sent | 00 00 00 00 00 00 00 00 |   |   |

This command causes the motor to perform an emergency stop and report the ESTOP_REQUESTED error.

#### **0x03 Get_Error**

Get detailed error information; for error codes, see the [end](https://cyberbeast.feishu.cn/docx/BPnQd8reEotLWVxqHFNc9qYZnKh#KKr4dVPH3oQ9r5x5KxocJLmQn4b) of this document

CMD ID: 0x03

> Input (Host -> Motor)
>

| Start Byte | Name | Type | Description |
|---|---|---|---|
| byte0 | Error_Type | uint8 | 0: Get motor error<br>1: Get encoder error<br>3: Get controller error<br>4: Get system error |

> Output (Motor -> Host)
>

| Start Byte | Name | Type | Description |
|---|---|---|---|
| byte0~7/byte0~3 | Error | uint32/uint64 | The data and length returned depend on the input Error_Type:<br>0: motor error (odrv0.axis0.motor.error), uint64, 8 bytes<br>1: encoder error (odrv0.axis0.encoder.error), uint32, 4 bytes<br>3: controller error (odrv0.axis0.controller.error), uint32, 4 bytes<br>4: system error (odrv0.error), uint32, 4 bytes |

#### **0x04 RxSdo**

Access all parameters; the motor responds to RxSdo read commands via 0x05 TxSdo

CMD ID: 0x04

> Input (Host -> Motor)
>

| Start Byte | Name | Type | Description |
|---|---|---|---|
| byte0 | opcode | uint8 | 0: Read<br>1: Write |
| byte1 | Endpoint_ID | uint16 | Please download the JSON file corresponding to the drive board's hardware/firmware version:<br>• V8 General<br>For drives whose hardware version number starts with V8<br>📎 [endpoints_v8.json](assets_protocol_manual/endpoints_v8.json)<br>• V3 General<br>Currently supports 3.8.3_0.6.5; 3.11.1_0.6.4; 3.12.1_xx;<br>📎 [endpoints_v3.json](assets_protocol_manual/endpoints_v3.json)<br>• 4.1.3<br>📎 [endpoint_4.1.3_0.6.0.json](assets_protocol_manual/endpoint_4.1.3_0.6.0.json)<br>• 4.1.25<br>📎 [endpoint_4.1.25.json](assets_protocol_manual/endpoint_4.1.25.json)<br>• 4.1.37<br>📎 [endpoint_4.1.37_0.6.2.5.json](assets_protocol_manual/endpoint_4.1.37_0.6.2.5.json)<br>📎 [endpoint_4.1.37_0.6.2.4.json](assets_protocol_manual/endpoint_4.1.37_0.6.2.4.json)<br>📎 [endpoint_4.1.37_0.6.2.json](assets_protocol_manual/endpoint_4.1.37_0.6.2.json)<br>• 4.2.51<br>📎 [endpoint_4.2.51_0.6.2.json](assets_protocol_manual/endpoint_4.2.51_0.6.2.json)<br>• 4.2.55<br>📎 [endpoint_4.2.55.json](assets_protocol_manual/endpoint_4.2.55.json)<br>• 3.11.12<br>📎 [endpoint_3.11.12_0.6.1.json](assets_protocol_manual/endpoint_3.11.12_0.6.1.json)<br>📎 [endpoint_3.11.12_0.6.0.json](assets_protocol_manual/endpoint_3.11.12_0.6.0.json)<br>• 3.12.3<br>📎 [endpoint_3.12.3_0.6.4.json](assets_protocol_manual/endpoint_3.12.3_0.6.4.json)<br>• 3.12.43<br>📎 [endpoint_3.12.43_0.6.2.2.json](assets_protocol_manual/endpoint_3.12.43_0.6.2.2.json)<br>📎 [endpoint_3.12.43_0.6.2.0.json](assets_protocol_manual/endpoint_3.12.43_0.6.2.0.json)<br>📎 [endpoint_3.12.43_0.6.0.json](assets_protocol_manual/endpoint_3.12.43_0.6.0.json)<br>• 3.8.3<br>📎 [endpoint_3.8.3_0.5.15.json](assets_protocol_manual/endpoint_3.8.3_0.5.15.json)<br>• SG version 0.6.0<br>📎 [endpoints_0.6.0_SG.json](assets_protocol_manual/endpoints_0.6.0_SG.json)<br>• STW version 0.6.0<br>📎 [endpoint_3.11.1_0.6.0.json](assets_protocol_manual/endpoint_3.11.1_0.6.0.json) |
| byte3 | Reserved | uint8 |   |
| byte4~7 | Value | uint8[4] | Varies depending on the Endpoint_ID; see the description in the JSON file mentioned above. If the Endpoint_ID corresponds to a readable/writable float value, these 4 bytes are the float value encoded per IEEE 754; when opcode=1, this value is written into that float value. |

#### **0x05 TxSdo**

CMD ID: 0x05

Used by the motor to respond to the host's 0x04 RxSdo read command

> Output (Motor -> Host)
>

| Start Byte | Name | Type | Description |
|---|---|---|---|
| byte0 | opcode | uint8 | Fixed at 0 |
| byte1 | Endpoint_ID | uint16 | Please download the JSON file corresponding to the drive board's hardware/firmware version |
| byte3 | Reserved | uint8 |   |
| byte4~7 | Value | uint8[4] | Varies depending on the Endpoint_ID; see the description in the JSON file mentioned above. If the Endpoint_ID corresponds to a readable uint32 value, these 4 bytes are the uint32 in **little-endian byte order**. |

#### **0x06 Set_Axis_Node_ID**

Set the Axis node's node_id

CMD ID: 0x06

> Input (Host -> Motor)
>

| Start Byte | Name | Type | Description |
|---|---|---|---|
| byte0~3 | Axis_Node_ID | uint32 | odrv0.axis0.config.can.node_id<br>The motor's unique ID on the CAN bus |

#### **0x07 Set_Axis_State**

Set the Axis requested state (the state the customer requests the motor to reach; the `odrv0.axis0.current_state` reported in the heartbeat is the motor's actual state)

CMD ID: 0x07

> Input (Host -> Motor)
>

| Start Byte | Name | Type | Description |
|---|---|---|---|
| byte0~3 | Axis_Requested_State | uint32 | `odrv0.axis0.requested_state`<br>• 0: Undefined<br>• 1: Idle<br>• 3: Calibration (motor calibration + encoder calibration)<br>• 4: Motor calibration<br>• 7: Encoder calibration<br>• 8: Closed loop |
#### **0x08 Mit_Control**

MIT Control

CMD ID: 0x08

This is an implementation that emulates the MIT open-source motion control protocol ([https://github.com/mit-biomimetics/Cheetah-Software](https://github.com/mit-biomimetics/Cheetah-Software)).

Note that when controlling via USB, the position, velocity, and torque values entered all refer to the rotor side, whereas when performing MIT control via CAN, the position, velocity, and torque in the protocol all refer to the output shaft side — this is done to keep consistency with the MIT open-source protocol!

For details on how to adjust the `mit_max_xxx` values below, see [ MIT Max Value Range Adjustment](https://bcnyljrhe70u.feishu.cn/docx/N3SMd4QyRobzHkx3wP3cT1qXnpf#share-SPP1dOodbor2PgxhrX2cWiOPnQd). This affects the valid mapping range of the parameters, so please set the maximum value parameters appropriately. For example, if the default `mit_max_pos` is 12.5, the position range is -12.5 to 12.5, and the drive internally maps this range onto 0–65535, which introduces some loss of decimal precision — please keep this in mind.

> Input (Host -> Motor)
>

| CAN Data Frame Bit | Meaning | Description |
|---|---|---|
| BYTE0 | **Position**: 16 bits total, BYTE0 is the high 8 bits, BYTE1 is the low 8 bits<br>Multi-turn position of the output shaft, in radians (RAD), range `-mit_max_pos` to `mit_max_pos` | The actual **position** is a double, which needs to be converted to a 16-bit int. The conversion is:<br>pos_int = (pos_double + `mit_max_pos`)*65535 / (`mit_max_pos`*2)<br>The default value of `mit_max_pos` is 12.5 |
| BYTE1 |   |   |
| BYTE2 | **Velocity**: 12 bits total, BYTE2 is its high 8 bits, BYTE3[7-4] (high 4 bits) is its low 4 bits. Represents the angular velocity of the output shaft, in (RAD/s), range `-mit_max_vel` to `mit_max_vel`<br>**KP value**: 12 bits total, BYTE3[3-0] (low 4 bits) is its high 4 bits, BYTE4 is its low 8 bits. Unit is (Nm/RAD), range 0 to `mit_max_kp` | The actual **velocity** is a double, which needs to be converted to a 12-bit int. The conversion is:<br>vel_int = (vel_double + `mit_max_vel`) * 4095 / (`mit_max_vel`*2)<br>The default value of `mit_max_vel` is 65<br>The actual **KP value** is a double, which needs to be converted to a 12-bit int. The conversion is:<br>kp_int = kp_double * 4095 / `mit_max_kp`<br>The default value of `mit_max_kp` is 500 |
| BYTE3 |   |   |
| BYTE4 |   |   |
| BYTE5 | **KD value**: 12 bits total, BYTE5 is its high 8 bits, BYTE6[7-4] (high 4 bits) is its low 4 bits. Unit is (Nm/RAD/s), range 0 to `mit_max_kd`<br>**Torque**: 12 bits total, BYTE6[3-0] (low 4 bits) is its high 4 bits, BYTE7 is its low 8 bits. Unit is N.m. Range `-mit_max_torque` to `mit_max_torque` | The actual **KD value** is a double, which needs to be converted to a 12-bit int. The conversion is:<br>kd_int = kd_double * 4095 / `mit_max_kd`<br>The default value of `mit_max_kd` is 5<br>The actual **torque** is a double, which needs to be converted to a 12-bit int. The conversion is:<br>t_int = (t_double + `mit_max_torque`) * 4095 / (`mit_max_torque`*2)<br>The default value of `mit_max_torque` is 50<br>The torque constant is in N.m/A |
| BYTE6 |   |   |
| BYTE7 |   |   |

> Output (Motor -> Host)
>

| CAN Data Frame Bit | Meaning | Description |
|---|---|---|
| BYTE0 | node id | Drive node id |
| BYTE1 | **Position**: 16 bits total, BYTE1 is the high 8 bits, BYTE2 is the low 8 bits<br>Multi-turn position of the output shaft, in radians (RAD), range `-mit_max_pos` to `mit_max_pos` | The actual **position** is a double, which needs to be converted to a 16-bit int. The conversion is:<br>pos_int = (pos_double + `mit_max_pos`)*65535 / 25<br>The default value of `mit_max_pos` is the same as above |
| BYTE2 |   |   |
| BYTE3 | **Velocity**: 12 bits total, BYTE3 is its high 8 bits, BYTE4[7-4] (high 4 bits) is its low 4 bits. Represents the angular velocity of the output shaft, in RAD/s, range<br>`-mit_max_vel` to `mit_max_vel`<br>**Torque**: 12 bits total, BYTE4[3-0] (low 4 bits) is its high 4 bits, BYTE5 is its low 8 bits. Unit is N.m. Range `-mit_max_torque` to `mit_max_torque` | The actual **velocity** is a double, which needs to be converted from a 12-bit int. The conversion is:<br>vel_double = vel_int * 130 / 4095 – `mit_max_vel`<br>The default value of `mit_max_vel` is the same as above<br>The actual **torque** is a double, which needs to be converted from a 12-bit int. The conversion is:<br>t_double = t_int * 100 / 4095 - `mit_max_torque`<br>The default value of `mit_max_torque` is the same as above<br>The torque constant is in N.m/A |
| BYTE4 |   |   |
| BYTE5 |   |   |

#### **0x09 Get_Encoder_Estimates**

Get encoder estimates

CMD ID: 0x09

> Output (Motor -> Host)
>

| Start Byte | Name | Type | Unit | Description |
|---|---|---|---|---|
| 0 | Pos_Estimate | float32 | rev (revolutions) | odrv0.axis0.encoder.pos_estimate<br>Current position of the motor's **rotor-side encoder** |
| 4 | Vel_Estimate | float32 | rev/s (revolutions/second) | odrv0.axis0.encoder.vel_estimate<br>Current speed of the motor's **rotor-side encoder** |

#### **0x0A Get_Encoder_Count**

Get encoder count

CMD ID: 0x0A

> Output (Motor -> Host)
>

| Start Byte | Name | Type | Description |
|---|---|---|---|
| 0 | Shadow_Count | int32 | odrv0.axis0.encoder.shadow_count<br>Encoder multi-turn count |
| 4 | Count_In_Cpr | int32 | odrv0.axis0.encoder.count_in_cpr<br>Encoder single-turn count |

#### **0x0B Set_Controller_Mode**

Set controller mode

CMD ID: 0x0B

> Input (Host -> Motor)
>

| Start Byte | Name | Type | Description |
|---|---|---|---|
| 0 | Control_Mode | uint32 | odrv0.axis0.controller.config.control_mode Control mode:<br>0: Voltage control<br>1: Torque control<br>2: Velocity control<br>3: Position control |
| 4 | Input_Mode | uint32 | odrv0.axis0.controller.config.input_mode<br>Input mode, indicating how the user's input control value is used to drive the motor:<br>0: Idle<br>1: Direct control<br>2: Velocity ramp<br>3: Position filter<br>5: Trapezoidal trajectory<br>6: Torque ramp<br>9: Motion control (MIT) |

#### **0x0C Set_Input_Pos**

Set input position

CMD ID: 0x0C

> Input (Host -> Motor)
>

| Start Byte | Name | Type | Unit | Description |
|---|---|---|---|---|
| 0 | Input_Pos | float32 | rev | odrv0.axis0.controller.input_pos<br>Target value for position control. |
| 4 | Vel_FF | int16 | 0.001rev/s | odrv0.axis0.controller.input_vel<br>Velocity feedforward for position control. |
| 6 | Torque_FF | int16 | 0.001Nm | odrv0.axis0.controller.input_torque<br>Torque feedforward for position control. |

#### **0x0D Set_Input_Vel**

Set input velocity

CMD ID: 0x0D

> Input (Host -> Motor)
>

| Start Byte | Name | Type | Unit | Description |
|---|---|---|---|---|
| 0 | Input_Vel | float32 | rev/s | odrv0.axis0.controller.input_vel<br>Target value for velocity control. |
| 4 | Torque_FF | float32 | Nm | odrv0.axis0.controller.input_torque<br>Torque feedforward for velocity control. |

#### **0x0E Set_Input_Torque**

Set input torque

CMD ID: 0x0E

> Input (Host -> Motor)
>

| Start Byte | Name | Type | Unit | Description |
|---|---|---|---|---|
| 0 | Input_Torque | float32 | Nm | odrv0.axis0.controller.input_torque<br>Target value for torque control. |

#### **0x0F Set_Limits**

Set velocity and current limits

CMD ID: 0x0F

> Input (Host -> Motor)
>

| Start Byte | Name | Type | Unit | Description |
|---|---|---|---|---|
| 0 | Velocity_Limit | float32 | rev/s | odrv0.axis0.controller.config.vel_limit<br>Global velocity limit. |
| 4 | Current_Limit | float32 | A | odrv0.axis0.motor.config.current_lim<br>Motor current limit. |

#### **0x10 Start_Anticogging**

CMD ID: 0x10 (Host -> Motor)

Perform torque ripple (anti-cogging) calibration.

|   | byte0~7 |   |   |
|---|---|---|---|
| Send data | 00 00 00 00 00 00 00 00 |   |   |

#### **0x11 Set_Traj_Vel_Limit**

Set trapezoidal trajectory mode velocity limit

CMD ID: 0x11

> Input (Host -> Motor)
>

| Start Byte | Name | Type | Unit | Description |
|---|---|---|---|---|
| 0 | Traj_Vel_Limit | float32 | rev/s | odrv0.axis0.trap_traj.config.vel_limit<br>Velocity limit for trapezoidal position control. |

#### **0x12 Set_Traj_Accel_Limits**

Set trapezoidal trajectory mode acceleration/deceleration limits

CMD ID: 0x12

> Input (Host -> Motor)
>

| Start Byte | Name | Type | Unit | Description |
|---|---|---|---|---|
| 0 | Traj_Accel_Limit | float32 | rev/s^2 | odrv0.axis0.trap_traj.config.accel_limit<br>Acceleration limit for trapezoidal position control. |
| 4 | Traj_Decel_Limit | float32 | rev/s^2 | odrv0.axis0.trap_traj.config.decel_limit<br>Deceleration limit for trapezoidal position control. |

#### **0x13 Set_Traj_Inertia**

Set trapezoidal trajectory mode inertia

CMD ID: 0x13

> Input (Host -> Motor)
>

| Start Byte | Name | Type | Unit | Description |
|---|---|---|---|---|
| 0 | Traj_Inertia | float32 | Nm/(rev/s^2) | odrv0.axis0.controller.config.inertia<br>Inertia. |

#### **0x14 Get_Iq**

Get Iq

CMD ID: 0x14

> Output (Motor -> Host)
>

| Start Byte | Name | Type | Unit | Description |
|---|---|---|---|---|
| 0 | Iq_Setpoint | float32 | A | odrv0.axis0.motor.current_control.Idq_setpoint<br>Q-axis current target value. |
| 4 | Iq_Measured | float32 | A | odrv0.axis0.motor.current_control.Iq_measured<br>Q-axis measured current value. |

#### **0x15 Get_Thermistor_Temperature**

Get temperature (Celsius)

CMD ID: 0x15

> Output (Motor -> Host)
>

| Start Byte | Name | Type | Unit | Description |
|---|---|---|---|---|
| 0 | Motor_Temperature | float32 | A | odrv0.axis0.motor.motor_thermistor.temperature<br>Motor temperature (Celsius). |
| 4 | Fet_Temperature | float32 | A | odrv0.axis0.motor.fet_thermistor.temperature<br>Drive temperature (Celsius). |

#### **0x16 Reboot**

`odrv0.reboot()`

Reboot the motor

CMD ID: 0x16

> Input (Host -> Motor)
>

|   | byte0~7 |   |   |
|---|---|---|---|
| Send data | 00 00 00 00 00 00 00 00 |   |   |

#### **0x17 Get_Bus_Voltage_Current**

Get Vbus voltage and Ibus current

CMD ID: 0x17

> Output (Motor -> Host)
>

| Start Byte | Name | Type | Unit | Description |
|---|---|---|---|---|
| 0 | Bus_Voltage | float32 | V | odrv0.vbus_voltage<br>Bus voltage. |
| 4 | Bus_Current | float32 | A | odrv0.ibus<br>Bus current. |

#### **0x18 Clear_Errors**

`odrv0.clear_errors()`

Clear all errors and faults.

CMD ID: 0x18

> Input (Host -> Motor)
>

|   | byte0~7 |   |   |
|---|---|---|---|
| Send data | 00 00 00 00 00 00 00 00 |   |   |

#### **0x19 Set_Move_Incremental**

Moves the motor by a given increment (`displacement`) relative to its current position. This increment is an offset relative to the motor's current position, not an absolute target position.

CMD ID: 0x19

> Input (Host -> Motor)
>

| Start Byte | Name | Type | Description |   |
|---|---|---|---|---|
| 0 | Displacement | float32  | rev | odrv0.axis0.controller.move_incremental(displacement, 1)<br>Incrementally moves the motor position; displacement is the increment value, in **rotor-side revolutions** |

#### **0x1A Set_Pos_Gain**

Set position loop Kp

CMD ID: 0x1A

> Input (Host -> Motor)
>

| Start Byte | Name | Type | Unit | Description |
|---|---|---|---|---|
| 0 | Pos_Gain | float32 | (rev/s)/rev | odrv0.axis0.controller.config.pos_gain<br>Position loop gain Kp. |

#### **0x1B Set_Vel_Gains**

Set velocity loop Kp and Ki

CMD ID: 0x1B

> Input (Host -> Motor)
>

| Start Byte | Name | Type | Unit | Description |
|---|---|---|---|---|
| 0 | Vel_Gain | float32 | Nm/(rev/s) | odrv0.axis0.controller.config.vel_gain<br>Velocity loop gain Kp. |
| 4 | Vel_Integrator_Gain | float32 | Nm/rev | odrv0.axis0.controller.config.vel_integrator_gain<br>Velocity loop gain Ki. |

#### **0x1C Get_Torques**

Get torque target value and current torque value

CMD ID: 0x1C

> Output (Motor -> Host)
>

| Start Byte | Name | Type | Unit | Description |
|---|---|---|---|---|
| 0 | Torque_Setpoint | float32 | Nm | odrv0.axis0.controller.torque_setpoint<br>Current loop torque target value. |
| 4 | Torque | float32 | Nm | Indicates the current torque value. |

#### **0x1D Get_Powers**

Get electrical power and mechanical power

CMD ID: 0x1D

> Output (Motor -> Host)
>

| Start Byte | Name | Type | Description  |
|---|---|---|---|
| 0 | Electrical_Power | float32 | odrv0.axis0.controller.electrical_power<br>Electrical power |
| 4 | Mechanical_Power | float32 | odrv0.axis0.controller.mechanical_power<br>Mechanical power |

#### **0x1E Disable_Can**

Disable CAN and reboot the drive.

CMD ID: 0x1E

> Input (Host -> Motor)
>

|   | byte0~7 |   |   |
|---|---|---|---|
| Send data | 00 00 00 00 00 00 00 00 |   |   |

#### **0x1F Save_Configuration**

`odrv0.save_configuration()`

Save the current configuration, apply it, and reboot.

CMD ID: 0x1F

> Input (Host -> Motor)
>

|   | byte0~7 |   |   |
|---|---|---|---|
| Send data | 00 00 00 00 00 00 00 00 |   |   |

### 1.4 CAN Protocol in Practice

Assume the motor's node_id = 1:

#### 1.4.1 Practice: Power-On Calibration

The sequence of CAN messages to send is as follows:

| CMD ID | Frame Type | Frame Data | Description |
|---|---|---|---|
| 0x27 | Data frame | 04 00 00 00 00 00 00 00 | Message: Set_Axis_State<br>Parameter: 4<br>Calibrate the motor |
| 0x27 | Data frame | 07 00 00 00 00 00 00 00 | Message: Set_Axis_State<br>Parameter: 7<br>Calibrate the encoder |

#### 1.4.2 Practice: Velocity Control

The sequence of CAN messages to send is as follows:

| CMD ID | Frame Type | Frame Data | Description |
|---|---|---|---|
| 0x2B | Data frame | 02 00 00 00 02 00 00 00 | Message: Set_Controller_Mode<br>Parameter: 2/2<br>Set the control mode to velocity control and the input mode to velocity ramp |
| 0x27 | Data frame | 08 00 00 00 00 00 00 00 | Message: Set_Axis_State<br>Parameter: 8<br>Enter closed-loop control state |
| 0x2D | Data frame | 00 00 20 41 00 00 00 00 | Message: Set_Input_Vel<br>Parameter: 10/0<br>Set the target velocity and torque feedforward, where the target velocity is 10 (float: 0x41200000) and the torque feedforward is 0 (float: 0x00000000) |

#### 1.4.3 Practice: Position Control

The sequence of CAN messages to send is as follows:

| CMD ID | Frame Type | Frame Data | Description |
|---|---|---|---|
| 0x2B | Data frame | 03 00 00 00 03 00 00 00 | Message: Set_Controller_Mode<br>Parameter: 3/3<br>Set the control mode to position control and the input mode to position filter |
| 0x27 | Data frame | 08 00 00 00 00 00 00 00 | Message: Set_Axis_State<br>Parameter: 8<br>Enter closed-loop control state |
| 0x2C | Data frame | CD CC 0C 40 00 00 00 00 | Message: Set_Input_Pos<br>Parameter: 2.2/0/0<br>Set the target position, velocity feedforward, and torque feedforward, where the target position is 2.2 (float: 0x400CCCCD) and the torque feedforward and velocity feedforward are 0 |

#### 

#### 1.4.4 Practice: Rx/TxSdo

Rx/TxSdo is a command that can read and write all parameters or functions inside the drive, enabling free adjustment of parameters under CAN Simple

Generally, RxSdo is used for Host -> Motor, and TxSdo is used for Motor -> Host.

    This practice demonstrates the use of Rx/TxSdo by reading the bus voltage, writing the motor gear ratio, and reading it back to verify

    Based on endpoint_V3, node_id =1. The [endpoint.json file](https://cyberbeast.feishu.cn/docx/BPnQd8reEotLWVxqHFNc9qYZnKh#share-EgmodaGUVoegCBxiVAtcflzxn1b) for the corresponding version number can be found at 0x04 RxSdo

    Read bus voltage:

    | CMD ID | Frame Type | Frame Data | Description |
    |---|---|---|---|
    | 24 | Data frame | 00 02 00 00 00 00 00 00 | Message: RxSdo, read bus voltage<br>Parameters:<br>00 Read data<br>02 00 Little-endian hex ID of bus voltage (vbus_voltage) in the endpoint<br>00 Reserved bit<br>00 00 00 00 Data to read, just write 0 |

Write gear ratio 9.25 and read back to verify:

| CMD ID | Frame Type | Frame Data | Description |
|---|---|---|---|
| 24 | Data frame | 01 F2 00 00 00 00 14 41 | Message: RxSdo, write gear ratio 9.25<br>Parameters:<br>01: Write data<br>F2 00: Little-endian hex ID of gear ratio (gear_ratio) in the endpoint<br>00: Reserved bit<br>00 00 14 41: Little-endian hex representation of the float 9.25 |
| 24 | Data frame | 00 F2 00 00 00 00 00 00 | Message: RxSdo, read gear ratio. Parameter meaning is the same as <Read bus voltage> |

## 2 RS485 Protocol (Modbus RTU)

### 2.1 Overview

**Communication Physical Layer: RS485**

RS485 is a differential-signal serial communication standard, featuring strong noise immunity, long communication distance (up to 1200 meters), and support for multiple nodes (up to 32 nodes). Since RS485 uses half-duplex communication, all devices share the same two communication lines (A, B) and must take turns sending data through a master-slave mechanism to avoid conflicts.

**Communication Protocol Layer: Modbus RTU**

Modbus RTU is a serial communication protocol widely used in industrial applications, adopting a master-slave architecture: the master actively initiates requests, and the slave responds passively. Each data frame has a fixed format and supports reliable read/write operations, making it suitable for controlling and monitoring devices (such as motor drives).

**Frame Format (Modbus RTU Standard Frame Structure)**

| **Field** | **Length** | **Description** |
|---|---|---|
| Slave address | 1 byte | Uniquely identifies a slave device (1 to 247)<br>This identifier is also called the station number, and can be changed via `odrv0.axis0.config.can.node_id` |
| Function code | 1 byte | Indicates the type of operation being performed (such as reading/writing registers, etc.) |
| Data field | N bytes | Depending on the function code, the content may be an address, data, etc. |
| CRC checksum | 2 bytes | Cyclic redundancy check, low byte first, high byte last; for details on generating the Modbus RTU CRC checksum, see [ cht.nahua.com](https://cht.nahua.com.tw/software/crc16/index.php)[.](https://cht.nahua.com.tw/software/crc16/index.php)[tw](https://cht.nahua.com.tw/software/crc16/index.php) |

### 2.2 Modbus RTU

Based on the industry-standard Modbus RTU, the currently supported function codes are as follows:

| Function Code | Meaning |
|---|---|
| 0x04 | Read one or more input registers |
| 0x03 | Read multiple holding registers |
| 0x06 | Write a single holding register |
| 0x10 | Write multiple holding registers |
### 2.3 Input Registers

Input registers are read-only and cannot be written; they represent certain system states or dynamic metrics, such as error information, voltage/current, etc. The currently supported input registers are as follows:

#### **0x01: Motor state**

Host request (`odrv0.axis0.request_state`):

| Address | Function code | Register address | Register count | Checksum |
|---|---|---|---|---|
| 1 byte | 1 byte | 2 bytes | 2 bytes | 2 bytes |
| XX | 04 | 00 01 | 00 01 | XX XX |

Motor response (`odrv0.axis0.current_state`):

| Address | Function code | Byte count | Register value | Checksum |
|---|---|---|---|---|
| 1 byte | 1 byte | 1 byte | 2 bytes | 2 bytes |
| XX | 04 | 02 | State0 State1 | XX XX |

Here State0 is always 0, and State1 is the motor state code:

| Motor state code | Description |
|---|---|
| 1 | Idle |
| 3 | Full calibration |
| 4 | Motor calibration |
| 7 | Encoder calibration |
| 8 | Closed-loop running |
| 11 | Homing |

#### **0x02: Fault flags**

Host request:

| Address | Function code | Register address | Register count | Checksum |
|---|---|---|---|---|
| 1 byte | 1 byte | 2 bytes | 2 bytes | 2 bytes |
| XX | 04 | 00 02 | 00 01 | XX XX |

Motor response:

| Address | Function code | Byte count | Register value | Checksum |
|---|---|---|---|---|
| 1 byte | 1 byte | 1 byte | 2 bytes | 2 bytes |
| XX | 04 | 02 | Flag0 Flag1 | XX XX |

Here Flag0 is always 0, and Flag1 is the fault flag, where each bit set to 1 indicates the presence of the corresponding fault. The bit meanings are as follows:

| bit7~bit5 | bit4 | bit3 | bit2 | bit1 | bit0 |
|---|---|---|---|---|---|
| No meaning | Drive fault bit (whether `odrv0.axis0.error` is nonzero) | System fault bit (whether `odrv0.error` is nonzero) | Controller fault bit (whether `odrv0.axis0.controller.error` is nonzero) | Encoder fault bit (whether `odrv0.axis0.encoder.error` is nonzero) | Motor fault bit (whether `odrv0.axis0.motor.error` is nonzero) |

#### **0x03 0x04: Drive fault code 1, Drive fault code 2**

`odrv0.axis0.error`

The drive fault code is a 32-bit unsigned integer, where Drive fault code 1 is its upper 16 bits and Drive fault code 2 is its lower 16 bits. The user can only obtain the complete drive fault code by accessing register address 0x03 and retrieving both register values at the same time. Neither register may be accessed individually.

Host request:

| Address | Function code | Register address | Register count | Checksum |
|---|---|---|---|---|
| 1 byte | 1 byte | 2 bytes | 2 bytes | 2 bytes |
| XX | 04 | 00 03 | 00 02 | XX XX |

Motor response:

| Address | Function code | Byte count | Register value | Checksum |
|---|---|---|---|---|
| 1 byte | 1 byte | 1 byte | 2 bytes | 2 bytes |
| XX | 04 | 04 | E0 E1 E2 E3 | XX XX |

E0 E1 E2 E3 together make up the drive fault code; see the [fault code table](https://cyberbeast.feishu.cn/docx/BPnQd8reEotLWVxqHFNc9qYZnKh#share-PC6HdxK3ioSdO1xWig5cjx0mnkh).

#### **0x05 0x06 0x07 0x08: Motor fault code 1, Motor fault code 2, Motor fault code 3, Motor fault code 4**

`odrv0.axis0.motor.error`

The motor fault code is a 64-bit unsigned integer, where Motor fault code 1 is bits 63 to 48, Motor fault code 2 is bits 47 to 32, Motor fault code 3 is bits 31 to 16, and Motor fault code 4 is bits 15 to 0. The user can only obtain the complete motor fault code by accessing register address 0x05 and retrieving all four register values at the same time. No single register may be accessed individually.

Host request:

| Address | Function code | Register address | Register count | Checksum |
|---|---|---|---|---|
| 1 byte | 1 byte | 2 bytes | 2 bytes | 2 bytes |
| XX | 04 | 00 05 | 00 04 | XX XX |

Motor response:

| Address | Function code | Byte count | Register value | Checksum |
|---|---|---|---|---|
| 1 byte | 1 byte | 1 byte | 8 bytes | 2 bytes |
| XX | 04 | 08 | E0 E1 E2 E3 E4 E5 E6 E7 | XX XX |

E0 E1 E2 E3 E4 E5 E6 E7 together make up the motor fault code; see the fault code table in section 5.2.

#### **0x09 0x0A: Encoder fault code 1, Encoder fault code 2**

`odrv0.axis0.encoder.error`

The encoder fault code is a 32-bit unsigned integer, where Encoder fault code 1 is its upper 16 bits and Encoder fault code 2 is its lower 16 bits. The user can only obtain the complete encoder fault code by accessing register address 0x09 and retrieving both register values at the same time. Neither register may be accessed individually.

Host request:

| Address | Function code | Register address | Register count | Checksum |
|---|---|---|---|---|
| 1 byte | 1 byte | 2 bytes | 2 bytes | 2 bytes |
| XX | 04 | 00 09 | 00 02 | XX XX |

Motor response:

| Address | Function code | Byte count | Register value | Checksum |
|---|---|---|---|---|
| 1 byte | 1 byte | 1 byte | 2 bytes | 2 bytes |
| XX | 04 | 04 | E0 E1 E2 E3 | XX XX |

E0 E1 E2 E3 together make up the encoder fault code; see the fault code table in section 5.2.

#### **0x0B 0x0C: Controller fault code 1, Controller fault code 2**

`odrv0.axis0.controller.error`

The controller fault code is a 32-bit unsigned integer, where Controller fault code 1 is its upper 16 bits and Controller fault code 2 is its lower 16 bits. The user can only obtain the complete controller fault code by accessing register address 0x0B and retrieving both register values at the same time. Neither register may be accessed individually.

Host request:

| Address | Function code | Register address | Register count | Checksum |
|---|---|---|---|---|
| 1 byte | 1 byte | 2 bytes | 2 bytes | 2 bytes |
| XX | 04 | 00 0B | 00 02 | XX XX |

Motor response:

| Address | Function code | Byte count | Register value | Checksum |
|---|---|---|---|---|
| 1 byte | 1 byte | 1 byte | 2 bytes | 2 bytes |
| XX | 04 | 04 | E0 E1 E2 E3 | XX XX |

E0 E1 E2 E3 together make up the controller fault code; see the fault code table in section 5.2.

#### **0x0D 0x0E: System fault code 1, System fault code 2**

`odrv0.error`

The system fault code is a 32-bit unsigned integer, where System fault code 1 is its upper 16 bits and System fault code 2 is its lower 16 bits. The user can only obtain the complete system fault code by accessing register address 0x0D and retrieving both register values at the same time. Neither register may be accessed individually.

Host request:

| Address | Function code | Register address | Register count | Checksum |
|---|---|---|---|---|
| 1 byte | 1 byte | 2 bytes | 2 bytes | 2 bytes |
| XX | 04 | 00 0D | 00 02 | XX XX |

Motor response:

| Address | Function code | Byte count | Register value | Checksum |
|---|---|---|---|---|
| 1 byte | 1 byte | 1 byte | 2 bytes | 2 bytes |
| XX | 04 | 04 | E0 E1 E2 E3 | XX XX |

E0 E1 E2 E3 together make up the system fault code; see the fault code table in section 5.2.

#### **0x0F 0x10: Position feedback 1, Position feedback 2**

`odrv0.axis0.encoder.pos_estimate`

Position feedback is a 32-bit floating-point number, where Position feedback 1 is its upper 16 bits and Position feedback 2 is its lower 16 bits. The user can only obtain the complete position feedback value by accessing register address 0x0F and retrieving both register values at the same time. Neither register may be accessed individually.

Host request:

| Address | Function code | Register address | Register count | Checksum |
|---|---|---|---|---|
| 1 byte | 1 byte | 2 bytes | 2 bytes | 2 bytes |
| XX | 04 | 00 0F | 00 02 | XX XX |

Motor response:

| Address | Function code | Byte count | Register value | Checksum |
|---|---|---|---|---|
| 1 byte | 1 byte | 1 byte | 2 bytes | 2 bytes |
| XX | 04 | 04 | P0 P1 P2 P3 | XX XX |

P0 P1 P2 P3 together make up the position feedback value, where the 32-bit floating-point number is encoded according to the IEEE 754 standard (you can test the encoding via the website [ https://www.h-schmidt.net/FloatConverter/IEEE754.html ](https://www.h-schmidt.net/FloatConverter/IEEE754.html)).

#### **0x11 0x12: Velocity feedback 1, Velocity feedback 2**

`odrv0.axis0.encoder.vel_estimate`

Velocity feedback is a 32-bit floating-point number, where Velocity feedback 1 is its upper 16 bits and Velocity feedback 2 is its lower 16 bits. The user can only obtain the complete velocity feedback value by accessing register address 0x11 and retrieving both register values at the same time. Neither register may be accessed individually.

Host request:

| Address | Function code | Register address | Register count | Checksum |
|---|---|---|---|---|
| 1 byte | 1 byte | 2 bytes | 2 bytes | 2 bytes |
| XX | 04 | 00 11 | 00 02 | XX XX |

Motor response:

| Address | Function code | Byte count | Register value | Checksum |
|---|---|---|---|---|
| 1 byte | 1 byte | 1 byte | 2 bytes | 2 bytes |
| XX | 04 | 04 | V0 V1 V2 V3 | XX XX |

V0 V1 V2 V3 together make up the velocity feedback value, where the 32-bit floating-point number is encoded according to the IEEE 754 standard (you can test the encoding via the website [ https://www.h-schmidt.net/FloatConverter/IEEE754.html ](https://www.h-schmidt.net/FloatConverter/IEEE754.html)).

#### **0x13 0x14: Bus voltage feedback 1, Bus voltage feedback 2**

`odrv0.vbus_voltage`

Bus voltage is a 32-bit floating-point number, where Bus voltage 1 is its upper 16 bits and Bus voltage 2 is its lower 16 bits. The user can only obtain the complete bus voltage value by accessing register address 0x13 and retrieving both register values at the same time. Neither register may be accessed individually.

Host request:

| Address | Function code | Register address | Register count | Checksum |
|---|---|---|---|---|
| 1 byte | 1 byte | 2 bytes | 2 bytes | 2 bytes |
| XX | 04 | 00 13 | 00 02 | XX XX |

Motor response:

| Address | Function code | Byte count | Register value | Checksum |
|---|---|---|---|---|
| 1 byte | 1 byte | 1 byte | 2 bytes | 2 bytes |
| XX | 04 | 04 | V0 V1 V2 V3 | XX XX |

V0 V1 V2 V3 together make up the bus voltage value, where the 32-bit floating-point number is encoded according to the IEEE 754 standard (you can test the encoding via the website [ https://www.h-schmidt.net/FloatConverter/IEEE754.html ](https://www.h-schmidt.net/FloatConverter/IEEE754.html)).

#### **0x15 0x16: Bus current feedback 1, Bus current feedback 2**

`odrv0.ibus`

Bus current feedback is a 32-bit floating-point number, where Bus current feedback 1 is its upper 16 bits and Bus current feedback 2 is its lower 16 bits. The user can only obtain the complete bus current feedback value by accessing register address 0x15 and retrieving both register values at the same time. Neither register may be accessed individually.

Host request:

| Address | Function code | Register address | Register count | Checksum |
|---|---|---|---|---|
| 1 byte | 1 byte | 2 bytes | 2 bytes | 2 bytes |
| XX | 04 | 00 15 | 00 02 | XX XX |

Motor response:

| Address | Function code | Byte count | Register value | Checksum |
|---|---|---|---|---|
| 1 byte | 1 byte | 1 byte | 2 bytes | 2 bytes |
| XX | 04 | 04 | I0 I1 I2 I3 | XX XX |

I0 I1 I2 I3 together make up the bus current feedback value, where the 32-bit floating-point number is encoded according to the IEEE 754 standard (you can test the encoding via the website [ https://www.h-schmidt.net/FloatConverter/IEEE754.html ](https://www.h-schmidt.net/FloatConverter/IEEE754.html)).

#### **0x17 0x18: Q-axis current setpoint 1, Q-axis current setpoint 2**

`odrv0.axis0.motor.current_control.Idq_setpoint`

The Q-axis current setpoint is a 32-bit floating-point number, where Q-axis current setpoint 1 is its upper 16 bits and Q-axis current setpoint 2 is its lower 16 bits. The user can only obtain the complete Q-axis current setpoint value by accessing register address 0x17 and retrieving both register values at the same time. Neither register may be accessed individually.

Host request:

| Address | Function code | Register address | Register count | Checksum |
|---|---|---|---|---|
| 1 byte | 1 byte | 2 bytes | 2 bytes | 2 bytes |
| XX | 04 | 00 17 | 00 02 | XX XX |

Motor response:

| Address | Function code | Byte count | Register value | Checksum |
|---|---|---|---|---|
| 1 byte | 1 byte | 1 byte | 2 bytes | 2 bytes |
| XX | 04 | 04 | I0 I1 I2 I3 | XX XX |

I0 I1 I2 I3 together make up the Q-axis current setpoint value, where the 32-bit floating-point number is encoded according to the IEEE 754 standard (you can test the encoding via the website [ https://www.h-schmidt.net/FloatConverter/IEEE754.html ](https://www.h-schmidt.net/FloatConverter/IEEE754.html)).

#### **0x19 0x1A: Q-axis current feedback 1, Q-axis current feedback 2**

`odrv0.axis0.motor.current_control.Iq_measured`

Q-axis current feedback is a 32-bit floating-point number, where Q-axis current feedback 1 is its upper 16 bits and Q-axis current feedback 2 is its lower 16 bits. The user can only obtain the complete Q-axis current feedback value by accessing register address 0x19 and retrieving both register values at the same time. Neither register may be accessed individually.

Host request:

| Address | Function code | Register address | Register count | Checksum |
|---|---|---|---|---|
| 1 byte | 1 byte | 2 bytes | 2 bytes | 2 bytes |
| XX | 04 | 00 19 | 00 02 | XX XX |

Motor response:

| Address | Function code | Byte count | Register value | Checksum |
|---|---|---|---|---|
| 1 byte | 1 byte | 1 byte | 2 bytes | 2 bytes |
| XX | 04 | 04 | I0 I1 I2 I3 | XX XX |

I0 I1 I2 I3 together make up the Q-axis current feedback value, where the 32-bit floating-point number is encoded according to the IEEE 754 standard (you can test the encoding via the website [ https://www.h-schmidt.net/FloatConverter/IEEE754.html ](https://www.h-schmidt.net/FloatConverter/IEEE754.html)).

#### **0x1B 0x1C: Torque feedback 1, Torque feedback 2**

`odrv0.axis0.motor.current_control.Iq_measured * odrv0.axis0.motor.config.torque_constant`

Torque feedback is a 32-bit floating-point number, where Torque feedback 1 is its upper 16 bits and Torque feedback 2 is its lower 16 bits. The user can only obtain the complete torque feedback value by accessing register address 0x1B and retrieving both register values at the same time. Neither register may be accessed individually.

Host request:

| Address | Function code | Register address | Register count | Checksum |
|---|---|---|---|---|
| 1 byte | 1 byte | 2 bytes | 2 bytes | 2 bytes |
| XX | 04 | 00 1B | 00 02 | XX XX |

Motor response:

| Address | Function code | Byte count | Register value | Checksum |
|---|---|---|---|---|
| 1 byte | 1 byte | 1 byte | 2 bytes | 2 bytes |
| XX | 04 | 04 | T0 T1 T2 T3 | XX XX |

T0 T1 T2 T3 together make up the torque feedback value, where the 32-bit floating-point number is encoded according to the IEEE 754 standard (you can test the encoding via the website [ https://www.h-schmidt.net/FloatConverter/IEEE754.html ](https://www.h-schmidt.net/FloatConverter/IEEE754.html)).

#### **0x1D: Motor temperature**

`odrv0.axis0.motor.motor_thermistor.temperature`

Motor temperature is a 16-bit signed integer, whose value equals the temperature in Celsius multiplied by 10.

Host request:

| Address | Function code | Register address | Register count | Checksum |
|---|---|---|---|---|
| 1 byte | 1 byte | 2 bytes | 2 bytes | 2 bytes |
| XX | 04 | 00 1D | 00 01 | XX XX |

Motor response:

| Address | Function code | Byte count | Register value | Checksum |
|---|---|---|---|---|
| 1 byte | 1 byte | 1 byte | 2 bytes | 2 bytes |
| XX | 04 | 02 | T0 T1 | XX XX |

T0 T1 together make up the 16-bit signed integer motor temperature value.

#### **0x1E: Drive temperature**

`odrv0.axis0.motor.fet_thermistor.temperature`

Drive temperature is a 16-bit signed integer, whose value equals the temperature in Celsius multiplied by 10.

Host request:

| Address | Function code | Register address | Register count | Checksum |
|---|---|---|---|---|
| 1 byte | 1 byte | 2 bytes | 2 bytes | 2 bytes |
| XX | 04 | 00 1D | 00 01 | XX XX |

Motor response:

| Address | Function code | Byte count | Register value | Checksum |
|---|---|---|---|---|
| 1 byte | 1 byte | 1 byte | 2 bytes | 2 bytes |
| XX | 04 | 02 | T0 T1 | XX XX |

T0 T1 together make up the 16-bit signed integer drive temperature value.

#### **0x31 0x32: Arbitrary endpoint access return value 1, Arbitrary endpoint access return value 2**

Similar to RxSdo in CANSimple.

Arbitrary endpoint access refers to the user's ability to read/write any supported parameter. The so-called "endpoint" refers to a parameter, and each parameter has a unique endpoint number. When the user wants to read a parameter, the endpoint number can be obtained from the endpoint description file, written into holding register 0xA1, and then input register 0x31 can be read to obtain that parameter's value. Refer to the description of holding register 0xA1 for the address from which to download the endpoint description file.

The arbitrary endpoint access return value is a 32-bit value, which may be a 16-bit integer, a 32-bit integer, or a 32-bit floating-point number, where Arbitrary endpoint access return value 1 is its upper 16 bits and Arbitrary endpoint access return value 2 is its lower 16 bits. The user can only obtain the complete return value (parameter value) by accessing register address 0x31 and reading both register values at the same time. Neither register may be accessed individually.

Host request:

| Address | Function code | Register address | Register count | Checksum |
|---|---|---|---|---|
| 1 byte | 1 byte | 2 bytes | 2 bytes | 2 bytes |
| XX | 04 | 00 31 | 00 02 | XX XX |

Motor response:

| Address | Function code | Byte count | Register value | Checksum |
|---|---|---|---|---|
| 1 byte | 1 byte | 1 byte | 2 bytes | 2 bytes |
| XX | 04 | 04 | V0 V1 V2 V3 | XX XX |

Here V0 V1 V2 V3 are the 4 bytes of the arbitrary endpoint access return value from high to low, where the 32-bit floating-point number is encoded according to the IEEE 754 standard (you can test the encoding via the website [ https://www.h-schmidt.net/FloatConverter/IEEE754.html ](https://www.h-schmidt.net/FloatConverter/IEEE754.html)).

### 2.4 Holding Registers

Holding registers are both readable and writable. The currently supported registers and their functions are as follows:

#### 0x5C 0x5D 0x5E 0x5F: Serial number

`odrv0.config.custom_serial_number`

The default product serial number, 8 bytes. When writing this serial number, the user must ensure its uniqueness and exclusivity. The user can only obtain the complete serial number by accessing register address 0x5C and retrieving all four register values at the same time. No single register may be accessed individually.

Write serial number request:

| Address | Function code | Register address | Register count | Byte count | Register value | Checksum |
|---|---|---|---|---|---|---|
| 1 byte | 1 byte | 2 bytes | 2 bytes | 1 byte | 8 bytes | 2 bytes |
| XX | 10 | 00 5C | 00 04 | 08 | S0 S1 S2 S3 S4 S5 S6 S7 | XX XX |

Motor response:

| Address | Function code | Register address | Register count | Checksum |
|---|---|---|---|---|
| 1 byte | 1 byte | 2 bytes | 2 bytes | 2 bytes |
| XX | 10 | 00 5C | 00 04 | XX XX |

Here S0 ~ S7 are the byte sequence of the serial number (from high to low).

Read serial number request:

| Address | Function code | Register address | Register count | Checksum |
|---|---|---|---|---|
| 1 byte | 1 byte | 2 bytes | 2 bytes | 2 bytes |
| XX | 03 | 00 5C | 00 04 | XX XX |

Motor response:

| Address | Function code | Byte count | Register value | Checksum |
|---|---|---|---|---|
| 1 byte | 1 byte | 1 byte | 8 bytes | 2 bytes |
| XX | 03 | 08 | S0 S1 S2 S3 S4 S5 S6 S7 | XX XX |

#### **0x61: Emergency stop**

Writing any value to this register will cause the motor to perform an emergency stop.

Host request:

| Address | Function code | Register address | Register value | Checksum |
|---|---|---|---|---|
| 1 byte | 1 byte | 2 bytes | 2 bytes | 2 bytes |
| XX | 06 | 00 61 | XX XX | XX XX |

Motor response:

| Address | Function code | Register address | Register value | Checksum |
|---|---|---|---|---|
| 1 byte | 1 byte | 2 bytes | 2 bytes | 2 bytes |
| XX | 06 | 00 61 | XX XX | XX XX |

#### **0x62: Node ID**

`odrv0.axis0.config.can.node_id`

Use function code 06 to write the node ID to this register, or use function code 03 to read the current node ID.

Write node ID request:

| Address | Function code | Register address | Register value | Checksum |
|---|---|---|---|---|
| 1 byte | 1 byte | 2 bytes | 2 bytes | 2 bytes |
| XX | 06 | 00 62 | ID0 ID1 | XX XX |

Motor response:

| Address | Function code | Register address | Register value | Checksum |
|---|---|---|---|---|
| 1 byte | 1 byte | 2 bytes | 2 bytes | 2 bytes |
| XX | 06 | 00 62 | ID0 ID1 | XX XX |

Here ID0 and ID1 are the upper 8 bits and lower 8 bits of the new node ID.

Read current node ID request:

| Address | Function code | Register address | Register count | Checksum |
|---|---|---|---|---|
| 1 byte | 1 byte | 2 bytes | 2 bytes | 2 bytes |
| XX | 03 | 00 62 | 00 01 | XX XX |

Motor response:

| Address | Function code | Byte count | Register value | Checksum |
|---|---|---|---|---|
| 1 byte | 1 byte | 1 byte | 2 bytes | 2 bytes |
| XX | 03 | 02 | ID0 ID1 | XX XX |

#### **0x63: State control**

`odrv0.axis0.requested_state`

Use function code 06 to write the motor state (changing the motor state) to this register.

Host request:

| Address | Function code | Register address | Register value | Checksum |
|---|---|---|---|---|
| 1 byte | 1 byte | 2 bytes | 2 bytes | 2 bytes |
| XX | 06 | 00 63 | S0 S1 | XX XX |

Motor response:

| Address | Function code | Register address | Register value | Checksum |
|---|---|---|---|---|
| 1 byte | 1 byte | 2 bytes | 2 bytes | 2 bytes |
| XX | 06 | 00 63 | S0 S1 | XX XX |

Here S0 and S1 are the upper 8 bits and lower 8 bits of the motor state. The state definitions are as follows:

| Motor state code | Description |
|---|---|
| 1 | Idle |
| 3 | Full calibration |
| 4 | Motor calibration |
| 7 | Encoder calibration |
| 8 | Closed-loop running |
| 11 | Homing |

For example, setting the motor state to 8 (closed-loop running) is what is generally referred to as "turning on the motor"; setting the motor state to 1 (idle) is what is generally referred to as "turning off the motor".
#### **0x64: Control Mode**

`odrv0.axis0.controller.config.control_mode`

The user can use function code 06 to modify the value of this register to enter torque control, velocity control, or position control mode, or use function code 03 to read the current control mode.

Request to modify the control mode:

| Address | Function Code | Register Address | Register Value | Checksum |
|---|---|---|---|---|
| 1 byte | 1 byte | 2 bytes | 2 bytes | 2 bytes |
| XX | 06 | 00 64 | M0 M1 | XX XX |

Motor response:

| Address | Function Code | Register Address | Register Value | Checksum |
|---|---|---|---|---|
| 1 byte | 1 byte | 2 bytes | 2 bytes | 2 bytes |
| XX | 06 | 00 64 | M0 M1 | XX XX |

Request to read the control mode:

| Address | Function Code | Register Address | Register Count | Checksum |
|---|---|---|---|---|
| 1 byte | 1 byte | 2 bytes | 2 bytes | 2 bytes |
| XX | 03 | 00 64 | 00 01 | XX XX |

Motor response:

| Address | Function Code | Byte Count | Register Value | Checksum |
|---|---|---|---|---|
| 1 byte | 1 byte | 1 byte | 2 bytes | 2 bytes |
| XX | 03 | 02 | M0 M1 | XX XX |

Where M0 and M1 are the high 8 bits and low 8 bits of the control mode. The control modes are defined as follows:

| Control Mode | Description |
|---|---|
| 1 | Torque control |
| 2 | Velocity control |
| 3 | Position control |

#### **0x65: Input Mode**

`odrv0.axis0.controller.config.input_mode`

The user can use function code 06 to modify the value of this register to select a different target value input mode, such as direct input or filtered input, or use function code 03 to read the current input mode.

Request to modify the input mode:

| Address | Function Code | Register Address | Register Value | Checksum |
|---|---|---|---|---|
| 1 byte | 1 byte | 2 bytes | 2 bytes | 2 bytes |
| XX | 06 | 00 65 | M0 M1 | XX XX |

Motor response:

| Address | Function Code | Register Address | Register Value | Checksum |
|---|---|---|---|---|
| 1 byte | 1 byte | 2 bytes | 2 bytes | 2 bytes |
| XX | 06 | 00 65 | M0 M1 | XX XX |

Request to read the input mode:

| Address | Function Code | Register Address | Register Count | Checksum |
|---|---|---|---|---|
| 1 byte | 1 byte | 2 bytes | 2 bytes | 2 bytes |
| XX | 03 | 00 65 | 00 01 | XX XX |

Motor response:

| Address | Function Code | Byte Count | Register Value | Checksum |
|---|---|---|---|---|
| 1 byte | 1 byte | 1 byte | 2 bytes | 2 bytes |
| XX | 03 | 02 | M0 M1 | XX XX |

Where M0 and M1 are the high 8 bits and low 8 bits of the input mode. The input modes are defined as follows:

| Control Mode | Description | Detailed Explanation |
|---|---|---|
| 1 | Direct input | The target value entered by the user is used directly as the drive's target value |
| 2 | Velocity ramp | The drive's velocity target value will reach the user-input velocity target value within a certain time at a certain rate, used only for velocity control |
| 3 | Position filter | The drive's position target value is the value obtained after filtering the user-input position target value, used only for position control |
| 5 | Trapezoidal trajectory | The drive will move the motor to the user-input target value following a trapezoidal velocity profile, in which the rising slope (acceleration), falling slope (deceleration), and cruising velocity of the trapezoid can all be set; please refer to registers 0x70–0x75 below |
| 6 | Torque ramp | The drive's torque target value will reach the user-input target torque value within a certain time at a certain rate, used only for torque control |

#### **0x66 0x67: Position Control Target 1, Position Control Target 2**

`odrv0.axis0.controller.input_pos`

The position control target is a 32-bit floating-point number, of which position control target 1 is its high 16 bits and position control target 2 is its low 16 bits. The user can only obtain/modify the complete position control target by accessing register address 0x66 and reading/writing both registers' data at the same time. Neither register can be accessed individually.

Request to modify/update the position control target:

| Address | Function Code | Register Address | Register Count | Byte Count | Register Value | Checksum |
|---|---|---|---|---|---|---|
| 1 byte | 1 byte | 2 bytes | 2 bytes | 1 byte | 4 bytes | 2 bytes |
| XX | 10 | 00 66 | 00 02 | 04 | P0 P1 P2 P3 | XX XX |

Motor response:

| Address | Function Code | Register Address | Register Count | Checksum |
|---|---|---|---|---|
| 1 byte | 1 byte | 2 bytes | 2 bytes | 2 bytes |
| XX | 10 | 00 66 | 00 02 | XX XX |

Request to read the position control target:

| Address | Function Code | Register Address | Register Count | Checksum |
|---|---|---|---|---|
| 1 byte | 1 byte | 2 bytes | 2 bytes | 2 bytes |
| XX | 03 | 00 66 | 00 02 | XX XX |

Motor response:

| Address | Function Code | Byte Count | Register Value | Checksum |
|---|---|---|---|---|
| 1 byte | 1 byte | 1 byte | 2 bytes | 2 bytes |
| XX | 03 | 04 | P0 P1 P2 P3 | XX XX |

Where P0 P1 P2 P3 are the 4 bytes of the position control target from high to low, and the 32-bit floating-point number is encoded according to the IEEE 754 standard (you can test the encoding via the website [ https://www.h-schmidt.net/FloatConverter/IEEE754.html ](https://www.h-schmidt.net/FloatConverter/IEEE754.html)).

#### **0x68 0x69: Velocity Control Target 1, Velocity Control Target 2**

`odrv0.axis0.controller.input_vel`

The velocity control target is a 32-bit floating-point number, of which velocity control target 1 is its high 16 bits and velocity control target 2 is its low 16 bits. The user can only obtain/modify the complete velocity control target by accessing register address 0x68 and reading/writing both registers' data at the same time. Neither register can be accessed individually.

Request to modify/update the velocity control target:

| Address | Function Code | Register Address | Register Count | Byte Count | Register Value | Checksum |
|---|---|---|---|---|---|---|
| 1 byte | 1 byte | 2 bytes | 2 bytes | 1 byte | 4 bytes | 2 bytes |
| XX | 10 | 00 68 | 00 02 | 04 | V0 V1 V2 V3 | XX XX |

Motor response:

| Address | Function Code | Register Address | Register Count | Checksum |
|---|---|---|---|---|
| 1 byte | 1 byte | 2 bytes | 2 bytes | 2 bytes |
| XX | 10 | 00 68 | 00 02 | XX XX |

Request to read the velocity control target:

| Address | Function Code | Register Address | Register Count | Checksum |
|---|---|---|---|---|
| 1 byte | 1 byte | 2 bytes | 2 bytes | 2 bytes |
| XX | 03 | 00 68 | 00 02 | XX XX |

Motor response:

| Address | Function Code | Byte Count | Register Value | Checksum |
|---|---|---|---|---|
| 1 byte | 1 byte | 1 byte | 2 bytes | 2 bytes |
| XX | 03 | 04 | V0 V1 V2 V3 | XX XX |

Where V0 V1 V2 V3 are the 4 bytes of the velocity control target from high to low, and the 32-bit floating-point number is encoded according to the IEEE 754 standard (you can test the encoding via the website [ https://www.h-schmidt.net/FloatConverter/IEEE754.html ](https://www.h-schmidt.net/FloatConverter/IEEE754.html)).

#### **0x6A 0x6B: Torque Control Target 1, Torque Control Target 2**

`odrv0.axis0.controller.input_torque`

The torque control target is a 32-bit floating-point number, of which torque control target 1 is its high 16 bits and torque control target 2 is its low 16 bits. The user can only obtain/modify the complete torque control target by accessing register address 0x6A and reading/writing both registers' data at the same time. Neither register can be accessed individually.

Request to modify/update the torque control target:

| Address | Function Code | Register Address | Register Count | Byte Count | Register Value | Checksum |
|---|---|---|---|---|---|---|
| 1 byte | 1 byte | 2 bytes | 2 bytes | 1 byte | 4 bytes | 2 bytes |
| XX | 10 | 00 6A | 00 02 | 04 | T0 T1 T2 T3 | XX XX |

Motor response:

| Address | Function Code | Register Address | Register Count | Checksum |
|---|---|---|---|---|
| 1 byte | 1 byte | 2 bytes | 2 bytes | 2 bytes |
| XX | 10 | 00 6A | 00 02 | XX XX |

Request to read the torque control target:

| Address | Function Code | Register Address | Register Count | Checksum |
|---|---|---|---|---|
| 1 byte | 1 byte | 2 bytes | 2 bytes | 2 bytes |
| XX | 03 | 00 6A | 00 02 | XX XX |

Motor response:

| Address | Function Code | Byte Count | Register Value | Checksum |
|---|---|---|---|---|
| 1 byte | 1 byte | 1 byte | 2 bytes | 2 bytes |
| XX | 03 | 04 | T0 T1 T2 T3 | XX XX |

Where T0 T1 T2 T3 are the 4 bytes of the torque control target from high to low, and the 32-bit floating-point number is encoded according to the IEEE 754 standard (you can test the encoding via the website [ https://www.h-schmidt.net/FloatConverter/IEEE754.html ](https://www.h-schmidt.net/FloatConverter/IEEE754.html)).

#### **0x6C 0x6D: Velocity Limit 1, Velocity Limit 2**

`odrv0.axis0.controller.config.vel_limit`

The velocity limit is a 32-bit floating-point number, of which velocity limit 1 is its high 16 bits and velocity limit 2 is its low 16 bits. The user can only obtain/modify the complete velocity limit by accessing register address 0x6C and reading/writing both registers' data at the same time. Neither register can be accessed individually.

Request to modify/update the velocity limit:

| Address | Function Code | Register Address | Register Count | Byte Count | Register Value | Checksum |
|---|---|---|---|---|---|---|
| 1 byte | 1 byte | 2 bytes | 2 bytes | 1 byte | 4 bytes | 2 bytes |
| XX | 10 | 00 6C | 00 02 | 04 | V0 V1 V2 V3 | XX XX |

Motor response:

| Address | Function Code | Register Address | Register Count | Checksum |
|---|---|---|---|---|
| 1 byte | 1 byte | 2 bytes | 2 bytes | 2 bytes |
| XX | 10 | 00 6C | 00 02 | XX XX |

Request to read the velocity limit:

| Address | Function Code | Register Address | Register Count | Checksum |
|---|---|---|---|---|
| 1 byte | 1 byte | 2 bytes | 2 bytes | 2 bytes |
| XX | 03 | 00 6C | 00 02 | XX XX |

Motor response:

| Address | Function Code | Byte Count | Register Value | Checksum |
|---|---|---|---|---|
| 1 byte | 1 byte | 1 byte | 2 bytes | 2 bytes |
| XX | 03 | 04 | V0 V1 V2 V3 | XX XX |

Where V0 V1 V2 V3 are the 4 bytes of the velocity limit from high to low, and the 32-bit floating-point number is encoded according to the IEEE 754 standard (you can test the encoding via the website [ https://www.h-schmidt.net/FloatConverter/IEEE754.html ](https://www.h-schmidt.net/FloatConverter/IEEE754.html)).

#### **0x6E 0x6F: Current Limit 1, Current Limit 2**

`odrv0.axis0.motor.config.current_lim`

The current limit is a 32-bit floating-point number, of which current limit 1 is its high 16 bits and current limit 2 is its low 16 bits. The user can only obtain/modify the complete current limit by accessing register address 0x6E and reading/writing both registers' data at the same time. Neither register can be accessed individually.

Request to modify/update the current limit:

| Address | Function Code | Register Address | Register Count | Byte Count | Register Value | Checksum |
|---|---|---|---|---|---|---|
| 1 byte | 1 byte | 2 bytes | 2 bytes | 1 byte | 4 bytes | 2 bytes |
| XX | 10 | 00 6E | 00 02 | 04 | I0 I1 I2 I3 | XX XX |

Motor response:

| Address | Function Code | Register Address | Register Count | Checksum |
|---|---|---|---|---|
| 1 byte | 1 byte | 2 bytes | 2 bytes | 2 bytes |
| XX | 10 | 00 6E | 00 02 | XX XX |

Request to read the current limit:

| Address | Function Code | Register Address | Register Count | Checksum |
|---|---|---|---|---|
| 1 byte | 1 byte | 2 bytes | 2 bytes | 2 bytes |
| XX | 03 | 00 6E | 00 02 | XX XX |

Motor response:

| Address | Function Code | Byte Count | Register Value | Checksum |
|---|---|---|---|---|
| 1 byte | 1 byte | 1 byte | 2 bytes | 2 bytes |
| XX | 03 | 04 | I0 I1 I2 I3 | XX XX |

Where I0 I1 I2 I3 are the 4 bytes of the current limit from high to low, and the 32-bit floating-point number is encoded according to the IEEE 754 standard (you can test the encoding via the website [ https://www.h-schmidt.net/FloatConverter/IEEE754.html ](https://www.h-schmidt.net/FloatConverter/IEEE754.html)).

#### **0x70 0x71: Trapezoidal Trajectory Position Control Velocity Limit 1, Trapezoidal Trajectory Position Control Velocity Limit 2**

`odrv0.axis0.trap_traj.config.vel_limit`

The trapezoidal trajectory position control velocity limit is a 32-bit floating-point number, of which trapezoidal trajectory position control velocity limit 1 is its high 16 bits and trapezoidal trajectory position control velocity limit 2 is its low 16 bits. The user can only obtain/modify the complete trapezoidal trajectory position control velocity limit by accessing register address 0x70 and reading/writing both registers' data at the same time. Neither register can be accessed individually.

Request to modify/update the trapezoidal trajectory position control velocity limit:

| Address | Function Code | Register Address | Register Count | Byte Count | Register Value | Checksum |
|---|---|---|---|---|---|---|
| 1 byte | 1 byte | 2 bytes | 2 bytes | 1 byte | 4 bytes | 2 bytes |
| XX | 10 | 00 70 | 00 02 | 04 | V0 V1 V2 V3 | XX XX |

Motor response:

| Address | Function Code | Register Address | Register Count | Checksum |
|---|---|---|---|---|
| 1 byte | 1 byte | 2 bytes | 2 bytes | 2 bytes |
| XX | 10 | 00 70 | 00 02 | XX XX |

Request to read the trapezoidal trajectory position control velocity limit:

| Address | Function Code | Register Address | Register Count | Checksum |
|---|---|---|---|---|
| 1 byte | 1 byte | 2 bytes | 2 bytes | 2 bytes |
| XX | 03 | 00 70 | 00 02 | XX XX |

Motor response:

| Address | Function Code | Byte Count | Register Value | Checksum |
|---|---|---|---|---|
| 1 byte | 1 byte | 1 byte | 2 bytes | 2 bytes |
| XX | 03 | 04 | V0 V1 V2 V3 | XX XX |

Where V0 V1 V2 V3 are the 4 bytes of the trapezoidal trajectory position control velocity limit from high to low, and the 32-bit floating-point number is encoded according to the IEEE 754 standard (you can test the encoding via the website [ https://www.h-schmidt.net/FloatConverter/IEEE754.html ](https://www.h-schmidt.net/FloatConverter/IEEE754.html)).

#### **0x72 0x73: Trapezoidal Trajectory Position Control Acceleration Limit 1, Trapezoidal Trajectory Position Control Acceleration Limit 2**

`odrv0.axis0.trap_traj.config.accel_limit`

The trapezoidal trajectory position control acceleration limit is a 32-bit floating-point number, of which trapezoidal trajectory position control acceleration limit 1 is its high 16 bits and trapezoidal trajectory position control acceleration limit 2 is its low 16 bits. The user can only obtain/modify the complete trapezoidal trajectory position control acceleration limit by accessing register address 0x72 and reading/writing both registers' data at the same time. Neither register can be accessed individually.

Request to modify/update the trapezoidal trajectory position control acceleration limit:

| Address | Function Code | Register Address | Register Count | Byte Count | Register Value | Checksum |
|---|---|---|---|---|---|---|
| 1 byte | 1 byte | 2 bytes | 2 bytes | 1 byte | 4 bytes | 2 bytes |
| XX | 10 | 00 72 | 00 02 | 04 | A0 A1 A2 A3 | XX XX |

Motor response:

| Address | Function Code | Register Address | Register Count | Checksum |
|---|---|---|---|---|
| 1 byte | 1 byte | 2 bytes | 2 bytes | 2 bytes |
| XX | 10 | 00 72 | 00 02 | XX XX |

Request to read the trapezoidal trajectory position control acceleration limit:

| Address | Function Code | Register Address | Register Count | Checksum |
|---|---|---|---|---|
| 1 byte | 1 byte | 2 bytes | 2 bytes | 2 bytes |
| XX | 03 | 00 72 | 00 02 | XX XX |

Motor response:

| Address | Function Code | Byte Count | Register Value | Checksum |
|---|---|---|---|---|
| 1 byte | 1 byte | 1 byte | 2 bytes | 2 bytes |
| XX | 03 | 04 | A0 A1 A2 A3 | XX XX |

Where A0 A1 A2 A3 are the 4 bytes of the trapezoidal trajectory position control acceleration limit from high to low, and the 32-bit floating-point number is encoded according to the IEEE 754 standard (you can test the encoding via the website [ https://www.h-schmidt.net/FloatConverter/IEEE754.html ](https://www.h-schmidt.net/FloatConverter/IEEE754.html)).

#### **0x74 0x75: Trapezoidal Trajectory Position Control Deceleration Limit 1, Trapezoidal Trajectory Position Control Deceleration Limit 2**

`odrv0.axis0.trap_traj.config.decel_limit`

The trapezoidal trajectory position control deceleration limit is a 32-bit floating-point number, of which trapezoidal trajectory position control deceleration limit 1 is its high 16 bits and trapezoidal trajectory position control deceleration limit 2 is its low 16 bits. The user can only obtain/modify the complete trapezoidal trajectory position control deceleration limit by accessing register address 0x74 and reading/writing both registers' data at the same time. Neither register can be accessed individually.

Request to modify/update the trapezoidal trajectory position control deceleration limit:

| Address | Function Code | Register Address | Register Count | Byte Count | Register Value | Checksum |
|---|---|---|---|---|---|---|
| 1 byte | 1 byte | 2 bytes | 2 bytes | 1 byte | 4 bytes | 2 bytes |
| XX | 10 | 00 74 | 00 02 | 04 | D0 D1 D2 D3 | XX XX |

Motor response:

| Address | Function Code | Register Address | Register Count | Checksum |
|---|---|---|---|---|
| 1 byte | 1 byte | 2 bytes | 2 bytes | 2 bytes |
| XX | 10 | 00 74 | 00 02 | XX XX |

Request to read the trapezoidal trajectory position control deceleration limit:

| Address | Function Code | Register Address | Register Count | Checksum |
|---|---|---|---|---|
| 1 byte | 1 byte | 2 bytes | 2 bytes | 2 bytes |
| XX | 03 | 00 74 | 00 02 | XX XX |

Motor response:

| Address | Function Code | Byte Count | Register Value | Checksum |
|---|---|---|---|---|
| 1 byte | 1 byte | 1 byte | 2 bytes | 2 bytes |
| XX | 03 | 04 | D0 D1 D2 D3 | XX XX |

Where D0 D1 D2 D3 are the 4 bytes of the trapezoidal trajectory position control deceleration limit from high to low, and the 32-bit floating-point number is encoded according to the IEEE 754 standard (you can test the encoding via the website [ https://www.h-schmidt.net/FloatConverter/IEEE754.html ](https://www.h-schmidt.net/FloatConverter/IEEE754.html)).

#### **0x76 0x77: Inertia 1, Inertia 2**

`odrv0.axis0.controller.config.inertia`

The inertia is a 32-bit floating-point number, of which inertia 1 is its high 16 bits and inertia 2 is its low 16 bits. The user can only obtain/modify the complete inertia value by accessing register address 0x76 and reading/writing both registers' data at the same time. Neither register can be accessed individually.

Request to modify/update the inertia:

| Address | Function Code | Register Address | Register Count | Byte Count | Register Value | Checksum |
|---|---|---|---|---|---|---|
| 1 byte | 1 byte | 2 bytes | 2 bytes | 1 byte | 4 bytes | 2 bytes |
| XX | 10 | 00 76 | 00 02 | 04 | I0 I1 I2 I3 | XX XX |

Motor response:

| Address | Function Code | Register Address | Register Count | Checksum |
|---|---|---|---|---|
| 1 byte | 1 byte | 2 bytes | 2 bytes | 2 bytes |
| XX | 10 | 00 76 | 00 02 | XX XX |

Request to read the inertia:

| Address | Function Code | Register Address | Register Count | Checksum |
|---|---|---|---|---|
| 1 byte | 1 byte | 2 bytes | 2 bytes | 2 bytes |
| XX | 03 | 00 76 | 00 02 | XX XX |

Motor response:

| Address | Function Code | Byte Count | Register Value | Checksum |
|---|---|---|---|---|
| 1 byte | 1 byte | 1 byte | 2 bytes | 2 bytes |
| XX | 03 | 04 | I0 I1 I2 I3 | XX XX |

Where I0 I1 I2 I3 are the 4 bytes of the inertia from high to low, and the 32-bit floating-point number is encoded according to the IEEE 754 standard (you can test the encoding via the website [ https://www.h-schmidt.net/FloatConverter/IEEE754.html ](https://www.h-schmidt.net/FloatConverter/IEEE754.html)).

#### **0x78 0x79: Position Gain 1, Position Gain 2**

`odrv0.axis0.controller.config.pos_gain`

The position gain is a 32-bit floating-point number, of which position gain 1 is its high 16 bits and position gain 2 is its low 16 bits. The user can only obtain/modify the complete position gain by accessing register address 0x78 and reading/writing both registers' data at the same time. Neither register can be accessed individually.

Request to modify/update the position gain:

| Address | Function Code | Register Address | Register Count | Byte Count | Register Value | Checksum |
|---|---|---|---|---|---|---|
| 1 byte | 1 byte | 2 bytes | 2 bytes | 1 byte | 4 bytes | 2 bytes |
| XX | 10 | 00 78 | 00 02 | 04 | G0 G1 G2 G3 | XX XX |

Motor response:

| Address | Function Code | Register Address | Register Count | Checksum |
|---|---|---|---|---|
| 1 byte | 1 byte | 2 bytes | 2 bytes | 2 bytes |
| XX | 10 | 00 78 | 00 02 | XX XX |

Request to read the position gain:

| Address | Function Code | Register Address | Register Count | Checksum |
|---|---|---|---|---|
| 1 byte | 1 byte | 2 bytes | 2 bytes | 2 bytes |
| XX | 03 | 00 78 | 00 02 | XX XX |

Motor response:

| Address | Function Code | Byte Count | Register Value | Checksum |
|---|---|---|---|---|
| 1 byte | 1 byte | 1 byte | 2 bytes | 2 bytes |
| XX | 03 | 04 | G0 G1 G2 G3 | XX XX |

Where G0 G1 G2 G3 are the 4 bytes of the position gain from high to low, and the 32-bit floating-point number is encoded according to the IEEE 754 standard (you can test the encoding via the website [ https://www.h-schmidt.net/FloatConverter/IEEE754.html ](https://www.h-schmidt.net/FloatConverter/IEEE754.html)).

#### **0x7A 0x7B: Velocity Gain 1, Velocity Gain 2**

`odrv0.axis0.controller.config.vel_gain`

The velocity gain is a 32-bit floating-point number, of which velocity gain 1 is its high 16 bits and velocity gain 2 is its low 16 bits. The user can only obtain/modify the complete velocity gain by accessing register address 0x7A and reading/writing both registers' data at the same time. Neither register can be accessed individually.

Request to modify/update the velocity gain:

| Address | Function Code | Register Address | Register Count | Byte Count | Register Value | Checksum |
|---|---|---|---|---|---|---|
| 1 byte | 1 byte | 2 bytes | 2 bytes | 1 byte | 4 bytes | 2 bytes |
| XX | 10 | 00 7A | 00 02 | 04 | G0 G1 G2 G3 | XX XX |

Motor response:

| Address | Function Code | Register Address | Register Count | Checksum |
|---|---|---|---|---|
| 1 byte | 1 byte | 2 bytes | 2 bytes | 2 bytes |
| XX | 10 | 00 7A | 00 02 | XX XX |

Request to read the velocity gain:

| Address | Function Code | Register Address | Register Count | Checksum |
|---|---|---|---|---|
| 1 byte | 1 byte | 2 bytes | 2 bytes | 2 bytes |
| XX | 03 | 00 7A | 00 02 | XX XX |

Motor response:

| Address | Function Code | Byte Count | Register Value | Checksum |
|---|---|---|---|---|
| 1 byte | 1 byte | 1 byte | 2 bytes | 2 bytes |
| XX | 03 | 04 | G0 G1 G2 G3 | XX XX |

Where G0 G1 G2 G3 are the 4 bytes of the velocity gain from high to low, and the 32-bit floating-point number is encoded according to the IEEE 754 standard (you can test the encoding via the website [ https://www.h-schmidt.net/FloatConverter/IEEE754.html ](https://www.h-schmidt.net/FloatConverter/IEEE754.html)).
#### **0x7C 0x7D: Velocity Integrator Gain 1, Velocity Integrator Gain 2**

`odrv0.axis0.controller.config.vel_integrator_gain`

The velocity integrator gain is a 32-bit float, where Velocity Integrator Gain 1 is its high 16 bits and Velocity Integrator Gain 2 is its low 16 bits. The user can only obtain/modify the complete velocity integrator gain by accessing register address 0x7C and reading/writing both registers' data simultaneously. Neither register can be accessed individually.

Request to modify/update the velocity integrator gain:

| Address | Function Code | Register Address | Number of Registers | Byte Length | Register Value | Checksum |
|---|---|---|---|---|---|---|
| 1 byte | 1 byte | 2 bytes | 2 bytes | 1 byte | 4 bytes | 2 bytes |
| XX | 10 | 00 7C | 00 02 | 04 | G0 G1 G2 G3 | XX XX |

Motor response:

| Address | Function Code | Register Address | Number of Registers | Checksum |
|---|---|---|---|---|
| 1 byte | 1 byte | 2 bytes | 2 bytes | 2 bytes |
| XX | 10 | 00 7C | 00 02 | XX XX |

Request to read the velocity integrator gain:

| Address | Function Code | Register Address | Number of Registers | Checksum |
|---|---|---|---|---|
| 1 byte | 1 byte | 2 bytes | 2 bytes | 2 bytes |
| XX | 03 | 00 7C | 00 02 | XX XX |

Motor response:

| Address | Function Code | Byte Length | Register Value | Checksum |
|---|---|---|---|---|
| 1 byte | 1 byte | 1 byte | 2 bytes | 2 bytes |
| XX | 03 | 04 | G0 G1 G2 G3 | XX XX |

Here, G0 G1 G2 G3 are the 4 bytes of the velocity integrator gain from high to low, where the 32-bit float is encoded per the IEEE 754 standard (you can test the encoding via the website [ https://www.h-schmidt.net/FloatConverter/IEEE754.html ](https://www.h-schmidt.net/FloatConverter/IEEE754.html)).

#### **0x7E: Reboot**

`odrv0.reboot()`

Writing any value to this register will reboot the motor drive.

Host request:

| Address | Function Code | Register Address | Register Value | Checksum |
|---|---|---|---|---|
| 1 byte | 1 byte | 2 bytes | 2 bytes | 2 bytes |
| XX | 06 | 00 7E | XX XX | XX XX |

No motor response.

#### **0x7F: Clear Errors**

`odrv0.clear_errors()`

Writing any value to this register will clear all errors and faults.

Host request:

| Address | Function Code | Register Address | Register Value | Checksum |
|---|---|---|---|---|
| 1 byte | 1 byte | 2 bytes | 2 bytes | 2 bytes |
| XX | 06 | 00 7F | XX XX | XX XX |

Motor response:

| Address | Function Code | Register Address | Register Value | Checksum |
|---|---|---|---|---|
| 1 byte | 1 byte | 2 bytes | 2 bytes | 2 bytes |
| XX | 06 | 00 7F | XX XX | XX XX |

#### **0x80: Save Configuration**

`odrv0.save_configuration()`

Writing any value to this register will save all current parameters to the drive's Flash memory for permanent storage.

Host request:

| Address | Function Code | Register Address | Register Value | Checksum |
|---|---|---|---|---|
| 1 byte | 1 byte | 2 bytes | 2 bytes | 2 bytes |
| XX | 06 | 00 80 | XX XX | XX XX |

No motor response.

#### **0xA1: Random Endpoint Access Endpoint Number**

Random endpoint access means the user can read/write any supported parameter. The so-called "endpoint" refers to a parameter, and each parameter has a unique endpoint number. When the user wants to read/write a certain parameter, they can obtain that parameter's endpoint number from the endpoint description file, write it into holding register 0xA1, and then either read input register 0x31 to obtain the parameter value, or write to holding register 0xA2 to update the parameter value.

The endpoint number is a 16-bit unsigned integer. Please download the [JSON file](https://cyberbeast.feishu.cn/docx/BPnQd8reEotLWVxqHFNc9qYZnKh#share-Jxl8dOnBMosICExZfihc1JCnn8b) with the endpoint numbers corresponding to all parameters and interface functions:

Request to write the random endpoint number:

| Address | Function Code | Register Address | Register Value | Checksum |
|---|---|---|---|---|
| 1 byte | 1 byte | 2 bytes | 2 bytes | 2 bytes |
| XX | 06 | 00 A1 | E0 E1 | XX XX |

Motor response:

| Address | Function Code | Register Address | Register Value | Checksum |
|---|---|---|---|---|
| 1 byte | 1 byte | 2 bytes | 2 bytes | 2 bytes |
| XX | 06 | 00 A1 | E0 E1 | XX XX |

Here, E0 E1 are the 2 bytes of the 16-bit endpoint number from high to low.

#### **0xA2 0xA3: Random Endpoint Access Write Value 1, Random Endpoint Access Write Value 2**

The random endpoint access write value is a 32-bit value, which may be a 16-bit integer, a 32-bit integer, or a 32-bit float. Random Endpoint Access Write Value 1 is its high 16 bits, and Random Endpoint Access Write Value 2 is its low 16 bits. The user can only update the endpoint value (parameter value) by accessing register address 0xA2 and writing both registers' data simultaneously. Neither register can be accessed individually.

Write request:

| Address | Function Code | Register Address | Number of Registers | Byte Length | Register Value | Checksum |
|---|---|---|---|---|---|---|
| 1 byte | 1 byte | 2 bytes | 2 bytes | 1 byte | 4 bytes | 2 bytes |
| XX | 10 | 00 A2 | 00 02 | 04 | V0 V1 V2 V3 | XX XX |

Motor response:

| Address | Function Code | Register Address | Number of Registers | Checksum |
|---|---|---|---|---|
| 1 byte | 1 byte | 2 bytes | 2 bytes | 2 bytes |
| XX | 10 | 00 A2 | 00 02 | XX XX |

Here, V0 V1 V2 V3 are the 4 bytes of the write value from high to low. If the endpoint is a 32-bit float, it is encoded per the IEEE 754 standard (you can test the encoding via the website [ IEEE-754 Floating Point Converter ](https://www.h-schmidt.net/FloatConverter/IEEE754.html)).

### 2.5 Modbus Protocol in Practice

Assume the motor's address is 1.

#### 2.5.1 Practice: Velocity Control

The sequence of frames sent is as follows:

| Frame Data | Register | Description |
|---|---|---|
| 01 06 00 64 00 02 XX XX | Holding register 0x64 | Set control mode to 2 (velocity control) |
| 01 06 00 65 00 02 XX XX | Holding register 0x65 | Set input mode to 2 (velocity ramp) |
| 01 06 00 63 00 08 XX XX | Holding register 0x63 | Change the motor state to 8 (closed-loop control). |
| 01 10 00 68 00 02 04 40 13 33 33 XX XX | Holding register 0x68 | Set the target velocity to 2.3 r/s, which converts to the 32-bit float 40 13 33 33. |

#### 2.5.2 Practice: Position Control

The sequence of frames sent is as follows:

| Frame Data | Register | Description |
|---|---|---|
| 01 06 00 64 00 03 XX XX | Holding register 0x64 | Set control mode to 3 (position control) |
| 01 06 00 65 00 03 XX XX | Holding register 0x65 | Set input mode to 3 (position filter) |
| 01 06 00 63 00 08 XX XX | Holding register 0x63 | Change the motor state to 8 (closed-loop control). |
| 01 10 00 66 00 02 04 40 49 0F F9 XX XX | Holding register 0x66 | Set the target position to 3.1416 r, which converts to the 32-bit float 40 49 0F F9. |

## 3 CANOpen Protocol

### 3.1 Overview

CANOpen is a communication protocol specification used in the field of automation. This product is implemented in compliance with the CiA301, CiA302, and CiA402 specifications.

**Main Features**

- **Standardized motion control modes**: CiA 402 defines multiple operating modes suited to different motion control needs:
Profile Position Mode: Used for precise position control (e.g., robotic arm positioning).

Cyclic Synchronous Velocity Mode: Simple velocity control (no complex trajectory planning required).

Cyclic Synchronous Torque Mode: Simple torque control (no complex trajectory planning required).

Cyclic Synchronous Position Mode: Simple position control (no complex trajectory planning required).

Homing Mode: Used to find the mechanical zero point (e.g., machine tool reference point return).

- **State machine management**: CiA 402 specifies the drive's state transition logic to ensure safe device operation.

### 3.2 Protocol Frame Format

The table below shows the frame format: CANOpen communication uses the standard frame format, a data frame, an 11-bit ID, and 8 bytes of data, as shown in the table below (MSB on the left, LSB on the right):

| **Data Field** | **COB-ID / CAN ID( 11bits)**  |   | **Data( 8 bytes)**  |
|---|---|---|---|
| Segment | Bit10 ~ Bit7 | Bit6 ~ Bit0 | Byte0 ~ Byte7 |
| Description | Function Code | node_id | Communication Data |

- Function code: Represents the type and priority of the CANOpen frame; the smaller the function code, the higher the priority. See the table below.
- node_id: Represents the unique ID of this motor on the bus; it can be read and set in odrivetool with `odrv0.axis0.config.can.node_id`.
- Communication data: 8 bytes, byte order is little-endian, content depends on the function code type; see 4.3–4.8.
The function code table is as follows:

| **Communication Object** | **Function Code** | **Node Address/Node-ID** | **COB-ID** | **Corresponding Object Index** |
|---|---|---|---|---|
| Network Management | 0000b | 0 | 0h | - |
| Sync Object | 0001b | 0 | 80h | 1005h, 1006h |
| Emergency Object | 0001b | 1-127 | 80h + node_id | 1014h |
| RPDO1 | 0100b | 1-127 | 200h + node_id | 1400h |
| RPDO2 | 0110b | 1-127 | 300h + node_id | 1401h |
| RPDO3 | 1000b | 1-127 | 400h + node_id | 1402h |
| RPDO4 | 1010b | 1-127 | 500h + node_id | 1403h |
| TPDO1 | 0011b | 1-127 | 180h + node_id | 1800h |
| TPDO2 | 0101b | 1-127 | 280h + node_id | 1801h |
| TPDO3 | 0111b | 1-127 | 380h + node_id | 1802h |
| TPD04 | 1001b | 1-127 | 480h + node_id | 1803h |
| TSDO | 1100b | 1-127 | 0x600+node_id | 1200h |
| RSDO | 1011b | 1-127 | 0x580+node_id | 1200h |
| NMT (Heartbeat) | 1110b | 1-127 | 0x700+node_id | 1017h |

### 3.3 NMT (Network Management)

NMT is used by the master to uniformly manage the operating status of all slaves. Its COB-ID is always 0, and the data consists of 2 bytes, as shown in the table below:

| **COB-ID** | **Data Byte0** | **Data Byte1** |
|---|---|---|
| 0x000 | Status Command Word | node_id |

When node_id is 0, it indicates a broadcast message, which is valid for all slave devices on the network.

The status command word is used to directly control the operating status of the slave device; its meaning is shown in the table below:

| Status Command Word | Description |
|---|---|
| 0x01 | Change state to "operational" |
| 0x02 | Change state to "stopped" |
| 0x80 | Change state to "pre-operational" |
| 0x81 | Reset node "reset node" |
| 0x82 | Reset communication "reset communication" |

For example, after the master connects to the drive, it first sends the following NMT command to bring all nodes on the bus into operation:

| COB-ID | Data |
|---|---|
| 0x000 | 01 00 |

### 3.4 SYNC (Synchronization Signal)

SYNC (synchronization signal) is used by the master to synchronize the time of all nodes on the bus, mainly to synchronize PDO messages between nodes. Its COB-ID is fixed at 0x80, and the data field is empty.

For example, after the host uses the NMT command to bring nodes on the bus into operation, it generally sends a SYNC message to have nodes start transmitting PDO data.

### 3.5 Emergency Frame

The emergency frame (EMCY) is used to report abnormal error conditions on the drive board.

The COB-ID is [0x80 + node_id]. When an abnormal error occurs, the drive board sends the CANopen error code at a 500 ms cycle.

The data consists of the values of index 603F and 1001, in little-endian order.

| COB-ID | byte0 | byte1 | byte2 | byte3 | byte4-7 |
|---|---|---|---|---|---|
| 0x80+node_id | 603F_L | 603F_H | 1001_L | 1001H | 0 |

### 3.6 PDO

      PDO (Process Data Object) is a communication method prepared for real-time data transmission. It is used for transmission whenever the user wants to periodically obtain node data, or wants to report data driven by events. PDO is only used when the node status is "operational", so before using PDO the user must first switch to the "operational" status using an NMT message. Its COB-ID follows the table below, and the data field length is fixed at 0–8 bytes, with the specific content determined by the PDO mapping.

Based on whether they are received or sent, PDOs can be divided into RPDO (host to slave) and TPDO (slave to host). The final transmission method and content of a PDO are jointly determined by the communication parameters and mapping parameters. This drive uses 4 RPDOs and 4 TPDOs to implement PDO transmission; the related object list is shown in the table below, and the specific mapping content is described in the object dictionary.

| Name |   | COB-ID | Communication Object | Mapping Object |
|---|---|---|---|---|
| RPDO | 1 | 200h + Node_ID | 1400h | 1600h |
|   | 2 | 300h + Node_ID | 1401h | 1601h |
|   | 3 | 400h + Node_ID | 1402h | 1602h |
|   | 4 | 500h + Node_ID | 1403h | 1603h |
| TPDO | 1 | 180h + Node_ID | 1800h | 1A00h |
|   | 2 | 280h + Node_ID | 1801h | 1A01h |
|   | 3 | 380h + Node_ID | 1802h | 1A02h |
|   | 4 | 480h + Node_ID | 1803h | 1A03h |

#### 3.6.1 PDO Transmission Framework

PDO belongs to process data, meaning it is a one-way transmission that does not require the receiving node to respond with a CAN frame for confirmation; in communication terminology, it belongs to the "producer-consumer" model.

#### 3.6.2 PDO Communication Configuration

- **PDO transmission type**
The PDO transmission type is located at sub-index 02 of the communication parameters (RPDO: 1400h–1403h, TPDO: 1800h–1803h). Different values represent different transmission types, defining the method by which the drive triggers TPDO transmission or processes a received RPDO; the specific correspondence is shown in the table below.

| Communication Type Value | Synchronous |   | Asynchronous |
|---|---|---|---|
|   | Cyclic | Acyclic |   |
| 0 |   | √ |   |
| 1~240 | √ |   |   |
| 241~253 | / | / | / |
| 254、255 |   |   | √ |

When the TPDO transmission type is 0, the TPDO is sent if the mapped data changes and a SYNC frame is received; when the TPDO transmission type is 1–240, the TPDO is sent upon receiving the corresponding number of SYNC frames.

When the TPDO transmission type is 254 or 255, the TPDO is sent when the mapped data changes or the event timer expires. Note: in this type, sub-index 03 (inhibit time) of the drive's TPDO communication parameters (TPDO: 1800h–1803h) must not be 0.

When the RPDO transmission type is 0–255, the received data is directly updated to the application.

- **Inhibit time**
An inhibit time is set for the TPDO, stored at sub-index 03 of the communication parameters (1800h–1803h), to prevent the CAN network from being continuously occupied by lower-priority PDOs. Once a value is set, the interval between transmissions of the same TPDO must not be less than the time corresponding to this parameter.

- **Event timer**
For TPDOs using asynchronous transmission (transmission type 254 or 255), an event timer is defined at sub-index 05 of the communication parameters (1800h–1803h). The event timer can also be regarded as a type of triggering event, and it will likewise trigger the corresponding TPDO transmission. If other events, such as a data change, occur during the timer's running cycle, the TPDO will also be triggered, and the event timer will be reset immediately.

#### 3.6.3 PDO Mapping Parameters

PDO mapping refers to the process of associating variables in the object dictionary with a PDO. The data length of each PDO signal can be up to 8 bytes, and one or more objects can be mapped simultaneously. Through mapping, the specific content and format of PDO transmission can be defined.

Sub-index 0 of the PDO mapping parameters (RPDO: 1600h–1603h, TPDO: 1A00h–1A03h) records the number of objects specifically mapped by that PDO, while sub-indices 1–8 hold the mapping content, i.e., pointers to variables in the object dictionary that can be mapped to the PDO, including the index, sub-index, and mapped object length.

The index and sub-index together determine the object's position in the object dictionary, while the object length specifies the object's specific bit length, expressed in hexadecimal, as follows:

| Object Length | Bit Length |
|---|---|
| 08h | 8 bits  |
| 10h | 16 bits |
| 20h | 32 bits |

Example:

The mapping content of RPDO1's sub-index 1 is 60710010, meaning the 16-bit target torque 6071h-00h is mapped to RPDO1,

If the host wants to send a target torque value of 1000,

according to the mapping, RPDO1's data segment during transmission has 2 bytes, and the transmitted signal is E8 03 00 00 00 00 00 00.

#### 3.6.4 **Changing PDO Mapping Parameters**

If the default PDO mapping parameters do not meet requirements, the mapping can be changed by following the steps below.

The PDO's COB-ID (RPDO: 0x201-0x501, TPDO: 0x181-0x481) is located at sub-index 01 of the communication parameters (RPDO: 1400h~1403h, TPDO: 1800h~1803h). The highest bit of sub-index 1 determines whether the PDO is valid (if the highest bit is 1, the PDO is in an invalid state).

Example:

For a node with Node_ID 1, TPDO2's COB-ID should be "80000281h" in the invalid state, and writing "281h" to this COB-ID indicates that the PDO is activated.

The configuration steps for changing the mapping are: pre-operational mode --> invalidate the PDO mapping (write "1" to the highest bit of the PDO's COB-ID) --> clear the existing mapping content (write "0" to sub-index 0 of the mapping object) --> write the new mapping content (write the mapping parameters to sub-indices 1~8 according to the mapping definition/change the mapping configuration parameters, then write the number of mappings written in the previous step to sub-index 0 of the mapping object) --> validate the PDO (write "0" to the highest bit of the PDO's COB-ID) --> end

Example: with NMT in pre-operational mode, change the existing mapping of 1A01 (status word and current position) to current speed

| CAN ID | Frame Type | Frame Data | Frame Data |
|---|---|---|---|
| 601 | Data Frame | 23 01 18 01 81 02 00 80 | Message: 1801's sub-index 01<br>Invalidate PDO mapping |
| 601 | Data Frame | 2f 01 1A 00 00 00 00 00 | Message: 1A01's sub-index 00<br>Clear existing mapping content |
| 601 | Data Frame | 23 01 1A 01 20 00 64 60 | Message: 1A01's sub-index 01<br>Write new mapping content 60640020 |
| 601 | Data Frame | 2f 01 1A 00 01 00 00 00 | Message: 1A01's sub-index 00<br>Write the number of existing mapping entries |
| 601 | Data Frame | 23 01 18 01 81 02 00 00 | Message: 1801's sub-index 00<br>Validate the PDO |

### 3.7 SDO

SDO (Service Data Object) can be used to read or write object data in the object dictionary. Generally speaking, SDO is only used for parameter configuration, not for data exchange during operation. For real-time operational data exchange, PDO is recommended. SDO's COB-ID follows the table below, and the data field length is fixed at 8 bytes, with the specific content following the table below.

#### 3.7.1 SDO Transmission Framework

The SDO transmission method follows the client-server model, i.e., a request-response scheme. It is initiated by the SDO client on the CAN bus network, and the SDO server provides the response. Therefore, data exchange between SDOs requires at least two CAN frames to complete, and the COB-IDs of the two CAN frames are different.

#### 3.7.2 SDO Transmission Frames

Currently, SDO transmission only supports object data transmission of no more than 4 bytes, i.e., only expedited SDO transmission is supported. The SDO transmission frame is shown in the table below. The data segment uses little-endian mode, i.e., the low byte comes first and the high byte comes last. All SDO frame data segments must be 8 bytes. The SDO transmission frame format is shown in the table below:

| COB-ID | Data |   |   |   |   |   |   |   |
|---|---|---|---|---|---|---|---|---|
| 580h+Node_ID/<br>600h+Node_ID | 0 | 1 | 2 | 3 | 4 | 5 | 6 | 7 |
|   | Command Code | Index |   | Sub-index | Data |   |   |   |

Here, the command code specifies the transmission type and data length of this SDO segment, the index and sub-index indicate the object's position in the object dictionary, and the data is the object's value.

- Expedited SDO write transmission frame
For writes of no more than 4 bytes, expedited SDO transmission (also called upload) is used. Depending on the read/write method and the content data length, the transmission frames differ. The expedited SDO write frame is shown in the table below:

|   | COB-ID | Byte0 | Byte1 | Byte2 | Byte3 | Byte4 | Byte5 | Byte6 | Byte7 |
|---|---|---|---|---|---|---|---|---|---|
| Client → | 0x600+<br>node_id | 0x23 | Index |   | Sub-index | Data |   |   |   |
|   |   | 0x27 | Index |   | Sub-index | Data |   |   | / |
|   |   | 0x2B | Index |   | Sub-index | Data |   | / | / |
|   |   | 0x2F | Index |   | Sub-index | Data | / | / | / |
| ← Server | 0x580+<br>node_id | 0x60 | Index |   | Sub-index | / | / | / | / |
|   |   | 0x80 | Index |   | Sub-index | Abort Code |   |   |   |

Note: "/" indicates that data is present but should be disregarded; when writing data, it is recommended to write 0. The same applies below.

- Expedited SDO read transmission frame
For reads of no more than 4 bytes, expedited SDO transmission (also called download) is used. Depending on the read/write method and the content data length, the transmission frames differ. The expedited SDO read frame is shown in the table below:

|   | COB-ID | Byte0 | Byte1 | Byte2 | Byte3 | Byte4 | Byte5 | Byte6 | Byte7 |
|---|---|---|---|---|---|---|---|---|---|
| Client → | 0x600+<br>node_id | 0x40 | Index |   | Sub-index | / | / | / | / |
| ← Server | 0x580+<br>node_id | 0x43 | Index |   | Sub-index | Data |   |   |   |
|   |   | 0x47 | Index |   | Sub-index | Data |   |   | / |
|   |   | 0x4B | Index |   | Sub-index | Data |   | / | / |
|   |   | 0x4F | Index |   | Sub-index | Data | / | / | / |
|   |   | 0x80 | Index |   | Sub-index | Abort Code |   |   |   |

### 3.8 NMT (Heartbeat)

The heartbeat message is a mechanism used to monitor node status. The slave device periodically sends a heartbeat frame to the master or the entire network to report its current operating status. The frame format is as follows:

| COB-ID | Data Byte0 |
|---|---|
| 0x700+node_id | Status Byte |

The status byte shows the slave's current operating status; its meaning is shown in the table below

| Status Value (Hex) | Status Name | Description |
|---|---|---|
| 0x00 | Boot-up | Node initialization complete |
| 0x04 | Stopped | Node is in the NMT stopped state and only responds to NMT frames |
| 0x05 | operational | Node allows all communication services |
| 0x7F | Pre-operational | Node can perform SDO communication but does not process PDO |
| 0x01-0x03, 0x06-0x7E | - | Reserved value; a normal node should not send this,<br>doing so indicates a problem with the protocol stack implementation |

### 3.9 Master Monitoring Mechanism

During motor operation, for safety and reliability, this drive has a function to monitor the master's communication status. When a master communication anomaly is detected, the motor will automatically perform a stop operation.

Two independent master monitoring mechanisms are currently provided, and the user can select and configure one based on the actual application scenario:

1. Enable communication monitoring by setting the odrv0.can.config.break_timeout parameter via Motor Genie; a detailed explanation is available at [Communication Timeout Stop](https://cyberbeast.feishu.cn/docx/N3SMd4QyRobzHkx3wP3cT1qXnpf#share-DSIUdbjyzohffxxyi1ecTyORnCv)
2. Enable communication monitoring by configuring index 0x1016 (consumer heartbeat time); a detailed explanation follows:
- Working principle
The master periodically broadcasts heartbeat frames according to its configured producer heartbeat time (object 1017h)

If the slave monitoring the master does not receive a heartbeat frame from the master within the time configured in the 1016h sub-index, it considers the master to be offline.

- Configuration requirements
The time of a given 1016h sub-index should preferably be >= 1.8x the host's producer time; otherwise, the slave may falsely report the host as offline.

This drive is both a heartbeat producer and a heartbeat consumer, and can act as a heartbeat consumer for up to 8 different nodes simultaneously. It is recommended that the producer heartbeat time not be lower than 20ms, and the consumer heartbeat time not be lower than 40ms, and be at least 1.8 times the corresponding producer heartbeat time.

Format requirements for configuring a given 1016h sub-index:

Sub-index 0: Indicates the highest supported sub-index number; it cannot be changed and defaults to 8. (A value of 0x08 means sub-indices 1, 2 ... 8 are supported)

Sub-indices 1 to N: Each sub-index is an UNSIGNED32 value used to configure a specific monitoring rule. The structure is as follows: Bits 31-24: reserved as 0x00. Bits 23-16: slave producer node ID (1-127). Bits 15-0: expected heartbeat time (in units of 1ms, 0 means disabled)

- Example
Ensure the slave status is PREOP or OP, and enable the function to monitor the master's heartbeat for the slave with node_id 1

The configuration steps to enable monitoring are: the master first periodically sends heartbeat information --> write to 0x1016 (consumer heartbeat time) --> end

| CAN ID | Frame Type | Frame Data | Frame Data |
|---|---|---|---|
| 701 | Data Frame | 7F | Message: master periodically sends heartbeat information |
| 601 | Data Frame | 23 16 10 01 88 13 01 00 | Message: consumer heartbeat time<br>For the slave with node_id 1, set the master monitoring time to 5s |
### 3.10 Servo Status

To use this drive, the servo drive must be guided through the process specified by the CiA402 protocol before it can operate in the designated state.

![image 926x844](assets_protocol_manual/img_O8z8b01BvoZwh9xrg5scTtkPn9f.png)

The control word and state transitions are explained in the table below:

*[Embedded spreadsheet — see original document online]*

Since bits 10–15 of status word 6041h (bit14 has no meaning) are related to the running status of each servo mode, they are all shown as "0" in the table above; for the specific bit states, please refer to each servo operating mode.

#### 3.10.1 Control Word 6040h

| Bit | Name | Description |
|---|---|---|
| 0 | Servo ready | 1: active, 0: inactive |
| 1 | Main circuit power on | 1: active, 0: inactive |
| 2 | Quick stop | 0: active, 1: inactive |
| 3 | Servo running | 1: active, 0: inactive |
| 4~6 | - | Related to each servo operating mode |
| 7 | Fault reset | 0→1: For resettable faults and warnings, performs the fault reset function<br>1: Held constantly at 1, all other control commands are invalid |
| 8 | Halt | 1: active, 0: inactive |
| 9~15 | NA (reserved) | Manufacturer-defined<br>bit9: In Pre-OP state, writing 1 triggers a full calibration of the drive board<br>bit10: In Pre-OP state, writing 1 triggers the application layer to zero the position |

Notes

● Assigning a value to any single bit of the control word in isolation has no meaning; it must combine with other bits to form a specific control command.

● bit0~bit3 and bit7 have the same meaning across all servo modes; commands must be sent in sequence to guide the servo drive through the CiA402 state machine transition process into the expected state, with each command corresponding to a specific state.

● bit4~bit6 are related to each servo mode (see the control commands for the different modes).

● bit9~15 have no defined function.

| Operating mode | Bit 4 | Bit5 | Bit6 |
|---|---|---|---|
| Cyclic synchronous mode | NA | NA | NA |
| Profile position mode | bit4: Enable new position command (edge-triggered)<br>0→1 indicates to the drive that there is a new target position; whether it is successfully enabled depends on the servo state;<br>1→0 clears bit12 of control word 6041h; whether the clear succeeds depends on the servo state | bit5: Position command update mode<br>0 - not immediate update<br>1 - immediate update | NA |
| Homing mode | bit4: Enable homing<br>0: Homing not activated;<br>0 → 1: Enable homing;<br>1: Homing in progress; (During homing, bit4 must remain at 1.)<br>1 → 0: Abort homing. | NA | NA |

Profile position mode description:

In profile position mode, the triggering and taking effect of the target position are determined by the timing relationship between bit4 of the control word (New set-point) and bit12 of the status word (Set-point acknowledge).

The controller notifies the drive of a new target position by setting New set-point (bit4 of the control word) from 0 to 1. After the drive receives the new target position, it sets Set-point acknowledge (bit12 of the status word) to 1. Once the controller sets New set-point (bit4 of the control word) back to 0, if the drive can currently accept a new target position, it sets Set-point acknowledge (bit12 of the status word) back to 0; otherwise it remains at 1.

Bit5 of the control word (Change set immediately) determines how position commands are connected. When this bit is 1, position commands are connected sequentially, referred to as sequential mode; conversely, when this bit is 0, position commands are connected through zero crossing, referred to as single set-point mode.

    Sequential mode:

    While the current segment's target position is being positioned, once the controller has prepared a new target position, it sets New set-point from 0 to 1, and the drive immediately moves to the new target position.

    In sequential mode, the timing relationship between bit4 of the control word (New set-point) and bit12 of the status word (Set-point acknowledge) is shown in "Figure 11-7" on page 222.

    Single set-point mode:

    While the current segment's target position is being positioned, once the controller has prepared a new target position, it sets the New set-point bit from 0 to 1; after the current segment's position command finishes sending, the drive then moves to the new target position.

    The timing relationship between bit4 of the control word (New set-point) and bit12 of the status word (Set-point acknowledge) is shown in the figure below.

    For single set-point mode, the servo drive supports one target position buffer, meaning that while the current target position is running, a new target position can be buffered. The timing is as follows.

    - If the buffer position is empty, the set position will run immediately.
    - If a position command is currently running, the new set position is stored in the buffer; once the current segment's command finishes sending, the buffered value starts running. Once the buffer is freed, a new set value can be accepted.
    - When the buffer is full, no new set value is accepted, unless the set value's attribute bit "Change set immediately" is 1, in which case the set value will start running immediately.
Example: profile position, non-immediate update:

| Operation sequence | Control command (6040h) | Status word (6041h) | Description |
|---|---|---|---|
| 1 | 0x06 | 0x0631 | Ready to accept new commands, the drive is ready. |
| 2 | 0x07 | 0x0633 | Ready to accept new commands, the drive is ready, and servo enable can be switched on. |
| 3 | 0x0F | 0x0637 | Ready to accept new commands, servo enable is on (since no other position command was executed before segment ① of the displacement command, the target position is considered 0 at this point, and the position-reached bit10 of 6041h = 1). |
| 4. Send target position value 607A |   | 0x1237 | New command received, target position not reached |
| 5 | 0x1F | 0x1237 | The servo has received the command and is running; target position not reached. |
| 6. Positioning for this displacement command segment is complete |   | 0x1637 | The servo has received the command, and the target position has been reached |
| To continue running, modify the displacement command related data (607Ah, 6081h, 6083h, 6084h) as needed, then repeat steps 3–6. |   |   |   |

| Operation sequence | Control command (6040h) | Status word (6041h) | Description |
|---|---|---|---|
| 1 | 0x06 | 0x0631 | Ready to accept new commands, the drive is ready. |
| 2 | 0x07 | 0x0633 | Ready to accept new commands, the drive is ready, and servo enable can be switched on. |
| 3. Send target position value 607A |   | 0x1233 | New command received, target position not reached |
| 4 | 0x0F | 0x1237 | New command received, target position not reached |
| 5 | 0x1F | 0x1237 | The servo has received the command and is running; target position not reached. |
| 6. Positioning for this displacement command segment is complete |   | 0x1637 | The servo has received the command, and the target position has been reached |
| To continue running, modify the displacement command related data (607Ah, 6081h, 6083h, 6084h) as needed, then repeat steps 3–6. |   |   |   |

Example: homing mode:

| Operation sequence | Control command (6040h) | Status word (6041h) | Description |
|---|---|---|---|
| 1 | 0x06 | 0x0631 | Ready to accept new commands, the drive is ready. |
| 2 | 0x07 | 0x0633 | Ready to accept new commands, the drive is ready, and servo enable can be switched on. |
| 3 | 0x0F | 0x0637 | Homing not started, target position reached (since no other position command was executed before segment ① of the displacement command, the target position is considered 0 at this point, and the position-reached bit10 of 6041h = 1). |
| 4 | 0x1F | 0x0237 | Homing started, not complete, target position not reached |
| 5. Positioning for this displacement command segment is complete |   | 0x1637 | Homing started, complete, target position reached |
| To continue running, modify the homing command related data as needed, then repeat steps 3–5. |   |   |   |

#### 3.10.2 Status Word 6041h

| Bit | Name | Description |
|---|---|---|
| 0 | Servo fault-free | 1: active, 0: inactive |
| 1 | Waiting to switch on servo enable | 1: active, 0: inactive |
| 2 | Servo running | 1: active, 0: inactive |
| 3 | Fault | 1: active, 0: inactive |
| 4 | Main circuit power on | 1: active, 0: inactive |
| 5 | Quick stop | 0: active, 1: inactive |
| 6 | Servo ready | 1: active, 0: inactive |
| 7 | Warning | 1: active, 0: inactive |
| 8 | NA | Undefined |
| 9 | Remote control | 0: Not in CANopen mode; some ISG20P standard software functions are available<br>1: CANopen remote control mode |
| 10 | Target reached | 0: Target position not reached<br>1: Target position reached |
| 11 | Internal software position limit exceeded | 0: Position command or feedback has not reached the internal software position limit<br>1: Position command or feedback has reached the internal software position limit |
| 12~13 | - | Related to each servo mode |
| 14~15 | NA | Undefined |

| Operating mode | Bit12 | Bit13 |
|---|---|---|
| Cyclic synchronous mode | NA, always 1 | NA, always 1 |
| Profile position mode | bit12: Position command can be accepted<br>0 - The servo can accept a new displacement command<br>1 - The servo cannot accept a new displacement command | NA |
| Homing mode | bit12: Homing complete<br>0 - Homing not complete<br>1 - Homing complete | NA |

#### 3.10.3 Stopping Methods

This drive - CANopen supports the following 4 stopping methods:

    Servo enable OFF stop;

    Servo fault stop;

    Quick stop;

    Halt stop.

Under all stopping methods, if the motor is running in position mode, the recommended setting for 605Ah/605Ch/605Dh/605Eh is 1; if the motor is running in velocity mode, the recommended setting for 605Ah/605Ch/605Dh/605Eh is 2; if the motor is running in torque mode, the recommended setting for 605Ah/605Ch/605Dh/605Eh is 3.

3. Servo Enable OFF Stop
In the servo running state, when control word 6040h = 0x07/0x06/0x0, servo enable is switched off, and the stopping method is determined by object dictionary 605Ch.

| Index | Set value | Stopping method | Default value |
|---|---|---|---|
| 605C | 0 | Coast to stop, remains in free-running state. | 1 |
|   | 1 | Ramp stop using 6084h; remains in free-running state after stopping is complete. |   |
|   | 2 | Ramp stop at the deceleration rate set by 6085h; remains in free-running state after stopping is complete. |   |
|   | 3 | Ramp stop using 6087h; remains in free-running state after stopping is complete. |   |

4. Servo Fault Stop
When a fault or warning occurs, the servo automatically enters the fault stop state, and the stopping method is determined by object dictionary 605Eh.

| Index | Set value | Stopping method | Default value |
|---|---|---|---|
| 605E | 0 | Coast to stop, remains in free-running state. | 2 |
|   | 1 | Ramp stop using 6084h; remains in free-running state after stopping is complete. |   |
|   | 2 | Ramp stop at the deceleration rate set by 6085h; remains in free-running state after stopping is complete. |   |
|   | 3 | Ramp stop using 6087h; remains in free-running state after stopping is complete. |   |

5. Quick Stop
In the non-fault state, when control word 6040h = 0x02, a quick stop is executed, and the stopping method is selected via object dictionary 605A.

Setting the quick stop stopping method:

| Index | Set value | Stopping method | Default value |
|---|---|---|---|
| 605A | 0 | Coast to stop, remains in free-running state. | 2 |
|   | 1 | Ramp stop using 6084h; remains in free-running state after stopping is complete. |   |
|   | 2 | Ramp stop at the deceleration rate set by 6085h; remains in free-running state after stopping is complete. |   |
|   | 3 | Ramp stop using 6087h; remains in free-running state after stopping is complete. |   |
|   | 4 | NA, no effect |   |
|   | 5 | Ramp stop using 6084h; remains in position-locked state after stopping is complete. |   |
|   | 6 | Ramp stop at the deceleration rate set by 6085h; remains in position-locked state after stopping is complete. |   |
|   | 7 | Ramp stop using 6087h; remains in position-locked state after stopping is complete. |   |

6. Halt
In the non-fault state, when bit8 (Halt) of control word 6040 = 1, the halt command is executed, and the halt method is selected via object dictionary 605D.

| Index | Set value | Stopping method | Default value |
|---|---|---|---|
| 605D | 0 | Coast to stop, remains in free-running state. | 1 |
|   | 1 | Ramp stop using 6084h; remains in position-locked state. |   |
|   | 2 | Ramp stop using 6085h; remains in position-locked state. |   |
|   | 3 | Ramp stop using 6087h; remains in position-locked state. |   |

Explanation:

Depending on the stopping method, stops can be classified into coast stop, ramp stop, and fast torque stop; depending on the stopping state, they can be classified into free-running state and position-hold locked state. Details are as follows:

Stopping Method Comparison

| Stopping method | Description | Characteristics |
|---|---|---|
| Coast stop | The servo motor is de-energized and coasts down to 0 freely; the deceleration time is affected by mechanical inertia, mechanical friction, etc. | Smooth deceleration with low mechanical shock, but the deceleration process is slow. |
| Ramp stop | The position/velocity/torque command decelerates smoothly to 0 to stop. | Smooth deceleration with low mechanical shock; the deceleration rate is controllable. |
| Fast torque stop | The servo drive outputs a reverse braking torque to stop. | Fast deceleration with some mechanical shock, but the deceleration process is short. |

Stopping State Comparison

| Stopping state | State description |
|---|---|
| Free-running state | After the motor stops rotating, it is de-energized and the motor shaft can rotate freely. |
| Position-hold locked | After the motor stops rotating, the motor shaft is locked and cannot rotate freely. (Servo running state) |

### 3.11 CANopen Protocol Practical Examples

Assume the motor acts as a slave with Node_ID 1. Note that in the following practical examples, CAN signal units are in command units; if conversion to user units is needed, refer to the conversion factor section of the [CyberBeast Drive CANopen Supplementary Manual](https://bcnyljrhe70u.feishu.cn/wiki/RkxswgglNiX2mckH9CdcVNmQnXe).

#### 3.11.1 Practical Example: Cyclic Synchronous Torque Mode

The sequence of CAN messages sent is as follows:

| CAN ID | Frame type | Frame data | Frame data |
|---|---|---|---|
| 00 | Data frame | 01 01 | Message: NMT<br>Parameter: 01<br>The NMT state of the motor with Node_ID 1 switches to "operational" |
| 601 | Data frame | 2F 60 60 00 0A 00 00 00  | Message: Operating mode<br>Parameter: 10<br>Set the control mode to torque control |
| 601 | Data frame | 2F 02 20 01 06 00 00 00  | Input mode<br>Parameter: 6<br>Set the input mode to torque ramp (if this command is not executed, the default is direct input mode) |
| 601 | Data frame | 2F 02 20 01 06 00 00 00  | Input mode<br>Parameter: 6<br>Set the input mode to torque ramp (if this command is not executed, the default is direct input mode) |
| 601 | Data frame | 2B 40 60 00 06 00 00 00 | Message: Control word<br>Parameter: 6<br>Set the servo state to servo ready |
| 601 | Data frame | 2B 40 60 00 07 00 00 00 | Message: Control word<br>Parameter: 7<br>Set the servo state to waiting to switch on servo enable |
| 601 | Data frame | 2B 40 60 00 0F 00 00 00 | Message: Control word<br>Parameter: 16<br>Set the servo state to servo running |
| 501 | Data frame | 0F 00 64 00  | Message: Target torque<br>Parameter: 0.5<br>Set the target torque in bytes 3 and 4 to 0.5Nm via PDO |

#### 3.11.2 Practical Example: Cyclic Synchronous Velocity Mode

The sequence of CAN messages sent is as follows:

| CAN ID | Frame type | Frame data | Frame data |
|---|---|---|---|
| 00 | Data frame | 01 01 | Message: NMT<br>Parameter: 01<br>The NMT state of the motor with Node_ID 1 switches to "operational" |
| 601 | Data frame | 2F 60 60 00 09 00 00 00  | Message: Operating mode<br>Parameter: 9<br>Set the control mode to velocity control |
| 601 | Data frame | 2F 02 20 01 02 00 00 00  | Input mode<br>Parameter: 2<br>Set the input mode to velocity ramp (this command may also be skipped; the default is direct input mode) |
| 601 | Data frame | 2B 40 60 00 06 00 00 00 | Message: Control word<br>Parameter: 6<br>Set the servo state to servo ready |
| 601 | Data frame | 2B 40 60 00 07 00 00 00 | Message: Control word<br>Parameter: 7<br>Set the servo state to waiting to switch on servo enable |
| 601 | Data frame | 2B 40 60 00 0F 00 00 00 | Message: Control word<br>Parameter: 16<br>Set the servo state to servo running |
| 401 | Data frame | 0F 00 00 08 00 00 | Message: Target velocity<br>Parameter: 0.125<br>Set the target velocity in bytes 3, 4, 5, 6 to 0.125 rev/s via PDO |

#### 3.11.3 Practical Example: Cyclic Synchronous Position Mode

The sequence of CAN messages sent is as follows:

| CAN ID | Frame type | Frame data | Frame data |
|---|---|---|---|
| 00 | Data frame | 01 01 | Message: NMT<br>Parameter: 01<br>The NMT state of the motor with Node_ID 1 switches to "operational" |
| 601 | Data frame | 2F 60 60 00 08 00 00 00  | Message: Operating mode<br>Parameter: 8<br>Set the control mode to position control |
| 601 | Data frame | 2F 02 20 01 03 00 00 00  | Input mode<br>Parameter: 3<br>Set the input mode to position filtering (this command may also be skipped; the default is direct input mode) |
| 601 | Data frame | 2B 40 60 00 06 00 00 00 | Message: Control word<br>Parameter: 6<br>Set the servo state to servo ready |
| 601 | Data frame | 2B 40 60 00 07 00 00 00 | Message: Control word<br>Parameter: 7<br>Set the servo state to waiting to switch on servo enable |
| 601 | Data frame | 2B 40 60 00 0F 00 00 00 | Message: Control word<br>Parameter: 16<br>Set the servo state to servo running |
| 301 | Data frame | 0F 00 00 40 00 00 | Message: Target position<br>Parameter: 1<br>Set the target position in bytes 3, 4, 5, 6 to 1 revolution via PDO |

#### 3.11.4 Practical Example: Profile Position Mode

The sequence of CAN messages sent is as follows:

| CAN ID | Frame type | Frame data | Frame data |
|---|---|---|---|
| 00 | Data frame | 01 01 | Message: NMT<br>Parameter: 01<br>The NMT state of the motor with Node_ID 1 switches to "operational" |
| 601 | Data frame | 2F 60 60 00 01 00 00 00  | Message: Operating mode<br>Parameter: 1<br>Set the control mode to position control, with input mode 5: trapezoidal profile position |
| 601 | Data frame | 23 81 60 00 00 80 0C 00 | Message: Profile velocity<br>Parameter: 50<br>Set the profile velocity to 50 rev/s |
| 601 | Data frame | 23 83 60 00 00 80 02 00 | Message: Profile acceleration<br>Parameter: 16<br>Set the profile acceleration to 10 rev/s^2 |
| 601 | Data frame | 23 84 60 00 00 80 02 00 | Message: Profile deceleration<br>Parameter: 16<br>Set the profile deceleration to 10 rev/s^2 |
| 601 | Data frame | 2B 40 60 00 06 00 00 00 | Message: Control word<br>Parameter: 6<br>Set the servo state to servo ready |
| 601 | Data frame | 2B 40 60 00 07 00 00 00 | Message: Control word<br>Parameter: 7<br>Set the servo state to waiting to switch on servo enable |
| 601 | Data frame | 2B 40 60 00 0F 00 00 00 | Message: Control word<br>Parameter: 16<br>Set the servo state to servo running |
| 301 | Data frame | 0F 00 00 40 00 00 | Message: Target position<br>Parameter: 1<br>Set the target position in bytes 3, 4, 5, 6 to 1 revolution via PDO; the motor does not rotate at this point |
| 601 | Data frame | 2B 40 60 00 1F 00 00 00 | Message: Control word<br>Parameter: 31<br>Set the servo state to servo running, target position not reached, motor rotating |

#### 3.11.5 Practical Example: Homing Mode

The sequence of CAN messages sent is as follows:

| CAN ID | Frame type | Frame data | Frame data |
|---|---|---|---|
| 00 | Data frame | 01 01 | Message: NMT<br>Parameter: 01<br>The NMT state of the motor with Node_ID 1 switches to "operational" |
| 601 | Data frame | 2F 60 60 00 06 00 00 00  | Message: Operating mode<br>Parameter: 6<br>Set the control mode to homing mode |
| 601 | Data frame | 2F 98 60 00 11 00 00 00 | Message: Homing method<br>Parameter: 17<br>Set the homing method to 17; only 17 can currently be set, others are not yet supported |
| 601 | Data frame | 23 7C 60 00 00 40 01 00 | Message: Home offset<br>Parameter: 5<br>Set the home offset to 5 revolutions |
| 601 | Data frame | 23 99 60 01 00 20 00 00 | Message: Homing velocity<br>Parameter: 16<br>Set the profile deceleration to 0.5 rev/s |
| 601 | Data frame | 23 9A 60 00 00 00 04 00 | Message: Homing acceleration<br>Parameter: 0.5<br>Set the profile deceleration to 16 rev/s^2 |
| 601 | Data frame | 2B 40 60 00 06 00 00 00 | Message: Control word<br>Parameter: 6<br>Set the servo state to servo ready |
| 601 | Data frame | 2B 40 60 00 07 00 00 00 | Message: Control word<br>Parameter: 7<br>Set the servo state to waiting to switch on servo enable |
| 601 | Data frame | 2B 40 60 00 0F 00 00 00 | Message: Control word<br>Parameter: 16<br>Set the servo state to servo running |
| 601 | Data frame | 2B 40 60 00 1F 00 00 00 | Message: Control word<br>Parameter: 31<br>Set the servo state to homing in progress |

#### 3.11.6 Practical Example: Zeroing Mode

If, during use, you find that the current position value is prone to overflow, you can configure a suitable conversion factor yourself, or perform this zeroing mode.

Note: Prerequisites for the zeroing operation:

1. The motor is in the idle state (not enabled)

2. In homing mode

3. The homing method is 35

When the above three conditions are met, the zeroing operation is performed and the current position is set to zero.

The sequence of CAN messages sent is as follows:

| CAN ID | Frame type | Frame data | Frame data |
|---|---|---|---|
| 601 | Data frame | 2F 60 60 00 06 00 00 00  | Message: Operating mode<br>Parameter: 6<br>Set the control mode to homing mode |
| 601 | Data frame | 2F 98 60 00 23 00 00 00 | Message: Homing method<br>Parameter: 17<br>Set the homing method to 35, clearing all actual values of the velocity loop and position loop to zero |
| 601 | Data frame | 2F 60 60 00 01 00 00 00  | Message: Operating mode<br>Parameter: 1<br>After completing the zeroing, remember to exit homing mode; as an example here, set the control mode to profile position mode |

#### 3.11.7 Practical Example: Save Configuration

Note: Prerequisite for the zeroing operation: the motor is in the idle state (not enabled)

| CAN ID | Frame type | Frame data | Frame data |
|---|---|---|---|
| 601 | Data frame | 23 10 10 01 65 76 61 73 | Message: Save_Configuration<br>Save parameters and restart the motor |

#### 3.11.8 Practical Example: Modifying PDO Mapping

PDO mapping can be modified while the state machine is in the Pre-operational (Pre-op) state.

Example: change TPDO1's original mapping of [01:6041] status word, [02:6061] mode display, to [01:6064] current position, [02:606c] current velocity.

| CAN ID | Frame type | Frame data | Meaning of frame data |
|---|---|---|---|
| 601 | Data frame | 23 00 18 01 81 01 00 80 | 23 (data type)   00 18 (index)   01 (sub-index)  81 01 (TPDO1 ID) 00 80 (disable) |
| 601 | Data frame | 2f 00 1a 00 00 00 00 00 | Clear the 1800 mapping parameters |
| 601  | Data frame | 23 00 1a 01 20 00 64 60 | 23 (data type)   00 1a (index)   01 (sub-index)   <br>20 (size of mapped data)  00 64 60 (mapped register) |
| 601 | Data frame | 23 00 1a 02 20 00 6c 60 | 23 (data type)   00 1a (index)   02 (sub-index)   <br>20 (size of mapped data)  00 6c 60 (mapped register) |
| 601 | Data frame | 2f 00 1a 00 02 00 00 00 | Write mapping count 2 into 1a00 |
| 601 | Data frame | 23 00 18 01 81 01 00 00 | Restore TPDO1 ID: 181 |
### 3.12 Object Dictionary

The object dictionary, i.e. the EDS file, describes the device's functions and communication parameters. To communicate with this product via CANopen, you must import the product's EDS file. Please download it from the link below: https://bl.cyberbeast.cn/actuator/ds402.eds

#### 3.12.1 Object Group 1000h Assignment List

| Index | Subindex | Name | Accessibility | Mappable | Data Type | Unit | Data Range | Factory Setting |
|---|---|---|---|---|---|---|---|---|
| 1000h | - | Device Type | RO | NO | Uint32 | VAR | Uint32 | 0x20192 |
| 1001h | - | Error Register | RO | RPDO | Uint8 | VAR | Uint8 | 0x0 |
| 1003h | 0h | Number of Errors | RO | NO | Uint32 | ARR | Uint8 | - |
|   | 1~10h | Standard Error Field | RW | NO | Uint32 | - | Uint32 | 0 |
| 1005h | - | Sync Message COB-ID | RW | NO | Uint32 | VAR | Uint32 | 0x80 |
| 1006h | - | Sync Cycle Period | RW | NO | Uint32 | VAR | Uint32 | 1000 |
| 1008h | - | Manufacturer Device Name | CONST | NO | String | VAR | String | MW6010-8 |
| 1009h | - | Hardware Version | CONST | NO | String | VAR | String | 3.10 |
| 100Ah | - | Software Version | CONST | NO | String | VAR | String | 0.5.16 |
| 1010h | 0h | Save Parameters | RO | NO | Uint8 | ARR | Uint8 | 4 |
|   | 1h | Save All Object Parameters | RW | NO | Uint32 | - | - | 0 |
|   | 2h | Save Communication Object Parameters | RW | NO | Uint32 | - | - | 0 |
|   | 3h | Save Application Object Parameters | RW | NO | Uint32 | - | - | 0 |
|   | 4h | Save Sub-Protocol Area Object Parameters | RW | NO | Uint32 | - | - | 0 |
| 1011h | 0h | Restore Default Parameters | RW | NO | Uint8 | ARR | - | 4 |
|   | 1h | Restore All Object Default Parameters | RW | NO | Uint32 | - | - | 0 |
|   | 2h | Restore Communication Object Default Parameters | RW | NO | Uint32 | - | - | 0 |
|   | 3h | Restore Application Object Parameters | RW | NO | Uint32 | - | - | 0 |
|   | 4h | Restore Sub-Protocol Area Object Parameters | RW | NO | Uint32 | - | - | 0 |
| 1014h | - | Emergency Message COB-ID | RW | NO | Uint32 | VAR | Uint32 | 0x80+<br>NODEID |
| 1016h | 0h | Consumer Heartbeat Time | RO | NO | Uint8 | ARR | - | 8 |
|   | 1~8h | Consumer Heartbeat Time | RW | NO | Uint32 | - | Uint32 | 0 |
| 1017h | - | Producer Heartbeat Time | RW | NO | Uint16 | VAR | Uint16 | 500 |
| 1018h | 0h | Device Object Description | RO | NO | Uint8 | REC | - | 4 |
|   | 1h | Vendor ID | RO | NO | Uint32 | - | Uint32 | 0 |
|   | 2h | Device Code | RO | NO | Uint32 | - | Uint32 | 0 |
|   | 3h | Device Revision Number | RO | NO | Uint32 | - | Uint32 | 0 |
|   | 4h | Serial Number | RO | NO | Uint32 | - | Uint32 | 0 |
| 1200h | 0h | SDO Server Parameters | RO | NO | Uint8 | REC | Uint8 | 2 |
|   | 1h | Client-to-Server COB-ID | RO | NO | Uint32 | - | Uint32 | 0x600+<br>Node_ID |
|   | 2h | Server-to-Client COB-ID | RO | NO | Uint32 | - | Uint32 | 0x580+<br>Node_ID |
| 1400h | 0h | RPDO1 Parameters | RO | NO | Uint8 | REC | Uint8 | 4 |
|   | 1h | COB-ID | RW | NO | Uint32 | - | Uint32 | 0x200<br>+Node_ID |
|   | 2h | Transmission Type | RW | NO | Uint8 | - | Uint8 | 0xFF |
|   | 3h | Inhibit Time | RW | NO | Uint16 | - | Uint16 | 0 |
|   | 4h | Reserved Field | RW | NO | Uint8 | - | Uint8 | 0 |
|   | 5h | Event Timer | RW | NO | Uint16 | - | Uint16 | 0 |
| 1401h | 0h | RPDO2 Parameters | RW | NO | Uint8 | REC | Uint8 | - |
|   | 1h | COB-ID | RW | NO | Uint32 | - | Uint32 | 0x300<br>+Node_ID |
|   | 2h | Transmission Type | RW | NO | Uint8 | - | Uint8 | 0xFF |
|   | 3h | Inhibit Time | RW | NO | Uint16 | - | Uint16 | 0 |
|   | 4h | Reserved Field | RW | NO | Uint8 | - | Uint8 | 0 |
|   | 5h | Event Timer | RW | NO | Uint16 | - | Uint16 | 0 |
| 1402h | 0h | RPDO3 Parameters | RW | NO | Uint8 | REC | Uint8 | - |
|   | 1h | COB-ID | RW | NO | Uint32 | - | Uint32 | 0x400<br>+Node_ID |
|   | 2h | Transmission Type | RW | NO | Uint8 | - | Uint8 | 0xFF |
|   | 3h | Inhibit Time | RW | NO | Uint16 | - | Uint16 | 0 |
|   | 4h | Reserved Field | RW | NO | Uint8 | - | Uint8 | 0 |
|   | 5h | Event Timer | RW | NO | Uint16 | - | Uint16 | 0 |
| 1403h | 0h | RPDO4 Parameters | RW | NO | Uint8 | REC | Uint8 | - |
|   | 1h | COB-ID | RW | NO | Uint32 | - | Uint32 | 0x500<br>+Node_ID |
|   | 2h | Transmission Type | RW | NO | Uint8 | - | Uint8 | 0xFF |
|   | 3h | Inhibit Time | RW | NO | Uint16 | - | Uint16 | 0 |
|   | 4h | Reserved Field | RW | NO | Uint8 | - | Uint8 | 0 |
|   | 5h | Event Timer | RW | NO | Uint16 | - | Uint16 | 0 |
| 1600h | 0h | RPDO1 Mapping Parameters | RW | NO | Uint8 | REC | Uint8 | 3 |
|   | 1h | RPDO1 Mapped Object 1 | RW | NO | Uint32 | - | Uint32 | 0x60400010 |
|   | 2h | RPDO1 Mapped Object 2 | RW | NO | Uint32 | - | Uint32 | 0x60600008<br>(Mode Selection) |
|   | 3h | RPDO1 Mapped Object 3 | RW | NO | Uint32 | - | Uint32 | 0 |
|   | 4h | RPDO1 Mapped Object 4 | RW | NO | Uint32 | - | Uint32 | 0 |
|   | 5h | RPDO1 Mapped Object 5 | RW | NO | Uint32 | - | Uint32 | 0 |
|   | 6h | RPDO1 Mapped Object 6 | RW | NO | Uint32 | - | Uint32 | 0 |
|   | 7h | RPDO1 Mapped Object 7 | RW | NO | Uint32 | - | Uint32 | 0 |
|   | 8h | RPDO1 Mapped Object 8 | RW | NO | Uint32 | - | Uint32 | 0 |
| 1601h | 0h | RPDO2 Mapping Parameters | RW | NO | Uint8 | REC | Uint8 | 3 |
|   | 1h | RPDO2 Mapped Object 1 | RW | NO | Uint32 | - | Uint32 | 0x60400010 |
|   | 2h | RPDO2 Mapped Object 2 | RW | NO | Uint32 | - | Uint32 | 0x607A0020<br>(Target Position) |
|   | 3h | RPDO2 Mapped Object 3 | RW | NO | Uint32 | - | Uint32 | 0 |
|   | 4h | RPDO2 Mapped Object 4 | RW | NO | Uint32 | - | Uint32 | 0 |
|   | 5h | RPDO2 Mapped Object 5 | RW | NO | Uint32 | - | Uint32 | 0 |
|   | 6h | RPDO2 Mapped Object 6 | RW | NO | Uint32 | - | Uint32 | 0 |
|   | 7h | RPDO2 Mapped Object 7 | RW | NO | Uint32 | - | Uint32 | 0 |
|   | 8h | RPDO2 Mapped Object 8 | RW | NO | Uint32 | - | Uint32 | 0 |
| 1602h | 0h | RPDO3 Mapping Parameters | RW | NO | Uint8 | REC | Uint8 | 3 |
|   | 1h | RPDO3 Mapped Object 1 | RW | NO | Uint32 | - | Uint32 | 0x60400010 |
|   | 2h | RPDO3 Mapped Object 2 | RW | NO | Uint32 | - | Uint32 | 0x60FF0020<br>(Target Velocity) |
|   | 3h | RPDO3 Mapped Object 3 | RW | NO | Uint32 | - | Uint32 | 0 |
|   | 4h | RPDO3 Mapped Object 4 | RW | NO | Uint32 | - | Uint32 | 0 |
|   | 5h | RPDO3 Mapped Object 5 | RW | NO | Uint32 | - | Uint32 | 0 |
|   | 6h | RPDO3 Mapped Object 6 | RW | NO | Uint32 | - | Uint32 | 0 |
|   | 7h | RPDO3 Mapped Object 7 | RW | NO | Uint32 | - | Uint32 | 0 |
|   | 8h | RPDO3 Mapped Object 8 | RW | NO | Uint32 | - | Uint32 | 0 |
| 1603h | 0h | RPDO4 Mapping Parameters | RW | NO | Uint8 | REC | Uint8 | 3 |
|   | 1h | RPDO4 Mapped Object 1 | RW | NO | Uint32 | - | Uint32 | 0x60400010 |
|   | 2h | RPDO4 Mapped Object 2 | RW | NO | Uint32 | - | Uint32 | 0x60710010 |
|   | 3h | RPDO4 Mapped Object 3 | RW | NO | Uint32 | - | Uint32 | 0 |
|   | 4h | RPDO4 Mapped Object 4 | RW | NO | Uint32 | - | Uint32 | 0 |
|   | 5h | RPDO4 Mapped Object 5 | RW | NO | Uint32 | - | Uint32 | 0 |
|   | 6h | RPDO4 Mapped Object 6 | RW | NO | Uint32 | - | Uint32 | 0 |
|   | 7h | RPDO4 Mapped Object 7 | RW | NO | Uint32 | - | Uint32 | 0 |
|   | 8h | RPDO4 Mapped Object 8 | RW | NO | Uint32 | - | Uint32 | 0 |
| 1800h | 0h | TPDO1 Communication Parameters | RO | NO | Uint8 | REC | - | 5 |
|   | 1h | COB-ID | RW | NO | Uint32 | - | Uint32 | 0x180<br>+Node_ID |
|   | 2h | Transmission Type | RW | NO | Uint8 | - | Uint8 | 0xFF |
|   | 3h | Inhibit Time | RW | NO | Uint16 | - | Uint16 | 200 |
|   | 5h | Event Timer | RW | NO | Uint16 | - | Uint16 | 500 |
| 1801h | 0h | TPDO2 Communication Parameters | RO | NO | Uint8 | REC | - | 5 |
|   | 1h | COB-ID | RW | NO | Uint32 | - | Uint32 | 0x280<br>+Node_ID |
|   | 2h | Transmission Type | RW | NO | Uint8 | - | Uint8 | 0xFF |
|   | 3h | Inhibit Time | RW | NO | Uint16 | - | Uint16 | 200 |
|   | 5h | Event Timer | RW | NO | Uint16 | - | Uint16 | 500 |
| 1802h | 0h | TPDO3 Communication Parameters | RO | NO | Uint8 | REC | - | 5 |
|   | 1h | COB-ID | RW | NO | Uint32 | - | Uint32 | 0x380<br>+Node_ID |
|   | 2h | Transmission Type | RW | NO | Uint8 | - | Uint8 | 0xFF |
|   | 3h | Inhibit Time | RW | NO | Uint16 | - | Uint16 | 200 |
|   | 5h | Event Timer | RW | NO | Uint16 | - | Uint16 | 500 |
| 1803h | 0h | TPDO4 Communication Parameters | RO | NO | Uint8 | REC | - | 5 |
|   | 1h | COB-ID | RW | NO | Uint32 | - | Uint32 | 0x480<br>+Node_ID |
|   | 2h | Transmission Type | RW | NO | Uint8 | - | Uint8 | 0x01 |
|   | 3h | Inhibit Time | RW | NO | Uint16 | - | Uint16 | 0 |
|   | 5h | Event Timer | RW | NO | Uint16 | - | Uint16 | 0 |
| 1A00h | 0h | TPDO1 Mapping Parameters | RO | NO | Uint8 | REC | Uint8 | 3 |
|   | 1h | TPDO1 Mapped Object 1 | RW | NO | Uint32 | - | Uint32 | 0x60410010 |
|   | 2h | TPDO1 Mapped Object 2 | RW | NO | Uint32 | - | Uint32 | 0x60610008<br>(Mode Display) |
|   | 3h | TPDO1 Mapped Object 3 | RW | NO | Uint32 | - | Uint32 | 0 |
|   | 4h | TPDO1 Mapped Object 4 | RW | NO | Uint32 | - | Uint32 | 0 |
|   | 5h | TPDO1 Mapped Object 5 | RW | NO | Uint32 | - | Uint32 | 0 |
|   | 6h | TPDO1 Mapped Object 6 | RW | NO | Uint32 | - | Uint32 | 0 |
|   | 7h | TPDO1 Mapped Object 7 | RW | NO | Uint32 | - | Uint32 | 0 |
|   | 8h | TPDO1 Mapped Object 8 | RW | NO | Uint32 | - | Uint32 | 0 |
| 1A01h | 0h | TPDO2 Mapping Parameters | RO | NO | Uint8 | REC | Uint8 | 3 |
|   | 1h | TPDO2 Mapped Object 1 | RW | NO | Uint32 | - | Uint32 | 0x60410010 |
|   | 2h | TPDO2 Mapped Object 2 | RW | NO | Uint32 | - | Uint32 | 0x60640020<br>(Current Position) |
|   | 3h | TPDO2 Mapped Object 3 | RW | NO | Uint32 | - | Uint32 | 0 |
|   | 4h | TPDO2 Mapped Object 4 | RW | NO | Uint32 | - | Uint32 | 0 |
|   | 5h | TPDO2 Mapped Object 5 | RW | NO | Uint32 | - | Uint32 | 0 |
|   | 6h | TPDO2 Mapped Object 6 | RW | NO | Uint32 | - | Uint32 | 0 |
|   | 7h | TPDO2 Mapped Object 7 | RW | NO | Uint32 | - | Uint32 | 0 |
|   | 8h | TPDO2 Mapped Object 8 | RW | NO | Uint32 | - | Uint32 | 0 |
| 1A02h | 0h | TPDO3 Mapping Parameters | RO | NO | Uint8 | REC | Uint8 | 3 |
|   | 1h | TPDO3 Mapped Object 1 | RW | NO | Uint32 | - | Uint32 | 0x60410010 |
|   | 2h | TPDO3 Mapped Object 2 | RW | NO | Uint32 | - | Uint32 | 0x606C0020<br>(Current Speed) |
|   | 3h | TPDO3 Mapped Object 3 | RW | NO | Uint16 | - | Uint16 | 0 |
|   | 4h | TPDO3 Mapped Object 4 | RW | NO | Uint32 | - | Uint32 | 0 |
|   | 5h | TPDO3 Mapped Object 5 | RW | NO | Uint32 | - | Uint32 | 0 |
|   | 6h | TPDO3 Mapped Object 6 | RW | NO | Uint32 | - | Uint32 | 0 |
|   | 7h | TPDO3 Mapped Object 7 | RW | NO | Uint32 | - | Uint32 | 0 |
|   | 8h | TPDO3 Mapped Object 8 | RW | NO | Uint32 | - | Uint32 | 0 |
| 1A03h | 0h | TPDO4 Mapping Parameters | RO | NO | Uint8 | REC | Uint8 | 3 |
|   | 1h | TPDO4 Mapped Object 1 | RW | NO | Uint32 | - | Uint32 | 0x60410010 |
|   | 2h | TPDO4 Mapped Object 2 | RW | NO | Uint32 | - | Uint32 | 0x60770010<br>(Current Torque) |
|   | 3h | TPDO4 Mapped Object 3 | RW | NO | Uint32 | - | Uint32 | 0 |
|   | 4h | TPDO4 Mapped Object 4 | RW | NO | Uint32 | - | Uint32 | 0 |
|   | 5h | TPDO4 Mapped Object 5 | RW | NO | Uint32 | - | Uint32 | 0 |
|   | 6h | TPDO4 Mapped Object 6 | RW | NO | Uint32 | - | Uint32 | 0 |
|   | 7h | TPDO4 Mapped Object 7 | RW | NO | Uint32 | - | Uint32 | 0 |
|   | 8h | TPDO4 Mapped Object 8 | RW | NO | Uint32 | - | Uint32 | 0 |

#### 3.10.2 Object Group 2000h Assignment List

| Index | Subindex | Name | Accessibility | Mappable | Data Type | Unit | Data Range | Factory Setting |
|---|---|---|---|---|---|---|---|---|
| 2000h | 0h | Motor Parameters | RO | NO | Uint8 | - | - | 10 |
|   | 1h | Number of Pole Pairs | RW | - | Int32 | - | - | 14 |
|   | 2h | Torque Constant | RW |   | FLoat32 | - |   | 0.097 |
|   | 3h | Phase-to-Phase Resistance | RW | - | Float32 | - | - | 0 |
|   | 4h | Phase-to-Phase Inductance | RW | - | Float32 | - | - | 0 |
|   | 5h | Motor Name | RO | - | String | - | - | MW6010-8 |
|   | 6h | Motor Number | RW | - | Uint16 | - | - | 0 |
|   | 7h | Non-standard Number | RO | - | Uint32 | - | - | 0 |
|   | 8h | Encoder Version Number | RO | - | Uint16 | - | - | 0 |
|   | 9h | Bus Motor Number | RO | - | Uint16 | - | - | 0 |
|   | Ah | Bus Encoder Type | RW | - | Uint16 | - | - | 0 |

| Index | Subindex | Name | Accessibility | Mappable | Data Type | Unit | Data Range | Factory Setting |
|---|---|---|---|---|---|---|---|---|
| 2002h | 0h | Basic Control Parameters | RO | NO | Uint8 | - | - | 2 |
|   | 1h | Input Mode Selection | RW | - | Uint8 | - | - | 1 |
|   | 2h | Rotation Direction Selection | RW | - | Uint16 | - | - | 8 |

| Index | Subindex | Name | Accessibility | Mappable | Data Type | Unit | Data Range | Factory Setting |
|---|---|---|---|---|---|---|---|---|
| 2008h | 0h | Gain Parameters | RO | NO | Uint8 | - | - | 10 |
|   | 1h | Velocity Loop kp | RW | - | Uint16 | 0.01 | Uint16 | 10 |
|   | 2h | Velocity Loop ki | RW | - | Uint16 | 0.01 | Uint16 | 100 |
|   | 3h | Velocity Loop kd | RW | - | Uint16 | 0.01 | Uint16 | 0 |
|   | 4h | Position Loop kp | RW | - | Uint16 | 0.01 | Uint16 | 2000 |
|   | 5h | Position Loop ki | RW | - | Uint16 | 0.01 | Uint16 | 0 |
|   | 6h | Position Loop kd | RW | - | Uint16 | 0.01 | Uint16 | 0 |
|   | 7h | Current Loop kp | RW | - | Uint16 | 0.01 | Uint16 | 0 |
|   | 8h | Current Loop ki | RW | - | Uint16 | 0.01 | Uint16 | 0 |
|   | 9h | Current Loop kd | RW | - | Uint16 | 0.01 | Uint16 | 0 |
|   | Ah | Current Loop Bandwidth | RW | - | Uint16 | 1 | Uint16 | 1000 |

| Index | Subindex | Name | Accessibility | Mappable | Data Type | Unit | Data Range | Factory Setting |
|---|---|---|---|---|---|---|---|---|
| 200Ah | 0h | Fault and Protection Parameters | RO | NO | Uint8 | - | - | 12 |
|   | 1h | MOSFET Temperature Protection Enable | RW | NO | Uint8 | 1 | - | 1 |
|   | 2h | Motor Temperature Protection Enable | RW | NO | Uint8 | 1 | - | 1 |
|   | 3h | MOSFET Temperature Protection Lower Limit | RW | NO | Uint16 | 0.1℃ | - | 1000 |
|   | 4h | MOSFET Temperature Protection Upper Limit | RW | NO | Uint16 | 0.1℃ | - | 1200 |
|   | 5h | Motor Temperature Protection Lower Limit | RW | NO | Uint16 | 0.1℃ | - | 1400 |
|   | 6h | Motor Temperature Protection Upper Limit | RW | NO | Uint16 | 0.1℃ | - | 1500 |
|   | 7h | Maximum Bus Current | RW | NO | UINT16 | 0.1% of rated current | - | 2000 |
|   | 8h | Maximum Reverse Bus Current | RW | NO | INT16 | 0.1% of rated current | - | -5000 |
|   | 9h | Maximum Bus Voltage | RO | NO | Uint32 | mV | - | 64000 |
|   | Ah | Minimum Bus Voltage | RO | NO | Uint32 | mV | - | 15000 |
|   | Bh | Speed Limit Switch Enable | RW | NO | Uint8 | - | - | 1 |
|   | Ch | Speed Limit Switch Enable in Torque Mode | RW | NO | Uint8 | - | - | 1 |

| Index | Subindex | Name | Accessibility | Mappable | Data Type | Unit | Data Range | Factory Setting |
|---|---|---|---|---|---|---|---|---|
| 200Bh | 0h | Monitoring Parameters | RO | NO | Uint8 | - | - | 3 |
|   | 1h | MOSFET Temperature Value | RO | NO | Int16 | 0.1℃ | Int16 | 0 |
|   | 2h | Motor Temperature Value | RO | NO | Int16 | 0.1℃ | Int16 | 0 |
|   | 3h | Bus Current Value | RO | NO | INT16 | command unit | INT16 | 0 |

| Index | Subindex | Name | Accessibility | Mappable | Data Type | Unit | Data Range | Factory Setting |
|---|---|---|---|---|---|---|---|---|
| 200Eh | 0h | Communication Function Parameters | RO | NO | Uint8 | - | - | 8 |
|   | 1h | Node Address | RW | - | Uint32 | - | 1-127 | 1 |
|   | 2h | CAN Baud Rate | RW | NO | Uint32 | bps | - | 500000 |

| Index | Subindex | Name | Accessibility | Mappable | Data Type | Unit | Data Range | Factory Setting |
|---|---|---|---|---|---|---|---|---|
| 203Eh | - | Extended Servo Fault Code | RO | TPDO | Uint32 | - | 0~65535 | 0 |
| 203Fh | - | Servo Fault Code | RO | TPDO | Uint32 | - | 0~65535 | 0 |

#### 3.10.3 Object Group 6000h Assignment List

| Index | Subindex | Name | Accessibility | Mappable | Data Type | Unit | Data Range | Factory Setting |
|---|---|---|---|---|---|---|---|---|
| 603Fh | - | Fault Code | RO | TPDO | Uint16 | - | 0~65535 | 0 |
| 6040h | - | Control Word | RW | YES | Uint16 | - | 0~65535 | 0 |
| 6041h | - | Status Word | RO | TPDO | Uint16 | - | 0~65535 | 0 |
| 605Ah | - | Quick Stop Mode Selection | RW | NO | Int16 | - | 0~7 | 2 |
| 605Dh | - | Halt Mode Selection | RW | NO | Int16 | - | 0~7 | 1 |
| 6060h | - | Mode Selection | RW | YES | Int8 | - | 0~10 | 1 |
| 6061h | - | Mode Display | RO | TPDO | Int8 | - | 0~10 | 2 |
| 6062h | - | User Position Command | RO | TPDO | Int32 | command unit | -231~(231-1) | 0 |
| 6064h | - | Current Actual Position | RO | TPDO | Int32 | command unit | -231~(231-1) | 0 |
| 6067h | - | Position Reached Threshold | RW | YES | Uint32 | command unit | 0~(232-1) | 0 |
| 6069h | - | Actual Speed Sensor Value | RO | TPDO | Int32 | command unit | -231~(231-1) | 0 |
| 606Bh | - | User Velocity Command | RO | TPDO | Int32 | command unit | -231~(231-1) | 0 |
| 606Ch | - | Current Actual Speed | RO | TPDO | Int32 | command unit | -231~(231-1) | 0 |
| 6071h | - | Target Torque | RW | RPDO | INT16 | 0.1% of rated torque | -215~(215-1) | 0 |
| 6072h | - | Maximum Torque | RW | RPDO | Uint16 | 0.1% of rated torque | -10000~10000 | 0 |
| 6073h | - | Maximum Current | RW | RPDO | Uint16 | 0.1% of rated torque | 0~(216-1) | 6000 |
| 6074h | - | User Torque Command | RO | TPDO | INT16 | 0.1% of rated torque | -215~(215-1) | 0 |
| 6075h | - | Rated Current Value | RW | YES | UINT32 | 0.1% | 0~(232-1) | 10000 |
| 6076h | - | Motor Rated Torque | RW | YES | Uint32 | 0.1% | 0~(232-1) | 10000 |
| 6077h | - | Current Actual Torque | RO | TPDO | INT16 | 0.1% of rated torque | -215~(215-1) | 0 |
| 6078h | - | Actual Current Value | RO | TPDO | INT16 | 0.1% of rated current | - | 0 |
| 6079h | - | Bus Voltage | RO | TPDO | Uint32 | mV | - | 0 |
| 607Ah | - | Target Position | RW | RPDO | Int32 | command unit | -231~(231-1) | 0 |
| 607Ch | - | Home Offset | RW | YES | Int32 | command unit | -231~(231-1) | 0 |
| 607Dh | 0h | Software Position Limit | RO | NO | Uint8 | - | - | 2 |
|   | 1h | Minimum Position Limit | RW | YES | Int32 | command unit | -231~(231-1) | 0 |
|   | 2h | Maximum Position Limit | RW | YES | Int32 | command unit | -231~(231-1) | 0 |
| 607Eh | - | Command Polarity | RW | YES | Uint8 | - | 0-255 | 0 |
| 607Fh | - | Maximum Profile Velocity | RW | YES | Uint32 | command unit | 0~(232-1) | 0 |
| 6080h | - | Maximum Motor Speed | RW | RPDO | Uint32 | command unit | 0~(232-1) | 1000 |
| 6081h | - | Profile Velocity | RW | YES | Uint32 | command unit | 0~(232-1) | 163840 |
| 6083h | - | Profile Acceleration | RW | YES | Uint32 | command unit | 0~(232-1) | 245760 |
| 6084h | - | Profile Deceleration | RW | YES | Uint32 | command unit | 0~(232-1) | 245760 |
| 6085h | - | Quick Stop Deceleration | RW | YES | Uint32 | command unit | 0~(232-1) | 245760 |
| 6086h | - | Motor Motion Profile Type | RW | YES | Int16 | - | -231~(231-1) | 0 |
| 6087h | - | Torque Slope | RW | RPDO | UINT32 | 0.1%/s of rated torque | 0~(232-1) | 100 |
| 608Fh | 0h | Position Encoder Resolution | RO | NO | Uint8 | - | - | 2 |
|   | 1h | Encoder Resolution | RW | TPDO | Uint32 | command unit | -231~(231-1) | 16384 |
|   | 2h | Motor Speed | RW | TPDO | Uint32 | command unit | -231~(231-1) | 1 |
| 6091h | 0h | Gear Ratio | RO | NO | UNIT8 | - | - | 2 |
|   | 1h | Motor Resolution | RW | RPDO | Uint32 | - | 1~(232-1) | 8 |
|   | 2h | Load Shaft Resolution | RW | RPDO | Uint32 | - | 1~(232-1) | 1 |
| 6098h | - | Homing Mode | RW | YES | Int8 | - | 0~35 | 0 |
| 6099h | 0h | Homing Speed | RO | NO | UNIT8 | - | - | 2 |
|   | 1h | Search Deceleration Point Signal Speed | RW | YES | Uint32 | command unit | 0~(232-1) | 65536 |
|   | 2h | Search Zero Point Signal Speed | RW | YES | Uint32 | command unit | 0~(232-1) | 0 |
| 609Ah | - | Homing Acceleration | RW | YES | Uint32 | command unit | 0~(232-1) | 65536 |
| 60B0h | - | Position Offset | RW | YES | Int32 | command unit | -231~(231-1) | 0 |
| 60B1h | - | Velocity Offset | RW | YES | Int32 | command unit | -231~(231-1) | 0 |
| 60B2h | - | Torque Offset | RW | YES | Int16 | 0.1% of rated torque | -215~(215-1) | 0 |
| 60C2h | 0h | Interpolation Time | RO | NO | Uint8 | - | - | 2 |
|   | 1h | Interpolation Time Unit | RW | YES | Uint8 | - | 0~255 | 1 |
|   | 2h | Interpolation Time Index | RW | YES | Int8 | - | -128~63 | 0 |
| 60C5h | - | Maximum Profile Acceleration | RW | YES | Uint32 | command unit | 0~(232-1) | 0 |
| 60C6h | - | Maximum Profile Deceleration | RW | YES | Uint32 | command unit | 0~(232-1) | 0 |
| 60F4h | - | User Position Deviation | RO | TPDO | Int32 | command unit | -231~(231-1) | 0 |
| 60FCh | - | Motor Position Command* | RO | TPDO | Int32 | command unit | -231~(231-1) | 0 |
| 60FFh | - | Target Velocity | RW | RPDO | Int32 | command unit | -231~(231-1) | 0 |
| 6502h | - | Supported Control Modes | RO | NO | UINT32 | - | - | 0x761 |
### 3.13 Object Dictionary to Motor Parameter Mapping Table

For an explanation of command units, refer to the conversion factor section of the [ CyberBeast Drive CANOpen Supplementary Manual](https://bcnyljrhe70u.feishu.cn/wiki/RkxswgglNiX2mckH9CdcVNmQnXe).

| Index | Subindex | Name | Type | Unit | Corresponding Motor Parameter |
|---|---|---|---|---|---|
| 6040h | - | Control word | Uint16 | - | odrv0.axis0.requested_state = 8<br>The servo's running state corresponds to Motor Genie's closed-loop control mode,<br>odrv0.axis0.requested_state = 1<br>other states in the servo state machine correspond to Motor Genie's idle state |
| 6041h | - | Status word | Uint16 | - | Same as above<br>In Profile Position mode, bit 10 of the status word indicates whether the target position has been reached<br>In Homing mode, bits 10 and 12 of the status word indicate whether the configured home position has been reached |
| 6060h | - | Mode of operation | Int8 | - | odrv0.axis0.controller.config.control_mode<br>odrv0.axis0.controller.config.input_mode<br>For the specific mapping, refer to Mode of Operation 6060 in the CyberBeast Drive CANOpen Supplementary Manual. |
| 6061h | - | Mode of operation display | Int8 | - | Same as above |
| 6098h | - | Homing method | Int8 | - | Currently supported homing methods are 17 and 35 |
| 6064h | - | Position actual value | int32 | command unit | odrv0.axis0.encoder.pos_estimate*conversion factor<br>Current position value; for detailed usage, refer to the conversion factor section of the [ CyberBeast Drive CANOpen Supplementary Manual](https://bcnyljrhe70u.feishu.cn/wiki/RkxswgglNiX2mckH9CdcVNmQnXe). |
| 607Ah | - | Target position | int32 | command unit | odrv0.axis0.controller.input_pos<br>Input position |
| 60FFh | - | Target velocity | int32 | command unit | odrv0.axis0.controller.input_vel<br>Input velocity |
| 6071h | - | Target torque | int16 | 0.001Nm*rated torque | odrv0.axis0.controller.input_torque<br>Input torque (rotor-side unit) |
| 6076h | - | Rated torque | Uint32 | 0.1% | Default motor rated torque is 10NM |
| 6077h | - | Torque actual value | int16 | command unit | odrv0.axis0.motor.current_control.Iq_measured* axis.motor_.config_.torque_constant * axis.motor_.config_.gear_ratio<br>Motor-side torque (motor-side unit) |
| 606Ch | - | Velocity actual value | int32 | command unit | odrv0.axis0.encoder.vel_estimate_counts<br>Current velocity |
| 608Fh | 1h | Encoder resolution | Uint32 | - | Equivalent to the conversion factor; for detailed usage, refer to the conversion factor section of the [ CyberBeast Drive CANOpen Supplementary Manual](https://bcnyljrhe70u.feishu.cn/wiki/RkxswgglNiX2mckH9CdcVNmQnXe). |
| 6081h | - | Profile velocity | Uint32 | command unit | odrv0.axis0.trap_traj.config.vel_limit<br>Velocity limit. |
| 6083h | - | Profile acceleration | Uint32 | command unit | odrv0.axis0.trap_traj.config.accel_limit<br>Acceleration limit for trapezoidal position control. |
| 6084h | - | Profile deceleration | Uint32 | command unit | odrv0.axis0.trap_traj.config.decel_limit<br>Deceleration limit for trapezoidal position control. |
| 607Ch | - | Home offset | Int32 | command unit | odrv0.axis0.min_endstop.config_.offset<br>Home offset |
| 6099H | 1h | Homing speed | Uint32 | command unit | odrv0.axis0.controller.config.homing_speed<br>Speed for searching the homing signal |
| 609Ah | - | Homing acceleration | Uint32 | command unit | odrv0.axis0.controller.config.vel_ramp_rate<br>Acceleration for searching the homing signal |
| 6087h | - | Torque slope | Uint32 | 0.001Nm/s*rated torque | odrv0.axis0.controller_.config_.torque_ramp_rate<br>Current loop ramp rate |
| 6080h | - | Max motor speed | UINT32 | command unit | odrv0.axis0.controller.config.vel_limit<br>Maximum motor speed |
| 60C5h | - | Max acceleration | Uint32 | command unit | odrv0.axis0.controller.config.vel_ramp_rate<br>Maximum motor acceleration |
| 6075h | - | Rated current value | UINT32 | 0.1% | Default motor rated current is 10A |
| 6078h | - | Current actual value | INT16 | command unit | odrv0.axis0.motor.current_control.Iq_measured<br>Iq current; for detailed usage, refer to the conversion factor section of the [ CyberBeast Drive CANOpen Supplementary Manual](https://bcnyljrhe70u.feishu.cn/wiki/RkxswgglNiX2mckH9CdcVNmQnXe). |
| 6073h | - | Max current | UINT16 | command unit | odrv0.axis0.motor.config.current_lim<br>Motor current limit. |
| 6079h | - | Bus voltage | Uint32 | 0.1% | odrv0.vbus_voltage<br>Bus voltage |
| 6091h | 1h | Gear ratio | Uint32 | - | odrv0.axis0.motor.config.gear_ratio<br>Reduction ratio |
| 6085h | - | Quick stop deceleration | Uint32 | command unit | odrv0.axis0.controller_.config_.vel_ramp_rate<br>Quick stop deceleration. |

| 2002h | 1h | Input mode selection | Uint16 | - | odrv0.axis0.controller.config.input_mode<br>Input command type: position filter, ramp, direct, etc. |
|---|---|---|---|---|---|
| 2008h | 10h | Current loop bandwidth | Uint16 | 0.01 | odrv0.axis0.motor.config.current_control_bandwidth<br>Current loop bandwidth |
| 2008h | 1h | Velocity loop kp | Uint16 | 0.01 | odrv0.axis0.controller.config.vel_gain<br>Velocity loop kp |
| 2008h | 2h | Velocity loop ki | Uint16 | 0.01 | odrv0.axis0.controller.config.vel_integrator_gain<br>Velocity loop ki |
| 2008h | 3h | Velocity loop kd | Uint16 | 0.01 | odrv0.axis0.controller.config.vel_diff_gain<br>Velocity loop kd |
| 2008h | 4h | Position loop kp | Uint16 | 0.01 | odrv0.axis0.controller.config.pos_gain<br>Position loop kp |
| 2008h | 5h | Position loop ki | Uint16 | 0.01 | odrv0.axis0.controller.config.pos_integrator_gain<br>Position loop ki |
| 2008h | 6h | Position loop kd | Uint16 | 1 | odrv0.axis0.controller.config.pos_diff_gain<br>Position loop kd |
| 200B h | 3h | Bus current value | Int16 | 0.1°C | odrv0.ibus<br>Bus current |
| 200B h | 2h | Motor temperature value | Int16 | 0.1°C | odrv0.axis0.motor.motor_thermistor.temperature<br>Motor temperature |
| 200B h | 1h | MOSFET temperature value | INT16 | command unit | odrv0.axis0.motor.fet_thermistor.temperature<br>Drive board temperature |
| 200Ah | 1h | MOSFET temperature protection enable | Uint8 | - | odrv0.axis0.motor.fet_thermistor.config.enabled<br>Drive board temperature protection enable |
| 200Ah | 2h | Motor temperature protection enable | Uint8 | - | odrv0.axis0.motor.motor_thermistor.config.enabled<br>Motor temperature protection enable |
| 200Ah | 3h | MOSFET temperature protection lower limit | Uint16 | 0.1°C | odrv0.axis0.motor.fet_thermistor.config.temp_limit_lower<br>Drive temperature protection lower limit |
| 200Ah | 4h | MOSFET temperature protection upper limit | Uint16 | 0.1°C | odrv0.axis0.motor.fet_thermistor.config.temp_limit_upper<br>Drive temperature protection upper limit |
| 200Ah | 5h | Motor temperature protection lower limit | Uint16 | 0.1°C | odrv0.axis0.motor.motor_thermistor.config.temp_limit_lower    <br>Motor temperature protection lower limit |
| 200Ah | 6h | Motor temperature protection upper limit | Uint16 | 0.1°C | odrv0.axis0.motor.motor_thermistor.config.temp_limit_upper   <br>Motor temperature protection threshold upper limit |
| 200Ah | 9h | Max bus voltage | FLoat32 | 0.1% | odrv0.config.dc_bus_undervoltage_trip_level<br>Acceptable upper limit of bus input voltage |
| 200Ah | 10h | Min bus voltage | FLoat32 | 0.1% | odrv0.config.dc_bus_overvoltage_trip_level<br>Acceptable lower limit of bus input voltage |
| 200Ah | 7h | Max bus current | FLoat32 | command unit | odrv0.config.dc_max_positive_current<br>Maximum supply current allowed by the power source |
| 200Ah | 8h | Max regenerative bus current | FLoat32 | command unit | odrv0.config.dc_max_negative_current<br>Maximum regenerative charging current allowed by the power source (negative value) |
| 200Ah | 11h | Speed limit switch enable | Uint8 | - | odrv0.axis0.controller.config.enable_vel_limit<br>Whether speed limiting is enabled |
| 200Ah | 12h | Speed limit switch enable in torque mode | Uint8 |   | odrv0.axis0.controller.config.enable_torque_mode_vel_limit<br>Whether speed is limited when torque control is enabled |
| 2000h | 1h | Pole pairs | Int32 | - | odrv0.axis0.motor.config.pole_pairs<br>Pole pairs |
| 2000h | 2h | Torque constant | FLoat32 | - | odrv0.axis0.motor.config.torque_constant<br>Torque constant |
| 2000h | 3h | Phase resistance | FLoat32 | - | odrv0.axis0.motor.config.phase_resistance<br>Motor phase resistance |
| 2000h | 4h | Phase inductance | FLoat32 | - | odrv0.axis0.motor.config.phase_inductance<br>Motor phase inductance |
| 200Eh | 01h | Node address | Uint32 | - | odrv0.axis0.config.can.node_id<br>can id |
| 200Eh | 02h | CAN baud rate | Uint32 | bps | odrv0.can.config.baud_rate<br>CAN baud rate |

Note: For detailed explanations of motor parameters, see the [User Manual](https://cyberbeast.feishu.cn/docx/N3SMd4QyRobzHkx3wP3cT1qXnpf)
### 3.14 Fault Code Reference Table

When Bit 3 of status word 6041 is set to 1, it indicates that a fault has occurred; refer to the table below.

When the drive experiences an error consistent with the CiA402 sub-protocol, 603Fh follows the CiA402 protocol specification, as detailed in the table below. When the drive experiences a manufacturer-defined exception condition, 603Fh is 0xFF00, and object dictionary 203Fh additionally presents the fault code as 32-bit data; if the fault code exceeds 32 bits, the high-order bits are presented by object dictionary 203Eh.

Error code major category (MSB): predefined by the CiA 402 protocol as 1001h, used for an initial classification of errors and to specify the particular cause of the error within that category. Its bit assignment rules are as follows:

| Error Bit | Meaning | Explanation |
|---|---|---|
| 0 | General error | Bit 0 must always be "1" whenever an error is present |
| 1 | Current | If this bit is "1", it indicates a current error |
| 2 | Voltage | If this bit is "1", it indicates a voltage error |
| 3 | Temperature | If this bit is "1", it indicates a temperature error |
| 4 | Communication error | If this bit is "1", it indicates a communication error |
| 5 | CiA402 standard error | If this bit is "1", it indicates an error consistent with the CiA402 sub-protocol; refer to 603F |
| 6 | Reserved | Always 0 |
| 7 | Manufacturer-defined | If this bit is "1", it indicates a user-defined error; refer to 203F |

| Error Category | ODrive Error Code | odrivetool Display | 0x603F | Description | 0x1001 Error Register Bit | 0x203E (Upper 32 Bits) + 0x203F (Lower 32 Bits) |
|---|---|---|---|---|---|---|
| System Fault | 0x00000001 | CONTROL_ITERATION_MISSED | 0x5220 | CONTROL_COMPUTING_CIRCUIT | Bit 5 (Device profile specific) | 0x00000001 |
|   | 0x00000002 | DC_BUS_UNDER_VOLTAGE | 0x3220 | DC_LINK_UNDER_VOLTAGE | Bit 2 (Voltage) | 0x00000002 |
|   | 0x00000004 | DC_BUS_OVER_VOLTAGE | 0x3210 | DC_LINK_OVER_VOLTAGE | Bit 2 (Voltage) | 0x00000004 |
|   | 0x00000008 | DC_BUS_OVER_REGEN_CURRENT | 0x2311 | CONTINUOUS_OVER_CURRENT_NO1 | Bit 1 (Current) | 0x00000008 |
|   | 0x00000010 | DC_BUS_OVER_CURRENT | 0x2311 | CONTINUOUS_OVER_CURRENT_NO1 | Bit 1 (Current) | 0x00000010 |
|   | 0x00000020 | BRAKE_DEADTIME_VIOLATION | 0x7111 | FAILURE_BRAKE_CHOPPER | Bit 5 (Device profile specific) | 0x00000020 |
|   | 0x00000040 | BRAKE_DUTY_CYCLE_NAN | 0x7111 | FAILURE_BRAKE_CHOPPER | Bit 5 (Device profile specific) | 0x00000040 |
|   | 0x00000080 | INVALID_BRAKE_RESISTANCE | 0x6320 | PARAMETER_ERROR | Bit 5 (Device profile specific) | 0x00000080 |
| Drive Fault | 0x00000001 | INVALID_STATE | 0xFF00 | MANUFACTURER_SPECIFIC | Bit 7 (Manufacturer-specific) | 0x00000001 |
|   | 0x00000040 | MOTOR_FAILED | 0x7122 | MOTOR_ERROR | Bit 5 (Device profile specific) | 0x00000040 |
|   | 0x00000080 | SENSORLESS_ESTIMATOR_FAILED | 0xFF00 | MANUFACTURER_SPECIFIC | Bit 7 (Manufacturer-specific) | 0x00000080 |
|   | 0x00000100 | ENCODER_FAILED | 0x7300 | SENSOR_ERROR | Bit 5 (Device profile specific) | 0x00000100 |
|   | 0x00000200 | CONTROLLER_FAILED | 0x8A00 | CONTROL | Bit 5 (Device profile specific) | 0x00000200 |
|   | 0x00000800 | WATCHDOG_TIMER_EXPIRED | 0x6010 | SOFTWARE_RESET_WATCHDOG | Bit 0 (Generic error) | 0x00000800 |
|   | 0x00001000 | MIN_ENDSTOP_PRESSED | 0x8612 | REFERENCE_LIMIT | Bit 5 (Device profile specific) | 0x00001000 |
|   | 0x00002000 | MAX_ENDSTOP_PRESSED | 0x8612 | REFERENCE_LIMIT | Bit 5 (Device profile specific) | 0x00002000 |
|   | 0x00004000 | ESTOP_REQUESTED | 0xFF00 | MANUFACTURER_SPECIFIC | Bit 7 (Manufacturer-specific) | 0x00004000 |
|   | 0x00020000 | HOMING_WITHOUT_ENDSTOP | 0xFF00 | MANUFACTURER_SPECIFIC | Bit 7 (Manufacturer-specific) | 0x00020000 |
|   | 0x00040000 | OVER_TEMP | 0x4310 | EXCESS_TEMPERATURE_DRIVE | Bit 3 (Temperature) | 0x00040000 |
|   | 0x00080000 | UNKNOWN_POSITION | 0x7320 | POSITION | Bit 5 (Device profile specific) | 0x00080000 |
|   | 0x00100000 | CAN_BUS_FAILED | 0x7500 | COMMUNICATION | Bit 4 (Communication error ) | 0x00100000 |
| Motor Fault | 0x00000001 | PHASE_RESISTANCE_OUT_OF_RANGE | 0x7122 | MOTOR_OR_COMMUTATION_MALFUNC | Bit 5 (Device profile specific) | 0x00000001 |
|   | 0x00000002 | PHASE_INDUCTANCE_OUT_OF_RANGE | 0x7122 | MOTOR_OR_COMMUTATION_MALFUNC | Bit 5 (Device profile specific) | 0x00000002 |
|   | 0x00000008 | DRV_FAULT | 0xFF00 | MANUFACTURER_SPECIFIC | Bit 7 (Manufacturer-specific) | 0x00000008 |
|   | 0x00000010 | CONTROL_DEADLINE_MISSED | 0xFF00 | MANUFACTURER_SPECIFIC | Bit 7 (Manufacturer-specific) | 0x00000010 |
|   | 0x00000080 | MODULATION_MAGNITUDE | 0xFF00 | MANUFACTURER_SPECIFIC | Bit 7 (Manufacturer-specific) | 0x00000080 |
|   | 0x00000400 | CURRENT_SENSE_SATURATION | 0x2311 | CONTINUOUS_OVER_CURRENT_NO1 | Bit 1 (Current) | 0x00000400 |
|   | 0x00001000 | CURRENT_LIMIT_VIOLATION | 0x2311 | CONTINUOUS_OVER_CURRENT_NO1 | Bit 1 (Current) | 0x00001000 |
|   | 0x00010000 | MODULATION_IS_NAN | 0xFF00 | MANUFACTURER_SPECIFIC | Bit 7 (Manufacturer-specific) | 0x00010000 |
|   | 0x00020000 | MOTOR_THERMISTOR_OVER_TEMP | 0x4210 | EXCESS_TEMPERATURE_DEVICE | Bit 3 (Temperature) | 0x00020000 |
|   | 0x00040000 | FET_THERMISTOR_OVER_TEMP | 0x4310 | EXCESS_TEMPERATURE_DRIVE | Bit 3 (Temperature) | 0x00040000 |
|   | 0x00080000 | TIMER_UPDATE_MISSED | 0xFF00 | MANUFACTURER_SPECIFIC | Bit 7 (Manufacturer-specific) | 0x00080000 |
|   | 0x00100000 | CURRENT_MEASUREMENT_UNAVAILABLE | 0xFF00 | MANUFACTURER_SPECIFIC | Bit 1 (Current) | 0x00100000 |
|   | 0x00200000 | CONTROLLER_FAILED | 0x8A00 | CONTROL | Bit 5 (Device profile specific) | 0x00200000 |
|   | 0x00400000 | I_BUS_OUT_OF_RANGE | 0x2311 | CONTINUOUS_OVER_CURRENT_NO1 | Bit 1 (Current) | 0x00400000 |
|   | 0x00800000 | BRAKE_RESISTOR_DISARMED | 0x7110 | BRAKE_CHOPPER | Bit 5 (Device profile specific) | 0x00800000 |
|   | 0x01000000 | SYSTEM_LEVEL | 0xFF00 | MANUFACTURER_SPECIFIC | Bit 7 (Manufacturer-specific) | 0x01000000 |
|   | 0x02000000 | BAD_TIMING | 0x5220 | CONTROL_COMPUTING_CIRCUIT | Bit 5 (Device profile specific) | 0x02000000 |
|   | 0x04000000 | UNKNOWN_PHASE_ESTIMATE | 0x7320 | POSITION | Bit 5 (Device profile specific) | 0x04000000 |
|   | 0x08000000 | UNKNOWN_PHASE_VEL | 0x7310 | SPEED | Bit 5 (Device profile specific) | 0x08000000 |
|   | 0x10000000 | UNKNOWN_TORQUE | 0x8331 | TORQUE_FAULT | Bit 5 (Device profile specific) | 0x10000000 |
|   | 0x20000000 | UNKNOWN_CURRENT_COMMAND | 0x8331 | TORQUE_FAULT | Bit 5 (Device profile specific) | 0x20000000 |
|   | 0x40000000 | UNKNOWN_CURRENT_MEASUREMENT | 0x5210 | CONTROL_MEASUREMENT_CIRCUIT | Bit 1 (Current) | 0x40000000 |
|   | 0x80000000 | UNKNOWN_VBUS_VOLTAGE | 0x5210 | CONTROL_MEASUREMENT_CIRCUIT | Bit 2 (Voltage) | 0x80000000 |
|   | 0x100000000 | UNKNOWN_VOLTAGE_COMMAND | 0x8A00 | CONTROL | Bit 2 (Voltage) | 0x100000000 |
|   | 0x200000000 | UNKNOWN_GAINS | 0x6320 | PARAMETER_ERROR | Bit 5 (Device profile specific) | 0x200000000 |
|   | 0x400000000 | CONTROLLER_INITIALIZING | 0x8A00 | CONTROL | Bit 5 (Device profile specific) | 0x400000000 |
|   | 0x800000000 | UNBALANCED_PHASES | 0x3130 | PHASE_FAILURE  | Bit 5 (Device profile specific) | 0x800000000 |
| Control Fault | 0x00000001 | OVERSPEED | 0x8400 | VELOCITY_SPEED_CONTROLLER | Bit 5 (Device profile specific) | 0x00000001 |
|   | 0x00000002 | INVALID_INPUT_MODE | 0x6320 | PARAMETER_ERROR | Bit 5 (Device profile specific) | 0x00000002 |
|   | 0x00000004 | UNSTABLE_GAIN | 0x6320 | PARAMETER_ERROR | Bit 5 (Device profile specific) | 0x00000004 |
|   | 0x00000008 | INVALID_MIRROR_AXIS | 0xFF00 | MANUFACTURER_SPECIFIC | Bit 7 (Manufacturer-specific) | 0x00000008 |
|   | 0x00000010 | INVALID_LOAD_ENCODER | 0x7300 | SENSOR_ERROR | Bit 5 (Device profile specific) | 0x00000010 |
|   | 0x00000020 | INVALID_ESTIMATE | 0x8611 | FOLLOWING_ERROR | Bit 5 (Device profile specific) | 0x00000020 |
|   | 0x00000040 | INVALID_CIRCULAR_RANGE | 0x6320 | PARAMETER_ERROR | Bit 5 (Device profile specific) | 0x00000040 |
|   | 0x00000080 | SPINOUT_DETECTED | 0x6320 | PARAMETER_ERROR | Bit 5 (Device profile specific | 0x00000080 |
| Encoder Fault | 0x00000001 | UNSTABLE_GAIN | 0x6320 | PARAMETER_ERROR | Bit 5 (Device profile specific) | 0x00000001 |
|   | 0x00000002 | CPR_POLEPAIRS_MISMATCH | 0x6320 | PARAMETER_ERROR | Bit 5 (Device profile specific) | 0x00000002 |
|   | 0x00000004 | NO_RESPONSE | 0x7305 | INCREMENTAL_SENSOR_1_FAULT | Bit 5 (Device profile specific) | 0x00000004 |
|   | 0x00000008 | UNSUPPORTED_ENCODER_MODE | 0x6320 | PARAMETER_ERROR | Bit 5 (Device profile specific) | 0x00000008 |
|   | 0x00000010 | ILLEGAL_HALL_STATE | 0x7300 | SENSOR_ERROR | Bit 5 (Device profile specific) | 0x00000010 |
|   | 0x00000020 | INDEX_NOT_FOUND_YET | 0x7305 | INCREMENTAL_SENSOR_1_FAULT | Bit 5 (Device profile specific) | 0x00000020 |
|   | 0x00000040 | ABS_SPI_TIMEOUT | 0x7305 | INCREMENTAL_SENSOR_1_FAULT | Bit 5 (Device profile specific) | 0x00000040 |
|   | 0x00000080 | ABS_SPI_COM_FAIL | 0x7305 | INCREMENTAL_SENSOR_1_FAULT | Bit 5 (Device profile specific) | 0x00000080 |
|   | 0x00000100 | ABS_SPI_NOT_READY | 0x7305 | INCREMENTAL_SENSOR_1_FAULT | Bit 5 (Device profile specific) | 0x00000100 |
|   | 0x00000200 | HALL_NOT_CALIBRATED_YET | 0x7300 | SENSOR_ERROR | Bit 5 (Device profile specific) | 0x00000200 |
|   | 0x00000400 | SEC_ENC_COM_FAIL | 0x7306 | INCREMENTAL_SENSOR_2_FAULT | Bit 5 (Device profile specific) | 0x00000400 |

## 4 Exception Codes (To Be Updated)

### 4.1 Exception Codes

| Error Category | Error Code | odrivetool Display | Description |
|---|---|---|---|
| System Fault | 0x00000002 | DC_BUS_UNDER_VOLTAGE | Supply voltage too low |
|   | 0x00000004 | DC_BUS_OVER_VOLTAGE | Supply voltage too high |
|   | 0x00000008 | DC_BUS_OVER_REGEN_CURRENT | Reverse (charging) supply current too high |
|   | 0x00000010 | DC_BUS_OVER_CURRENT | Forward (discharging) supply current too high |
| Drive Fault | 0x00000001 | INVALID_STATE | Invalid drive state |
|   | 0x00000040 | MOTOR_FAILED | Motor fault |
|   | 0x00000100 | ENCODER_FAILED | Encoder fault |
|   | 0x00000200 | CONTROLLER_FAILED | Controller fault |
|   | 0x00001000 | MIN_ENDSTOP_PRESSED | Lower limit switch triggered |
|   | 0x00002000 | MAX_ENDSTOP_PRESSED | Upper limit switch triggered |
|   | 0x00004000 | ESTOP_REQUESTED | Emergency stop |
|   | 0x00020000 | HOMING_WITHOUT_ENDSTOP | Homing requested but no limit switch present |
|   | 0x00080000 | UNKNOWN_POSITION | No position information |
| Motor Fault | 0x00000001 | PHASE_RESISTANCE_OUT_OF_RANGE | Phase-to-phase resistance out of normal range |
|   | 0x00000002 | PHASE_INDUCTANCE_OUT_OF_RANGE | Phase-to-phase inductance out of normal range |
|   | 0x00000010 | CONTROL_DEADLINE_MISSED | FOC frequency too high |
|   | 0x00000080 | MODULATION_MAGNITUDE | SVM modulation fault |
|   | 0x00000400 | CURRENT_SENSE_SATURATION | Phase current saturation |
|   | 0x00001000 | CURRENT_LIMIT_VIOLATION | Motor current too high |
|   | 0x00020000 | MOTOR_THERMISTOR_OVER_TEMP | Motor temperature too high |
|   | 0x00040000 | FET_THERMISTOR_OVER_TEMP | Drive temperature too high |
|   | 0x00080000 | TIMER_UPDATE_MISSED | FOC processing not completed in time |
|   | 0x00100000 | CURRENT_MEASUREMENT_UNAVAILABLE | Phase current sample lost |
|   | 0x00200000 | CONTROLLER_FAILED | Control fault |
|   | 0x00400000 | I_BUS_OUT_OF_RANGE | Bus current out of range |
|   | 0x00800000 | BRAKE_RESISTOR_DISARMED | Brake resistor drive fault |
|   | 0x01000000 | SYSTEM_LEVEL | System-level fault |
|   | 0x02000000 | BAD_TIMING | Phase current sampling not completed in time |
|   | 0x04000000 | UNKNOWN_PHASE_ESTIMATE | Motor position unknown |
|   | 0x08000000 | UNKNOWN_PHASE_VEL | Motor velocity unknown |
|   | 0x10000000 | UNKNOWN_TORQUE | Torque unknown |
|   | 0x20000000 | UNKNOWN_CURRENT_COMMAND | Torque control unknown |
|   | 0x40000000 | UNKNOWN_CURRENT_MEASUREMENT | Current sample value unknown |
|   | 0x80000000 | UNKNOWN_VBUS_VOLTAGE | Voltage sample value unknown |
|   | 0x100000000 | UNKNOWN_VOLTAGE_COMMAND | Voltage control unknown |
|   | 0x200000000 | UNKNOWN_GAINS | Current loop gain unknown |
|   | 0x400000000 | CONTROLLER_INITIALIZING | Controller initialization fault |
|   | 0x800000000 | UNBALANCED_PHASES | Three-phase imbalance |
| Control Fault | 0x00000001 | OVERSPEED | Speed too high |
|   | 0x00000002 | INVALID_INPUT_MODE | Incorrect control input mode |
|   | 0x00000004 | UNSTABLE_GAIN | PLL gain unstable |
|   | 0x00000020 | INVALID_ESTIMATE | Position/velocity unstable |
|   | 0x00000080 | SPINOUT_DETECTED | Mechanical power and electrical power mismatch (incorrect encoder calibration, or unstable magnet) |
| Encoder Fault | 0x00000001 | UNSTABLE_GAIN | Encoder bandwidth too high |
|   | 0x00000002 | CPR_POLEPAIRE_MISMATCH | CPR and pole pair count mismatch |
|   | 0x00000004 | NO_RESPONSE | Encoder not responding |
|   | 0x00000400 | SEC_ENC_COM_FAIL | Secondary encoder communication error |
