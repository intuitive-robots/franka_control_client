# RemoteFranka – Protocol Documentation

This README describes the binary protocol for the client franka robot.

---

## 1 RobotState Payload

Sent both as reply to `GET_STATE_REQ` and periodically via the PUB socket.
Messages are published under the topic `franka_arm`.

| Offset | Field                  | Type / Count   | Description                                             |
| -----: | ---------------------- | -------------- | ------------------------------------------------------- |
|      0 | `timestamp_ms`         | `uint32`       | timestamp \[ms]                                         |
|      4 | `O_T_EE`               | 16 × `float64` | 4×4 homogeneous EE pose                                 |
|    132 | `O_T_EE_d`             | 16 × `float64` | pose set‑point                                          |
|    260 | `q`                    |  7 × `float64` | joint angles actual \[rad]                              |
|    316 | `q_d`                  |  7 × `float64` | joint angles target \[rad]                              |
|    372 | `dq`                   |  7 × `float64` | joint velocities actual \[rad s⁻¹]                      |
|    428 | `dq_d`                 |  7 × `float64` | joint velocities target \[rad s⁻¹]                      |
|    484 | `tau_ext_hat_filtered` |  7 × `float64` | filtered external torque estimate \[N m]                |
|    540 | `O_F_ext_hat_K`        |  6 × `float64` | Cartesian force estimate \[N, N m]                      |
|    588 | `K_F_ext_hat_K`        |  6 × `float64` | desired wrench \[N, N m]                                |

Total size: **636 B**

Python struct: `!I 16d16d 7d7d7d7d7d 6d6d`

---

## 2 Message Identifiers (MsgID)

| Hex    | Symbol              | Direction | Description                       |
|--------|---------------------| --------- |-----------------------------------|
| `0x01` | `GET_STATE_REQ`     | C → S     | request a single state            |
| `0x02` | `QUERY_STATE_REQ`   | C → S     | ask for active control mode       |
| `0x03` | `START_CONTROL_REQ` | C → S     | switch to desired mode            |
| `0x04` | `GET_SUB_PORT_REQ`  | C → S     | query PUB port number             |
| `0x05` | `MOVE_TO_POSITION_REQ`                   |   C → S         | move robot to a specific position |
| `0x51` | `GET_STATE_RESP`    | S → C     | contains one                      |
| `0x52` | `QUERY_STATE_RESP`  | S → C     | 1 × `uint8` (see table 5)         |
| `0x53` | `START_CONTROL_RESP` | S → C     | 1 × `uint8` status (0 = OK)       |
| `0x54` | `GET_SUB_PORT_RESP` | S → C     | 2 × `uint8` TCP port (big‑endian) |
| `0xFF` | `ERROR`             | S → C     | 1 × error code                    |

---

## 3 ControlMode Constants

| Value | Symbol               | Meaning                                |
| ----: | -------------------- | -------------------------------------- |
|     0 | `CARTESIAN_POSITION` | control end‑effector position          |
|     1 | `CARTESIAN_VELOCITY` | control end‑effector velocity          |
|     2 | `JOINT_POSITION`     | control joint angles                   |
|     3 | `JOINT_VELOCITY`     | control joint velocities               |
|     4 | `HUMAN_MODE`         | free‑floating                          |

---
