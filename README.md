# FS-AI CAN Communication System - Complete Documentation

Complete bidirectional CAN communication system between STM32F1 AI Computer and Arduino Mega VCU Simulator.

**[View on GitHub](https://github.com/AkhilaNisal/FS-AI)**

## 📋 Table of Contents

- [Overview](#overview)
- [System Architecture](#system-architecture)
- [Hardware Configuration](#hardware-configuration)
- [CAN Protocol Specifications](#can-protocol-specifications)
- [Message Definitions](#message-definitions)
- [Data Frame Details](#data-frame-details)
- [File Structure](#file-structure)
- [Function Reference](#function-reference)
- [Arduino Code](#arduino-code)
- [Testing & Troubleshooting](#testing--troubleshooting)
- [Performance Metrics](#performance-metrics)
- [Getting Started](#getting-started)
- [Version History](#version-history)
- [License](#license)
- [Support & Contact](#support--contact)
- [Quick Reference](#quick-reference)

---

## Overview

The **FS-AI CAN Communication System** enables complete bidirectional communication between an **STM32F1 AI Computer** and an **Arduino Mega VCU Simulator** using the **MCP2515 CAN Controller** at **500 kbps**.

### ⚡ Key Features

✅ **Instant Command Response** - AI commands applied immediately (1-2 ms latency)  
✅ **Bidirectional Communication** - 12 total messages (7 VCU TX, 5 AI TX)  
✅ **Offline Detection** - Automatically detects STM32 disconnection after 2 seconds  
✅ **Vehicle Simulation** - Realistic dynamics simulation (acceleration, braking, steering)  
✅ **Real-time Monitoring** - Serial output every 1 second with detailed diagnostics  
✅ **100 Hz Message Rate** - 10 ms cycle time for all communications  

### 📊 Key Specifications

| Parameter | Value |
|-----------|-------|
| **CAN Bus Speed** | 500 kbps |
| **Message Cycle** | 10 ms (100 Hz) |
| **Total Messages** | 12 (7 VCU + 5 AI) |
| **Data Per Cycle** | 51 bytes |
| **Bandwidth Used** | ~4% of 500 kbps |
| **Response Latency** | Instant (1-2 ms) |
| **Timeout Detection** | 2000 ms |
| **Handshake Frequency** | Every 500 ms |

---

## System Architecture

### Block Diagram

```text
┌─────────────────────────────────────┐
│      STM32F1 AI COMPUTER            │
│  (Autonomous Vehicle Control)       │
│                                     │
│  • Path Planning                    │
│  • Decision Making                  │
│  • Sends: 0x510-0x514               │
│  • Receives: 0x520-0x526            │
└────────────┬────────────────────────┘
             │
             │ CAN Bus (500 kbps)
             │ 12 Messages / 10 ms
             │
┌────────────▼────────────────────────┐
│   Arduino Mega + MCP2515            │
│   (VCU Simulator)                   │
│                                     │
│  • Receives: 0x510-0x514            │
│  • Vehicle Dynamics Simulation      │
│  • Transmits: 0x520-0x526           │
│  • Wheel Speed Simulation           │
└─────────────────────────────────────┘
```
---

## Communication Timeline

0 ms    AI sends commands (0x510-0x514)
        ↓ 1 ms
1 ms    Arduino receives
        ↓ 2 ms
3 ms    Arduino applies instantly
        ↓ 7 ms
10 ms   Arduino sends feedback (0x520-0x526)
        ↓ 1 ms
11 ms   AI receives
        ↓
20 ms   Next TX cycle

---

## Hardware Configuration
Arduino Mega Pinout

|Pin	|Signal	|Device  	|Purpose          |
|-----|-------|---------|-----------------|
|51	|MOSI	|MCP2515 |SPI Data Out|
|50	|MISO	|MCP2515 |SPI Data In|
|52	|SCK	|MCP2515 |SPI Clock|
|10	|CS	|MCP2515 |Chip Select|
|2	|INT	|MCP2515 |Interrupt (RX Ready)|

Note: PC13 and PA4 are STM32-style pin names, not Arduino Mega pin names. Keep them only if they belong to the STM32 side of your system

MCP2515 Configuration
CAN Speed:     500 kbps
Crystal:       8 MHz
Mode:          Normal
Filters:       Accept All
RX Buffers:    2 (RXB0, RXB1)
TX Buffers:    3 (TXB0, TXB1, TXB2)
Interrupt:     RX Ready

---

## CAN Protocol Specifications
### Frame Format
Standard 11-bit CAN ID Frame

```text
┌───────┬────┬─────────────────────────────────┐
│ ID    │DLC │ DATA (0-8 bytes)                │
├───────┼────┼─────────────────────────────────┤
│11-bit │4-bit│ B0 B1 B2 B3 B4 B5 B6 B7        │
└───────┴────┴─────────────────────────────────┘
```

Byte Order: Little Endian (LSB first)

### Message ID Ranges
VCU → AI:  0x520 - 0x526 (7 messages)
AI  → VCU: 0x510 - 0x514 (5 messages)

### Data Encoding Standards

| Type     | Scale       | Range         | Example         |
| -------- | ----------- | ------------- | --------------- |
| Torque   | 0.1 Nm/unit | 0-195 Nm      | 50 Nm = 500     |
| Speed    | 1 RPM/unit  | 0-65535 RPM   | 2000 RPM = 2000 |
| Angle    | 0.1°/unit   | -210 to +210° | 5.5° = 55       |
| Pressure | 0.5%/unit   | 0-100%        | 50% = 100       |

---

## Message Definitions

### VCU → AI Messages (0x520-0x526)
**0x520: VCU Status**
Vehicle state, handshake, and AS state.
| Byte | Field         | Type     | Range | Description         |
| ---- | ------------- | -------- | ----- | ------------------- |
| 0    | Handshake     | bit 0    | 0-1   | Toggle every 500 ms |
| 0    | GO Signal     | bit 4    | 0-1   | RES go signal       |
| 1    | AS State      | bits 0-2 | 1-5   | System state        |
| 1    | AMI State     | bits 4-6 | 0-7   | Mission state       |
| 2    | Fault Flags   | bits 0-7 | 0-255 | Fault bitmask       |
| 3    | Warning Flags | bits 0-7 | 0-255 | Warning bitmask     |

DLC: 8 bytes
AS States: 1=OFF, 2=READY, 3=DRIVING, 4=E-BRAKE, 5=FINISHED

---

**0x521: Front Motor Feedback**
| Bytes | Field        | Type     | Scale  | Description    |
| ----- | ------------ | -------- | ------ | -------------- |
| 0-1   | Front Torque | int16_t  | 0.1 Nm | Current torque |
| 4-5   | Max Torque   | uint16_t | 0.1 Nm | Max capability |

DLC: 6 bytes

---

**0x522: Rear Motor Feedback**
| Bytes | Field       | Type     | Scale  | Description    |
| ----- | ----------- | -------- | ------ | -------------- |
| 0-1   | Rear Torque | int16_t  | 0.1 Nm | Current torque |
| 4-5   | Max Torque  | uint16_t | 0.1 Nm | Max capability |

DLC: 6 bytes

---

**0x523: Steering Feedback**
| Bytes | Field          | Type     | Scale | Description    |
| ----- | -------------- | -------- | ----- | -------------- |
| 0-1   | Steering Angle | int16_t  | 0.1°  | Current angle  |
| 2-3   | Max Angle      | uint16_t | 0.1°  | Max capability |

DLC: 6 bytes

---

**0x524: Brake Feedback**
| Byte | Field       | Type    | Scale | Description      |
| ---- | ----------- | ------- | ----- | ---------------- |
| 0    | Front Brake | uint8_t | 0.5%  | Current pressure |
| 2    | Rear Brake  | uint8_t | 0.5%  | Current pressure |

DLC: 5 bytes

---

**0x525: Wheel Speeds**
| Bytes | Field    | Type     | Scale | Description |
| ----- | -------- | -------- | ----- | ----------- |
| 0-1   | FL Speed | uint16_t | 1 RPM | Front-left  |
| 2-3   | FR Speed | uint16_t | 1 RPM | Front-right |
| 4-5   | RL Speed | uint16_t | 1 RPM | Rear-left   |
| 6-7   | RR Speed | uint16_t | 1 RPM | Rear-right  |

DLC: 8 bytes

---

**0x526: Encoder Counts**
| Bytes | Field    | Type     | Description        |
| ----- | -------- | -------- | ------------------ |
| 0-1   | FL Count | uint16_t | Front-left pulses  |
| 2-3   | FR Count | uint16_t | Front-right pulses |
| 4-5   | RL Count | uint16_t | Rear-left pulses   |
| 6-7   | RR Count | uint16_t | Rear-right pulses  |

DLC: 8 bytes

---

### AI → VCU Messages (0x510-0x514)
**0x510: AI Status**
| Byte | Field           | Type     | Range   | Description       |
| ---- | --------------- | -------- | ------- | ----------------- |
| 0    | Handshake       | bit 0    | 0-1     | AI heartbeat      |
| 0    | E-STOP          | bit 1    | 0-1     | Emergency stop    |
| 0    | Mission         | bits 2-3 | 0-3     | Mission state     |
| 0    | Direction       | bit 4    | 0-1     | Forward/Neutral   |
| 1    | Lap Counter     | bits 0-7 | 0-255   | Lap number        |
| 2    | Cones Hit       | bits 0-7 | 0-255   | Cones this lap    |
| 3-4  | Total Cones Hit | uint16_t | 0-65535 | Total cones count |

DLC: 8 bytes
Mission: 0=NOT_SELECTED, 1=SELECTED, 2=RUNNING, 3=FINISHED

---

**0x511: Front Motor Command**
| Bytes | Field  | Type     | Scale  | Description      |
| ----- | ------ | -------- | ------ | ---------------- |
| 0-1   | Torque | int16_t  | 0.1 Nm | Requested torque |
| 2-3   | Speed  | uint16_t | 1 RPM  | Requested speed  |

DLC: 4 bytes

---

**0x512: Rear Motor Command**
| Bytes | Field  | Type     | Scale  | Description      |
| ----- | ------ | -------- | ------ | ---------------- |
| 0-1   | Torque | int16_t  | 0.1 Nm | Requested torque |
| 2-3   | Speed  | uint16_t | 1 RPM  | Requested speed  |

DLC: 4 bytes

---

**0x513: Steering Command**
| Bytes | Field | Type    | Scale | Description     |
| ----- | ----- | ------- | ----- | --------------- |
| 0-1   | Angle | int16_t | 0.1°  | Requested angle |

DLC: 2 bytes

---

**0x514: Brake Command**
| Byte | Field | Type    | Scale | Description   |
| ---- | ----- | ------- | ----- | ------------- |
| 0    | Front | uint8_t | 0.5%  | Front brake % |
| 1    | Rear  | uint8_t | 0.5%  | Rear brake %  |

DLC: 2 bytes

---

## Data Frame Details
### Encoding Examples

**Example 1: 50.0 Nm Torque**
Value: 50.0 Nm
Raw:   50.0 × 10 = 500 (0x01F4)
LE:    [0xF4, 0x01]

**Example 2: 5.5° Steering**
Value: 5.5°
Raw:   5.5 × 10 = 55 (0x0037)
LE:    [0x37, 0x00]

**Example 3: -10.0° (Left Turn)**
Value: -10.0°
Raw:   -10.0 × 10 = -100 (0xFF9C)
LE:    [0x9C, 0xFF]

---

## File Structure

FS-AI/
  ├── main.c
  ├── main.h
  ├── ai_controller.c
  ├── ai_controller.h
  ├── can_transport.c
  ├── can_transport.h
  ├── can_interface.c
  ├── can_interface.h
  ├── fs_ai_protocol.c
  └── fs_ai_protocol.h

---

## Function Reference

### Core Update Functions

| Function                       | Purpose                                        |
| ------------------------------ | ---------------------------------------------- |
| `update_vcu_state_from_ai()`   | Applies AI command values to current VCU state |
| `simulate_vehicle_dynamics()`  | Simulates wheel speed and pulse counts         |
| `update_as_state_from_ai()`    | Updates autonomous system state from AI status |
| `check_stm32_alive()`          | Detects communication timeout                  |
| `print_bidirectional_status()` | Prints diagnostics to serial monitor           |

### Transmission Functions

| Function                     | Purpose                     |
| ---------------------------- | --------------------------- |
| `send_vcu2ai_status()`       | Sends message `0x520`       |
| `send_vcu2ai_drive_f()`      | Sends message `0x521`       |
| `send_vcu2ai_drive_r()`      | Sends message `0x522`       |
| `send_vcu2ai_steer()`        | Sends message `0x523`       |
| `send_vcu2ai_brake()`        | Sends message `0x524`       |
| `send_vcu2ai_speeds()`       | Sends message `0x525`       |
| `send_vcu2ai_wheel_counts()` | Sends message `0x526`       |
| `send_all_vcu_messages()`    | Sends all 7 feedback frames |

### Reception Functions

| Function          | Purpose                    |
| ----------------- | -------------------------- |
| `handle_ai_510()` | Parses AI status frame     |
| `handle_ai_511()` | Parses front motor command |
| `handle_ai_512()` | Parses rear motor command  |
| `handle_ai_513()` | Parses steering command    |
| `handle_ai_514()` | Parses brake command       |

---

##  Arduino Code
### Complete VCU Simulator Code
```cpp

/*
 * ============================================================================
 * FS-AI VCU SIMULATOR - Arduino Mega + MCP2515
 * INSTANT AI COMMAND RESPONSE (NO DELAY)
 * ============================================================================
 */

#include <mcp_can.h>
#include <SPI.h>

// ============================================================================
// HARDWARE CONFIGURATION
// ============================================================================
#define CAN0_INT 2
#define CAN0_CS  10

MCP_CAN CAN0(CAN0_CS);

// ============================================================================
// CAN MESSAGE BUFFERS
// ============================================================================
long unsigned int rxId = 0;
unsigned char rxLen = 0;
unsigned char rxBuf[8] = {0};

// ============================================================================
// MESSAGE IDS - VCU TRANSMITS (0x520-0x526)
// ============================================================================
static const uint16_t ID_VCU2AI_STATUS       = 0x520;
static const uint16_t ID_VCU2AI_DRIVE_F      = 0x521;
static const uint16_t ID_VCU2AI_DRIVE_R      = 0x522;
static const uint16_t ID_VCU2AI_STEER        = 0x523;
static const uint16_t ID_VCU2AI_BRAKE        = 0x524;
static const uint16_t ID_VCU2AI_SPEEDS       = 0x525;
static const uint16_t ID_VCU2AI_WHEEL_COUNTS = 0x526;

// ============================================================================
// MESSAGE IDS - AI SENDS (0x510-0x514)
// ============================================================================
static const uint16_t ID_AI2VCU_STATUS  = 0x510;
static const uint16_t ID_AI2VCU_DRIVE_F = 0x511;
static const uint16_t ID_AI2VCU_DRIVE_R = 0x512;
static const uint16_t ID_AI2VCU_STEER   = 0x513;
static const uint16_t ID_AI2VCU_BRAKE   = 0x514;

// ============================================================================
// TIMING
// ============================================================================
static const uint32_t VCU_SEND_INTERVAL_MS    = 10;
static const uint32_t PRINT_INTERVAL_MS       = 1000;
static const uint32_t HANDSHAKE_TOGGLE_PERIOD = 500;
static const uint32_t COMMS_TIMEOUT_MS        = 2000;

// ============================================================================
// TIMING VARIABLES
// ============================================================================
static uint32_t last_vcu_send_ms = 0;
static uint32_t last_print_ms = 0;
static uint32_t last_handshake_toggle_ms = 0;
static bool comms_lost = false;

// ============================================================================
// VCU STATE
// ============================================================================
struct VCU_State {
  uint8_t handshake_bit = 0;
  uint8_t as_state = 1;
  uint8_t ami_state = 0;
  uint8_t go_signal = 0;
  int16_t front_torque = 0;
  uint16_t front_torque_max = 1950;
  int16_t rear_torque = 0;
  uint16_t rear_torque_max = 1950;
  int16_t steer_angle = 0;
  uint16_t steer_angle_max = 210;
  uint8_t brake_press_f = 0;
  uint8_t brake_press_r = 0;
  uint16_t fl_wheel_speed = 0;
  uint16_t fr_wheel_speed = 0;
  uint16_t rl_wheel_speed = 0;
  uint16_t rr_wheel_speed = 0;
  uint16_t fl_pulse_count = 0;
  uint16_t fr_pulse_count = 0;
  uint16_t rl_pulse_count = 0;
  uint16_t rr_pulse_count = 0;
} vcu_state;

// ============================================================================
// AI DATA
// ============================================================================
struct AI_Data {
  struct {
    bool handshake;
    uint8_t mission_status;
    uint8_t direction;
    uint8_t estop;
    uint8_t lap_counter;
    uint8_t cones_count;
    uint16_t cones_count_all;
  } status;

  struct {
    int16_t torque_request;
    uint16_t speed_request;
  } drive_f, drive_r;

  struct {
    int16_t steer_request;
  } steer;

  struct {
    uint8_t front_pct;
    uint8_t rear_pct;
  } brake;
} ai_data = {};

// ============================================================================
// FRAME HEALTH
// ============================================================================
struct FrameHealth {
  bool seen = false;
  uint32_t last_ms = 0;
  uint32_t count = 0;
  uint32_t dlc_errors = 0;
};

static FrameHealth h_ai_status, h_ai_driveF, h_ai_driveR, h_ai_steer, h_ai_brake;

// ============================================================================
// MACROS
// ============================================================================
#define LE_U16(d, v) do { (d)[0] = (v) & 0xFF; (d)[1] = ((v) >> 8) & 0xFF; } while(0)
#define LE_I16(d, v) LE_U16(d, (uint16_t)(v))
#define FROM_LE_U16(d) ((uint16_t)(d)[0] | ((uint16_t)(d)[1] << 8))
#define FROM_LE_I16(d) ((int16_t)FROM_LE_U16(d))

// ============================================================================
// ENUMS
// ============================================================================
#define MISSION_NOT_SELECTED 0
#define MISSION_SELECTED 1
#define MISSION_RUNNING 2
#define MISSION_FINISHED 3
#define ESTOP_NO 0
#define ESTOP_YES 1

// ============================================================================
// UPDATE FUNCTIONS
// ============================================================================

void update_vcu_state_from_ai() {
  vcu_state.front_torque = ai_data.drive_f.torque_request;
  vcu_state.rear_torque = ai_data.drive_r.torque_request;
  vcu_state.steer_angle = ai_data.steer.steer_request;
  vcu_state.brake_press_f = ai_data.brake.front_pct;
  vcu_state.brake_press_r = ai_data.brake.rear_pct;
}

void simulate_vehicle_dynamics() {
  uint16_t ai_target_speed = ai_data.drive_f.speed_request;
  uint8_t ai_brake_f = ai_data.brake.front_pct;
  uint8_t ai_brake_r = ai_data.brake.rear_pct;
  int16_t ai_torque = ai_data.drive_f.torque_request;

  if (ai_torque > 0 && ai_brake_f == 0 && ai_brake_r == 0) {
    if (vcu_state.fl_wheel_speed < ai_target_speed) {
      vcu_state.fl_wheel_speed = (vcu_state.fl_wheel_speed + 50 <= ai_target_speed)
                                   ? vcu_state.fl_wheel_speed + 50
                                   : ai_target_speed;
    } else if (vcu_state.fl_wheel_speed > ai_target_speed) {
      vcu_state.fl_wheel_speed = (vcu_state.fl_wheel_speed - 50 >= ai_target_speed)
                                   ? vcu_state.fl_wheel_speed - 50
                                   : ai_target_speed;
    }
  }
  else if (ai_brake_f > 0 || ai_brake_r > 0) {
    if (vcu_state.fl_wheel_speed > 0) {
      uint16_t brake_decel = ((ai_brake_f + ai_brake_r) / 2) * 2;
      vcu_state.fl_wheel_speed = (vcu_state.fl_wheel_speed > brake_decel)
                                   ? vcu_state.fl_wheel_speed - brake_decel
                                   : 0;
    }
  }
  else {
    if (vcu_state.fl_wheel_speed > 0) {
      vcu_state.fl_wheel_speed = (vcu_state.fl_wheel_speed > 10)
                                   ? vcu_state.fl_wheel_speed - 10
                                   : 0;
    }
  }

  vcu_state.fr_wheel_speed = vcu_state.fl_wheel_speed;
  vcu_state.rl_wheel_speed = vcu_state.fl_wheel_speed;
  vcu_state.rr_wheel_speed = vcu_state.fl_wheel_speed;

  vcu_state.fl_pulse_count++;
  vcu_state.fr_pulse_count++;
  vcu_state.rl_pulse_count++;
  vcu_state.rr_pulse_count++;
}

void update_as_state_from_ai() {
  if (ai_data.status.estop == ESTOP_YES) {
    vcu_state.as_state = 4;
    vcu_state.go_signal = 0;
    return;
  }

  switch (ai_data.status.mission_status) {
    case MISSION_NOT_SELECTED:
      vcu_state.as_state = 1;
      vcu_state.go_signal = 0;
      break;
    case MISSION_SELECTED:
      vcu_state.as_state = 2;
      vcu_state.go_signal = 1;
      break;
    case MISSION_RUNNING:
      vcu_state.as_state = 3;
      vcu_state.go_signal = 1;
      break;
    case MISSION_FINISHED:
      vcu_state.as_state = 5;
      vcu_state.go_signal = 0;
      break;
    default:
      vcu_state.as_state = 1;
      vcu_state.go_signal = 0;
      break;
  }
}

// ============================================================================
// TRANSMISSION (VCU → AI)
// ============================================================================

void send_vcu2ai_status() {
  uint8_t data[8] = {0};
  data[0] = ((vcu_state.handshake_bit & 0x01) << 0) |
            ((vcu_state.go_signal & 0x01) << 4);
  data[1] = ((vcu_state.as_state & 0x07) << 0) |
            ((vcu_state.ami_state & 0x07) << 4);
  CAN0.sendMsgBuf(ID_VCU2AI_STATUS, 0, 8, data);
}

void send_vcu2ai_drive_f() {
  uint8_t data[6] = {0};
  LE_I16(&data[0], vcu_state.front_torque);
  LE_U16(&data[2], 0);
  LE_U16(&data[4], vcu_state.front_torque_max);
  CAN0.sendMsgBuf(ID_VCU2AI_DRIVE_F, 0, 6, data);
}

void send_vcu2ai_drive_r() {
  uint8_t data[6] = {0};
  LE_I16(&data[0], vcu_state.rear_torque);
  LE_U16(&data[2], 0);
  LE_U16(&data[4], vcu_state.rear_torque_max);
  CAN0.sendMsgBuf(ID_VCU2AI_DRIVE_R, 0, 6, data);
}

void send_vcu2ai_steer() {
  uint8_t data[6] = {0};
  LE_I16(&data[0], vcu_state.steer_angle);
  LE_U16(&data[2], vcu_state.steer_angle_max);
  LE_I16(&data[4], 0);
  CAN0.sendMsgBuf(ID_VCU2AI_STEER, 0, 6, data);
}

void send_vcu2ai_brake() {
  uint8_t data[5] = {0};
  data[0] = vcu_state.brake_press_f;
  data[2] = vcu_state.brake_press_r;
  CAN0.sendMsgBuf(ID_VCU2AI_BRAKE, 0, 5, data);
}

void send_vcu2ai_speeds() {
  uint8_t data[8] = {0};
  LE_U16(&data[0], vcu_state.fl_wheel_speed);
  LE_U16(&data[2], vcu_state.fr_wheel_speed);
  LE_U16(&data[4], vcu_state.rl_wheel_speed);
  LE_U16(&data[6], vcu_state.rr_wheel_speed);
  CAN0.sendMsgBuf(ID_VCU2AI_SPEEDS, 0, 8, data);
}

void send_vcu2ai_wheel_counts() {
  uint8_t data[8] = {0};
  LE_U16(&data[0], vcu_state.fl_pulse_count);
  LE_U16(&data[2], vcu_state.fr_pulse_count);
  LE_U16(&data[4], vcu_state.rl_pulse_count);
  LE_U16(&data[6], vcu_state.rr_pulse_count);
  CAN0.sendMsgBuf(ID_VCU2AI_WHEEL_COUNTS, 0, 8, data);
}

void send_all_vcu_messages() {
  send_vcu2ai_status();
  send_vcu2ai_drive_f();
  send_vcu2ai_drive_r();
  send_vcu2ai_steer();
  send_vcu2ai_brake();
  send_vcu2ai_speeds();
  send_vcu2ai_wheel_counts();
}

// ============================================================================
// RECEPTION (AI → VCU)
// ============================================================================

void handle_ai_510(uint8_t* d, uint8_t dlc) {
  if (dlc != 8) { h_ai_status.dlc_errors++; return; }
  h_ai_status.seen = true;
  h_ai_status.last_ms = millis();
  h_ai_status.count++;

  uint8_t b0 = d[0];
  ai_data.status.handshake = (b0 >> 0) & 0x01;
  ai_data.status.estop = (b0 >> 1) & 0x01;
  ai_data.status.mission_status = (b0 >> 2) & 0x03;
  ai_data.status.direction = (b0 >> 4) & 0x01;
  ai_data.status.lap_counter = d[1];
  ai_data.status.cones_count = d[2];
  ai_data.status.cones_count_all = FROM_LE_U16(&d[3]);
}

void handle_ai_511(uint8_t* d, uint8_t dlc) {
  if (dlc != 4) { h_ai_driveF.dlc_errors++; return; }
  h_ai_driveF.seen = true;
  h_ai_driveF.last_ms = millis();
  h_ai_driveF.count++;
  ai_data.drive_f.torque_request = FROM_LE_I16(&d[0]);
  ai_data.drive_f.speed_request = FROM_LE_U16(&d[2]);
}

void handle_ai_512(uint8_t* d, uint8_t dlc) {
  if (dlc != 4) { h_ai_driveR.dlc_errors++; return; }
  h_ai_driveR.seen = true;
  h_ai_driveR.last_ms = millis();
  h_ai_driveR.count++;
  ai_data.drive_r.torque_request = FROM_LE_I16(&d[0]);
  ai_data.drive_r.speed_request = FROM_LE_U16(&d[2]);
}

void handle_ai_513(uint8_t* d, uint8_t dlc) {
  if (dlc != 2) { h_ai_steer.dlc_errors++; return; }
  h_ai_steer.seen = true;
  h_ai_steer.last_ms = millis();
  h_ai_steer.count++;
  ai_data.steer.steer_request = FROM_LE_I16(&d[0]);
}

void handle_ai_514(uint8_t* d, uint8_t dlc) {
  if (dlc != 2) { h_ai_brake.dlc_errors++; return; }
  h_ai_brake.seen = true;
  h_ai_brake.last_ms = millis();
  h_ai_brake.count++;
  ai_data.brake.front_pct = d[0];
  ai_data.brake.rear_pct = d[1];
}

// ============================================================================
// DISPLAY
// ============================================================================

const char* as_state_str(uint8_t state) {
  switch (state) {
    case 1: return "OFF";
    case 2: return "READY";
    case 3: return "DRIVING";
    case 4: return "E-BRAKE";
    case 5: return "FINISHED";
    default: return "INVALID";
  }
}

const char* mission_str(uint8_t mission) {
  switch (mission) {
    case MISSION_NOT_SELECTED: return "NOT_SELECTED";
    case MISSION_SELECTED:     return "SELECTED";
    case MISSION_RUNNING:      return "RUNNING";
    case MISSION_FINISHED:     return "FINISHED";
    default:                   return "UNKNOWN";
  }
}

const char* direction_str(uint8_t dir) {
  return (dir == 0) ? "NEUTRAL" : "FORWARD";
}

const char* estop_str(uint8_t estop) {
  return (estop == 0) ? "NO" : "YES";
}

// ============================================================================
// TIMEOUT DETECTION
// ============================================================================

uint32_t min5(uint32_t a, uint32_t b, uint32_t c, uint32_t d, uint32_t e) {
  uint32_t m = a;
  if (b < m) m = b;
  if (c < m) m = c;
  if (d < m) m = d;
  if (e < m) m = e;
  return m;
}

void check_stm32_alive() {
  uint32_t now = millis();
  uint32_t age_status = h_ai_status.count > 0 ? (now - h_ai_status.last_ms) : 99999;
  uint32_t age_driveF = h_ai_driveF.count > 0 ? (now - h_ai_driveF.last_ms) : 99999;
  uint32_t age_driveR = h_ai_driveR.count > 0 ? (now - h_ai_driveR.last_ms) : 99999;
  uint32_t age_steer  = h_ai_steer.count > 0 ? (now - h_ai_steer.last_ms) : 99999;
  uint32_t age_brake  = h_ai_brake.count > 0 ? (now - h_ai_brake.last_ms) : 99999;

  uint32_t most_recent_age = min5(age_status, age_driveF, age_driveR, age_steer, age_brake);
  comms_lost = (most_recent_age > COMMS_TIMEOUT_MS);
}

// ============================================================================
// STATUS PRINT
// ============================================================================

void print_bidirectional_status() {
  check_stm32_alive();

  Serial.println("\n╔════════════════════════════════════════════════════════╗");
  Serial.println("║          BIDIRECTIONAL COMMUNICATION STATUS            ║");
  Serial.println("║              (INSTANT AI COMMAND RESPONSE)             ║");
  Serial.println("╚════════════════════════════════════════════════════════╝");

  Serial.print("\n[STM32 STATUS]: ");
  if (comms_lost) {
    Serial.println("✗✗✗ OFFLINE - NO COMMUNICATION ✗✗✗");
  } else {
    Serial.println("✓✓✓ ONLINE - Communication Active ✓✓✓");
  }

  Serial.println("\n[VCU TRANSMITTING] → STM32 AI");
  Serial.print("  0x520: HS=");
  Serial.print(vcu_state.handshake_bit);
  Serial.print(" AS_STATE=");
  Serial.println(as_state_str(vcu_state.as_state));

  Serial.print("  0x521: Front Torque=");
  Serial.print(vcu_state.front_torque * 0.1, 1);
  Serial.print(" Nm | 0x522: Rear Torque=");
  Serial.print(vcu_state.rear_torque * 0.1, 1);
  Serial.println(" Nm");

  Serial.print("  0x523: Steering=");
  Serial.print(vcu_state.steer_angle * 0.1, 1);
  Serial.print("° | 0x524: Brake F=");
  Serial.print(vcu_state.brake_press_f * 0.5, 1);
  Serial.print("% R=");
  Serial.print(vcu_state.brake_press_r * 0.5, 1);
  Serial.println("%");

  Serial.print("  0x525: Speeds - FL=");
  Serial.print(vcu_state.fl_wheel_speed);
  Serial.print(" FR=");
  Serial.print(vcu_state.fr_wheel_speed);
  Serial.print(" RL=");
  Serial.print(vcu_state.rl_wheel_speed);
  Serial.print(" RR=");
  Serial.print(vcu_state.rr_wheel_speed);
  Serial.println(" RPM");

  Serial.println("\n[AI RECEIVING] ← STM32 AI");
  if (comms_lost) {
    Serial.println("  ✗ NO DATA - STM32 IS OFFLINE");
  }

  Serial.print("  0x510: MISSION=");
  Serial.print(mission_str(ai_data.status.mission_status));
  Serial.print(" DIR=");
  Serial.print(direction_str(ai_data.status.direction));
  Serial.print(" ESTOP=");
  Serial.println(estop_str(ai_data.status.estop));

  Serial.print("  0x511: Front Torque=");
  Serial.print(ai_data.drive_f.torque_request * 0.1, 1);
  Serial.print(" Nm | 0x512: Rear Torque=");
  Serial.print(ai_data.drive_r.torque_request * 0.1, 1);
  Serial.println(" Nm");

  Serial.println("\n╚════════════════════════════════════════════════════════╝\n");
}

// ============================================================================
// SETUP
// ============================================================================

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("\n╔════════════════════════════════════════════════════════╗");
  Serial.println("║   FS-AI VCU SIMULATOR - WITH STM32 OFFLINE DETECTION   ║");
  Serial.println("║        Arduino Mega + MCP2515 (500k CAN)               ║");
  Serial.println("╚════════════════════════════════════════════════════════╝\n");

  if (CAN0.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) == CAN_OK) {
    Serial.println("✓ MCP2515 Initialized");
  } else {
    Serial.println("✗ MCP2515 FAILED");
    while (1) {}
  }

  CAN0.setMode(MCP_NORMAL);
  pinMode(CAN0_INT, INPUT);

  Serial.println("✓ CAN Bus Ready");
  Serial.println("✓ Listening for AI commands...\n");

  last_vcu_send_ms = millis();
  last_print_ms = millis();
  last_handshake_toggle_ms = millis();
}

// ============================================================================
// LOOP
// ============================================================================

void loop() {
  uint32_t now = millis();

  // RECEIVE AI MESSAGES
  while (!digitalRead(CAN0_INT)) {
    CAN0.readMsgBuf(&rxId, &rxLen, rxBuf);
    if ((rxId & 0x80000000UL) == 0) {
      uint16_t sid = (uint16_t)(rxId & 0x7FF);
      switch (sid) {
        case ID_AI2VCU_STATUS:  handle_ai_510(rxBuf, rxLen); break;
        case ID_AI2VCU_DRIVE_F: handle_ai_511(rxBuf, rxLen); break;
        case ID_AI2VCU_DRIVE_R: handle_ai_512(rxBuf, rxLen); break;
        case ID_AI2VCU_STEER:   handle_ai_513(rxBuf, rxLen); break;
        case ID_AI2VCU_BRAKE:   handle_ai_514(rxBuf, rxLen); break;
      }
    }
  }

  // APPLY COMMANDS INSTANTLY
  update_vcu_state_from_ai();
  update_as_state_from_ai();
  simulate_vehicle_dynamics();

  // TOGGLE HANDSHAKE
  if ((now - last_handshake_toggle_ms) >= HANDSHAKE_TOGGLE_PERIOD) {
    last_handshake_toggle_ms = now;
    vcu_state.handshake_bit ^= 1;
  }

  // SEND FEEDBACK EVERY 10 ms
  if ((now - last_vcu_send_ms) >= VCU_SEND_INTERVAL_MS) {
    last_vcu_send_ms = now;
    send_all_vcu_messages();
  }

  // PRINT STATUS EVERY 1 SECOND
  if ((now - last_print_ms) >= PRINT_INTERVAL_MS) {
    last_print_ms = now;
    print_bidirectional_status();
  }
}
```

---

## Performance Metrics

| Metric              | Value             |
| ------------------- | ----------------- |
| CAN Bus Speed       | 500 kbps          |
| Message Cycle       | 10 ms (100 Hz)    |
| Messages per Second | 120 total         |
| Data per Cycle      | 51 bytes          |
| Bandwidth Used      | ~4% of 500 kbps   |
| Latency             | 2-3 ms end-to-end |
| Handshake           | Every 500 ms      |
| Timeout Detection   | 2000 ms           |

---

## Quick Reference
### CAN IDs at a Glance

VCU TRANSMITS (→ AI):
  0x520 = Status
  0x521 = Front Motor Feedback
  0x522 = Rear Motor Feedback
  0x523 = Steering Feedback
  0x524 = Brake Feedback
  0x525 = Wheel Speeds
  0x526 = Encoder Counts

AI TRANSMITS (→ VCU):
  0x510 = Status
  0x511 = Front Motor Command
  0x512 = Rear Motor Command
  0x513 = Steering Command
  0x514 = Brake Command

  ---

  

