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
|Pin	|Signal	|Device	|Purpose|
|-----|-------|---------|-----------------|
|51	|MOSI	|MCP2515 |SPI Data Out|
|50	|MISO	|MCP2515 |SPI Data In|
|52	|SCK	|MCP2515 |SPI Clock|
|10	|CS	|MCP2515 |Chip Select|
|2	|INT	|MCP2515 |Interrupt (RX Ready)|

Note: PC13 and PA4 are STM32-style pin names, not Arduino Mega pin names. Keep them only if they belong to the STM32 side of your system
