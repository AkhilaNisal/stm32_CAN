# STM32 + MCP2515 CAN Bus Communication (8MHz Crystal)

This repository contains firmware for using an **STM32 (Blue Pill / F1) microcontroller** with an **MCP2515 CAN controller** over SPI, and an **Arduino Mega** to receive CAN messages.

📺 This project was developed following the tutorial:  
https://www.micropeta.com/video138

It is also based on the MCP2515 driver structure from this reference repository:  
https://github.com/eziya/STM32_SPI_MCP2515

---

## 🧠 Description

- **STM32** initializes the MCP2515 CAN controller (with 8 MHz crystal) and transmits CAN messages periodically.
- **Arduino Mega** uses an MCP2515 module to receive CAN messages and prints them over USB Serial.
- Communication occurs via **standard 11-bit CAN IDs** at **500 kbps**.

---

## 🔧 Hardware Required

| Module | Quantity |
|--------|----------|
| STM32 Blue Pill | 1 |
| MCP2515 CAN module (8 MHz) | 1 |
| Arduino Mega | 1 |
| CAN Bus wiring (terminated with 120 Ω resistors) | Yes |
| Common GND between modules | Required |

---

## 📍 Wiring

### STM32 → MCP2515
3.3V → VIO (logic)
5V → VCC
GND → GND
PA5 → SCK
PA7 → MOSI
PA6 ← MISO
PB12 → CS
(any GPIO) → INT (optional)
CANH ↔ CANH
CANL ↔ CANL

### Arduino Mega → MCP2515
5V → VCC
GND → GND
50 ← MISO
51 → MOSI
52 → SCK
10 → CS
2 ← INT
CANH ↔ CANH
CANL ↔ CANL


⚠ Make sure both boards share **common ground** and the CAN bus has proper termination.

---

## 🧾 STM32 Firmware

STM32 code initializes the MCP2515 (8 MHz crystal) and sets bit timing accordingly:

```c
// Configure bit timing for 8MHz MCP2515, 500 kbps
MCP2515_WriteByte(MCP2515_CNF1, 0x00);
MCP2515_WriteByte(MCP2515_CNF2, 0x90);
MCP2515_WriteByte(MCP2515_CNF3, 0x02);

###📡 Arduino Mega Code

The Arduino Mega snippet below receives CAN messages from MCP2515 and prints them over Serial:


```cpp
#include <SPI.h>
#include <mcp_can.h>

#define CAN0_INT 2
MCP_CAN CAN0(10); // CS pin 10

void setup()
{
  Serial.begin(115200);

  // Initialize MCP2515 with 8 MHz crystal and 500 kbps
  if (CAN0.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) == CAN_OK)
    Serial.println("MCP2515 Initialized Successfully!");
  else
    Serial.println("Error Initializing MCP2515...");

  CAN0.setMode(MCP_NORMAL); // Normal mode to send ACKs
  pinMode(CAN0_INT, INPUT);
}

void loop()
{
  long unsigned int rxId;
  unsigned char len = 0;
  unsigned char rxBuf[8];

  if (!digitalRead(CAN0_INT)) {
    CAN0.readMsgBuf(&rxId, &len, rxBuf);

    Serial.print("Standard ID: 0x");
    Serial.print(rxId, HEX);
    Serial.print("  DLC: ");
    Serial.print(len);
    Serial.print("  Data:");

    for (byte i = 0; i < len; i++) {
      Serial.print(" 0x");
      Serial.print(rxBuf[i], HEX);
    }
    Serial.println();
  }
}
