# DJIRonin – Professional C++ Library for DJI Ronin R SDK

A clean, modular, hardware-independent C++ library that implements the official **DJI R SDK Protocol** (v2.5) for controlling Ronin / RS series gimbals over CAN.

## Features

- Absolute & relative position control
- Official DJI Joystick / External Device Control (periodic)
- Camera record / photo / center-focus
- Focus motor control
- ActiveTrack toggle
- Automatic CRC16 + CRC32 (exact parameters from DJI documentation)
- Automatic sequence number handling
- `Result<T, Error>` error handling (no exceptions)
- Protocol layer completely independent of Arduino / MCP2515
- Desktop-testable with MockCANDriver
- Ready for ESP32 TWAI, STM32 CAN, Linux SocketCAN (only transport changes)

## Architecture

```
User Application
       │
       ▼
 High Level API  (Motion, Camera, Focus, ActiveTrack)
       │
       ▼
 Protocol Layer  (PacketBuilder / PacketParser)
       │
       ▼
 CRC Layer       (custom_crc16 / custom_crc32)
       │
       ▼
 ICANDriver      (abstract)
       │
   ┌───┴───┐
   ▼       ▼
MCP2515  MockCAN
```

## Quick Start (Arduino + MCP2515)

```cpp
#include <SPI.h>
#include "DJIRonin.h"
#include "MCP2515Driver.h"

using namespace dji::ronin;

MCP2515Driver can(10);   // CS pin
DJIRonin ronin(can);

void setup() {
    Serial.begin(115200);
    if (!ronin.begin()) {
        Serial.println("CAN init failed");
        while (1) {}
    }

    // Move to 45° yaw in 2 seconds (angles in 0.1° units)
    ronin.motion.moveTo(450, 0, 0, 20);

    // Start recording
    ronin.camera.recordStart();
}

void loop() {
    // Joystick must be sent periodically (~20 ms)
    static unsigned long last = 0;
    if (millis() - last >= 20) {
        last = millis();
        ronin.motion.joystick(yaw, pitch, roll);
    }
}
```

## Important Notes

### Periodic Commands
Speed / Joystick commands (**CmdID 0x0A** and **0x01**) are **periodic**.  
The Ronin stops motion approximately **0.5 s** after the last packet.  
The library does **not** create timers – the user is responsible for the send rate.

### Units
- Position angles: **0.1°** (e.g. 900 = 90.0°)
- Time for action: **0.1 s** (e.g. 20 = 2.0 s)
- Joystick values: **-15000 … +15000**

### CAN Parameters (DJI RS 2)
- Baud rate: **1 Mbps**
- TX ID (third-party → gimbal): **0x223**
- RX ID (gimbal → third-party): **0x222**
- Frame type: Standard (11-bit)

## Building & Testing on Desktop

```bash
# CRC + PacketBuilder + PacketParser tests (no Arduino needed)
g++ -std=c++11 -I src/protocol -I src/crc \
    tests/PacketBuilderTests.cpp \
    src/protocol/PacketBuilder.cpp \
    src/crc/custom_crc16.c src/crc/custom_crc32.c \
    -o packet_test && ./packet_test

g++ -std=c++11 -I src/protocol -I src/crc \
    tests/PacketParserTests.cpp \
    src/protocol/PacketParser.cpp src/protocol/PacketBuilder.cpp \
    src/crc/custom_crc16.c src/crc/custom_crc32.c \
    -o parser_test && ./parser_test
```

All tests pass against the official sample packet from the DJI documentation.

## Folder Structure

See the repository root. Protocol, CRC and high-level API have zero Arduino dependency.

## License

MIT (see LICENSE)
