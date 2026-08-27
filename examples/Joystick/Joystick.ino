/**
 * Joystick example – continuous speed control
 *
 * IMPORTANT: Joystick / Speed commands are PERIODIC.
 * The Ronin stops after ~0.5 s without new packets.
 * Send every 20 ms (or similar).
 *
 * Hardware: Arduino + MCP2515 (CS = pin 10)
 * Optional: potentiometers on A0/A1/A2 for yaw/pitch/roll
 */

#include <SPI.h>
#include "DJIRonin.h"
#include "MCP2515Driver.h"

using namespace dji::ronin;

MCP2515Driver can(10);
DJIRonin      ronin(can);

unsigned long lastSend = 0;
const unsigned long INTERVAL_MS = 20;   // 50 Hz

void setup() {
    Serial.begin(115200);
    if (!ronin.begin()) {
        Serial.println("CAN init failed");
        while (1) {}
    }
    // Optional intensity scaling (50 %)
    ronin.motion.setSpeedScale(50);
    Serial.println("Joystick mode ready – send every 20 ms");
}

void loop() {
    if (millis() - lastSend >= INTERVAL_MS) {
        lastSend = millis();

        // Example: map analog sticks (user is responsible for calibration)
        // Range expected by protocol: -15000 .. +15000
        int16_t yaw   = map(analogRead(A0), 0, 1023, -15000, 15000);
        int16_t pitch = map(analogRead(A1), 0, 1023, -15000, 15000);
        int16_t roll  = map(analogRead(A2), 0, 1023, -15000, 15000);

        // Dead-zone example (optional)
        if (abs(yaw)   < 500) yaw   = 0;
        if (abs(pitch) < 500) pitch = 0;
        if (abs(roll)  < 500) roll  = 0;

        ronin.motion.joystick(yaw, pitch, roll);
    }
}
