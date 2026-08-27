/**
 * Motion example – absolute and relative moves + home
 *
 * Hardware: Arduino + MCP2515 (CS = pin 10)
 */

#include <SPI.h>
#include "DJIRonin.h"
#include "MCP2515Driver.h"

using namespace dji::ronin;

MCP2515Driver can(10);          // CS pin
DJIRonin      ronin(can);

void setup() {
    Serial.begin(115200);
    while (!Serial) {}

    if (!ronin.begin()) {
        Serial.println("CAN init failed");
        while (1) {}
    }
    Serial.println("Ronin ready");

    // Move to yaw = 45.0°, pitch = 0, roll = 0 in 2 seconds
    // (angles are in 0.1° units → 450 = 45.0°)
    ronin.motion.moveTo(450, 0, 0, 20);

    delay(3000);

    // Relative move: +10° yaw
    ronin.motion.moveBy(100, 0, 0, 10);

    delay(2000);

    // Return to center
    ronin.motion.home();
}

void loop() {
    // nothing
}
