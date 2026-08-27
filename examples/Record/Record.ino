/**
 * Camera record / photo example
 */

#include <SPI.h>
#include "DJIRonin.h"
#include "MCP2515Driver.h"

using namespace dji::ronin;

MCP2515Driver can(10);
DJIRonin      ronin(can);

void setup() {
    Serial.begin(115200);
    if (!ronin.begin()) {
        Serial.println("CAN init failed");
        while (1) {}
    }

    // Take a photo
    ronin.camera.photo();
    delay(1000);

    // Start recording
    ronin.camera.recordStart();
    delay(5000);

    // Stop recording
    ronin.camera.recordStop();
}

void loop() {}
