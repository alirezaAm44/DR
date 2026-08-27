/**
 * Focus motor example
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

    // Move focus to middle (0..4095)
    ronin.focus.moveTo(2048);
    delay(2000);

    // Move to near end
    ronin.focus.moveTo(0);
    delay(2000);

    // Move to far end
    ronin.focus.moveTo(4095);
}

void loop() {}
