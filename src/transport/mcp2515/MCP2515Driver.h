/**
 * @file MCP2515Driver.h
 * @brief Arduino + MCP2515 implementation of ICANDriver
 *
 * This is the ONLY file allowed to include Arduino.h / SPI.h / MCP2515 library.
 * Protocol layer never includes this header.
 */

#ifndef DJI_RONIN_MCP2515_DRIVER_H
#define DJI_RONIN_MCP2515_DRIVER_H

#include "../interfaces/ICANDriver.h"
#include "../../protocol/RoninConstants.h"

// Hardware includes are isolated here.
// The user must install the MCP2515 library (e.g. autowp/arduino-mcp2515)
// and provide the correct CS / interrupt pins.

#if defined(ARDUINO)
#include <Arduino.h>
#include <SPI.h>
#include <mcp2515.h>   // https://github.com/autowp/arduino-mcp2515
#endif

namespace dji {
namespace ronin {

class MCP2515Driver : public ICANDriver {
public:
    /**
     * @param csPin   Chip-select pin for MCP2515
     * @param baud    Desired CAN baud rate (default 1 Mbps for DJI RS)
     */
    explicit MCP2515Driver(uint8_t csPin, uint32_t baud = CAN_BAUD_RATE);

    Result<void> begin() override;
    Result<void> send(const CANFrame& frame) override;
    Result<void> receive(CANFrame& frame) override;
    Result<void> setFilter(uint32_t id, uint32_t mask) override;

private:
#if defined(ARDUINO)
    MCP2515 mcp_;
    uint32_t baud_;
    bool initialized_;
#endif
};

} // namespace ronin
} // namespace dji

#endif // DJI_RONIN_MCP2515_DRIVER_H
