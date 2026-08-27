/**
 * @file ICANDriver.h
 * @brief Abstract CAN interface – the only place the protocol layer talks to transport.
 *
 * Protocol layer must never include Arduino.h, SPI.h or any hardware library.
 */

#ifndef DJI_RONIN_ICAN_DRIVER_H
#define DJI_RONIN_ICAN_DRIVER_H

#include <stdint.h>
#include "../../protocol/Result.h"
#include "../../protocol/RoninEnums.h"

namespace dji {
namespace ronin {

/**
 * Simple CAN frame used by the transport layer.
 * Protocol layer never sees this structure directly in high-level API,
 * but PacketBuilder/Parser work with byte buffers; the driver converts.
 */
struct CANFrame {
    uint32_t id;        // 11-bit or 29-bit identifier
    uint8_t  data[8];
    uint8_t  length;    // 0..8
    bool     extended;  // true = 29-bit, false = 11-bit (DJI uses standard 11-bit)

    CANFrame() : id(0), length(0), extended(false) {
        for (int i = 0; i < 8; ++i) data[i] = 0;
    }
};

/**
 * Abstract CAN driver interface.
 * Concrete implementations: MCP2515Driver, MockCANDriver, future SocketCAN, TWAI, etc.
 */
class ICANDriver {
public:
    virtual ~ICANDriver() {}

    /**
     * Initialize the hardware / driver.
     * @return Ok or TransportError
     */
    virtual Result<void> begin() = 0;

    /**
     * Send a single CAN frame.
     */
    virtual Result<void> send(const CANFrame& frame) = 0;

    /**
     * Try to receive one CAN frame (non-blocking).
     * @param frame  Output frame
     * @return Ok if a frame was received, NoData if nothing available, or TransportError
     */
    virtual Result<void> receive(CANFrame& frame) = 0;

    /**
     * Optional: set acceptance filter (implementation may ignore).
     */
    virtual Result<void> setFilter(uint32_t id, uint32_t mask) {
        (void)id;
        (void)mask;
        return Result<void>::success();
    }
};

} // namespace ronin
} // namespace dji

#endif // DJI_RONIN_ICAN_DRIVER_H
