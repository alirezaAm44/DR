/**
 * @file MCP2515Driver.cpp
 * @brief Arduino + MCP2515 implementation
 *
 * TX notes (MCP2515 has only 3 hardware TX buffers):
 * - A 25–26 byte DJI packet needs 4 CAN frames.
 * - Frames are queued faster over SPI than the bus can drain them at 1 Mbps.
 * - Without spacing / wait-for-free-buffer, the 4th frame returns ERROR_ALLTXBUSY
 *   and the packet is truncated (CRC32 never leaves the controller).
 */

#include "MCP2515Driver.h"

namespace dji {
namespace ronin {

#if defined(ARDUINO)

/** Inter-frame gap so TX buffers can drain on a 1 Mbps bus (~120 µs/frame). */
static const uint16_t kInterFrameDelayUs = 350;

/** How long to wait for a free TX buffer before giving up. */
static const uint16_t kTxBusyTimeoutUs = 5000;

MCP2515Driver::MCP2515Driver(uint8_t csPin, uint32_t baud)
    : mcp_(csPin), baud_(baud), initialized_(false)
{
}

Result<void> MCP2515Driver::begin()
{
    mcp_.reset();
    // DJI RS 2 uses 1 Mbps, standard frames
    MCP2515::ERROR err = mcp_.setBitrate(CAN_1000KBPS, MCP_8MHZ); // adjust crystal if needed
    if (err != MCP2515::ERROR_OK) {
        return Result<void>::failure(Error::TransportError);
    }
    err = mcp_.setNormalMode();
    if (err != MCP2515::ERROR_OK) {
        return Result<void>::failure(Error::TransportError);
    }
    initialized_ = true;
    return Result<void>::success();
}

Result<void> MCP2515Driver::send(const CANFrame& frame)
{
    if (!initialized_) {
        return Result<void>::failure(Error::NotInitialized);
    }

    can_frame f;
    f.can_id  = frame.id;
    f.can_dlc = frame.length;
    for (uint8_t i = 0; i < frame.length; ++i) {
        f.data[i] = frame.data[i];
    }

    // Retry while all 3 TX buffers are busy (packet still on the wire / waiting ACK).
    const unsigned long t0 = micros();
    MCP2515::ERROR err = MCP2515::ERROR_FAIL;
    for (;;) {
        err = mcp_.sendMessage(&f);
        if (err == MCP2515::ERROR_OK) {
            break;
        }
        if (err != MCP2515::ERROR_ALLTXBUSY) {
            return Result<void>::failure(Error::TransportError);
        }
        if ((micros() - t0) >= kTxBusyTimeoutUs) {
            return Result<void>::failure(Error::TransportError);
        }
        delayMicroseconds(50);
    }

    // Give the bus time to shift the frame out before the next SPI queue.
    // Prevents the classic "4th frame death" on 25–26 byte DJI packets.
    delayMicroseconds(kInterFrameDelayUs);

    return Result<void>::success();
}

Result<void> MCP2515Driver::receive(CANFrame& frame)
{
    if (!initialized_) {
        return Result<void>::failure(Error::NotInitialized);
    }

    can_frame f;
    MCP2515::ERROR err = mcp_.readMessage(&f);
    if (err == MCP2515::ERROR_NOMSG) {
        return Result<void>::failure(Error::NoData);
    }
    if (err != MCP2515::ERROR_OK) {
        return Result<void>::failure(Error::TransportError);
    }

    frame.id       = f.can_id & 0x7FF;  // standard 11-bit
    frame.length   = f.can_dlc;
    frame.extended = false;
    for (uint8_t i = 0; i < frame.length; ++i) {
        frame.data[i] = f.data[i];
    }
    return Result<void>::success();
}

Result<void> MCP2515Driver::setFilter(uint32_t id, uint32_t mask)
{
    (void)id;
    (void)mask;
    return Result<void>::success();
}

#else
MCP2515Driver::MCP2515Driver(uint8_t, uint32_t) {}
Result<void> MCP2515Driver::begin()   { return Result<void>::failure(Error::TransportError); }
Result<void> MCP2515Driver::send(const CANFrame&) { return Result<void>::failure(Error::TransportError); }
Result<void> MCP2515Driver::receive(CANFrame&)    { return Result<void>::failure(Error::TransportError); }
Result<void> MCP2515Driver::setFilter(uint32_t, uint32_t) { return Result<void>::failure(Error::TransportError); }
#endif

} // namespace ronin
} // namespace dji
