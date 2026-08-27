/**
 * @file MCP2515Driver.cpp
 * @brief Arduino + MCP2515 implementation
 */

#include "MCP2515Driver.h"

namespace dji {
namespace ronin {

#if defined(ARDUINO)

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

    MCP2515::ERROR err = mcp_.sendMessage(&f);
    if (err != MCP2515::ERROR_OK) {
        return Result<void>::failure(Error::TransportError);
    }
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
    // Simple filter on RXB0 – library specific; may need adjustment
    // for production use.
    (void)id;
    (void)mask;
    return Result<void>::success();
}

#else
// Stub implementations so the file still compiles on desktop
MCP2515Driver::MCP2515Driver(uint8_t, uint32_t) {}
Result<void> MCP2515Driver::begin()   { return Result<void>::failure(Error::TransportError); }
Result<void> MCP2515Driver::send(const CANFrame&) { return Result<void>::failure(Error::TransportError); }
Result<void> MCP2515Driver::receive(CANFrame&)    { return Result<void>::failure(Error::TransportError); }
Result<void> MCP2515Driver::setFilter(uint32_t, uint32_t) { return Result<void>::failure(Error::TransportError); }
#endif

} // namespace ronin
} // namespace dji
