/**
 * @file DJIRonin.h
 * @brief Public high-level API for controlling DJI Ronin via R SDK protocol
 *
 * Usage example (Arduino):
 *
 *   MCP2515Driver can(10);          // CS pin 10
 *   DJIRonin ronin(can);
 *   ronin.begin();
 *
 *   ronin.motion.moveTo(900, 0, 0, 20);   // 90.0°, 2 s
 *   ronin.camera.recordStart();
 */

#ifndef DJI_RONIN_API_H
#define DJI_RONIN_API_H

#include "../transport/interfaces/ICANDriver.h"
#include "../protocol/PacketBuilder.h"
#include "../protocol/PacketParser.h"
#include "../protocol/RoninCommands.h"
#include "../protocol/RoninPayload.h"
#include "../protocol/RoninEnums.h"
#include "../protocol/Result.h"
#include "../protocol/RoninConstants.h"

namespace dji {
namespace ronin {

// Forward declarations
class DJIRonin;

// ---------------------------------------------------------------------------
// Motion module
// ---------------------------------------------------------------------------
class Motion {
public:
    explicit Motion(DJIRonin& parent) : parent_(parent), speedScale_(100) {}

    /**
     * Absolute position control (CmdID 0x00).
     * Angles are in 0.1° units as required by the protocol.
     * timeForAction is in 0.1 s units (pass the value exactly, no conversion).
     */
    Result<void> moveTo(int16_t yaw, int16_t pitch, int16_t roll, uint8_t timeForAction);

    /**
     * Relative (incremental) position control.
     */
    Result<void> moveBy(int16_t yaw, int16_t pitch, int16_t roll, uint8_t timeForAction);

    /**
     * Official DJI Joystick / External Device Control command (CmdID 0x0A).
     * Values must already be in the protocol range (-15000 .. +15000).
     *
     * IMPORTANT: This command is periodic.
     * The Ronin stops motion ~0.5 s after the last packet.
     * User must call it repeatedly (e.g. every 20 ms). The library does NOT
     * start internal timers.
     */
    Result<void> joystick(int16_t yaw, int16_t pitch, int16_t roll);

    /**
     * Scale outgoing joystick values (0..100 %). Optional helper.
     */
    void setSpeedScale(uint8_t percent);

    Result<void> stop();
    Result<void> home();   // recenter

private:
    DJIRonin& parent_;
    uint8_t   speedScale_;
};

// ---------------------------------------------------------------------------
// Camera module
// ---------------------------------------------------------------------------
class Camera {
public:
    explicit Camera(DJIRonin& parent) : parent_(parent) {}

    Result<void> recordStart();
    Result<void> recordStop();
    Result<void> photo();
    Result<void> centerFocus();

private:
    DJIRonin& parent_;
    Result<void> sendMotionCommand(CameraMotionCommand cmd);
};

// ---------------------------------------------------------------------------
// Focus module
// ---------------------------------------------------------------------------
class Focus {
public:
    explicit Focus(DJIRonin& parent) : parent_(parent) {}

    Result<void> moveTo(uint16_t position);   // 0..4095
    Result<void> stop();
    Result<void> calibrate();

private:
    DJIRonin& parent_;
};

// ---------------------------------------------------------------------------
// ActiveTrack module
// ---------------------------------------------------------------------------
class ActiveTrack {
public:
    explicit ActiveTrack(DJIRonin& parent) : parent_(parent) {}

    Result<void> start();
    Result<void> stop();

private:
    DJIRonin& parent_;
    Result<void> sendToggle();
};

// ---------------------------------------------------------------------------
// Main library class
// ---------------------------------------------------------------------------
class DJIRonin {
public:
    explicit DJIRonin(ICANDriver& driver)
        : driver_(driver)
        , motion(*this)
        , camera(*this)
        , focus(*this)
        , activeTrack(*this)
        , rxAccLen_(0)
        , rxExpectedLen_(0)
    {}

    /**
     * Initialize transport and internal state.
     * Must be called before any command.
     */
    Result<void> begin();

    // Public modules – simple names as required by the specification
    Motion      motion;
    Camera      camera;
    Focus       focus;
    ActiveTrack activeTrack;

    // Internal helpers used by the modules
    Result<void> sendPacket(const PacketBuffer& packet);

    /**
     * Receive and reassemble a full DJI protocol packet from multiple
     * 8-byte CAN frames (ID = CAN_ID_RX).
     *
     * Blocks up to timeoutMs while collecting frames. Uses the Length field
     * in the DJI header to know when the packet is complete, then runs CRC
     * validation via PacketParser.
     */
    Result<ParsedPacket> receivePacket(uint32_t timeoutMs = 100);

    PacketBuilder& builder() { return builder_; }
    PacketParser&  parser()  { return parser_; }
    ICANDriver&    driver()  { return driver_; }

private:
    ICANDriver&   driver_;
    PacketBuilder builder_;
    PacketParser  parser_;

    // Multi-frame RX reassembly state
    uint8_t  rxAcc_[MAX_PACKET_SIZE];
    uint16_t rxAccLen_;
    uint16_t rxExpectedLen_;

    void resetRxAssembly();
    Result<void> sendAsCanFrames(const PacketBuffer& packet);
};

} // namespace ronin
} // namespace dji

#endif // DJI_RONIN_API_H
