/**
 * @file RoninPayload.h
 * @brief Binary payload structures that exactly match DJI R SDK Protocol v2.5
 *
 * All multi-byte fields are little-endian as required by the protocol.
 * Structures are packed to guarantee binary layout.
 */

#ifndef DJI_RONIN_PAYLOAD_H
#define DJI_RONIN_PAYLOAD_H

#include <stdint.h>
#include "RoninEnums.h"

namespace dji {
namespace ronin {

#pragma pack(push, 1)

// ---------------------------------------------------------------------------
// 2.3.4.1 Position Control (CmdSet 0x0E, CmdID 0x00)
// ---------------------------------------------------------------------------
struct PositionControlPayload {
    int16_t  yaw_angle;       // unit 0.1°, range -1800 .. +1800
    int16_t  roll_angle;      // unit 0.1°, range -300  .. +300
    int16_t  pitch_angle;     // unit 0.1°, range -560  .. +1460
    uint8_t  ctrl_byte;       // bit0 = mode, bit1=yaw invalid, bit2=roll invalid, bit3=pitch invalid
    uint8_t  time_for_action; // unit 0.1 s
};

// Helper to build ctrl_byte
inline uint8_t makePositionCtrlByte(ControlMode mode,
                                    AxisValid yawValid,
                                    AxisValid rollValid,
                                    AxisValid pitchValid)
{
    uint8_t b = 0;
    b |= (static_cast<uint8_t>(mode) & 0x01);
    b |= (static_cast<uint8_t>(yawValid)   & 0x01) << 1;
    b |= (static_cast<uint8_t>(rollValid)  & 0x01) << 2;
    b |= (static_cast<uint8_t>(pitchValid) & 0x01) << 3;
    // bits 7:4 must be 0
    return b;
}

// ---------------------------------------------------------------------------
// 2.3.4.2 Speed Control (CmdSet 0x0E, CmdID 0x01)
// ---------------------------------------------------------------------------
struct SpeedControlPayload {
    int16_t  yaw_speed;   // unit 0.1°/s
    int16_t  roll_speed;  // unit 0.1°/s
    int16_t  pitch_speed; // unit 0.1°/s
    uint8_t  ctrl_byte;   // bit7 = take-over, bit3 = focal length
};

inline uint8_t makeSpeedCtrlByte(SpeedControlBit takeOver,
                                 FocalLengthConsideration focal)
{
    uint8_t b = 0;
    b |= (static_cast<uint8_t>(takeOver) & 0x01) << 7;
    b |= (static_cast<uint8_t>(focal)    & 0x01) << 3;
    // other bits must be 0
    return b;
}

// ---------------------------------------------------------------------------
// 2.3.4.3 Obtain Information (CmdSet 0x0E, CmdID 0x02)
// ---------------------------------------------------------------------------
struct ObtainInfoRequestPayload {
    uint8_t ctrl_byte;  // 0x00 none, 0x01 attitude, 0x02 joint
};

struct ObtainInfoReplyPayload {
    uint8_t  return_code;
    uint8_t  data_type;   // 0x00 not ready, 0x01 attitude, 0x02 joint
    int16_t  yaw;
    int16_t  roll;
    int16_t  pitch;
};

// ---------------------------------------------------------------------------
// 2.3.4.11 External Device / Joystick (CmdSet 0x0E, CmdID 0x0A)
// ---------------------------------------------------------------------------
struct JoystickPayload {
    uint8_t  device_type;  // 0x01 = Joystick
    int16_t  pitch_speed;  // -15000 .. +15000
    int16_t  roll_speed;   // -15000 .. +15000
    int16_t  yaw_speed;    // -15000 .. +15000
};

struct DialPayload {
    uint8_t  device_type;  // 0x02 = Dial
    int16_t  dial_speed;   // -2048 .. +2048
};

// ---------------------------------------------------------------------------
// 2.3.4.14 / 2.3.4.15 Operating Mode / Recenter / Selfie / Follow
// ---------------------------------------------------------------------------
struct OperatingModePayload {
    uint8_t operating_mode;
    uint8_t recenter_selfie;  // only used for Recenter/Selfie commands
};

// ---------------------------------------------------------------------------
// 2.3.4.18 ActiveTrack
// ---------------------------------------------------------------------------
struct ActiveTrackPayload {
    uint8_t enable;  // 0x03 = toggle
};

// ---------------------------------------------------------------------------
// 2.3.4.19 Focus Motor Position Control
// ---------------------------------------------------------------------------
struct FocusPositionPayload {
    uint8_t  sub_id;        // 0x01
    uint8_t  control_type;  // 0x00
    uint8_t  data_length;   // 0x02
    uint16_t absolute_pos;  // 0 .. 4095
};

// ---------------------------------------------------------------------------
// 2.3.5.1 Camera Motion
// ---------------------------------------------------------------------------
struct CameraMotionPayload {
    uint16_t command;  // CameraMotionCommand
};

// ---------------------------------------------------------------------------
// 2.3.5.2 Camera Status
// ---------------------------------------------------------------------------
struct CameraStatusRequestPayload {
    uint8_t query;  // 0x01 = recording status
};

struct CameraStatusReplyPayload {
    uint8_t return_code;
    uint8_t status;  // 0x00 not recording, 0x02 recording
};

// ---------------------------------------------------------------------------
// Generic single-byte return code (most reply frames)
// ---------------------------------------------------------------------------
struct ReturnCodePayload {
    uint8_t return_code;
};

#pragma pack(pop)

} // namespace ronin
} // namespace dji

#endif // DJI_RONIN_PAYLOAD_H
