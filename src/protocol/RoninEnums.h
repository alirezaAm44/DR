/**
 * @file RoninEnums.h
 * @brief Enumerations extracted from DJI R SDK Protocol v2.5
 *
 * All values come directly from the official documentation.
 */

#ifndef DJI_RONIN_ENUMS_H
#define DJI_RONIN_ENUMS_H

#include <stdint.h>

namespace dji {
namespace ronin {

// ---------------------------------------------------------------------------
// Return / Error Codes (section 2.3.2 / Figure 5)
// ---------------------------------------------------------------------------
enum class ReturnCode : uint8_t {
    Success          = 0x00,  // Command execution succeeds
    ParseError       = 0x01,  // Command parse error
    ExecutionFailed  = 0x02,  // Command execution fails
    UndefinedError   = 0xFF   // Undefined error
};

// ---------------------------------------------------------------------------
// Library-level Error (for Result<T, Error>)
// ---------------------------------------------------------------------------
enum class Error : uint8_t {
    Ok = 0,
    InvalidParameter,
    PacketTooLarge,
    Crc16Mismatch,
    Crc32Mismatch,
    InvalidSof,
    InvalidLength,
    InvalidResponse,
    Timeout,
    TransportError,
    ProtocolVersionMismatch,
    UnknownCommand,
    BufferTooSmall,
    NotInitialized,
    NoData
};

// ---------------------------------------------------------------------------
// CmdType helpers (section 2.2)
// ---------------------------------------------------------------------------
enum class FrameType : uint8_t {
    Command = 0,   // bit5 = 0
    Reply   = 1    // bit5 = 1
};

enum class ReplyRequirement : uint8_t {
    NoReply         = 0,   // 0
    OptionalReply   = 1,   // 1
    ReplyRequired   = 2    // 2-31 treated as required
};

// ---------------------------------------------------------------------------
// Position Control – ctrl_byte bits (section 2.3.4.1)
// ---------------------------------------------------------------------------
enum class ControlMode : uint8_t {
    Incremental = 0,  // bit0 = 0
    Absolute    = 1   // bit0 = 1
};

// Axis validity flags (inverted logic in protocol: 0 = valid, 1 = invalid)
enum class AxisValid : uint8_t {
    Valid   = 0,
    Invalid = 1
};

// ---------------------------------------------------------------------------
// Speed Control – ctrl_byte bits (section 2.3.4.2)
// ---------------------------------------------------------------------------
enum class SpeedControlBit : uint8_t {
    Release = 0,  // bit7 = 0
    TakeOver = 1  // bit7 = 1
};

enum class FocalLengthConsideration : uint8_t {
    Consider   = 0,  // bit3 = 0
    Ignore     = 1   // bit3 = 1
};

// ---------------------------------------------------------------------------
// Information Obtain – data type (section 2.3.4.3)
// ---------------------------------------------------------------------------
enum class AngleType : uint8_t {
    NoOperation   = 0x00,
    AttitudeAngle = 0x01,
    JointAngle    = 0x02
};

// ---------------------------------------------------------------------------
// External Device / Joystick (section 2.3.4.11)
// ---------------------------------------------------------------------------
enum class ControllerType : uint8_t {
    Unknown  = 0x00,
    Joystick = 0x01,
    Dial     = 0x02
};

// ---------------------------------------------------------------------------
// Operating Mode (section 2.3.4.14 / 2.3.4.15)
// ---------------------------------------------------------------------------
enum class OperatingMode : uint8_t {
    Unchanged               = 0xFE,
    NoLandscapePortraitSwitch = 0x00,
    Landscape0              = 0x01,  // 0° around X
    Landscape180            = 0x02,  // 180° around X
    Portrait90              = 0x03,  // 90° around X
    PortraitMinus90         = 0x04,  // -90° around X
    AutoSwitch              = 0x05,
    RestoreDefault          = 0xFF,

    // Follow / Lock modes (used with CmdID 0x0E)
    LockMode                = 0x00,
    YawFollowMode           = 0x02,
    SportMode               = 0x03
};

enum class RecenterSelfieCommand : uint8_t {
    Unchanged = 0x00,
    Recenter  = 0x01,
    Selfie    = 0x02
};

// ---------------------------------------------------------------------------
// ActiveTrack (section 2.3.4.18)
// ---------------------------------------------------------------------------
enum class ActiveTrackCommand : uint8_t {
    Toggle = 0x03   // switch start/stop status of tracking
};

// ---------------------------------------------------------------------------
// Focus Motor (section 2.3.4.19)
// ---------------------------------------------------------------------------
enum class FocusSubId : uint8_t {
    Reserved          = 0x00,
    PositionControl   = 0x01,
    Calibration       = 0x02,
    ObtainPosition    = 0x15
};

enum class FocusControlType : uint8_t {
    Focus = 0x00
};

enum class FocusCalibrationAction : uint8_t {
    NoControl           = 0x00,
    EnableAuto          = 0x01,
    EnableManual        = 0x02,
    SetMinRange         = 0x04,
    SetMaxRange         = 0x05,
    Stop                = 0x06
};

enum class FocusCalibrationStatus : uint8_t {
    NoCalibration     = 0x01,
    Calibrating       = 0x02,
    CalibrationComplete = 0x03
};

// ---------------------------------------------------------------------------
// Camera commands (section 2.3.5.1)
// ---------------------------------------------------------------------------
enum class CameraMotionCommand : uint16_t {
    Shutter         = 0x0001,
    StopShutter     = 0x0002,
    StartRecording  = 0x0003,
    StopRecording   = 0x0004,
    CenterFocus     = 0x0005
};

enum class CameraStatusQuery : uint8_t {
    QueryRecording = 0x01
};

enum class CameraRecordingStatus : uint8_t {
    NotRecording = 0x00,
    Recording    = 0x02
};

} // namespace ronin
} // namespace dji

#endif // DJI_RONIN_ENUMS_H
