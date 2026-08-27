/**
 * @file RoninCommands.h
 * @brief Command Set and Command ID definitions from DJI R SDK Protocol v2.5
 *
 * Values taken exactly from section 2.3.1 / Figure 4.
 */

#ifndef DJI_RONIN_COMMANDS_H
#define DJI_RONIN_COMMANDS_H

#include <stdint.h>
#include "RoninConstants.h"

namespace dji {
namespace ronin {

// ---------------------------------------------------------------------------
// Gimbal Command Set (CmdSet = 0x0E)
// ---------------------------------------------------------------------------
namespace GimbalCmd {

    constexpr uint8_t CMDSET = CMDSET_GIMBAL;  // 0x0E

    /** 2.3.4.1 Handheld Gimbal Position Control */
    constexpr uint8_t PositionControl          = 0x00;

    /** 2.3.4.2 Handheld Gimbal Speed Control */
    constexpr uint8_t SpeedControl             = 0x01;

    /** 2.3.4.3 Obtain Handheld Gimbal Information */
    constexpr uint8_t ObtainInformation        = 0x02;

    /** 2.3.4.4 Handheld Gimbal Limit Angle Settings */
    constexpr uint8_t SetLimitAngle            = 0x03;

    /** 2.3.4.5 Obtain Handheld Gimbal Limit Angle */
    constexpr uint8_t ObtainLimitAngle         = 0x04;

    /** 2.3.4.6 Handheld Gimbal Motor Stiffness Settings */
    constexpr uint8_t SetMotorStiffness        = 0x05;

    /** 2.3.4.7 Obtain Handheld Gimbal Motor Stiffness */
    constexpr uint8_t ObtainMotorStiffness     = 0x06;

    /** 2.3.4.8 Handheld Gimbal Parameter Push Settings */
    constexpr uint8_t SetParameterPush         = 0x07;

    /** 2.3.4.9 Push Handheld Gimbal Parameters */
    constexpr uint8_t ParameterPush            = 0x08;

    /** 2.3.4.10 Obtain Module Version Number */
    constexpr uint8_t ObtainModuleVersion      = 0x09;

    /** 2.3.4.11 External Device Control Command Push (Joystick / Dial) */
    constexpr uint8_t ExternalDeviceControl    = 0x0A;

    /** 2.3.4.12 Obtain Handheld Gimbal User Parameters */
    constexpr uint8_t ObtainUserParameters     = 0x0B;

    /** 2.3.4.13 Handheld Gimbal User Parameters Settings */
    constexpr uint8_t SetUserParameters        = 0x0C;

    /** 2.3.4.14 Handheld Gimbal Operating Mode Settings */
    constexpr uint8_t SetOperatingMode         = 0x0D;

    /** 2.3.4.15 Handheld Gimbal Recenter, Selfie, and Follow Modes Settings */
    constexpr uint8_t SetRecenterSelfieFollow  = 0x0E;

    /** 2.3.4.16 Gimbal Auto Calibration Settings */
    constexpr uint8_t SetAutoCalibration       = 0x0F;

    /** 2.3.4.17 Gimbal Auto Calibration Status Push */
    constexpr uint8_t AutoCalibrationStatusPush = 0x10;

    /** 2.3.4.18 Gimbal ActiveTrack Settings */
    constexpr uint8_t SetActiveTrack           = 0x11;

    /** 2.3.4.19 Focus Motor Control Command */
    constexpr uint8_t FocusMotorControl        = 0x12;

} // namespace GimbalCmd

// ---------------------------------------------------------------------------
// Camera Command Set (CmdSet = 0x0D)
// ---------------------------------------------------------------------------
namespace CameraCmd {

    constexpr uint8_t CMDSET = CMDSET_CAMERA;  // 0x0D

    /** 2.3.5.1 Third-Party Camera Motion Command */
    constexpr uint8_t Motion                   = 0x00;

    /** 2.3.5.2 Third-Party Camera Status Obtain Command */
    constexpr uint8_t ObtainStatus             = 0x01;

} // namespace CameraCmd

} // namespace ronin
} // namespace dji

#endif // DJI_RONIN_COMMANDS_H
