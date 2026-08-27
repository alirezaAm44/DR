/**
 * @file DJIRonin.cpp
 * @brief High-level API implementation
 */

#include "DJIRonin.h"

namespace dji {
namespace ronin {

// ---------------------------------------------------------------------------
// DJIRonin core
// ---------------------------------------------------------------------------
Result<void> DJIRonin::begin()
{
    builder_.resetSequence(0);
    return driver_.begin();
}

Result<void> DJIRonin::sendAsCanFrames(const PacketBuffer& packet)
{
    // Send the DJI packet as consecutive 8-byte CAN frames.
    // Receiver uses the length field inside the protocol header to reassemble.
    size_t offset = 0;
    while (offset < packet.length) {
        CANFrame frame;
        frame.id       = CAN_ID_TX;
        frame.extended = false;
        frame.length   = 0;
        while (frame.length < 8 && offset < packet.length) {
            frame.data[frame.length++] = packet.data[offset++];
        }
        Result<void> r = driver_.send(frame);
        if (r.isError()) return r;
    }
    return Result<void>::success();
}

Result<void> DJIRonin::sendPacket(const PacketBuffer& packet)
{
    return sendAsCanFrames(packet);
}

Result<ParsedPacket> DJIRonin::receivePacket(uint32_t /*timeoutMs*/)
{
    // Simplified non-blocking receive (single frame).
    // Full multi-frame reassembly can be added later.
    CANFrame frame;
    Result<void> r = driver_.receive(frame);
    if (r.isError()) {
        return Result<ParsedPacket>::failure(r.error());
    }
    return parser_.parse(frame.data, frame.length);
}

// ---------------------------------------------------------------------------
// Motion
// ---------------------------------------------------------------------------
Result<void> Motion::moveTo(int16_t yaw, int16_t pitch, int16_t roll, uint8_t timeForAction)
{
    PositionControlPayload p;
    p.yaw_angle       = yaw;
    p.roll_angle      = roll;
    p.pitch_angle     = pitch;
    p.ctrl_byte       = makePositionCtrlByte(ControlMode::Absolute,
                                             AxisValid::Valid,
                                             AxisValid::Valid,
                                             AxisValid::Valid);
    p.time_for_action = timeForAction;

    PacketBuffer pkt;
    auto r = parent_.builder().buildCommand(
        GimbalCmd::CMDSET,
        GimbalCmd::PositionControl,
        p,
        ReplyRequirement::NoReply,
        pkt);
    if (r.isError()) return r;
    return parent_.sendPacket(pkt);
}

Result<void> Motion::moveBy(int16_t yaw, int16_t pitch, int16_t roll, uint8_t timeForAction)
{
    PositionControlPayload p;
    p.yaw_angle       = yaw;
    p.roll_angle      = roll;
    p.pitch_angle     = pitch;
    p.ctrl_byte       = makePositionCtrlByte(ControlMode::Incremental,
                                             AxisValid::Valid,
                                             AxisValid::Valid,
                                             AxisValid::Valid);
    p.time_for_action = timeForAction;

    PacketBuffer pkt;
    auto r = parent_.builder().buildCommand(
        GimbalCmd::CMDSET,
        GimbalCmd::PositionControl,
        p,
        ReplyRequirement::NoReply,
        pkt);
    if (r.isError()) return r;
    return parent_.sendPacket(pkt);
}

Result<void> Motion::joystick(int16_t yaw, int16_t pitch, int16_t roll)
{
    if (speedScale_ != 100) {
        yaw   = static_cast<int16_t>((static_cast<int32_t>(yaw)   * speedScale_) / 100);
        pitch = static_cast<int16_t>((static_cast<int32_t>(pitch) * speedScale_) / 100);
        roll  = static_cast<int16_t>((static_cast<int32_t>(roll)  * speedScale_) / 100);
    }

    JoystickPayload p;
    p.device_type = static_cast<uint8_t>(ControllerType::Joystick);
    p.pitch_speed = pitch;
    p.roll_speed  = roll;
    p.yaw_speed   = yaw;

    PacketBuffer pkt;
    auto r = parent_.builder().buildCommand(
        GimbalCmd::CMDSET,
        GimbalCmd::ExternalDeviceControl,
        p,
        ReplyRequirement::NoReply,
        pkt);
    if (r.isError()) return r;
    return parent_.sendPacket(pkt);
}

void Motion::setSpeedScale(uint8_t percent)
{
    if (percent > 100) percent = 100;
    speedScale_ = percent;
}

Result<void> Motion::stop()
{
    return joystick(0, 0, 0);
}

Result<void> Motion::home()
{
    OperatingModePayload p;
    p.operating_mode  = static_cast<uint8_t>(OperatingMode::Unchanged);
    p.recenter_selfie = static_cast<uint8_t>(RecenterSelfieCommand::Recenter);

    PacketBuffer pkt;
    auto r = parent_.builder().buildCommand(
        GimbalCmd::CMDSET,
        GimbalCmd::SetRecenterSelfieFollow,
        p,
        ReplyRequirement::NoReply,
        pkt);
    if (r.isError()) return r;
    return parent_.sendPacket(pkt);
}

// ---------------------------------------------------------------------------
// Camera
// ---------------------------------------------------------------------------
Result<void> Camera::sendMotionCommand(CameraMotionCommand cmd)
{
    CameraMotionPayload p;
    p.command = static_cast<uint16_t>(cmd);

    PacketBuffer pkt;
    auto r = parent_.builder().buildCommand(
        CameraCmd::CMDSET,
        CameraCmd::Motion,
        p,
        ReplyRequirement::NoReply,
        pkt);
    if (r.isError()) return r;
    return parent_.sendPacket(pkt);
}

Result<void> Camera::recordStart() { return sendMotionCommand(CameraMotionCommand::StartRecording); }
Result<void> Camera::recordStop()  { return sendMotionCommand(CameraMotionCommand::StopRecording); }
Result<void> Camera::photo()       { return sendMotionCommand(CameraMotionCommand::Shutter); }
Result<void> Camera::centerFocus() { return sendMotionCommand(CameraMotionCommand::CenterFocus); }

// ---------------------------------------------------------------------------
// Focus
// ---------------------------------------------------------------------------
Result<void> Focus::moveTo(uint16_t position)
{
    if (position > 4095) {
        return Result<void>::failure(Error::InvalidParameter);
    }
    FocusPositionPayload p;
    p.sub_id       = static_cast<uint8_t>(FocusSubId::PositionControl);
    p.control_type = static_cast<uint8_t>(FocusControlType::Focus);
    p.data_length  = 0x02;
    p.absolute_pos = position;

    PacketBuffer pkt;
    auto r = parent_.builder().buildCommand(
        GimbalCmd::CMDSET,
        GimbalCmd::FocusMotorControl,
        p,
        ReplyRequirement::NoReply,
        pkt);
    if (r.isError()) return r;
    return parent_.sendPacket(pkt);
}

Result<void> Focus::stop()
{
    return moveTo(0);
}

Result<void> Focus::calibrate()
{
    uint8_t payload[3] = { 0x00, 0x01, 0x01 }; // TLV: start self-tuning
    PacketBuffer pkt;
    auto r = parent_.builder().buildCommand(
        GimbalCmd::CMDSET,
        GimbalCmd::SetAutoCalibration,
        payload, 3,
        ReplyRequirement::NoReply,
        pkt);
    if (r.isError()) return r;
    return parent_.sendPacket(pkt);
}

// ---------------------------------------------------------------------------
// ActiveTrack
// ---------------------------------------------------------------------------
Result<void> ActiveTrack::sendToggle()
{
    ActiveTrackPayload p;
    p.enable = static_cast<uint8_t>(ActiveTrackCommand::Toggle);

    PacketBuffer pkt;
    auto r = parent_.builder().buildCommand(
        GimbalCmd::CMDSET,
        GimbalCmd::SetActiveTrack,
        p,
        ReplyRequirement::NoReply,
        pkt);
    if (r.isError()) return r;
    return parent_.sendPacket(pkt);
}

Result<void> ActiveTrack::start() { return sendToggle(); }
Result<void> ActiveTrack::stop()  { return sendToggle(); }

} // namespace ronin
} // namespace dji
