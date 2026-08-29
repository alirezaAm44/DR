/**
 * @file DJIRonin.cpp
 * @brief High-level API implementation
 */

#include "DJIRonin.h"

#if defined(ARDUINO)
#include <Arduino.h>
#endif

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
    size_t offset = 0;

    while (offset < packet.length) {

        CANFrame frame;
        frame.id = CAN_ID_TX;
        frame.extended = false;
        frame.length = 0;

        // Split complete DJI packet into standard CAN frames,
        // maximum 8 data bytes per frame.
        while (frame.length < 8 && offset < packet.length) {
            frame.data[frame.length++] = packet.data[offset++];
        }

        // Send the CAN frame.
        Result<void> r = driver_.send(frame);

        if (r.isError()) {
            return r;
        }

        /*
         * MCP2515 has three hardware TX buffers.
         *
         * At 1 Mbps, an 8-byte CAN frame takes roughly
         * 100+ microseconds on the physical bus.
         *
         * Waiting 250 us gives the MCP2515 enough time to
         * transmit the previous frame and free a TX buffer
         * before the next DJI CAN frame is queued.
         *
         * Do not wait after the final frame.
         */
#if defined(ARDUINO)
        if (offset < packet.length) {
            delayMicroseconds(250);
        }
#endif
    }

    return Result<void>::success();
}

Result<void> DJIRonin::sendPacket(const PacketBuffer& packet)
{
    return sendAsCanFrames(packet);
}

Result<ParsedPacket> DJIRonin::receivePacket(uint32_t /*timeoutMs*/)
{
    CANFrame frame;

    /*
     * Read all CAN frames currently available.
     *
     * A single DJI R SDK packet may be split across multiple
     * standard CAN frames (maximum 8 bytes per frame).
     *
     * We therefore accumulate frames until the total packet
     * length specified in the DJI Ver/Length field is reached.
     */

    while (true) {

        Result<void> r = driver_.receive(frame);

        if (r.isError()) {

            /*
             * No CAN frame available right now.
             *
             * IMPORTANT:
             * If we are already assembling a packet, keep the
             * accumulated data. The next call to receivePacket()
             * will continue the same packet.
             */
            return Result<ParsedPacket>::failure(r.error());
        }

        /*
         * We only expect standard 11-bit CAN frames from the Ronin.
         *
         * The MCP2515 driver already masks received IDs to 11 bits,
         * but keeping this check here makes the protocol layer safer.
         */
        if (frame.extended) {
            continue;
        }

        /*
         * Ronin → third-party device.
         */
        if (frame.id != CAN_ID_RX) {
            continue;
        }

        /*
         * Ignore invalid CAN DLC.
         */
        if (frame.length == 0 || frame.length > 8) {
            continue;
        }

        /*
         * -------------------------------------------------------------------
         * Start of a new DJI packet
         * -------------------------------------------------------------------
         */
        if (!rxReceiving_) {

            /*
             * The first byte of every DJI R SDK packet must be 0xAA.
             */
            if (frame.data[0] != SOF) {
                continue;
            }

            /*
             * Start a fresh packet.
             */
            rxPacket_.clear();
            rxExpectedLength_ = 0;
            rxReceiving_ = true;
        }

        /*
         * -------------------------------------------------------------------
         * Append current CAN frame to the packet accumulator.
         * -------------------------------------------------------------------
         */
        if (static_cast<size_t>(rxPacket_.length) + frame.length >
            PACKET_BUFFER_SIZE) {

            /*
             * Packet is larger than our supported buffer.
             * Drop the incomplete packet and resynchronize.
             */
            rxPacket_.clear();
            rxExpectedLength_ = 0;
            rxReceiving_ = false;

            return Result<ParsedPacket>::failure(Error::InvalidLength);
        }

        for (uint8_t i = 0; i < frame.length; ++i) {
            rxPacket_.data[rxPacket_.length++] = frame.data[i];
        }

        /*
         * -------------------------------------------------------------------
         * Extract total packet length once the first 3 bytes are available.
         *
         * DJI layout:
         *
         * byte 0 = SOF
         * byte 1 = Ver/Length LSB
         * byte 2 = Ver/Length MSB
         *
         * Length occupies bits [9:0].
         * -------------------------------------------------------------------
         */
        if (rxExpectedLength_ == 0 &&
            rxPacket_.length >= 3) {

            const uint16_t verLength =
                static_cast<uint16_t>(rxPacket_.data[OFFSET_VER_LENGTH]) |
                (static_cast<uint16_t>(
                    rxPacket_.data[OFFSET_VER_LENGTH + 1]) << 8);

            const uint16_t expectedLength =
                verLength & LENGTH_MASK;

            /*
             * Basic sanity checks before accepting the length.
             */
            if (expectedLength < MIN_PACKET_SIZE ||
                expectedLength > PACKET_BUFFER_SIZE) {

                rxPacket_.clear();
                rxExpectedLength_ = 0;
                rxReceiving_ = false;

                return Result<ParsedPacket>::failure(Error::InvalidLength);
            }

            rxExpectedLength_ = expectedLength;
        }

        /*
         * -------------------------------------------------------------------
         * Packet not complete yet.
         * -------------------------------------------------------------------
         */
        if (rxExpectedLength_ == 0 ||
            rxPacket_.length < rxExpectedLength_) {

            continue;
        }

        /*
         * -------------------------------------------------------------------
         * Complete DJI packet received.
         * -------------------------------------------------------------------
         */

        if (rxPacket_.length != rxExpectedLength_) {

            /*
             * We should never have more bytes than the declared
             * packet length because CAN frames are appended sequentially.
             *
             * If it happens, discard and resynchronize.
             */
            rxPacket_.clear();
            rxExpectedLength_ = 0;
            rxReceiving_ = false;

            return Result<ParsedPacket>::failure(Error::InvalidLength);
        }

        /*
         * Parse and validate:
         *
         * - SOF
         * - Version
         * - Length
         * - CRC16
         * - CRC32
         * - CmdSet
         * - CmdID
         * - Payload
         */
        Result<ParsedPacket> parsed =
            parser_.parse(rxPacket_.data, rxPacket_.length);

        /*
         * Reset RX state AFTER parsing.
         *
         * ParsedPacket contains pointers into rxPacket_.data,
         * so do not clear the buffer before parser.parse() returns.
         */
        rxPacket_.clear();
        rxExpectedLength_ = 0;
        rxReceiving_ = false;

        return parsed;
    }
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
