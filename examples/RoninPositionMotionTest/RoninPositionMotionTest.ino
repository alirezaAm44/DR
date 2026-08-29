/**
 * RoninPositionMotionTest.ino
 *
 * Deterministic field test for DJI Ronin position control.
 *
 * Purpose:
 *   Prove that CAN transport + packet builder + position-control command
 *   work independently of the joystick path.
 *
 * Sequence (repeatable):
 *   1. Request Ronin information.
 *   2. Move yaw to +20.0 degrees.
 *   3. Move yaw to -20.0 degrees.
 *   4. Move yaw back to 0.
 *   5. Move pitch to +10.0 degrees.
 *   6. Move pitch to -10.0 degrees.
 *   7. Return all axes to zero.
 *   8. Pause and repeat.
 *
 * Protocol angles are 0.1 degree units. The library's Motion::moveTo()
 * already creates the official PositionControl command and uses the same
 * CAN fragmentation path as the joystick command.
 *
 * IMPORTANT:
 *   Start with small angles and a properly balanced, unlocked Ronin.
 *   Keep hands clear of the axes and camera.
 */

#include <SPI.h>
#include <DJIRonin.h>

using namespace dji::ronin;

const uint8_t MCP2515_CS_PIN = 10;
const uint32_t STEP_SETTLE_MS = 2500;
const uint32_t INFO_RETRY_MS = 1000;

MCP2515Driver can(MCP2515_CS_PIN);
DJIRonin ronin(can);

bool roninSeen = false;
unsigned long lastInfo = 0;

struct MotionStep {
    const char* name;
    int16_t yaw;
    int16_t pitch;
    int16_t roll;
    uint8_t timeForAction;
};

// 0.1 degree units. Start deliberately conservative.
const MotionStep STEPS[] = {
    {"Yaw +20 deg",  200,    0,    0, 20},
    {"Yaw -20 deg", -200,    0,    0, 20},
    {"Yaw center",     0,    0,    0, 20},
    {"Pitch +10 deg",  0,  100,   0, 20},
    {"Pitch -10 deg",  0, -100,   0, 20},
    {"All center",     0,    0,    0, 20}
};

const size_t STEP_COUNT = sizeof(STEPS) / sizeof(STEPS[0]);
size_t stepIndex = 0;
unsigned long stepAt = 0;

const char* errorName(Error e)
{
    switch (e) {
        case Error::Ok: return "Ok";
        case Error::InvalidParameter: return "InvalidParameter";
        case Error::PacketTooLarge: return "PacketTooLarge";
        case Error::Crc16Mismatch: return "Crc16Mismatch";
        case Error::Crc32Mismatch: return "Crc32Mismatch";
        case Error::InvalidSof: return "InvalidSof";
        case Error::InvalidLength: return "InvalidLength";
        case Error::InvalidResponse: return "InvalidResponse";
        case Error::Timeout: return "Timeout";
        case Error::TransportError: return "TransportError";
        case Error::ProtocolVersionMismatch: return "ProtocolVersionMismatch";
        case Error::UnknownCommand: return "UnknownCommand";
        case Error::BufferTooSmall: return "BufferTooSmall";
        case Error::NotInitialized: return "NotInitialized";
        case Error::NoData: return "NoData";
        default: return "Unknown";
    }
}

void printHex(const uint8_t* data, size_t len)
{
    for (size_t i = 0; i < len; ++i) {
        if (data[i] < 0x10) Serial.print('0');
        Serial.print(data[i], HEX);
        Serial.print(i + 1 == len ? '\n' : ' ');
    }
}

void requestInfo()
{
    ObtainInfoRequestPayload p;
    p.ctrl_byte = 0x01;
    PacketBuffer packet;

    auto build = ronin.builder().buildCommand(
        GimbalCmd::CMDSET,
        GimbalCmd::ObtainInformation,
        p,
        ReplyRequirement::ReplyRequired,
        packet);

    if (build.isError()) {
        Serial.print(F("[INFO BUILD ERROR] "));
        Serial.println(errorName(build.error()));
        return;
    }

    Serial.print(F("[INFO TX] length="));
    Serial.println(packet.length);
    printHex(packet.data, packet.length);

    auto tx = ronin.sendPacket(packet);
    if (tx.isError()) {
        Serial.print(F("[INFO TX ERROR] "));
        Serial.println(errorName(tx.error()));
        return;
    }

    auto rx = ronin.receivePacket(100);
    if (rx.isError()) {
        Serial.print(F("[INFO] No Ronin response: "));
        Serial.println(errorName(rx.error()));
        return;
    }

    roninSeen = true;
    const ParsedPacket& pRx = rx.value();
    Serial.println(F("[INFO] *** RONIN RESPONSE RECEIVED ***"));
    Serial.print(F("  CmdSet=0x")); Serial.println(pRx.command.cmdSet, HEX);
    Serial.print(F("  CmdID=0x")); Serial.println(pRx.command.cmdId, HEX);
    Serial.print(F("  Length=")); Serial.println(pRx.header.length);
    Serial.print(F("  Seq=")); Serial.println(pRx.header.seq);
    Serial.print(F("  Payload=")); Serial.println(pRx.payloadLen);
    if (pRx.hasReturnCode) {
        Serial.print(F("  ReturnCode=0x"));
        Serial.println(static_cast<uint8_t>(pRx.returnCode), HEX);
    }
}

void runStep(const MotionStep& s)
{
    Serial.println();
    Serial.println(F("------------------------------------------------------------"));
    Serial.print(F("[MOTION STEP] "));
    Serial.println(s.name);
    Serial.print(F("Target yaw=")); Serial.print(s.yaw);
    Serial.print(F(" pitch=")); Serial.print(s.pitch);
    Serial.print(F(" roll=")); Serial.print(s.roll);
    Serial.print(F(" time=")); Serial.print(s.timeForAction);
    Serial.println(F(" x0.1s"));

    // Build once here so the exact packet can be inspected before transmit.
    PositionControlPayload p;
    p.yaw_angle = s.yaw;
    p.roll_angle = s.roll;
    p.pitch_angle = s.pitch;
    p.ctrl_byte = makePositionCtrlByte(
        ControlMode::Absolute,
        AxisValid::Valid,
        AxisValid::Valid,
        AxisValid::Valid);
    p.time_for_action = s.timeForAction;

    PacketBuffer packet;
    auto build = ronin.builder().buildCommand(
        GimbalCmd::CMDSET,
        GimbalCmd::PositionControl,
        p,
        ReplyRequirement::NoReply,
        packet);

    if (build.isError()) {
        Serial.print(F("[BUILD ERROR] "));
        Serial.println(errorName(build.error()));
        return;
    }

    Serial.print(F("[POSITION TX] packet length="));
    Serial.println(packet.length);
    Serial.print(F("[POSITION TX] packet="));
    printHex(packet.data, packet.length);

    auto tx = ronin.motion.moveTo(
        s.yaw, s.pitch, s.roll, s.timeForAction);

    if (tx.isError()) {
        Serial.print(F("[POSITION TX ERROR] "));
        Serial.println(errorName(tx.error()));
        return;
    }

    Serial.println(F("[POSITION TX OK] command handed to CAN transport"));
    Serial.println(F("[NOTE] Position command is NoReply; no CAN response is expected."));
}

void setup()
{
    Serial.begin(115200);
    delay(500);

    Serial.println();
    Serial.println(F("============================================================"));
    Serial.println(F(" DJI R SDK - POSITION CONTROL FIELD TEST"));
    Serial.println(F("============================================================"));
    Serial.println(F("MCP2515: 8 MHz / CAN 1 Mbps / standard CAN"));
    Serial.println(F("Small deterministic movements; joystick is NOT used."));
    Serial.println(F("WARNING: balance and unlock the Ronin before running."));
    Serial.println();

    auto begin = ronin.begin();
    if (begin.isError()) {
        Serial.print(F("[FATAL] Ronin/CAN begin failed: "));
        Serial.println(errorName(begin.error()));
        while (true) delay(1000);
    }

    Serial.println(F("[OK] CAN transport initialized"));
    Serial.println(F("[STEP 0] Requesting Ronin information..."));
    requestInfo();

    if (!roninSeen)
        Serial.println(F("[WARNING] Ronin did not answer; motion test will still run."));

    stepIndex = 0;
    stepAt = millis() - STEP_SETTLE_MS;
}

void loop()
{
    unsigned long now = millis();

    // Keep checking for a Ronin response if the first information request
    // was sent before the Ronin was ready.
    if (!roninSeen && now - lastInfo >= INFO_RETRY_MS) {
        lastInfo = now;
        requestInfo();
    }

    if (now - stepAt < STEP_SETTLE_MS)
        return;

    stepAt = now;

    runStep(STEPS[stepIndex]);
    stepIndex++;

    if (stepIndex >= STEP_COUNT) {
        stepIndex = 0;
        Serial.println();
        Serial.println(F("[SEQUENCE] Completed. Repeating after settle interval."));
    }
}
