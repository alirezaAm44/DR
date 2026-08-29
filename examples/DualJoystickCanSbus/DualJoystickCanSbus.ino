/**
 * DualJoystickCanSbus.ino
 *
 * DJI Ronin R SDK + MCP2515 CAN + SBUS
 *
 * Uses the standard mcp2515 Arduino library directly.
 * MCP2515Driver.h is intentionally NOT used.
 */

#include <SPI.h>
#include <SoftwareSerial.h>
#include <mcp2515.h>
#include <DJIRonin.h>

using namespace dji::ronin;

const bool ENABLE_CAN  = true;
const bool ENABLE_SBUS = true;
const bool USE_SBUS_MOTION = true;

const int JOYSTICK_X_PIN  = A1;
const int JOYSTICK_Y_PIN  = A2;
const int JOYSTICK_SW_PIN = 2;
const int MCP2515_CS_PIN  = 10;
const int SBUS_TX_PIN     = 8;

const CAN_CLOCK MCP_CLOCK = MCP_8MHZ;

const int JOYSTICK_CENTER = 512;
const int DEAD_ZONE       = 40;
const int16_t DJI_MIN     = -7500;
const int16_t DJI_MAX     = 7500;

const uint16_t SBUS_MIN = 352;
const uint16_t SBUS_MID = 1024;
const uint16_t SBUS_MAX = 1696;
const unsigned long SBUS_INTERVAL_MS = 14;

const uint8_t SBUS_CH_YAW   = 0;
const uint8_t SBUS_CH_PITCH = 1;
const uint8_t SBUS_CH_ROLL  = 3;

SoftwareSerial sbusSerial(9, SBUS_TX_PIN);

const unsigned long JOYSTICK_INTERVAL_MS = 20;
const unsigned long INFO_INTERVAL_MS     = 200;
const unsigned long STATUS_PRINT_MS      = 200;
const unsigned long DEBOUNCE_MS          = 40;

MCP2515 mcp2515(MCP2515_CS_PIN);
PacketBuilder builder;
PacketParser parser;

unsigned long lastJoystickMs = 0;
unsigned long lastInfoMs     = 0;
unsigned long lastStatusMs   = 0;
unsigned long lastSbusMs     = 0;

uint32_t txJoystickCount  = 0;
uint32_t txInfoCount      = 0;
uint32_t txCameraCount    = 0;
uint32_t txSbusCount      = 0;
uint32_t rxPacketCount    = 0;
uint32_t rxErrorCount     = 0;
uint32_t canSendFailCount = 0;

int16_t lastYaw = 0;
int16_t lastPitch = 0;
int16_t lastRoll = 0;

bool haveAttitude = false;
int16_t attYaw = 0;
int16_t attRoll = 0;
int16_t attPitch = 0;

bool isRecording = false;
bool lastButtonStable = HIGH;
bool lastButtonRead = HIGH;
unsigned long lastDebounceMs = 0;

uint8_t rxAcc[MAX_PACKET_SIZE];
uint16_t rxAccLen = 0;
uint16_t rxExpectedLen = 0;

uint16_t sbusChannels[16];

int16_t mapJoystickToDJI(int rawValue, bool invert = false)
{
    int delta = rawValue - JOYSTICK_CENTER;
    if (abs(delta) < DEAD_ZONE) return 0;
    if (invert) delta = -delta;

    long mapped = (long)delta * DJI_MAX / (JOYSTICK_CENTER - DEAD_ZONE);

    if (mapped > DJI_MAX) mapped = DJI_MAX;
    if (mapped < DJI_MIN) mapped = DJI_MIN;

    return (int16_t)mapped;
}

uint16_t mapDjiToSbus(int16_t v)
{
    long span = (long)SBUS_MAX - (long)SBUS_MIN;
    long mapped = (long)SBUS_MID + ((long)v * (span / 2)) / DJI_MAX;

    if (mapped < SBUS_MIN) mapped = SBUS_MIN;
    if (mapped > SBUS_MAX) mapped = SBUS_MAX;

    return (uint16_t)mapped;
}

void printHex(const uint8_t* data, size_t len)
{
    for (size_t i = 0; i < len; i++) {
        if (data[i] < 0x10) Serial.print('0');
        Serial.print(data[i], HEX);
        Serial.print(' ');
    }
    Serial.println();
}

void sbusResetChannels()
{
    for (uint8_t i = 0; i < 16; i++) {
        sbusChannels[i] = SBUS_MID;
    }
}

void sbusPackAndSend()
{
    uint8_t packet[25];
    packet[0] = 0x0F;

    uint8_t byteIdx = 1;
    uint32_t bitBuf = 0;
    uint8_t bitCnt = 0;

    for (uint8_t ch = 0; ch < 16; ch++) {
        uint16_t v = sbusChannels[ch] & 0x07FF;
        bitBuf |= ((uint32_t)v) << bitCnt;
        bitCnt += 11;

        while (bitCnt >= 8) {
            packet[byteIdx++] = (uint8_t)(bitBuf & 0xFF);
            bitBuf >>= 8;
            bitCnt -= 8;
        }
    }

    if (bitCnt > 0 && byteIdx < 23) {
        packet[byteIdx++] = (uint8_t)(bitBuf & 0xFF);
    }

    while (byteIdx < 23) packet[byteIdx++] = 0;

    packet[23] = 0x00;
    packet[24] = 0x00;

    for (uint8_t i = 0; i < 25; i++) {
        sbusSerial.write(packet[i]);
    }

    txSbusCount++;
}

void updateSbusFromJoystick(int16_t yaw, int16_t pitch, int16_t roll)
{
    sbusResetChannels();

    if (!USE_SBUS_MOTION) return;

    sbusChannels[SBUS_CH_YAW]   = mapDjiToSbus(yaw);
    sbusChannels[SBUS_CH_PITCH] = mapDjiToSbus(pitch);
    sbusChannels[SBUS_CH_ROLL]  = mapDjiToSbus(roll);
}

bool sendPacketMultiFrame(const PacketBuffer& packet)
{
    size_t offset = 0;

    while (offset < packet.length) {
        struct can_frame frame;
        frame.can_id = CAN_ID_TX;
        frame.can_dlc = 0;

        while (frame.can_dlc < 8 && offset < packet.length) {
            frame.data[frame.can_dlc++] = packet.data[offset++];
        }

        MCP2515::ERROR e = mcp2515.sendMessage(&frame);

        if (e != MCP2515::ERROR_OK) {
            canSendFailCount++;
            return false;
        }
    }

    return true;
}

void handleCompleteRxPacket(const uint8_t* data, uint16_t len)
{
    rxPacketCount++;

    auto result = parser.parse(data, len);

    if (result.isError()) {
        rxErrorCount++;
        Serial.println(F("----- RX PARSE ERROR -----"));
        printHex(data, len);
        return;
    }

    ParsedPacket pkt = result.value();

    Serial.println(F("----- RX FROM RONIN -----"));
    Serial.print(F("CmdSet=0x"));
    Serial.print(pkt.command.cmdSet, HEX);
    Serial.print(F(" CmdID=0x"));
    Serial.println(pkt.command.cmdId, HEX);

    Serial.print(F("RAW: "));
    printHex(data, len);

    if (pkt.hasReturnCode) {
        Serial.print(F("ReturnCode=0x"));
        Serial.println(static_cast<uint8_t>(pkt.returnCode), HEX);
        if (pkt.returnCode != ReturnCode::Success) rxErrorCount++;
    }

    if (pkt.command.cmdSet == GimbalCmd::CMDSET &&
        pkt.command.cmdId == GimbalCmd::ObtainInformation &&
        pkt.payloadLen >= sizeof(ObtainInfoReplyPayload)) {

        const ObtainInfoReplyPayload* rep =
            reinterpret_cast<const ObtainInfoReplyPayload*>(pkt.payload);

        haveAttitude = true;
        attYaw = rep->yaw;
        attRoll = rep->roll;
        attPitch = rep->pitch;

        Serial.print(F("Attitude Y="));
        Serial.print(attYaw / 10.0f, 1);
        Serial.print(F(" P="));
        Serial.println(attPitch / 10.0f, 1);
    }

    Serial.println(F("--------------------------"));
}

void pollIncoming()
{
    if (!ENABLE_CAN) return;

    struct can_frame frame;

    while (mcp2515.readMessage(&frame) == MCP2515::ERROR_OK) {
        if ((frame.can_id & 0x7FF) != CAN_ID_RX) continue;

        for (uint8_t i = 0; i < frame.can_dlc; i++) {
            uint8_t b = frame.data[i];

            if (rxAccLen == 0) {
                if (b != SOF) continue;
                rxAcc[0] = b;
                rxAccLen = 1;
                rxExpectedLen = 0;
                continue;
            }

            if (rxAccLen < MAX_PACKET_SIZE) {
                rxAcc[rxAccLen++] = b;
            } else {
                rxAccLen = 0;
                rxExpectedLen = 0;
                continue;
            }

            if (rxAccLen == 3) {
                uint16_t verLen =
                    (uint16_t)rxAcc[1] |
                    ((uint16_t)rxAcc[2] << 8);

                rxExpectedLen = verLen & LENGTH_MASK;

                if (rxExpectedLen < MIN_PACKET_SIZE ||
                    rxExpectedLen > MAX_PACKET_SIZE) {
                    rxAccLen = 0;
                    rxExpectedLen = 0;
                }
            }

            if (rxExpectedLen > 0 &&
                rxAccLen >= rxExpectedLen) {
                handleCompleteRxPacket(rxAcc, rxExpectedLen);
                rxAccLen = 0;
                rxExpectedLen = 0;
            }
        }
    }
}

bool sendJoystick(int16_t yaw, int16_t pitch, int16_t roll)
{
    JoystickPayload payload;
    payload.device_type = static_cast<uint8_t>(ControllerType::Joystick);
    payload.pitch_speed = pitch;
    payload.roll_speed = roll;
    payload.yaw_speed = yaw;

    PacketBuffer packet;

    auto result = builder.buildCommand(
        GimbalCmd::CMDSET,
        GimbalCmd::ExternalDeviceControl,
        payload,
        ReplyRequirement::NoReply,
        packet
    );

    if (result.isError()) return false;

    return sendPacketMultiFrame(packet);
}

bool sendObtainInfo(uint8_t infoType)
{
    ObtainInfoRequestPayload payload;
    payload.ctrl_byte = infoType;

    PacketBuffer packet;

    auto result = builder.buildCommand(
        GimbalCmd::CMDSET,
        GimbalCmd::ObtainInformation,
        payload,
        ReplyRequirement::ReplyRequired,
        packet
    );

    if (result.isError()) return false;

    return sendPacketMultiFrame(packet);
}

bool sendCameraRecord(bool start)
{
    CameraMotionPayload payload;
    payload.command = static_cast<uint16_t>(
        start ? CameraMotionCommand::StartRecording
              : CameraMotionCommand::StopRecording
    );

    PacketBuffer packet;

    auto result = builder.buildCommand(
        CameraCmd::CMDSET,
        CameraCmd::Motion,
        payload,
        ReplyRequirement::NoReply,
        packet
    );

    if (result.isError()) return false;

    bool ok = sendPacketMultiFrame(packet);

    if (ok) {
        txCameraCount++;
        Serial.print(F(">>> CAMERA "));
        Serial.println(start ? F("START") : F("STOP"));
    }

    return ok;
}

void handleRecordButton()
{
    bool reading = digitalRead(JOYSTICK_SW_PIN);

    if (reading != lastButtonRead) {
        lastDebounceMs = millis();
        lastButtonRead = reading;
    }

    if (millis() - lastDebounceMs < DEBOUNCE_MS) return;

    if (lastButtonStable == HIGH && reading == LOW) {
        isRecording = !isRecording;
        if (ENABLE_CAN) sendCameraRecord(isRecording);
    }

    lastButtonStable = reading;
}

void setup()
{
    Serial.begin(115200);
    delay(500);

    pinMode(JOYSTICK_SW_PIN, INPUT_PULLUP);

    Serial.println();
    Serial.println(F("========================================"));
    Serial.println(F(" DJI CAN + SBUS simultaneous test"));
    Serial.println(F("========================================"));

    if (ENABLE_SBUS) {
        sbusSerial.begin(100000);
        sbusResetChannels();
        Serial.print(F("[OK] SBUS TX on D"));
        Serial.println(SBUS_TX_PIN);
        Serial.println(F("     -> 74HC04 -> RSA pin 3"));
    }

    if (ENABLE_CAN) {
        SPI.begin();

        mcp2515.reset();
        delay(50);

        if (mcp2515.setBitrate(CAN_1000KBPS, MCP_CLOCK) !=
            MCP2515::ERROR_OK) {
            Serial.println(F("[ERR] setBitrate"));
            while (1) {}
        }

        if (mcp2515.setNormalMode() != MCP2515::ERROR_OK) {
            Serial.println(F("[ERR] setNormalMode"));
            while (1) {}
        }

        mcp2515.setFilterMask(MCP2515::MASK0, false, 0);
        mcp2515.setFilterMask(MCP2515::MASK1, false, 0);
        mcp2515.setFilter(MCP2515::RXF0, false, 0);

        builder.resetSequence(0);

        Serial.println(F("[OK] CAN Normal 1Mbps"));
    }

    Serial.println(F("AD_COM: 10k/47k to COMMON GND (Arduino GND = RSA pin6)"));
    Serial.println(F("========================================"));
}

void loop()
{
    if (ENABLE_CAN) {
        pollIncoming();
        handleRecordButton();
    }

    unsigned long now = millis();

    int rawX = analogRead(JOYSTICK_X_PIN);
    int rawY = analogRead(JOYSTICK_Y_PIN);

    lastYaw = mapJoystickToDJI(rawX, false);
    lastPitch = mapJoystickToDJI(rawY, true);
    lastRoll = 0;

    if (ENABLE_CAN && now - lastJoystickMs >= JOYSTICK_INTERVAL_MS) {
        lastJoystickMs = now;
        if (sendJoystick(lastYaw, lastPitch, lastRoll)) {
            txJoystickCount++;
        }
    }

    if (ENABLE_CAN && now - lastInfoMs >= INFO_INTERVAL_MS) {
        lastInfoMs = now;
        if (sendObtainInfo(0x01)) txInfoCount++;
    }

    if (ENABLE_SBUS && now - lastSbusMs >= SBUS_INTERVAL_MS) {
        lastSbusMs = now;
        updateSbusFromJoystick(lastYaw, lastPitch, lastRoll);
        sbusPackAndSend();
    }

    if (now - lastStatusMs >= STATUS_PRINT_MS) {
        lastStatusMs = now;

        Serial.print(F("JS Y="));
        Serial.print(lastYaw);
        Serial.print(F(" P="));
        Serial.print(lastPitch);
        Serial.print(F(" | CAN_tx="));
        Serial.print(txJoystickCount);
        Serial.print(F(" fail="));
        Serial.print(canSendFailCount);
        Serial.print(F(" RX="));
        Serial.print(rxPacketCount);
        Serial.print(F(" | SBUS_tx="));
        Serial.print(txSbusCount);
        Serial.print(F(" | REC="));
        Serial.print(isRecording ? F("ON") : F("OFF"));

        if (haveAttitude) {
            Serial.print(F(" | AttY="));
            Serial.print(attYaw / 10.0f, 1);
        }

        Serial.println();
    }
}
