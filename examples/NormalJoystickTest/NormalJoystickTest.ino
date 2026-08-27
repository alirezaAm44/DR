/**
 * NormalJoystickTest.ino
 *
 * اتصال واقعی به رونین (MCP2515 Normal Mode)
 *
 * - جوی‌استیک هر ~20 ms (CmdSet 0x0E / CmdID 0x0A)
 * - دکمه ماژول جوی‌استیک: Toggle Record Start/Stop دوربین (CmdSet 0x0D)
 * - هر ~200 ms زاویه موتورها با Obtain Information
 * - پکت‌های دریافتی از رونین روی Serial
 *
 * سیم‌کشی دکمه:
 *   پایه SW ماژول جوی‌استیک → پین دیجیتال (پیش‌فرض D2)
 *   معمولاً با INPUT_PULLUP: فشردن = LOW
 */

#include <SPI.h>
#include <mcp2515.h>
#include <DJIRonin.h>

using namespace dji::ronin;

// ================================================================
// سخت‌افزار
// ================================================================
const int JOYSTICK_X_PIN = A1;   // Yaw
const int JOYSTICK_Y_PIN = A2;   // Pitch
const int JOYSTICK_SW_PIN = 2;   // دکمه SW ماژول جوی‌استیک (عوض کن اگر پین دیگری است)
const int MCP2515_CS_PIN  = 10;

const CAN_CLOCK MCP_CLOCK = MCP_8MHZ;  // یا MCP_16MHZ

// ================================================================
// جوی‌استیک
// ================================================================
const int     JOYSTICK_CENTER = 512;
const int     DEAD_ZONE       = 40;
const int16_t DJI_MIN         = -15000;
const int16_t DJI_MAX         = 15000;

// ================================================================
// زمان‌بندی
// ================================================================
const unsigned long JOYSTICK_INTERVAL_MS = 20;
const unsigned long INFO_INTERVAL_MS     = 200;
const unsigned long STATUS_PRINT_MS      = 200;
const unsigned long DEBOUNCE_MS          = 40;

// ================================================================
// اشیاء
// ================================================================
MCP2515       mcp2515(MCP2515_CS_PIN);
PacketBuilder builder;
PacketParser  parser;

unsigned long lastJoystickMs = 0;
unsigned long lastInfoMs     = 0;
unsigned long lastStatusMs   = 0;

uint32_t txJoystickCount = 0;
uint32_t txInfoCount     = 0;
uint32_t txCameraCount   = 0;
uint32_t rxPacketCount   = 0;
uint32_t rxErrorCount    = 0;

int16_t lastYaw = 0, lastPitch = 0, lastRoll = 0;

bool    haveAttitude = false;
int16_t attYaw = 0, attRoll = 0, attPitch = 0;
uint8_t attDataType = 0;

// وضعیت رکورد (نرم‌افزاری – toggle)
bool isRecording = false;

// دیبانس دکمه
bool          lastButtonStable = HIGH;  // INPUT_PULLUP → آزاد = HIGH
bool          lastButtonRead   = HIGH;
unsigned long lastDebounceMs   = 0;

uint8_t  rxAcc[MAX_PACKET_SIZE];
uint16_t rxAccLen = 0;
uint16_t rxExpectedLen = 0;

// ================================================================
// کمکی
// ================================================================
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

void printHex(const uint8_t* data, size_t len)
{
    for (size_t i = 0; i < len; i++) {
        if (data[i] < 0x10) Serial.print('0');
        Serial.print(data[i], HEX);
        Serial.print(' ');
    }
    Serial.println();
}

bool sendPacketMultiFrame(const PacketBuffer& packet)
{
    size_t offset = 0;
    while (offset < packet.length) {
        struct can_frame frame;
        frame.can_id  = CAN_ID_TX;
        frame.can_dlc = 0;
        while (frame.can_dlc < 8 && offset < packet.length) {
            frame.data[frame.can_dlc++] = packet.data[offset++];
        }
        if (mcp2515.sendMessage(&frame) != MCP2515::ERROR_OK) {
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
        Serial.print(F("Raw ("));
        Serial.print(len);
        Serial.print(F(" bytes): "));
        printHex(data, len);
        return;
    }

    ParsedPacket pkt = result.value();

    Serial.println(F("----- RX FROM RONIN -----"));
    Serial.print(F("len="));
    Serial.print(len);
    Serial.print(F(" CmdSet=0x"));
    Serial.print(pkt.command.cmdSet, HEX);
    Serial.print(F(" CmdID=0x"));
    Serial.print(pkt.command.cmdId, HEX);
    Serial.print(F(" SEQ="));
    Serial.println(pkt.header.seq);

    Serial.print(F("RAW: "));
    printHex(data, len);

    if (pkt.hasReturnCode) {
        Serial.print(F("ReturnCode=0x"));
        Serial.print(static_cast<uint8_t>(pkt.returnCode), HEX);
        if (pkt.returnCode == ReturnCode::Success) {
            Serial.println(F(" (Success)"));
        } else if (pkt.returnCode == ReturnCode::ParseError) {
            Serial.println(F(" (ParseError)"));
            rxErrorCount++;
        } else if (pkt.returnCode == ReturnCode::ExecutionFailed) {
            Serial.println(F(" (ExecutionFailed)"));
            rxErrorCount++;
        } else {
            Serial.println();
        }
    }

    // پاسخ Obtain Information
    if (pkt.command.cmdSet == GimbalCmd::CMDSET &&
        pkt.command.cmdId  == GimbalCmd::ObtainInformation &&
        pkt.payloadLen >= sizeof(ObtainInfoReplyPayload)) {

        const ObtainInfoReplyPayload* rep =
            reinterpret_cast<const ObtainInfoReplyPayload*>(pkt.payload);

        haveAttitude = true;
        attDataType  = rep->data_type;
        attYaw       = rep->yaw;
        attRoll      = rep->roll;
        attPitch     = rep->pitch;

        Serial.print(F("Motor/Attitude: Y="));
        Serial.print(attYaw / 10.0f, 1);
        Serial.print(F(" R="));
        Serial.print(attRoll / 10.0f, 1);
        Serial.print(F(" P="));
        Serial.print(attPitch / 10.0f, 1);
        Serial.println(F(" deg"));
    }

    // پاسخ وضعیت دوربین (اگر Query کرده باشیم)
    if (pkt.command.cmdSet == CameraCmd::CMDSET &&
        pkt.command.cmdId  == CameraCmd::ObtainStatus &&
        pkt.payloadLen >= sizeof(CameraStatusReplyPayload)) {

        const CameraStatusReplyPayload* rep =
            reinterpret_cast<const CameraStatusReplyPayload*>(pkt.payload);
        Serial.print(F("Camera status: "));
        if (rep->status == static_cast<uint8_t>(CameraRecordingStatus::Recording)) {
            Serial.println(F("RECORDING"));
            isRecording = true;
        } else {
            Serial.println(F("NOT recording"));
            isRecording = false;
        }
    }

    Serial.println(F("--------------------------"));
}

void pollIncoming()
{
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
                uint16_t verLen = (uint16_t)rxAcc[1] | ((uint16_t)rxAcc[2] << 8);
                rxExpectedLen = verLen & LENGTH_MASK;
                if (rxExpectedLen < MIN_PACKET_SIZE || rxExpectedLen > MAX_PACKET_SIZE) {
                    rxAccLen = 0;
                    rxExpectedLen = 0;
                }
            }

            if (rxExpectedLen > 0 && rxAccLen >= rxExpectedLen) {
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
    payload.roll_speed  = roll;
    payload.yaw_speed   = yaw;

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

/** دستور دوربین: Start/Stop Recording (CmdSet 0x0D, CmdID 0x00) */
bool sendCameraRecord(bool start)
{
    CameraMotionPayload payload;
    payload.command = static_cast<uint16_t>(
        start ? CameraMotionCommand::StartRecording
              : CameraMotionCommand::StopRecording
    );

    PacketBuffer packet;
    auto result = builder.buildCommand(
        CameraCmd::CMDSET,   // 0x0D
        CameraCmd::Motion,   // 0x00
        payload,
        ReplyRequirement::NoReply,
        packet
    );
    if (result.isError()) return false;

    bool ok = sendPacketMultiFrame(packet);
    if (ok) {
        txCameraCount++;
        Serial.print(F(">>> CAMERA "));
        Serial.print(start ? F("RECORD START") : F("RECORD STOP"));
        Serial.print(F(" | packet: "));
        printHex(packet.data, packet.length);
    } else {
        Serial.println(F("[ERR] Camera command TX failed"));
    }
    return ok;
}

/** خواندن دکمه با دیبانس – یک‌بار فشار = یک toggle */
void handleRecordButton()
{
    bool reading = digitalRead(JOYSTICK_SW_PIN);

    if (reading != lastButtonRead) {
        lastDebounceMs = millis();
        lastButtonRead = reading;
    }

    if ((millis() - lastDebounceMs) < DEBOUNCE_MS) {
        return;
    }

    // لبهٔ پایین‌رونده: آزاد → فشرده (با PULLUP)
    if (lastButtonStable == HIGH && reading == LOW) {
        isRecording = !isRecording;
        sendCameraRecord(isRecording);
    }

    lastButtonStable = reading;
}

// ================================================================
// Setup
// ================================================================
void setup()
{
    Serial.begin(115200);
    delay(500);

    pinMode(JOYSTICK_SW_PIN, INPUT_PULLUP);

    Serial.println();
    Serial.println(F("========================================"));
    Serial.println(F(" DJI R SDK – Normal Mode + Camera Toggle"));
    Serial.println(F("========================================"));

    SPI.begin();
    mcp2515.reset();
    delay(50);

    if (mcp2515.setBitrate(CAN_1000KBPS, MCP_CLOCK) != MCP2515::ERROR_OK) {
        Serial.println(F("[ERR] setBitrate failed"));
        while (1) {}
    }
    Serial.println(F("[OK] Bitrate 1 Mbps"));

    if (mcp2515.setNormalMode() != MCP2515::ERROR_OK) {
        Serial.println(F("[ERR] setNormalMode failed"));
        while (1) {}
    }
    Serial.println(F("[OK] Normal mode"));

    mcp2515.setFilterMask(MCP2515::MASK0, false, 0);
    mcp2515.setFilterMask(MCP2515::MASK1, false, 0);
    mcp2515.setFilter(MCP2515::FILTER0, false, 0);

    builder.resetSequence(0);

    Serial.print(F("[OK] Button on D"));
    Serial.print(JOYSTICK_SW_PIN);
    Serial.println(F(" (press = toggle record)"));
    Serial.println(F("========================================"));
}

// ================================================================
// Loop
// ================================================================
void loop()
{
    pollIncoming();
    handleRecordButton();

    unsigned long now = millis();

    int rawX = analogRead(JOYSTICK_X_PIN);
    int rawY = analogRead(JOYSTICK_Y_PIN);
    lastYaw   = mapJoystickToDJI(rawX, false);
    lastPitch = mapJoystickToDJI(rawY, true);
    lastRoll  = 0;

    if (now - lastJoystickMs >= JOYSTICK_INTERVAL_MS) {
        lastJoystickMs = now;
        if (sendJoystick(lastYaw, lastPitch, lastRoll)) {
            txJoystickCount++;
        }
    }

    if (now - lastInfoMs >= INFO_INTERVAL_MS) {
        lastInfoMs = now;
        if (sendObtainInfo(0x01)) {
            txInfoCount++;
        }
    }

    if (now - lastStatusMs >= STATUS_PRINT_MS) {
        lastStatusMs = now;

        Serial.print(F("JS Y="));
        Serial.print(lastYaw);
        Serial.print(F(" P="));
        Serial.print(lastPitch);
        Serial.print(F(" | REC="));
        Serial.print(isRecording ? F("ON") : F("OFF"));
        Serial.print(F(" | TX_js="));
        Serial.print(txJoystickCount);
        Serial.print(F(" TX_cam="));
        Serial.print(txCameraCount);
        Serial.print(F(" RX="));
        Serial.print(rxPacketCount);

        if (haveAttitude) {
            Serial.print(F(" | Ronin Y="));
            Serial.print(attYaw / 10.0f, 1);
            Serial.print(F(" P="));
            Serial.print(attPitch / 10.0f, 1);
        }
        Serial.println();
    }
}
