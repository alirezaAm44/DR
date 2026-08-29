/**
 * DualJoystickCanSbus.ino
 *
 * Final Ronin field-test example:
 *   A1 -> Yaw joystick
 *   A2 -> Pitch joystick
 *   CAN  -> DJI R SDK External Device Control (0x0E/0x0A)
 *   SBUS -> CH1=Yaw, CH2=Pitch, CH4=Record
 *
 * CAN:
 *   MCP2515, 8 MHz crystal, 1 Mbps, standard 11-bit frames
 *   TX ID = 0x223, RX ID = 0x222
 *   Multi-frame packets are sent through DJIRonin::sendPacket(),
 *   including the 250 us inter-frame guard implemented in the library.
 *
 * Startup:
 *   1) Initialize CAN.
 *   2) Send Obtain Information request.
 *   3) Try to receive and parse a Ronin response.
 *   4) If no response is available, keep running and retry periodically.
 *   5) Continue joystick/SBUS output even when CAN has no response.
 *
 * SBUS:
 *   Uses Bolder Flight Systems SBUS library (SbusTx), not a hand-written
 *   bit-bang implementation. SBUS is 100000 baud, 8E2, 25 bytes/frame.
 *
 * IMPORTANT BOARD NOTE:
 *   This example requires a HardwareSerial port that supports the SBUS
 *   transmitter library. ESP32 is supported directly and uses D8 by default.
 *   Mega2560 uses Serial1. An ATmega328P/UNO has no second hardware UART;
 *   do not replace this with SoftwareSerial because standard SBUS requires
 *   100000 baud, 8 data bits, even parity, 2 stop bits.
 *
 * Install dependency:
 *   Bolder Flight Systems SBUS
 *   https://github.com/bolderflight/sbus
 */

#include <SPI.h>
#include <mcp2515.h>
#include <DJIRonin.h>
#include <sbus.h>

using namespace dji::ronin;

// -----------------------------------------------------------------------------
// User configuration
// -----------------------------------------------------------------------------
const bool ENABLE_CAN  = true;
const bool ENABLE_SBUS = true;

const uint8_t JOYSTICK_YAW_PIN   = A1;
const uint8_t JOYSTICK_PITCH_PIN = A2;
const uint8_t RECORD_BUTTON_PIN  = 2;
const uint8_t MCP2515_CS_PIN     = 10;

// Ronin software can select normal/inverted SBUS. Set this to the state
// expected by the Ronin input configuration.
const bool SBUS_INVERTED = false;

#if defined(ESP32)
HardwareSerial SbusSerial(1);
const int8_t SBUS_RX_UNUSED = -1;
const int8_t SBUS_TX_PIN = 8;
bfs::SbusTx sbus(&SbusSerial, SBUS_RX_UNUSED, SBUS_TX_PIN, SBUS_INVERTED);
#elif defined(ARDUINO_AVR_MEGA2560)
bfs::SbusTx sbus(&Serial1, SBUS_INVERTED);
#else
#error "DualJoystickCanSbus requires ESP32 or Arduino Mega2560 for a real SBUS HardwareSerial transmitter. Do not use SoftwareSerial for SBUS."
#endif

const CAN_CLOCK MCP_CLOCK = MCP_8MHZ;

const int JOYSTICK_CENTER = 512;
const int DEAD_ZONE = 40;
const int16_t DJI_MIN = -7500;
const int16_t DJI_MAX = 7500;

const uint16_t SBUS_MIN = 352;
const uint16_t SBUS_MID = 1024;
const uint16_t SBUS_MAX = 1696;

const uint8_t CH_YAW    = 0;  // CH1
const uint8_t CH_PITCH  = 1;  // CH2
const uint8_t CH_RECORD = 3;  // CH4

const uint32_t JOYSTICK_PERIOD_MS = 20;
const uint32_t SBUS_PERIOD_MS = 14;
const uint32_t INFO_RETRY_MS = 500;
const uint32_t STATUS_PERIOD_MS = 250;
const uint32_t DEBOUNCE_MS = 40;

MCP2515Driver can(MCP2515_CS_PIN);
DJIRonin ronin(can);
PacketBuilder rawBuilder;

uint16_t sequenceStart = 0;
uint32_t canJoystickOk = 0;
uint32_t canJoystickFail = 0;
uint32_t canInfoTx = 0;
uint32_t canInfoResponses = 0;
uint32_t canInfoNoResponse = 0;
uint32_t canRxErrors = 0;
uint32_t sbusFrames = 0;
uint32_t sbusErrors = 0;

unsigned long lastJoystickMs = 0;
unsigned long lastSbusMs = 0;
unsigned long lastInfoMs = 0;
unsigned long lastStatusMs = 0;

int16_t yaw = 0;
int16_t pitch = 0;
int16_t roll = 0;
bool recording = false;
bool roninSeen = false;

bool buttonStable = HIGH;
bool buttonLast = HIGH;
unsigned long buttonChangedAt = 0;

// -----------------------------------------------------------------------------
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

int16_t mapJoystick(int raw, bool invert)
{
    int delta = raw - JOYSTICK_CENTER;
    if (abs(delta) <= DEAD_ZONE) return 0;
    if (invert) delta = -delta;

    if (delta > 0) {
        long v = (long)(delta - DEAD_ZONE) * DJI_MAX /
                 (1023 - JOYSTICK_CENTER - DEAD_ZONE);
        if (v > DJI_MAX) v = DJI_MAX;
        return (int16_t)v;
    }

    long v = (long)(delta + DEAD_ZONE) * DJI_MAX /
             (JOYSTICK_CENTER - DEAD_ZONE);
    if (v < DJI_MIN) v = DJI_MIN;
    return (int16_t)v;
}

uint16_t djiToSbus(int16_t value)
{
    long half = ((long)SBUS_MAX - SBUS_MIN) / 2;
    long out = SBUS_MID + ((long)value * half) / DJI_MAX;
    if (out < SBUS_MIN) out = SBUS_MIN;
    if (out > SBUS_MAX) out = SBUS_MAX;
    return (uint16_t)out;
}

// -----------------------------------------------------------------------------
// Packet builders with complete TX logging
// -----------------------------------------------------------------------------
bool buildJoystickPacket(PacketBuffer& packet)
{
    JoystickPayload p;
    p.device_type = static_cast<uint8_t>(ControllerType::Joystick);
    p.pitch_speed = pitch;
    p.roll_speed  = roll;
    p.yaw_speed   = yaw;

    auto r = rawBuilder.buildCommand(
        GimbalCmd::CMDSET,
        GimbalCmd::ExternalDeviceControl,
        p,
        ReplyRequirement::NoReply,
        packet);

    if (r.isError()) {
        Serial.print(F("[CAN BUILDER ERROR] "));
        Serial.println(errorName(r.error()));
        return false;
    }

    return true;
}

bool buildInfoPacket(PacketBuffer& packet)
{
    ObtainInfoRequestPayload p;
    p.ctrl_byte = 0x01;

    auto r = rawBuilder.buildCommand(
        GimbalCmd::CMDSET,
        GimbalCmd::ObtainInformation,
        p,
        ReplyRequirement::ReplyRequired,
        packet);

    if (r.isError()) {
        Serial.print(F("[INFO BUILDER ERROR] "));
        Serial.println(errorName(r.error()));
        return false;
    }

    return true;
}

void logPacket(const char* label, const PacketBuffer& packet)
{
    Serial.print(F("["));
    Serial.print(label);
    Serial.print(F("] length="));
    Serial.println(packet.length);
    printHex(packet.data, packet.length);
}

// -----------------------------------------------------------------------------
// Ronin initialization / version information
// -----------------------------------------------------------------------------
void requestRoninInformation()
{
    PacketBuffer packet;
    if (!buildInfoPacket(packet)) return;

    logPacket("CAN TX INFO", packet);

    ++canInfoTx;
    auto tx = ronin.sendPacket(packet);
    if (tx.isError()) {
        Serial.print(F("[CAN INFO TX ERROR] "));
        Serial.println(errorName(tx.error()));
        ++canJoystickFail;
        return;
    }

    Serial.println(F("[CAN INFO] Request sent; waiting briefly for Ronin response..."));

    // receivePacket() is non-blocking with the current MCP2515 transport:
    // if no frame is available it returns the transport error and the main
    // loop simply retries later. The program never stops because CAN is silent.
    auto rx = ronin.receivePacket(100);
    if (rx.isError()) {
        ++canInfoNoResponse;
        Serial.print(F("[CAN INFO] No valid Ronin response yet: "));
        Serial.println(errorName(rx.error()));
        return;
    }

    ++canInfoResponses;
    roninSeen = true;

    const ParsedPacket& p = rx.value();
    Serial.println(F("[CAN INFO] *** RONIN RESPONSE RECEIVED ***"));
    Serial.print(F("  CmdSet = 0x"));
    Serial.println(p.command.cmdSet, HEX);
    Serial.print(F("  CmdID  = 0x"));
    Serial.println(p.command.cmdId, HEX);
    Serial.print(F("  Length = "));
    Serial.println(p.header.length);
    Serial.print(F("  Seq    = "));
    Serial.println(p.header.seq);
    Serial.print(F("  Payload bytes = "));
    Serial.println(p.payloadLen);
    if (p.hasReturnCode) {
        Serial.print(F("  ReturnCode = 0x"));
        Serial.println(static_cast<uint8_t>(p.returnCode), HEX);
    }
}

// -----------------------------------------------------------------------------
// Joystick CAN
// -----------------------------------------------------------------------------
bool sendCanJoystick()
{
    PacketBuffer packet;
    if (!buildJoystickPacket(packet)) return false;

    Serial.print(F("[CAN JOY] Yaw="));
    Serial.print(yaw);
    Serial.print(F(" Pitch="));
    Serial.print(pitch);
    Serial.print(F(" Roll="));
    Serial.println(roll);
    logPacket("CAN TX JOY", packet);

    auto tx = ronin.sendPacket(packet);
    if (tx.isError()) {
        Serial.print(F("[CAN JOY TX ERROR] "));
        Serial.println(errorName(tx.error()));
        return false;
    }
    return true;
}

// -----------------------------------------------------------------------------
// SBUS
// -----------------------------------------------------------------------------
void sendSbus()
{
    bfs::SbusData data;
    data.lost_frame = false;
    data.failsafe = false;
    data.ch17 = false;
    data.ch18 = false;

    for (uint8_t i = 0; i < bfs::SbusData::NUM_CH; ++i)
        data.ch[i] = SBUS_MID;

    data.ch[CH_YAW] = djiToSbus(yaw);
    data.ch[CH_PITCH] = djiToSbus(pitch);
    data.ch[CH_RECORD] = recording ? SBUS_MAX : SBUS_MIN;

    sbus.data(data);
    sbus.Write();
    ++sbusFrames;

    Serial.print(F("[SBUS TX] CH1="));
    Serial.print(data.ch[CH_YAW]);
    Serial.print(F(" CH2="));
    Serial.print(data.ch[CH_PITCH]);
    Serial.print(F(" CH4="));
    Serial.print(data.ch[CH_RECORD]);
    Serial.println(recording ? F(" START") : F(" STOP"));
}

// -----------------------------------------------------------------------------
void handleRecordButton()
{
    bool raw = digitalRead(RECORD_BUTTON_PIN);

    if (raw != buttonLast) {
        buttonChangedAt = millis();
        buttonLast = raw;
    }

    if (millis() - buttonChangedAt < DEBOUNCE_MS) return;

    if (buttonStable == HIGH && raw == LOW) {
        recording = !recording;
        Serial.print(F("[REC] "));
        Serial.println(recording ? F("START") : F("STOP"));

        auto r = recording ? ronin.camera.recordStart()
                           : ronin.camera.recordStop();
        if (r.isError()) {
            Serial.print(F("[REC CAN ERROR] "));
            Serial.println(errorName(r.error()));
        } else {
            Serial.println(F("[REC CAN] command sent"));
        }
    }

    buttonStable = raw;
}

// -----------------------------------------------------------------------------
void setup()
{
    Serial.begin(115200);
    delay(500);

    pinMode(RECORD_BUTTON_PIN, INPUT_PULLUP);

#if defined(ESP32)
    SbusSerial.begin(100000, SERIAL_8E2, SBUS_RX_UNUSED, SBUS_TX_PIN);
#endif

    Serial.println();
    Serial.println(F("============================================================"));
    Serial.println(F(" DJI R SDK - DUAL CAN + SBUS FIELD TEST"));
    Serial.println(F("============================================================"));
    Serial.println(F("A1 -> Yaw | A2 -> Pitch | D2 -> Record"));
    Serial.println(F("CAN: MCP2515 / 8 MHz / 1 Mbps / 0x223 -> Ronin"));
    Serial.print(F("SBUS inverted: "));
    Serial.println(SBUS_INVERTED ? F("YES") : F("NO"));
    Serial.println(F("SBUS CH1=Yaw CH2=Pitch CH4=Record"));
    Serial.println(F("============================================================"));

    rawBuilder.resetSequence(sequenceStart);

    if (ENABLE_CAN) {
        auto r = ronin.begin();
        if (r.isError()) {
            Serial.print(F("[CAN FATAL] begin failed: "));
            Serial.println(errorName(r.error()));
            Serial.println(F("CAN disabled logically; program continues."));
        } else {
            Serial.println(F("[CAN OK] MCP2515 initialized"));
            Serial.println(F("[CAN] Sending Ronin Obtain Information request..."));
            requestRoninInformation();
        }
    }

    if (ENABLE_SBUS) {
        sbus.Begin();
        Serial.println(F("[SBUS OK] Bolder Flight SbusTx initialized"));
    }

    Serial.println(F("[RUN] Program will continue even if Ronin sends no CAN response."));
}

void loop()
{
    const unsigned long now = millis();

    const int rawYaw = analogRead(JOYSTICK_YAW_PIN);
    const int rawPitch = analogRead(JOYSTICK_PITCH_PIN);

    yaw = mapJoystick(rawYaw, false);
    pitch = mapJoystick(rawPitch, true);
    roll = 0;

    handleRecordButton();

    if (ENABLE_CAN && now - lastJoystickMs >= JOYSTICK_PERIOD_MS) {
        lastJoystickMs = now;
        if (sendCanJoystick()) ++canJoystickOk;
        else ++canJoystickFail;
    }

    if (ENABLE_CAN && now - lastInfoMs >= INFO_RETRY_MS) {
        lastInfoMs = now;
        if (!roninSeen) requestRoninInformation();
    }

    if (ENABLE_SBUS && now - lastSbusMs >= SBUS_PERIOD_MS) {
        lastSbusMs = now;
        sendSbus();
    }

    if (now - lastStatusMs >= STATUS_PERIOD_MS) {
        lastStatusMs = now;

        Serial.print(F("[STATUS] A1="));
        Serial.print(rawYaw);
        Serial.print(F(" A2="));
        Serial.print(rawPitch);
        Serial.print(F(" | Yaw="));
        Serial.print(yaw);
        Serial.print(F(" Pitch="));
        Serial.print(pitch);
        Serial.print(F(" | Ronin="));
        Serial.print(roninSeen ? F("RESPONDED") : F("NO RESPONSE"));
        Serial.print(F(" | CANjoy="));
        Serial.print(canJoystickOk);
        Serial.print(F("/"));
        Serial.print(canJoystickFail);
        Serial.print(F(" | Info="));
        Serial.print(canInfoResponses);
        Serial.print(F("/"));
        Serial.print(canInfoNoResponse);
        Serial.print(F(" | SBUS="));
        Serial.println(sbusFrames);
    }
}
