/**
 * LoopbackJoystickDiagnostic.ino
 *
 * هدف:
 *  - Joystick روی A1/A2
 *  - A1 -> Yaw ، A2 -> Pitch
 *  - محدوده سرعت: -7500 ... +7500
 *  - نرخ ارسال: 50Hz (هر 20ms)
 *  - MCP2515 با کریستال 8MHz و CAN 1Mbps
 *  - ارسال Packet در CAN frame های حداکثر 8 بایتی
 *  - 250us فاصله بین frame های یک packet
 *  - Loopback و بازسازی packet
 *  - مقایسه byte به byte
 *  - اجرای PacketParser روی packet برگشتی
 *
 * توجه:
 * این مثال مسیر PacketBuilder/Parser و fragmentation را تست می‌کند.
 * چون Loopback مستقیماً MCP2515 را صدا می‌زند، تابع
 * DJIRonin::sendAsCanFrames() را به‌صورت مستقیم تست نمی‌کند.
 */

#include <SPI.h>
#include <mcp2515.h>
#include <DJIRonin.h>

using namespace dji::ronin;

const uint8_t JOYSTICK_YAW_PIN   = A1;
const uint8_t JOYSTICK_PITCH_PIN = A2;
const uint8_t MCP2515_CS_PIN     = 10;

const CAN_CLOCK MCP_CLOCK = MCP_8MHZ;
const uint32_t SEND_INTERVAL_MS = 20;
const uint16_t INTER_FRAME_DELAY_US = 250;

const int JOYSTICK_CENTER = 512;
const int DEAD_ZONE = 40;
const int16_t DJI_MIN = -7500;
const int16_t DJI_MAX = 7500;

MCP2515 mcp2515(MCP2515_CS_PIN);
PacketBuilder builder;
PacketParser parser;

uint8_t rxBuffer[MAX_PACKET_SIZE];
uint16_t rxLength = 0;
uint16_t rxExpectedLength = 0;

uint32_t packetCount = 0;
uint32_t txFrameCount = 0;
uint32_t rxFrameCount = 0;
uint32_t passCount = 0;
uint32_t failCount = 0;
uint32_t parserPassCount = 0;
uint32_t parserFailCount = 0;
uint32_t txErrorCount = 0;
uint32_t rxErrorCount = 0;

unsigned long nextSendMs = 0;

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
        if (i + 1 < len) Serial.print(' ');
    }
    Serial.println();
}

int16_t mapJoystick(int raw, bool invert)
{
    int delta = raw - JOYSTICK_CENTER;

    if (abs(delta) <= DEAD_ZONE)
        return 0;

    if (invert)
        delta = -delta;

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

void resetRx()
{
    rxLength = 0;
    rxExpectedLength = 0;
}

bool updateExpectedLength()
{
    if (rxLength < 3)
        return true;

    if (rxBuffer[0] != SOF) {
        Serial.println(F("[RX BUFFER ERROR] SOF != 0xAA"));
        return false;
    }

    uint16_t verLength =
        (uint16_t)rxBuffer[OFFSET_VER_LENGTH] |
        ((uint16_t)rxBuffer[OFFSET_VER_LENGTH + 1] << 8);

    rxExpectedLength = verLength & LENGTH_MASK;

    Serial.print(F("[RX BUFFER] Ver/Length = 0x"));
    Serial.println(verLength, HEX);

    Serial.print(F("[RX BUFFER] Expected length = "));
    Serial.println(rxExpectedLength);

    if (rxExpectedLength < MIN_PACKET_SIZE ||
        rxExpectedLength > MAX_PACKET_SIZE) {
        Serial.println(F("[RX BUFFER ERROR] Invalid packet length"));
        return false;
    }

    return true;
}

bool comparePackets(const PacketBuffer& tx)
{
    Serial.println(F("[COMPARE] TX vs reconstructed RX"));

    if (tx.length != rxLength) {
        Serial.print(F("[FAIL] Length TX="));
        Serial.print(tx.length);
        Serial.print(F(" RX="));
        Serial.println(rxLength);
        return false;
    }

    for (uint16_t i = 0; i < tx.length; ++i) {
        if (tx.data[i] != rxBuffer[i]) {
            Serial.print(F("[FAIL] Byte "));
            Serial.print(i);
            Serial.print(F(": TX=0x"));
            if (tx.data[i] < 0x10) Serial.print('0');
            Serial.print(tx.data[i], HEX);
            Serial.print(F(" RX=0x"));
            if (rxBuffer[i] < 0x10) Serial.print('0');
            Serial.println(rxBuffer[i], HEX);
            return false;
        }
    }

    Serial.println(F("[OK] Every byte matches"));
    return true;
}

bool parseReconstructedPacket()
{
    Serial.println(F("[PARSER] Parsing reconstructed packet..."));

    Result<ParsedPacket> result =
        parser.parse(rxBuffer, rxLength);

    if (result.isError()) {
        ++parserFailCount;
        Serial.print(F("[PARSER FAIL] "));
        Serial.println(errorName(result.error()));
        return false;
    }

    ++parserPassCount;

    const ParsedPacket& p = result.value();

    Serial.println(F("[PARSER OK] Packet accepted"));
    Serial.print(F("  SOF       = 0x"));
    Serial.println(p.header.sof, HEX);
    Serial.print(F("  Version   = "));
    Serial.println(p.header.version);
    Serial.print(F("  Length    = "));
    Serial.println(p.header.length);
    Serial.print(F("  CmdSet    = 0x"));
    Serial.println(p.command.cmdSet, HEX);
    Serial.print(F("  CmdID     = 0x"));
    Serial.println(p.command.cmdId, HEX);
    Serial.print(F("  Sequence  = "));
    Serial.println(p.header.seq);
    Serial.print(F("  Payload   = "));
    Serial.println(p.payloadLen);
    Serial.print(F("  CRC32     = 0x"));
    Serial.println(p.crc32, HEX);

    return true;
}

bool sendAndLoopback(const PacketBuffer& packet)
{
    resetRx();

    size_t offset = 0;
    uint8_t frameNo = 0;
    const unsigned long startUs = micros();

    while (offset < packet.length) {
        can_frame tx;
        memset(&tx, 0, sizeof(tx));

        tx.can_id = CAN_ID_TX;
        tx.can_dlc = 0;

        while (tx.can_dlc < 8 && offset < packet.length)
            tx.data[tx.can_dlc++] = packet.data[offset++];

        ++frameNo;
        ++txFrameCount;

        Serial.print(F("[TX FRAME "));
        Serial.print(frameNo);
        Serial.print(F("] ID=0x"));
        Serial.print(tx.can_id, HEX);
        Serial.print(F(" DLC="));
        Serial.print(tx.can_dlc);
        Serial.print(F(" DATA="));
        printHex(tx.data, tx.can_dlc);

        MCP2515::ERROR txErr = mcp2515.sendMessage(&tx);

        if (txErr != MCP2515::ERROR_OK) {
            ++txErrorCount;
            Serial.print(F("[TX ERROR] MCP2515 code="));
            Serial.println((int)txErr);
            return false;
        }

        if (offset < packet.length)
            delayMicroseconds(INTER_FRAME_DELAY_US);

        can_frame rx;
        memset(&rx, 0, sizeof(rx));

        MCP2515::ERROR rxErr = mcp2515.readMessage(&rx);

        if (rxErr != MCP2515::ERROR_OK) {
            delayMicroseconds(50);
            rxErr = mcp2515.readMessage(&rx);
        }

        if (rxErr != MCP2515::ERROR_OK) {
            ++rxErrorCount;
            Serial.print(F("[RX ERROR] Loopback code="));
            Serial.println((int)rxErr);
            return false;
        }

        ++rxFrameCount;

        Serial.print(F("[RX FRAME "));
        Serial.print(frameNo);
        Serial.print(F("] ID=0x"));
        Serial.print(rx.can_id, HEX);
        Serial.print(F(" DLC="));
        Serial.print(rx.can_dlc);
        Serial.print(F(" DATA="));
        printHex(rx.data, rx.can_dlc);

        if ((rx.can_id & 0x7FF) != CAN_ID_TX) {
            Serial.println(F("[RX ERROR] Unexpected CAN ID"));
            return false;
        }

        if (rx.can_dlc == 0 || rx.can_dlc > 8) {
            Serial.println(F("[RX ERROR] Invalid DLC"));
            return false;
        }

        if (rxLength + rx.can_dlc > MAX_PACKET_SIZE) {
            Serial.println(F("[RX ERROR] BUFFER OVERFLOW"));
            resetRx();
            return false;
        }

        for (uint8_t i = 0; i < rx.can_dlc; ++i)
            rxBuffer[rxLength++] = rx.data[i];

        Serial.print(F("[RX BUFFER] "));
        Serial.print(rxLength);
        Serial.print(F("/"));
        Serial.println(rxExpectedLength);

        if (rxExpectedLength == 0 && rxLength >= 3) {
            if (!updateExpectedLength()) {
                resetRx();
                return false;
            }
        }

        if (rxExpectedLength > 0 &&
            rxLength >= rxExpectedLength)
            break;
    }

    const unsigned long elapsedUs = micros() - startUs;

    Serial.print(F("[CAN] Frames="));
    Serial.print(frameNo);
    Serial.print(F(" RX bytes="));
    Serial.print(rxLength);
    Serial.print(F(" expected="));
    Serial.print(rxExpectedLength);
    Serial.print(F(" elapsed="));
    Serial.print(elapsedUs);
    Serial.println(F(" us"));

    if (rxLength != packet.length ||
        rxLength != rxExpectedLength) {
        Serial.println(F("[FAIL] Incomplete reconstructed packet"));
        return false;
    }

    Serial.print(F("[RX PACKET] "));
    printHex(rxBuffer, rxLength);

    if (!comparePackets(packet))
        return false;

    if (!parseReconstructedPacket())
        return false;

    return true;
}

void setup()
{
    Serial.begin(115200);
    delay(500);

    Serial.println();
    Serial.println(F("================================================"));
    Serial.println(F(" DJI R SDK - LOOPBACK JOYSTICK DIAGNOSTIC"));
    Serial.println(F("================================================"));
    Serial.println(F("A1 -> Yaw"));
    Serial.println(F("A2 -> Pitch"));
    Serial.println(F("Range: -7500 ... +7500"));
    Serial.println(F("Rate: 50 Hz / 20 ms"));
    Serial.println(F("CAN: 1 Mbps / MCP2515 8 MHz"));
    Serial.println(F("Inter-frame delay: 250 us"));
    Serial.println();

    SPI.begin();

    if (mcp2515.reset() != MCP2515::ERROR_OK) {
        Serial.println(F("[FATAL] MCP2515 reset failed"));
        while (true) {}
    }

    if (mcp2515.setBitrate(CAN_1000KBPS, MCP_CLOCK) != MCP2515::ERROR_OK) {
        Serial.println(F("[FATAL] setBitrate failed"));
        while (true) {}
    }

    Serial.println(F("[OK] CAN 1 Mbps / MCP_8MHZ"));

    if (mcp2515.setLoopbackMode() != MCP2515::ERROR_OK) {
        Serial.println(F("[FATAL] Loopback mode failed"));
        while (true) {}
    }

    Serial.println(F("[OK] MCP2515 Loopback ON"));

    builder.resetSequence(0);

    Serial.println(F("[OK] PacketBuilder ready"));
    Serial.println(F("[OK] PacketParser ready"));
    Serial.println(F("Move joystick. First packet starts after 20 ms."));
    Serial.println();
}

void loop()
{
    const unsigned long now = millis();

    if ((long)(now - nextSendMs) < 0)
        return;

    nextSendMs = now + SEND_INTERVAL_MS;

    const int rawYaw = analogRead(JOYSTICK_YAW_PIN);
    const int rawPitch = analogRead(JOYSTICK_PITCH_PIN);

    const int16_t yaw = mapJoystick(rawYaw, false);
    const int16_t pitch = mapJoystick(rawPitch, true);

    JoystickPayload payload;
    payload.device_type =
        static_cast<uint8_t>(ControllerType::Joystick);
    payload.pitch_speed = pitch;
    payload.roll_speed = 0;
    payload.yaw_speed = yaw;

    PacketBuffer packet;

    Result<void> buildResult = builder.buildCommand(
        GimbalCmd::CMDSET,
        GimbalCmd::ExternalDeviceControl,
        payload,
        ReplyRequirement::NoReply,
        packet
    );

    ++packetCount;

    Serial.println();
    Serial.print(F("================ PACKET #"));
    Serial.print(packetCount);
    Serial.println(F(" ================"));

    Serial.print(F("[JOYSTICK] A1="));
    Serial.print(rawYaw);
    Serial.print(F(" -> Yaw="));
    Serial.print(yaw);
    Serial.print(F(" | A2="));
    Serial.print(rawPitch);
    Serial.print(F(" -> Pitch="));
    Serial.println(pitch);

    if (buildResult.isError()) {
        Serial.print(F("[BUILDER FAIL] "));
        Serial.println(errorName(buildResult.error()));
        ++failCount;
        return;
    }

    Serial.print(F("[BUILDER OK] Packet length="));
    Serial.println(packet.length);

    Serial.print(F("[TX PACKET] "));
    printHex(packet.data, packet.length);

    if (packet.length >= 14) {
        Serial.print(F("[PACKET] CmdSet=0x"));
        Serial.print(packet.data[OFFSET_DATA], HEX);
        Serial.print(F(" CmdID=0x"));
        Serial.print(packet.data[OFFSET_DATA + 1], HEX);
        Serial.print(F(" Seq="));

        uint16_t seq =
            (uint16_t)packet.data[OFFSET_SEQ] |
            ((uint16_t)packet.data[OFFSET_SEQ + 1] << 8);

        Serial.println(seq);
    }

    if (sendAndLoopback(packet)) {
        ++passCount;
        Serial.println(F("[RESULT] PASS - TX/RX/PARSER"));
    } else {
        ++failCount;
        Serial.println(F("[RESULT] FAIL"));
    }

    Serial.print(F("[STATS] packets="));
    Serial.print(packetCount);
    Serial.print(F(" pass="));
    Serial.print(passCount);
    Serial.print(F(" fail="));
    Serial.print(failCount);
    Serial.print(F(" parserOK="));
    Serial.print(parserPassCount);
    Serial.print(F(" parserFAIL="));
    Serial.print(parserFailCount);
    Serial.print(F(" txErr="));
    Serial.print(txErrorCount);
    Serial.print(F(" rxErr="));
    Serial.println(rxErrorCount);
}
