/**
 * LoopbackJoystickTest.ino
 *
 * تست پکت واقعی DJI R SDK با MCP2515 در حالت Loopback
 *
 * فقط دو کتابخانه اصلی (مثل سبک خود MCP2515):
 *   #include <mcp2515.h>
 *   #include <DJIRonin.h>
 *
 * نصب کتابخانه:
 *   پوشه DJIRonin را در Arduino/libraries/ کپی کن
 *   سپس این مثال را از:
 *     File → Examples → DJIRonin → LoopbackJoystickTest
 *   باز کن (نه از Downloads به‌صورت جدا)
 */

#include <SPI.h>
#include <mcp2515.h>
#include <DJIRonin.h>          // کتابخانه اصلی

using namespace dji::ronin;

// ================================================================
// تنظیمات سخت‌افزار
// ================================================================
const int JOYSTICK_X_PIN = A1;   // Yaw
const int JOYSTICK_Y_PIN = A2;   // Pitch
const int MCP2515_CS_PIN = 10;

// اگر کریستال ماژول ۱۶MHz است → MCP_16MHZ
const CAN_CLOCK MCP_CLOCK = MCP_8MHZ;

// ================================================================
// تنظیمات جوی‌استیک
// ================================================================
const int     JOYSTICK_CENTER = 512;
const int     DEAD_ZONE       = 40;
const int16_t DJI_MIN         = -15000;
const int16_t DJI_MAX         = 15000;

// ================================================================
// زمان‌بندی
// ================================================================
const unsigned long DISPLAY_INTERVAL_MS = 100;  // 10 Hz

// ================================================================
// اشیاء
// ================================================================
MCP2515       mcp2515(MCP2515_CS_PIN);
PacketBuilder builder;

unsigned long lastDisplayTime = 0;
uint32_t      packetCounter   = 0;
uint32_t      matchCount      = 0;
uint32_t      mismatchCount   = 0;

uint8_t  rxBuffer[MAX_PACKET_SIZE];
uint16_t rxLength = 0;

// ================================================================
// توابع کمکی
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

/**
 * ارسال پکت کامل DJI در چند فریم CAN و دریافت مجدد در Loopback
 */
bool sendAndReceiveLoopback(const PacketBuffer& packet)
{
    rxLength = 0;
    size_t offset = 0;

    while (offset < packet.length) {
        struct can_frame txFrame;
        txFrame.can_id  = CAN_ID_TX;  // 0x223
        txFrame.can_dlc = 0;

        while (txFrame.can_dlc < 8 && offset < packet.length) {
            txFrame.data[txFrame.can_dlc++] = packet.data[offset++];
        }

        if (mcp2515.sendMessage(&txFrame) != MCP2515::ERROR_OK) {
            Serial.println(F("[ERR] CAN send failed"));
            return false;
        }

        struct can_frame rxFrame;
        MCP2515::ERROR err = mcp2515.readMessage(&rxFrame);
        if (err != MCP2515::ERROR_OK) {
            delay(1);
            err = mcp2515.readMessage(&rxFrame);
        }
        if (err != MCP2515::ERROR_OK) {
            Serial.println(F("[ERR] CAN receive failed (loopback)"));
            return false;
        }

        for (uint8_t i = 0; i < rxFrame.can_dlc; i++) {
            if (rxLength < MAX_PACKET_SIZE) {
                rxBuffer[rxLength++] = rxFrame.data[i];
            }
        }
    }
    return true;
}

bool compareBuffers(const uint8_t* a, size_t aLen, const uint8_t* b, size_t bLen)
{
    if (aLen != bLen) {
        Serial.print(F("  Length mismatch TX="));
        Serial.print(aLen);
        Serial.print(F(" RX="));
        Serial.println(bLen);
        return false;
    }
    bool ok = true;
    for (size_t i = 0; i < aLen; i++) {
        if (a[i] != b[i]) {
            if (ok) Serial.println(F("  Byte mismatches:"));
            Serial.print(F("    ["));
            Serial.print(i);
            Serial.print(F("] TX=0x"));
            if (a[i] < 0x10) Serial.print('0');
            Serial.print(a[i], HEX);
            Serial.print(F(" RX=0x"));
            if (b[i] < 0x10) Serial.print('0');
            Serial.println(b[i], HEX);
            ok = false;
        }
    }
    return ok;
}

// ================================================================
// Setup
// ================================================================
void setup()
{
    Serial.begin(115200);
    delay(500);

    Serial.println();
    Serial.println(F("========================================"));
    Serial.println(F(" DJI R SDK Loopback Packet Test"));
    Serial.println(F(" #include <DJIRonin.h> + MCP2515 Loopback"));
    Serial.println(F("========================================"));

    SPI.begin();
    mcp2515.reset();
    delay(50);

    if (mcp2515.setBitrate(CAN_1000KBPS, MCP_CLOCK) != MCP2515::ERROR_OK) {
        Serial.println(F("[ERR] setBitrate failed – check crystal (8/16 MHz)"));
        while (1) {}
    }
    Serial.println(F("[OK] Bitrate 1 Mbps"));

    if (mcp2515.setLoopbackMode() != MCP2515::ERROR_OK) {
        Serial.println(F("[ERR] setLoopbackMode failed"));
        while (1) {}
    }
    Serial.println(F("[OK] Loopback mode ON"));

    builder.resetSequence(0);
    Serial.println(F("[OK] DJIRonin PacketBuilder ready"));
    Serial.println();
    Serial.println(F("Move joystick. Packets every 100 ms."));
    Serial.println(F("========================================"));
}

// ================================================================
// Loop
// ================================================================
void loop()
{
    int rawX = analogRead(JOYSTICK_X_PIN);
    int rawY = analogRead(JOYSTICK_Y_PIN);

    int16_t yaw   = mapJoystickToDJI(rawX, false);
    int16_t pitch = mapJoystickToDJI(rawY, true);
    int16_t roll  = 0;

    if (millis() - lastDisplayTime < DISPLAY_INTERVAL_MS) return;
    lastDisplayTime = millis();
    packetCounter++;

    // ----- ساخت پکت واقعی جوی‌استیک با کتابخانه -----
    JoystickPayload payload;
    payload.device_type = static_cast<uint8_t>(ControllerType::Joystick); // 0x01
    payload.pitch_speed = pitch;
    payload.roll_speed  = roll;
    payload.yaw_speed   = yaw;

    PacketBuffer packet;
    auto result = builder.buildCommand(
        GimbalCmd::CMDSET,                  // 0x0E
        GimbalCmd::ExternalDeviceControl,   // 0x0A
        payload,
        ReplyRequirement::NoReply,
        packet
    );

    if (result.isError()) {
        Serial.println(F("[ERR] PacketBuilder failed"));
        return;
    }

    // ----- نمایش -----
    Serial.print(F("#"));
    Serial.print(packetCounter);
    Serial.print(F(" Yaw="));
    Serial.print(yaw);
    Serial.print(F(" Pitch="));
    Serial.print(pitch);
    Serial.print(F(" | len="));
    Serial.print(packet.length);
    Serial.print(F(" SEQ="));
    Serial.println(builder.currentSequence() - 1);

    Serial.print(F("TX: "));
    printHex(packet.data, packet.length);

    Serial.print(F("   SOF=0x"));
    Serial.print(packet.data[0], HEX);
    Serial.print(F(" CmdSet=0x"));
    Serial.print(packet.data[12], HEX);
    Serial.print(F(" CmdID=0x"));
    Serial.println(packet.data[13], HEX);

    // ----- Loopback -----
    if (!sendAndReceiveLoopback(packet)) {
        mismatchCount++;
        Serial.println(F("RESULT: SEND/RECV FAILED"));
        Serial.println(F("----------------------------------------"));
        return;
    }

    Serial.print(F("RX: "));
    printHex(rxBuffer, rxLength);

    if (compareBuffers(packet.data, packet.length, rxBuffer, rxLength)) {
        matchCount++;
        Serial.println(F("RESULT: MATCH"));
    } else {
        mismatchCount++;
        Serial.println(F("RESULT: MISMATCH"));
    }

    uint32_t total = matchCount + mismatchCount;
    Serial.print(F("Stats OK="));
    Serial.print(matchCount);
    Serial.print(F(" FAIL="));
    Serial.print(mismatchCount);
    if (total) {
        Serial.print(F(" ("));
        Serial.print((matchCount * 100UL) / total);
        Serial.print(F("%)"));
    }
    Serial.println();
    Serial.println(F("----------------------------------------"));
}
