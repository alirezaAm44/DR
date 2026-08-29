
/**
 * DualJoystickCanSbus.ino
 *
 * DJI Ronin R SDK:
 *   - CAN: Joystick control + Camera Record + Obtain Information
 *   - SBUS: Same joystick values on RC channels
 *
 * Serial Monitor:
 *   115200 baud
 *
 * CAN:
 *   MCP2515 + mcp2515.h
 *   1 Mbps
 *   8 MHz crystal
 *
 * IMPORTANT:
 *   This version DOES NOT use MCP2515Driver.h.
 *   It uses the standard mcp2515 library directly.
 */

#include <SPI.h>
#include <SoftwareSerial.h>
#include <mcp2515.h>
#include <DJIRonin.h>

using namespace dji::ronin;

// ================================================================
// CONFIGURATION
// ================================================================

const bool ENABLE_CAN      = true;
const bool ENABLE_SBUS     = true;
const bool USE_SBUS_MOTION = true;

// Detailed logging
const bool LOG_JOYSTICK        = true;
const bool LOG_CAN_TX          = true;
const bool LOG_CAN_TX_FRAMES   = true;
const bool LOG_CAN_RX_FRAMES   = true;
const bool LOG_CAN_RX_PACKET   = true;
const bool LOG_SBUS             = true;
const bool LOG_SBUS_FRAME_HEX   = false;
const bool LOG_STATUS            = true;

// ================================================================
// HARDWARE
// ================================================================

const int JOYSTICK_X_PIN = A1;   // Yaw
const int JOYSTICK_Y_PIN = A2;   // Pitch
const int JOYSTICK_SW_PIN = 2;   // Record button

const int MCP2515_CS_PIN = 10;

const int SBUS_TX_PIN = 8;

// RSA/NATO Ronin connector:
// CANH  -> Pin 4
// CANL  -> Pin 2
// GND   -> Pin 6
// AD_COM -> resistor -> GND
// SBUS  -> 74HC04 -> Pin 3

const CAN_CLOCK MCP_CLOCK = MCP_8MHZ;

// ================================================================
// JOYSTICK
// ================================================================

const int JOYSTICK_CENTER = 512;
const int DEAD_ZONE       = 40;

const int16_t DJI_MIN = -7500;
const int16_t DJI_MAX = 7500;

// ================================================================
// SBUS
// ================================================================

const uint16_t SBUS_MIN = 352;
const uint16_t SBUS_MID = 1024;
const uint16_t SBUS_MAX = 1696;

const unsigned long SBUS_INTERVAL_MS = 14;

// SBUS channel mapping
const uint8_t SBUS_CH_YAW   = 0;   // CH1
const uint8_t SBUS_CH_PITCH = 1;   // CH2
const uint8_t SBUS_CH_ROLL  = 3;   // CH4

SoftwareSerial sbusSerial(9, SBUS_TX_PIN);

// ================================================================
// TIMING
// ================================================================

const unsigned long JOYSTICK_INTERVAL_MS = 20;
const unsigned long INFO_INTERVAL_MS     = 200;
const unsigned long STATUS_PRINT_MS      = 200;
const unsigned long DEBOUNCE_MS          = 40;

// ================================================================
// OBJECTS
// ================================================================

MCP2515 mcp2515(MCP2515_CS_PIN);

PacketBuilder builder;
PacketParser  parser;

// ================================================================
// STATE
// ================================================================

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

int16_t lastYaw   = 0;
int16_t lastPitch = 0;
int16_t lastRoll  = 0;

bool haveAttitude = false;

int16_t attYaw   = 0;
int16_t attRoll  = 0;
int16_t attPitch = 0;

bool isRecording = false;

// Button debounce
bool lastButtonStable = HIGH;
bool lastButtonRead   = HIGH;

unsigned long lastDebounceMs = 0;

// RX packet accumulator
uint8_t  rxAcc[MAX_PACKET_SIZE];
uint16_t rxAccLen      = 0;
uint16_t rxExpectedLen = 0;

// SBUS channels
uint16_t sbusChannels[16];

// ================================================================
// HEX PRINT
// ================================================================

void printHex(const uint8_t* data, size_t len)
{
    for (size_t i = 0; i < len; i++)
    {
        if (data[i] < 0x10)
            Serial.print('0');

        Serial.print(data[i], HEX);
        Serial.print(' ');
    }

    Serial.println();
}

// ================================================================
// JOYSTICK MAPPING
// ================================================================

int16_t mapJoystickToDJI(int rawValue, bool invert = false)
{
    int delta = rawValue - JOYSTICK_CENTER;

    if (abs(delta) < DEAD_ZONE)
        return 0;

    if (invert)
        delta = -delta;

    long mapped =
        (long)delta * DJI_MAX /
        (JOYSTICK_CENTER - DEAD_ZONE);

    if (mapped > DJI_MAX)
        mapped = DJI_MAX;

    if (mapped < DJI_MIN)
        mapped = DJI_MIN;

    return (int16_t)mapped;
}

uint16_t mapDjiToSbus(int16_t v)
{
    long span =
        (long)SBUS_MAX -
        (long)SBUS_MIN;

    long mapped =
        (long)SBUS_MID +
        ((long)v * (span / 2)) /
        DJI_MAX;

    if (mapped < SBUS_MIN)
        mapped = SBUS_MIN;

    if (mapped > SBUS_MAX)
        mapped = SBUS_MAX;

    return (uint16_t)mapped;
}

// ================================================================
// SBUS
// ================================================================

void sbusResetChannels()
{
    for (uint8_t i = 0; i < 16; i++)
    {
        sbusChannels[i] = SBUS_MID;
    }
}

void updateSbusFromJoystick(
    int16_t yaw,
    int16_t pitch,
    int16_t roll)
{
    sbusResetChannels();

    if (USE_SBUS_MOTION)
    {
        sbusChannels[SBUS_CH_YAW] =
            mapDjiToSbus(yaw);

        sbusChannels[SBUS_CH_PITCH] =
            mapDjiToSbus(pitch);

        sbusChannels[SBUS_CH_ROLL] =
            mapDjiToSbus(roll);
    }
}

void sbusPackAndSend()
{
    uint8_t packet[25];

    packet[0] = 0x0F;

    uint8_t byteIdx = 1;

    uint32_t bitBuf = 0;
    uint8_t bitCnt = 0;

    for (uint8_t ch = 0; ch < 16; ch++)
    {
        uint16_t v =
            sbusChannels[ch] & 0x07FF;

        bitBuf |=
            ((uint32_t)v) << bitCnt;

        bitCnt += 11;

        while (bitCnt >= 8)
        {
            packet[byteIdx++] =
                (uint8_t)(bitBuf & 0xFF);

            bitBuf >>= 8;
            bitCnt -= 8;
        }
    }

    if (bitCnt > 0 && byteIdx < 23)
    {
        packet[byteIdx++] =
            (uint8_t)(bitBuf & 0xFF);
    }

    while (byteIdx < 23)
    {
        packet[byteIdx++] = 0;
    }

    packet[23] = 0x00;
    packet[24] = 0x00;

    for (uint8_t i = 0; i < 25; i++)
    {
        sbusSerial.write(packet[i]);
    }

    txSbusCount++;

    if (LOG_SBUS)
    {
        Serial.print(F("[SBUS TX] Yaw="));
        Serial.print(sbusChannels[SBUS_CH_YAW]);

        Serial.print(F(" Pitch="));
        Serial.print(sbusChannels[SBUS_CH_PITCH]);

        Serial.print(F(" Roll="));
        Serial.print(sbusChannels[SBUS_CH_ROLL]);

        Serial.print(F(" Count="));
        Serial.println(txSbusCount);
    }

    if (LOG_SBUS_FRAME_HEX)
    {
        Serial.print(F("[SBUS HEX] "));
        printHex(packet, 25);
    }
}

// ================================================================
// CAN MULTIFRAME TRANSMISSION
// ================================================================

bool sendPacketMultiFrame(
    const PacketBuffer& packet)
{
    size_t offset = 0;

    uint16_t frameNumber = 0;

    while (offset < packet.length)
    {
        struct can_frame frame;

        frame.can_id = CAN_ID_TX;
        frame.can_dlc = 0;

        while (
            frame.can_dlc < 8 &&
            offset < packet.length)
        {
            frame.data[frame.can_dlc++] =
                packet.data[offset++];
        }

        if (LOG_CAN_TX_FRAMES)
        {
            Serial.print(F("[CAN TX FRAME #"));
            Serial.print(frameNumber);
            Serial.print(F("] ID=0x"));
            Serial.print(frame.can_id, HEX);

            Serial.print(F(" DLC="));
            Serial.print(frame.can_dlc);

            Serial.print(F(" DATA="));

            for (uint8_t i = 0;
                 i < frame.can_dlc;
                 i++)
            {
                if (frame.data[i] < 0x10)
                    Serial.print('0');

                Serial.print(
                    frame.data[i],
                    HEX);

                Serial.print(' ');
            }

            Serial.println();
        }

        MCP2515::ERROR e =
            mcp2515.sendMessage(&frame);

        if (e != MCP2515::ERROR_OK)
        {
            canSendFailCount++;

            Serial.print(
                F("[CAN TX ERROR] code="));

            Serial.println(
                static_cast<int>(e));

            return false;
        }

        frameNumber++;
    }

    return true;
}

// ================================================================
// CAN JOYSTICK
// ================================================================

bool sendJoystick(
    int16_t yaw,
    int16_t pitch,
    int16_t roll)
{
    JoystickPayload payload;

    payload.device_type =
        static_cast<uint8_t>(
            ControllerType::Joystick);

    payload.pitch_speed = pitch;
    payload.roll_speed  = roll;
    payload.yaw_speed   = yaw;

    PacketBuffer packet;

    auto result =
        builder.buildCommand(
            GimbalCmd::CMDSET,
            GimbalCmd::ExternalDeviceControl,
            payload,
            ReplyRequirement::NoReply,
            packet);

    if (result.isError())
    {
        Serial.println(
            F("[CAN] Joystick build ERROR"));

        return false;
    }

    if (LOG_CAN_TX)
    {
        Serial.print(
            F("[CAN TX JOYSTICK] "));

        Serial.print(F("Yaw="));
        Serial.print(yaw);

        Serial.print(F(" Pitch="));
        Serial.print(pitch);

        Serial.print(F(" Roll="));
        Serial.print(roll);

        Serial.print(F(" PacketLen="));
        Serial.println(packet.length);

        Serial.print(F("[CAN TX PACKET] "));
        printHex(packet.data, packet.length);
    }

    return sendPacketMultiFrame(packet);
}

// ================================================================
// OBTAIN INFORMATION
// ================================================================

bool sendObtainInfo(uint8_t infoType)
{
    ObtainInfoRequestPayload payload;

    payload.ctrl_byte = infoType;

    PacketBuffer packet;

    auto result =
        builder.buildCommand(
            GimbalCmd::CMDSET,
            GimbalCmd::ObtainInformation,
            payload,
            ReplyRequirement::ReplyRequired,
            packet);

    if (result.isError())
    {
        Serial.println(
            F("[CAN] ObtainInfo build ERROR"));

        return false;
    }

    Serial.print(
        F("[CAN TX] ObtainInformation "));

    Serial.print(F("type=0x"));
    Serial.print(infoType, HEX);

    Serial.print(F(" len="));
    Serial.println(packet.length);

    if (LOG_CAN_TX)
    {
        Serial.print(F("[CAN TX PACKET] "));
        printHex(packet.data, packet.length);
    }

    return sendPacketMultiFrame(packet);
}

// ================================================================
// CAMERA RECORD
// ================================================================

bool sendCameraRecord(bool start)
{
    CameraMotionPayload payload;

    payload.command =
        static_cast<uint16_t>(
            start
                ? CameraMotionCommand::StartRecording
                : CameraMotionCommand::StopRecording);

    PacketBuffer packet;

    auto result =
        builder.buildCommand(
            CameraCmd::CMDSET,
            CameraCmd::Motion,
            payload,
            ReplyRequirement::NoReply,
            packet);

    if (result.isError())
    {
        Serial.println(
            F("[CAN] Camera build ERROR"));

        return false;
    }

    Serial.print(F("[CAN TX] CAMERA "));

    Serial.println(
        start
            ? F("START RECORDING")
            : F("STOP RECORDING"));

    if (LOG_CAN_TX)
    {
        Serial.print(F("[CAN TX PACKET] "));
        printHex(packet.data, packet.length);
    }

    bool ok =
        sendPacketMultiFrame(packet);

    if (ok)
    {
        txCameraCount++;
    }

    return ok;
}

// ================================================================
// RX PACKET
// ================================================================

void handleCompleteRxPacket(
    const uint8_t* data,
    uint16_t len)
{
    rxPacketCount++;

    Serial.println();
    Serial.println(
        F("========================================"));

    Serial.println(
        F("[RONIN RX] COMPLETE PACKET"));

    Serial.print(F("Length = "));
    Serial.println(len);

    if (LOG_CAN_RX_PACKET)
    {
        Serial.print(F("RAW HEX = "));
        printHex(data, len);
    }

    auto result =
        parser.parse(data, len);

    if (result.isError())
    {
        rxErrorCount++;

        Serial.println(
            F("[RX PARSE ERROR]"));

        Serial.println(
            F("========================================"));

        return;
    }

    ParsedPacket pkt =
        result.value();

    Serial.print(F("CmdSet = 0x"));
    Serial.println(
        pkt.command.cmdSet,
        HEX);

    Serial.print(F("CmdID  = 0x"));
    Serial.println(
        pkt.command.cmdId,
        HEX);

    Serial.print(F("PayloadLen = "));
    Serial.println(pkt.payloadLen);

    Serial.print(F("HasReturnCode = "));
    Serial.println(
        pkt.hasReturnCode
            ? F("YES")
            : F("NO"));

    if (pkt.hasReturnCode)
    {
        Serial.print(F("ReturnCode = 0x"));

        Serial.println(
            static_cast<uint8_t>(
                pkt.returnCode),
            HEX);

        if (
            pkt.returnCode ==
            ReturnCode::Success)
        {
            Serial.println(
                F("Return Status = SUCCESS"));
        }
        else
        {
            Serial.println(
                F("Return Status = ERROR"));

            rxErrorCount++;
        }
    }

    // ------------------------------------------------------------
    // Obtain Information reply
    // ------------------------------------------------------------

    if (
        pkt.command.cmdSet ==
            GimbalCmd::CMDSET &&
        pkt.command.cmdId ==
            GimbalCmd::ObtainInformation &&
        pkt.payloadLen >=
            sizeof(ObtainInfoReplyPayload))
    {
        const ObtainInfoReplyPayload* rep =
            reinterpret_cast<
                const ObtainInfoReplyPayload*>(
                    pkt.payload);

        haveAttitude = true;

        attYaw   = rep->yaw;
        attRoll  = rep->roll;
        attPitch = rep->pitch;

        Serial.println(
            F("[ATTITUDE]"));

        Serial.print(F("Yaw   = "));
        Serial.print(
            attYaw / 10.0f,
            1);

        Serial.println(F(" deg"));

        Serial.print(F("Roll  = "));
        Serial.print(
            attRoll / 10.0f,
            1);

        Serial.println(F(" deg"));

        Serial.print(F("Pitch = "));
        Serial.print(
            attPitch / 10.0f,
            1);

        Serial.println(F(" deg"));
    }

    Serial.println(
        F("========================================"));
}

// ================================================================
// CAN RX POLLING
// ================================================================

void pollIncoming()
{
    if (!ENABLE_CAN)
        return;

    struct can_frame frame;

    while (
        mcp2515.readMessage(&frame) ==
        MCP2515::ERROR_OK)
    {
        if (LOG_CAN_RX_FRAMES)
        {
            Serial.print(F("[CAN RX FRAME] ID=0x"));
            Serial.print(
                frame.can_id,
                HEX);

            Serial.print(F(" DLC="));
            Serial.print(frame.can_dlc);

            Serial.print(F(" DATA="));

            for (uint8_t i = 0;
                 i < frame.can_dlc;
                 i++)
            {
                if (frame.data[i] < 0x10)
                    Serial.print('0');

                Serial.print(
                    frame.data[i],
                    HEX);

                Serial.print(' ');
            }

            Serial.println();
        }

        if (
            (frame.can_id & 0x7FF) !=
            CAN_ID_RX)
        {
            continue;
        }

        for (
            uint8_t i = 0;
            i < frame.can_dlc;
            i++)
        {
            uint8_t b = frame.data[i];

            // Wait for SOF
            if (rxAccLen == 0)
            {
                if (b != SOF)
                    continue;

                rxAcc[0] = b;
                rxAccLen = 1;
                rxExpectedLen = 0;

                continue;
            }

            // Append byte
            if (
                rxAccLen <
                MAX_PACKET_SIZE)
            {
                rxAcc[rxAccLen++] = b;
            }
            else
            {
                Serial.println(
                    F("[RX ERROR] Packet buffer overflow"));

                rxAccLen = 0;
                rxExpectedLen = 0;

                continue;
            }

            // Header length
            if (rxAccLen == 3)
            {
                uint16_t verLen =
                    (uint16_t)rxAcc[1] |
                    ((uint16_t)rxAcc[2] << 8);

                rxExpectedLen =
                    verLen & LENGTH_MASK;

                Serial.print(
                    F("[RX] Expected packet length = "));

                Serial.println(
                    rxExpectedLen);

                if (
                    rxExpectedLen <
                        MIN_PACKET_SIZE ||
                    rxExpectedLen >
                        MAX_PACKET_SIZE)
                {
                    Serial.println(
                        F("[RX ERROR] Invalid packet length"));

                    rxAccLen = 0;
                    rxExpectedLen = 0;
                }
            }

            // Complete packet
            if (
                rxExpectedLen > 0 &&
                rxAccLen >= rxExpectedLen)
            {
                handleCompleteRxPacket(
                    rxAcc,
                    rxExpectedLen);

                rxAccLen = 0;
                rxExpectedLen = 0;
            }
        }
    }
}

// ================================================================
// RECORD BUTTON
// ================================================================

void handleRecordButton()
{
    bool reading =
        digitalRead(JOYSTICK_SW_PIN);

    if (reading != lastButtonRead)
    {
        lastDebounceMs = millis();
        lastButtonRead = reading;
    }

    if (
        millis() - lastDebounceMs <
        DEBOUNCE_MS)
    {
        return;
    }

    if (
        lastButtonStable == HIGH &&
        reading == LOW)
    {
        isRecording = !isRecording;

        Serial.print(F("[BUTTON] Record -> "));

        Serial.println(
            isRecording
                ? F("START")
                : F("STOP"));

        if (ENABLE_CAN)
        {
            sendCameraRecord(
                isRecording);
        }
    }

    lastButtonStable = reading;
}

// ================================================================
// SETUP
// ================================================================

void setup()
{
    Serial.begin(115200);

    delay(500);

    pinMode(
        JOYSTICK_SW_PIN,
        INPUT_PULLUP);

    Serial.println();
    Serial.println(
        F("========================================"));

    Serial.println(
        F(" DJI RONIN CAN + SBUS TEST"));

    Serial.println(
        F(" DualJoystickCanSbus"));

    Serial.println(
        F("========================================"));

    // ------------------------------------------------------------
    // Hardware information
    // ------------------------------------------------------------

    Serial.println(
        F("[CONFIG]"));

    Serial.print(F("CAN = "));
    Serial.println(
        ENABLE_CAN
            ? F("ENABLED")
            : F("DISABLED"));

    Serial.print(F("SBUS = "));
    Serial.println(
        ENABLE_SBUS
            ? F("ENABLED")
            : F("DISABLED"));

    Serial.print(F("SBUS Motion = "));
    Serial.println(
        USE_SBUS_MOTION
            ? F("ENABLED")
            : F("DISABLED"));

    Serial.println();

    // ------------------------------------------------------------
    // SBUS
    // ------------------------------------------------------------

    if (ENABLE_SBUS)
    {
        sbusSerial.begin(100000);

        sbusResetChannels();

        Serial.println(
            F("[SBUS]"));

        Serial.print(F("TX Pin = D"));
        Serial.println(SBUS_TX_PIN);

        Serial.println(
            F("Baud = 100000"));

        Serial.println(
            F("Format = 8E2 expected"));

        Serial.println(
            F("Hardware inverter = 74HC04"));

        Serial.println(
            F("Ronin SBUS = RSA Pin 3"));
    }

    // ------------------------------------------------------------
    // CAN
    // ------------------------------------------------------------

    if (ENABLE_CAN)
    {
        Serial.println();
        Serial.println(F("[CAN] Initializing..."));

        SPI.begin();

        mcp2515.reset();

        delay(50);

        MCP2515::ERROR bitrateResult =
            mcp2515.setBitrate(
                CAN_1000KBPS,
                MCP_CLOCK);

        if (
            bitrateResult !=
            MCP2515::ERROR_OK)
        {
            Serial.print(
                F("[CAN ERROR] setBitrate code="));

            Serial.println(
                static_cast<int>(
                    bitrateResult));

            while (1) {}
        }

        MCP2515::ERROR modeResult =
            mcp2515.setNormalMode();

        if (
            modeResult !=
            MCP2515::ERROR_OK)
        {
            Serial.print(
                F("[CAN ERROR] setNormalMode code="));

            Serial.println(
                static_cast<int>(
                    modeResult));

            while (1) {}
        }

        // Accept all messages
        mcp2515.setFilterMask(
            MCP2515::MASK0,
            false,
            0);

        mcp2515.setFilterMask(
            MCP2515::MASK1,
            false,
            0);

        mcp2515.setFilter(
            MCP2515::RXF0,
            false,
            0);

        builder.resetSequence(0);

        Serial.println(
            F("[CAN] MCP2515 OK"));

        Serial.println(
            F("[CAN] Speed = 1 Mbps"));

        Serial.println(
            F("[CAN] Clock = 8 MHz"));

        Serial.print(F("[CAN] TX ID = 0x"));
        Serial.println(
            CAN_ID_TX,
            HEX);

        Serial.print(F("[CAN] RX ID = 0x"));
        Serial.println(
            CAN_ID_RX,
            HEX);
    }

    Serial.println();

    Serial.println(
        F("[WIRING]"));

    Serial.println(
        F("CANH -> Ronin RSA Pin 4"));

    Serial.println(
        F("CANL -> Ronin RSA Pin 2"));

    Serial.println(
        F("GND  -> Ronin RSA Pin 6"));

    Serial.println(
        F("AD_COM -> resistor -> GND"));

    Serial.println(
        F("SBUS -> 74HC04 -> Ronin RSA Pin 3"));

    Serial.println();

    Serial.println(
        F("========================================"));

    Serial.println(
        F(" SETUP COMPLETE"));

    Serial.println(
        F("========================================"));
}

// ================================================================
// LOOP
// ================================================================

void loop()
{
    if (ENABLE_CAN)
    {
        pollIncoming();

        handleRecordButton();
    }

    unsigned long now =
        millis();

    // ------------------------------------------------------------
    // Read joystick
    // ------------------------------------------------------------

    int rawX =
        analogRead(JOYSTICK_X_PIN);

    int rawY =
        analogRead(JOYSTICK_Y_PIN);

    lastYaw =
        mapJoystickToDJI(
            rawX,
            false);

    lastPitch =
        mapJoystickToDJI(
            rawY,
            true);

    lastRoll = 0;

    // ------------------------------------------------------------
    // Joystick debug
    // ------------------------------------------------------------

    if (
        LOG_JOYSTICK &&
        ENABLE_CAN &&
        now - lastJoystickMs >=
            JOYSTICK_INTERVAL_MS)
    {
        Serial.print(F("[JOYSTICK] RAW X="));
        Serial.print(rawX);

        Serial.print(F(" Y="));
        Serial.print(rawY);

        Serial.print(F(" -> Yaw="));
        Serial.print(lastYaw);

        Serial.print(F(" Pitch="));
        Serial.print(lastPitch);

        Serial.print(F(" Roll="));
        Serial.println(lastRoll);
    }

    // ------------------------------------------------------------
    // CAN joystick
    // ------------------------------------------------------------

    if (
        ENABLE_CAN &&
        now - lastJoystickMs >=
            JOYSTICK_INTERVAL_MS)
    {
        lastJoystickMs = now;

        if (
            sendJoystick(
                lastYaw,
                lastPitch,
                lastRoll))
        {
            txJoystickCount++;
        }
    }

    // ------------------------------------------------------------
    // Obtain Information
    // ------------------------------------------------------------

    if (
        ENABLE_CAN &&
        now - lastInfoMs >=
            INFO_INTERVAL_MS)
    {
        lastInfoMs = now;

        if (
            sendObtainInfo(0x01))
        {
            txInfoCount++;
        }
    }

    // ------------------------------------------------------------
    // SBUS
    // ------------------------------------------------------------

    if (
        ENABLE_SBUS &&
        now - lastSbusMs >=
            SBUS_INTERVAL_MS)
    {
        lastSbusMs = now;

        updateSbusFromJoystick(
            lastYaw,
            lastPitch,
            lastRoll);

        sbusPackAndSend();
    }

    // ------------------------------------------------------------
    // STATUS
    // ------------------------------------------------------------

    if (
        LOG_STATUS &&
        now - lastStatusMs >=
            STATUS_PRINT_MS)
    {
        lastStatusMs = now;

        Serial.println();
        Serial.println(F("[STATUS]"));

        Serial.print(F("Joystick: Yaw="));
        Serial.print(lastYaw);

        Serial.print(F(" Pitch="));
        Serial.print(lastPitch);

        Serial.print(F(" Roll="));
        Serial.println(lastRoll);

        Serial.print(F("CAN Joystick TX="));
        Serial.print(txJoystickCount);

        Serial.print(F(" | Info TX="));
        Serial.print(txInfoCount);

        Serial.print(F(" | Camera TX="));
        Serial.print(txCameraCount);

        Serial.print(F(" | TX Fail="));
        Serial.println(canSendFailCount);

        Serial.print(F("CAN RX Packets="));
        Serial.print(rxPacketCount);

        Serial.print(F(" | RX Errors="));
        Serial.println(rxErrorCount);

        Serial.print(F("SBUS TX="));
        Serial.println(txSbusCount);

        Serial.print(F("Recording="));
        Serial.println(
            isRecording
                ? F("ON")
                : F("OFF"));

        if (haveAttitude)
        {
            Serial.print(F("Ronin Attitude: Yaw="));

            Serial.print(
                attYaw / 10.0f,
                1);

            Serial.print(F(" Pitch="));

            Serial.print(
                attPitch / 10.0f,
                1);

            Serial.print(F(" Roll="));

            Serial.println(
                attRoll / 10.0f,
                1);
        }
        else
        {
            Serial.println(
                F("Ronin Attitude: NO DATA"));
        }

        Serial.println(
            F("----------------------------------------"));
    }
}
