/**
 * @file PacketParserTests.cpp
 * @brief Unit tests for PacketParser
 *
 * Uses the official sample packet from the DJI documentation.
 */

#include <cstdio>
#include <cstdint>
#include <cstring>

#include "PacketParser.h"
#include "PacketBuilder.h"
#include "RoninCommands.h"
#include "RoninPayload.h"
#include "RoninEnums.h"

using namespace dji::ronin;

static int failures = 0;

#define CHECK(cond, msg) \
    do { \
        if (!(cond)) { \
            printf("FAIL: %s\n", msg); \
            ++failures; \
        } else { \
            printf("PASS: %s\n", msg); \
        } \
    } while (0)

int main()
{
    printf("=== DJI R SDK PacketParser Unit Tests ===\n\n");

    // Official sample packet
    const uint8_t sample[] = {
        0xAA, 0x1A, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00,
        0x22, 0x11, 0xA2, 0x42, 0x0E, 0x00, 0x20, 0x00,
        0x30, 0x00, 0x40, 0x00, 0x01, 0x14, 0x7B, 0x40,
        0x97, 0xBE
    };

    PacketParser parser;
    auto result = parser.parse(sample, sizeof(sample));

    CHECK(result.isOk(), "parse succeeded on official sample");

    if (result.isOk()) {
        const ParsedPacket& p = result.value();

        CHECK(p.header.sof == 0xAA, "SOF correct");
        CHECK(p.header.length == 26, "length correct");
        CHECK(p.header.seq == 0x1122, "SEQ correct");
        CHECK(p.header.crc16 == 0x42A2, "CRC16 field correct");
        CHECK(p.header.frameType == FrameType::Command, "frame type = Command");
        CHECK(p.command.cmdSet == 0x0E, "CmdSet = Gimbal");
        CHECK(p.command.cmdId == 0x00, "CmdID = PositionControl");
        CHECK(p.payloadLen == 8, "payload length = 8");
        CHECK(p.crc32 == 0xBE97407B, "CRC32 field correct");

        // Payload content
        if (p.payloadLen >= 8) {
            int16_t yaw   = static_cast<int16_t>(p.payload[0] | (p.payload[1] << 8));
            int16_t roll  = static_cast<int16_t>(p.payload[2] | (p.payload[3] << 8));
            int16_t pitch = static_cast<int16_t>(p.payload[4] | (p.payload[5] << 8));
            uint8_t ctrl  = p.payload[6];
            uint8_t time  = p.payload[7];

            CHECK(yaw   == 0x0020, "yaw payload correct");
            CHECK(roll  == 0x0030, "roll payload correct");
            CHECK(pitch == 0x0040, "pitch payload correct");
            CHECK(ctrl  == 0x01,   "ctrl_byte correct");
            CHECK(time  == 0x14,   "time_for_action correct");
        }
    }

    // ----- Negative tests -----
    // Bad SOF
    uint8_t badSof[26];
    memcpy(badSof, sample, 26);
    badSof[0] = 0x55;
    CHECK(parser.parse(badSof, 26).isError(), "rejects bad SOF");

    // Bad CRC16
    uint8_t badCrc16[26];
    memcpy(badCrc16, sample, 26);
    badCrc16[10] ^= 0xFF;
    CHECK(parser.parse(badCrc16, 26).error() == Error::Crc16Mismatch, "detects CRC16 mismatch");

    // Bad CRC32
    uint8_t badCrc32[26];
    memcpy(badCrc32, sample, 26);
    badCrc32[25] ^= 0xFF;
    CHECK(parser.parse(badCrc32, 26).error() == Error::Crc32Mismatch, "detects CRC32 mismatch");

    // Too short
    CHECK(parser.parse(sample, 10).error() == Error::InvalidLength, "rejects too-short buffer");

    // Round-trip: build → parse
    PositionControlPayload payload;
    payload.yaw_angle       = 100;
    payload.roll_angle      = -50;
    payload.pitch_angle     = 200;
    payload.ctrl_byte       = makePositionCtrlByte(ControlMode::Absolute,
                                                   AxisValid::Valid,
                                                   AxisValid::Valid,
                                                   AxisValid::Valid);
    payload.time_for_action = 15;

    PacketBuilder builder;
    builder.resetSequence(0xABCD);
    PacketBuffer pkt;
    auto buildRes = builder.buildCommand(GimbalCmd::CMDSET,
                                         GimbalCmd::PositionControl,
                                         payload,
                                         ReplyRequirement::NoReply,
                                         pkt);
    CHECK(buildRes.isOk(), "round-trip build succeeded");

    auto parseRes = parser.parse(pkt);
    CHECK(parseRes.isOk(), "round-trip parse succeeded");
    if (parseRes.isOk()) {
        CHECK(parseRes.value().command.cmdId == GimbalCmd::PositionControl, "round-trip CmdID");
        CHECK(parseRes.value().header.seq == 0xABCD, "round-trip SEQ");
    }

    printf("\n=== Summary: %d failure(s) ===\n", failures);
    return failures == 0 ? 0 : 1;
}
