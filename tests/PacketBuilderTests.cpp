/**
 * @file PacketBuilderTests.cpp
 * @brief Unit tests for PacketBuilder against official DJI sample
 *
 * Official sample (section 3.3):
 * AA 1A 00 03 00 00 00 00 22 11 A2 42 0E 00 20 00 30 00 40 00 01 14 7B 40 97 BE
 *
 * Compile:
 *   g++ -std=c++11 -I../src/protocol -I../src/crc \
 *       PacketBuilderTests.cpp \
 *       ../src/protocol/PacketBuilder.cpp \
 *       ../src/crc/custom_crc16.c \
 *       ../src/crc/custom_crc32.c \
 *       -o packet_test && ./packet_test
 */

#include <cstdio>
#include <cstdint>
#include <cstring>

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

static void printHex(const uint8_t* data, size_t len)
{
    for (size_t i = 0; i < len; ++i) {
        printf("%02X ", data[i]);
    }
    printf("\n");
}

int main()
{
    printf("=== DJI R SDK PacketBuilder Unit Tests ===\n\n");

    // Official expected packet (from PDF section 3.3)
    const uint8_t expected[] = {
        0xAA, 0x1A, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00,
        0x22, 0x11, 0xA2, 0x42, 0x0E, 0x00, 0x20, 0x00,
        0x30, 0x00, 0x40, 0x00, 0x01, 0x14, 0x7B, 0x40,
        0x97, 0xBE
    };
    const size_t expectedLen = sizeof(expected);

    // Build the same packet using PacketBuilder
    // Payload from the sample: yaw=0x0020, roll=0x0030, pitch=0x0040,
    // ctrl_byte=0x01 (absolute), time_for_action=0x14
    PositionControlPayload payload;
    payload.yaw_angle      = 0x0020;   // 3.2°
    payload.roll_angle     = 0x0030;   // 4.8°
    payload.pitch_angle    = 0x0040;   // 6.4°
    payload.ctrl_byte      = 0x01;     // absolute mode, all axes valid
    payload.time_for_action = 0x14;    // 2.0 s

    PacketBuilder builder;
    builder.resetSequence(0x1122);     // so that SEQ becomes 22 11 (little-endian)

    PacketBuffer pkt;
    auto result = builder.buildCommand(
        GimbalCmd::CMDSET,
        GimbalCmd::PositionControl,
        payload,
        ReplyRequirement::ReplyRequired,  // 0x03 in sample → required
        pkt
    );

    CHECK(result.isOk(), "buildCommand succeeded");
    CHECK(pkt.length == expectedLen, "packet length matches official sample (26 bytes)");

    printf("Generated : ");
    printHex(pkt.data, pkt.length);
    printf("Expected  : ");
    printHex(expected, expectedLen);

    bool match = (pkt.length == expectedLen) &&
                 (memcmp(pkt.data, expected, expectedLen) == 0);
    CHECK(match, "full packet matches official DJI sample byte-for-byte");

    // Extra checks on individual fields
    CHECK(pkt.data[0] == 0xAA, "SOF correct");
    CHECK(pkt.data[3] == 0x03, "CmdType correct (reply required)");
    CHECK(pkt.data[8] == 0x22 && pkt.data[9] == 0x11, "SEQ little-endian");
    CHECK(pkt.data[10] == 0xA2 && pkt.data[11] == 0x42, "CRC16 correct");
    CHECK(pkt.data[12] == 0x0E && pkt.data[13] == 0x00, "CmdSet/CmdID correct");
    CHECK(pkt.data[22] == 0x7B && pkt.data[23] == 0x40 &&
          pkt.data[24] == 0x97 && pkt.data[25] == 0xBE, "CRC32 correct");

    printf("\n=== Summary: %d failure(s) ===\n", failures);
    return failures == 0 ? 0 : 1;
}
