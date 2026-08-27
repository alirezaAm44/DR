/**
 * @file PacketBuilder.cpp
 * @brief Implementation of PacketBuilder
 */

#include "PacketBuilder.h"
#include "../crc/custom_crc16.h"
#include "../crc/custom_crc32.h"

namespace dji {
namespace ronin {

PacketBuilder::PacketBuilder()
    : seq_(0)
{
}

void PacketBuilder::resetSequence(uint16_t start)
{
    seq_ = start;
}

void PacketBuilder::writeUint16LE(uint8_t* buf, uint16_t value)
{
    buf[0] = static_cast<uint8_t>(value & 0xFF);
    buf[1] = static_cast<uint8_t>((value >> 8) & 0xFF);
}

void PacketBuilder::writeUint32LE(uint8_t* buf, uint32_t value)
{
    buf[0] = static_cast<uint8_t>(value & 0xFF);
    buf[1] = static_cast<uint8_t>((value >> 8) & 0xFF);
    buf[2] = static_cast<uint8_t>((value >> 16) & 0xFF);
    buf[3] = static_cast<uint8_t>((value >> 24) & 0xFF);
}

uint16_t PacketBuilder::computeCrc16(const uint8_t* data, size_t len)
{
    crc16_t crc = crc16_init();           // 0x3AA3
    crc = crc16_update(crc, data, len);
    crc = crc16_finalize(crc);
    return static_cast<uint16_t>(crc);
}

uint32_t PacketBuilder::computeCrc32(const uint8_t* data, size_t len)
{
    crc32_t crc = crc32_init();           // 0x00003AA3
    crc = crc32_update(crc, data, len);
    crc = crc32_finalize(crc);
    return static_cast<uint32_t>(crc);
}

Result<void> PacketBuilder::buildCommand(uint8_t cmdSet,
                                         uint8_t cmdId,
                                         const uint8_t* payload,
                                         size_t payloadLen,
                                         ReplyRequirement replyReq,
                                         PacketBuffer& out)
{
    // DATA segment = CmdSet(1) + CmdID(1) + payload
    const size_t dataLen = 2 + payloadLen;
    const size_t totalLen = HEADER_SIZE + dataLen + CRC32_SIZE;

    if (totalLen > PACKET_BUFFER_SIZE) {
        return Result<void>::failure(Error::PacketTooLarge);
    }

    out.clear();
    out.length = static_cast<uint16_t>(totalLen);

    uint8_t* p = out.data;

    // ----- SOF -----
    p[OFFSET_SOF] = SOF;

    // ----- Ver/Length (LSB first) -----
    // bits [15:10] = version (0), bits [9:0] = length
    uint16_t verLength = (static_cast<uint16_t>(PROTOCOL_VERSION) << VERSION_SHIFT)
                       | (static_cast<uint16_t>(totalLen) & LENGTH_MASK);
    writeUint16LE(&p[OFFSET_VER_LENGTH], verLength);

    // ----- CmdType -----
    // bits [4:0] = reply type
    //   0     = no reply
    //   1     = optional
    //   2-31  = reply required
    // bit 5   = frame type (0 = command)
    // bits [7:6] = reserved (0)
    uint8_t cmdType = 0;
    if (replyReq == ReplyRequirement::NoReply) {
        cmdType = 0x00;
    } else if (replyReq == ReplyRequirement::OptionalReply) {
        cmdType = 0x01;
    } else {
        // Use 0x03 to match the official sample packet in the documentation
        cmdType = 0x03;
    }
    p[OFFSET_CMDTYPE] = cmdType;

    // ----- ENC (unencrypted) -----
    p[OFFSET_ENC] = ENC_TYPE_NONE;

    // ----- RES (3 bytes reserved = 0) -----
    p[OFFSET_RES + 0] = 0;
    p[OFFSET_RES + 1] = 0;
    p[OFFSET_RES + 2] = 0;

    // ----- SEQ -----
    writeUint16LE(&p[OFFSET_SEQ], seq_);
    ++seq_;  // auto-increment, wraps naturally

    // ----- DATA: CmdSet + CmdID + payload -----
    p[OFFSET_DATA + 0] = cmdSet;
    p[OFFSET_DATA + 1] = cmdId;
    if (payload && payloadLen > 0) {
        for (size_t i = 0; i < payloadLen; ++i) {
            p[OFFSET_DATA + 2 + i] = payload[i];
        }
    }

    // ----- CRC-16 (covers SOF .. SEQ inclusive, i.e. first 10 bytes) -----
    // Protocol: CRC-16 is frame header check → bytes 0..9
    uint16_t crc16 = computeCrc16(p, OFFSET_CRC16);  // 10 bytes
    writeUint16LE(&p[OFFSET_CRC16], crc16);

    // ----- CRC-32 (covers entire frame except the CRC-32 itself) -----
    // i.e. SOF .. end of DATA
    const size_t crc32Len = HEADER_SIZE + dataLen;  // up to but not including CRC32
    uint32_t crc32 = computeCrc32(p, crc32Len);
    writeUint32LE(&p[crc32Len], crc32);

    return Result<void>::success();
}

} // namespace ronin
} // namespace dji
