/**
 * @file PacketParser.cpp
 * @brief Implementation of PacketParser
 */

#include "PacketParser.h"
#include "../crc/custom_crc16.h"
#include "../crc/custom_crc32.h"

namespace dji {
namespace ronin {

uint16_t PacketParser::readUint16LE(const uint8_t* p) const
{
    return static_cast<uint16_t>(p[0]) |
           (static_cast<uint16_t>(p[1]) << 8);
}

uint32_t PacketParser::readUint32LE(const uint8_t* p) const
{
    return static_cast<uint32_t>(p[0]) |
           (static_cast<uint32_t>(p[1]) << 8) |
           (static_cast<uint32_t>(p[2]) << 16) |
           (static_cast<uint32_t>(p[3]) << 24);
}

uint16_t PacketParser::computeCrc16(const uint8_t* data, size_t len) const
{
    crc16_t crc = crc16_init();
    crc = crc16_update(crc, data, len);
    crc = crc16_finalize(crc);
    return static_cast<uint16_t>(crc);
}

uint32_t PacketParser::computeCrc32(const uint8_t* data, size_t len) const
{
    crc32_t crc = crc32_init();
    crc = crc32_update(crc, data, len);
    crc = crc32_finalize(crc);
    return static_cast<uint32_t>(crc);
}

Result<ParsedPacket> PacketParser::parse(const uint8_t* buffer, size_t length)
{
    if (buffer == nullptr || length < MIN_PACKET_SIZE) {
        return Result<ParsedPacket>::failure(Error::InvalidLength);
    }

    // ----- SOF -----
    if (buffer[OFFSET_SOF] != SOF) {
        return Result<ParsedPacket>::failure(Error::InvalidSof);
    }

    // ----- Ver/Length -----
    const uint16_t verLength = readUint16LE(&buffer[OFFSET_VER_LENGTH]);
    const uint16_t version   = (verLength >> VERSION_SHIFT) & 0x3F;
    const uint16_t frameLen  = verLength & LENGTH_MASK;

    if (version != PROTOCOL_VERSION) {
        return Result<ParsedPacket>::failure(Error::ProtocolVersionMismatch);
    }

    if (frameLen != length || frameLen < MIN_PACKET_SIZE) {
        return Result<ParsedPacket>::failure(Error::InvalidLength);
    }

    // ----- CRC-16 (covers first 10 bytes: SOF .. SEQ) -----
    const uint16_t receivedCrc16 = readUint16LE(&buffer[OFFSET_CRC16]);
    const uint16_t computedCrc16 = computeCrc16(buffer, OFFSET_CRC16);
    if (receivedCrc16 != computedCrc16) {
        return Result<ParsedPacket>::failure(Error::Crc16Mismatch);
    }

    // ----- CRC-32 (covers everything except the last 4 bytes) -----
    const size_t crc32DataLen = frameLen - CRC32_SIZE;
    const uint32_t receivedCrc32 = readUint32LE(&buffer[crc32DataLen]);
    const uint32_t computedCrc32 = computeCrc32(buffer, crc32DataLen);
    if (receivedCrc32 != computedCrc32) {
        return Result<ParsedPacket>::failure(Error::Crc32Mismatch);
    }

    // ----- CmdType -----
    const uint8_t cmdTypeRaw = buffer[OFFSET_CMDTYPE];
    const FrameType frameType = ((cmdTypeRaw & CMDTYPE_FRAME_TYPE_MASK) != 0)
                                ? FrameType::Reply
                                : FrameType::Command;

    ReplyRequirement replyReq = ReplyRequirement::NoReply;
    const uint8_t replyBits = cmdTypeRaw & CMDTYPE_REPLY_TYPE_MASK;
    if (replyBits == 0) {
        replyReq = ReplyRequirement::NoReply;
    } else if (replyBits == 1) {
        replyReq = ReplyRequirement::OptionalReply;
    } else {
        replyReq = ReplyRequirement::ReplyRequired;
    }

    // ----- SEQ -----
    const uint16_t seq = readUint16LE(&buffer[OFFSET_SEQ]);

    // ----- DATA segment -----
    // DATA starts at OFFSET_DATA and ends just before CRC32
    const size_t dataLen = crc32DataLen - OFFSET_DATA;
    if (dataLen < 2) {
        // Must contain at least CmdSet + CmdID
        return Result<ParsedPacket>::failure(Error::InvalidLength);
    }

    const uint8_t cmdSet = buffer[OFFSET_DATA + 0];
    const uint8_t cmdId  = buffer[OFFSET_DATA + 1];
    const uint8_t* payloadPtr = (dataLen > 2) ? &buffer[OFFSET_DATA + 2] : nullptr;
    const size_t payloadLen   = (dataLen > 2) ? (dataLen - 2) : 0;

    // ----- Build result -----
    ParsedPacket result;
    result.header.sof       = SOF;
    result.header.version   = version;
    result.header.length    = frameLen;
    result.header.cmdType   = cmdTypeRaw;
    result.header.frameType = frameType;
    result.header.replyReq  = replyReq;
    result.header.enc       = buffer[OFFSET_ENC];
    result.header.seq       = seq;
    result.header.crc16     = receivedCrc16;

    result.command.cmdSet   = cmdSet;
    result.command.cmdId    = cmdId;
    result.payload          = payloadPtr;
    result.payloadLen       = payloadLen;
    result.crc32            = receivedCrc32;

    // Many reply frames start with a return_code byte
    result.hasReturnCode = false;
    result.returnCode    = ReturnCode::Success;
    if (frameType == FrameType::Reply && payloadLen >= 1) {
        result.hasReturnCode = true;
        result.returnCode    = static_cast<ReturnCode>(payloadPtr[0]);
    }

    return Result<ParsedPacket>::success(result);
}

} // namespace ronin
} // namespace dji
