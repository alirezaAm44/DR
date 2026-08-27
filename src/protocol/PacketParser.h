/**
 * @file PacketParser.h
 * @brief Parses and validates incoming DJI R SDK packets
 *
 * Completely independent of Arduino / CAN hardware.
 */

#ifndef DJI_RONIN_PACKET_PARSER_H
#define DJI_RONIN_PACKET_PARSER_H

#include <stdint.h>
#include <stddef.h>
#include "RoninPacket.h"
#include "RoninConstants.h"
#include "RoninEnums.h"
#include "Result.h"

namespace dji {
namespace ronin {

/**
 * Result of a successful parse.
 * Contains views into the original buffer (no copy of payload).
 */
struct ParsedPacket {
    PacketHeaderView header;
    CommandId        command;
    const uint8_t*   payload;       // points into the original buffer (after CmdSet+CmdID)
    size_t           payloadLen;    // bytes after CmdSet+CmdID
    uint32_t         crc32;         // the CRC32 that was present in the packet
    ReturnCode       returnCode;    // only valid for reply frames that contain one
    bool             hasReturnCode;
};

class PacketParser {
public:
    PacketParser() = default;

    /**
     * Parse a complete raw packet buffer.
     *
     * Performs:
     *  - SOF check
     *  - Length consistency check
     *  - CRC16 validation (header)
     *  - CRC32 validation (whole frame)
     *  - Extraction of CmdSet / CmdID / payload
     *
     * @param buffer  Raw bytes received
     * @param length  Number of bytes available
     * @return Result containing ParsedPacket on success, or Error on failure
     */
    Result<ParsedPacket> parse(const uint8_t* buffer, size_t length);

    /**
     * Convenience overload for PacketBuffer.
     */
    Result<ParsedPacket> parse(const PacketBuffer& packet) {
        return parse(packet.data, packet.length);
    }

private:
    uint16_t readUint16LE(const uint8_t* p) const;
    uint32_t readUint32LE(const uint8_t* p) const;

    uint16_t computeCrc16(const uint8_t* data, size_t len) const;
    uint32_t computeCrc32(const uint8_t* data, size_t len) const;
};

} // namespace ronin
} // namespace dji

#endif // DJI_RONIN_PACKET_PARSER_H
