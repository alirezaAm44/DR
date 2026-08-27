/**
 * @file RoninPacket.h
 * @brief Packet structures and buffer definitions for DJI R SDK protocol
 */

#ifndef DJI_RONIN_PACKET_H
#define DJI_RONIN_PACKET_H

#include <stdint.h>
#include <stddef.h>
#include "RoninConstants.h"
#include "RoninEnums.h"

namespace dji {
namespace ronin {

/**
 * Maximum size of a complete packet buffer.
 * Header (12) + Data (up to MAX_DATA_SIZE) + CRC32 (4)
 */
constexpr size_t PACKET_BUFFER_SIZE = MAX_PACKET_SIZE;

/**
 * Raw packet buffer. Protocol layer works only with this.
 * Transport layer receives/sends CAN frames built from it.
 */
struct PacketBuffer {
    uint8_t data[PACKET_BUFFER_SIZE];
    uint16_t length;   // total bytes currently used

    void clear() {
        length = 0;
        for (size_t i = 0; i < PACKET_BUFFER_SIZE; ++i) {
            data[i] = 0;
        }
    }
};

/**
 * Lightweight view of a parsed packet header.
 * Does not own memory.
 */
struct PacketHeaderView {
    uint8_t  sof;
    uint16_t version;
    uint16_t length;          // entire frame length
    uint8_t  cmdType;
    FrameType frameType;
    ReplyRequirement replyReq;
    uint8_t  enc;
    uint16_t seq;
    uint16_t crc16;
    // DATA starts at OFFSET_DATA
    // CRC32 is at the end
};

/**
 * Parsed command identification inside DATA segment.
 */
struct CommandId {
    uint8_t cmdSet;
    uint8_t cmdId;
};

} // namespace ronin
} // namespace dji

#endif // DJI_RONIN_PACKET_H
