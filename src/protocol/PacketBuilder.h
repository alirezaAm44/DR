/**
 * @file PacketBuilder.h
 * @brief Builds complete DJI R SDK packets (header + data + CRC16 + CRC32)
 *
 * Completely independent of Arduino / CAN hardware.
 */

#ifndef DJI_RONIN_PACKET_BUILDER_H
#define DJI_RONIN_PACKET_BUILDER_H

#include <stdint.h>
#include <stddef.h>
#include "RoninPacket.h"
#include "RoninConstants.h"
#include "RoninEnums.h"
#include "Result.h"

namespace dji {
namespace ronin {

class PacketBuilder {
public:
    PacketBuilder();

    /**
     * Reset sequence counter (call once at startup).
     */
    void resetSequence(uint16_t start = 0);

    /**
     * Current sequence number (for debugging / tests).
     */
    uint16_t currentSequence() const { return seq_; }

    /**
     * Build a command packet.
     *
     * @param cmdSet   Command set (e.g. 0x0E)
     * @param cmdId    Command ID
     * @param payload  Pointer to payload bytes (may be nullptr if payloadLen == 0)
     * @param payloadLen  Length of payload
     * @param replyReq Reply requirement
     * @param out      Output packet buffer
     * @return Result indicating success or error
     */
    Result<void> buildCommand(uint8_t cmdSet,
                              uint8_t cmdId,
                              const uint8_t* payload,
                              size_t payloadLen,
                              ReplyRequirement replyReq,
                              PacketBuffer& out);

    /**
     * Convenience overload for typed payloads.
     */
    template <typename PayloadT>
    Result<void> buildCommand(uint8_t cmdSet,
                              uint8_t cmdId,
                              const PayloadT& payload,
                              ReplyRequirement replyReq,
                              PacketBuffer& out)
    {
        return buildCommand(cmdSet, cmdId,
                            reinterpret_cast<const uint8_t*>(&payload),
                            sizeof(PayloadT),
                            replyReq, out);
    }

private:
    uint16_t seq_;

    void writeUint16LE(uint8_t* buf, uint16_t value);
    void writeUint32LE(uint8_t* buf, uint32_t value);

    uint16_t computeCrc16(const uint8_t* data, size_t len);
    uint32_t computeCrc32(const uint8_t* data, size_t len);
};

} // namespace ronin
} // namespace dji

#endif // DJI_RONIN_PACKET_BUILDER_H
