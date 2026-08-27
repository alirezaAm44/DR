/**
 * @file RoninConstants.h
 * @brief Protocol constants extracted exactly from DJI R SDK Protocol and User Interface v2.5
 *
 * All values are taken directly from the official documentation.
 * Do not invent or guess any values.
 */

#ifndef DJI_RONIN_CONSTANTS_H
#define DJI_RONIN_CONSTANTS_H

#include <stdint.h>

namespace dji {
namespace ronin {

// ---------------------------------------------------------------------------
// Frame Header / SOF
// ---------------------------------------------------------------------------
/** Start of Frame. Fixed value defined in protocol section 2.2 */
constexpr uint8_t SOF = 0xAA;

// ---------------------------------------------------------------------------
// Packet Structure Sizes (bytes)
// ---------------------------------------------------------------------------
/** Size of SOF field */
constexpr uint8_t SOF_SIZE = 1;

/** Size of Ver/Length field */
constexpr uint8_t VER_LENGTH_SIZE = 2;

/** Size of CmdType field */
constexpr uint8_t CMD_TYPE_SIZE = 1;

/** Size of ENC field */
constexpr uint8_t ENC_SIZE = 1;

/** Size of RES (reserved) field */
constexpr uint8_t RES_SIZE = 3;

/** Size of SEQ (sequence number) field */
constexpr uint8_t SEQ_SIZE = 2;

/** Size of CRC-16 field (frame header check) */
constexpr uint8_t CRC16_SIZE = 2;

/** Size of CRC-32 field (entire frame check) */
constexpr uint8_t CRC32_SIZE = 4;

/**
 * Fixed header size before DATA segment.
 * SOF(1) + Ver/Length(2) + CmdType(1) + ENC(1) + RES(3) + SEQ(2) + CRC16(2) = 12
 */
constexpr uint8_t HEADER_SIZE = 12;

/**
 * Minimum packet size (header + empty DATA + CRC32)
 * 12 + 0 + 4 = 16
 */
constexpr uint8_t MIN_PACKET_SIZE = HEADER_SIZE + CRC32_SIZE;

/**
 * Maximum practical DATA payload size.
 * Reduced to 64 for Arduino Nano (2 KB SRAM) compatibility.
 * All official DJI R SDK commands fit comfortably within this limit
 * (typical packets are 20–30 bytes total).
 */
constexpr uint16_t MAX_DATA_SIZE = 64;

/** Maximum complete packet size */
constexpr uint16_t MAX_PACKET_SIZE = HEADER_SIZE + MAX_DATA_SIZE + CRC32_SIZE;

// ---------------------------------------------------------------------------
// Version / Length field bit layout (section 2.2)
// ---------------------------------------------------------------------------
/** Version is stored in bits [15:10] of the 16-bit Ver/Length field. Default = 0 */
constexpr uint8_t PROTOCOL_VERSION = 0;

/** Mask for length part (bits [9:0]) */
constexpr uint16_t LENGTH_MASK = 0x03FF;

/** Shift for version part */
constexpr uint8_t VERSION_SHIFT = 10;

// ---------------------------------------------------------------------------
// CmdType bit layout (section 2.2)
// ---------------------------------------------------------------------------
/** Reply type bits [4:0] */
constexpr uint8_t CMDTYPE_REPLY_TYPE_MASK = 0x1F;
constexpr uint8_t CMDTYPE_NO_REPLY        = 0x00;  // No reply required
constexpr uint8_t CMDTYPE_OPTIONAL_REPLY  = 0x01;  // Can reply or not
// 2-31 = Reply required

/** Frame type bit [5] */
constexpr uint8_t CMDTYPE_FRAME_TYPE_MASK = 0x20;
constexpr uint8_t CMDTYPE_COMMAND_FRAME   = 0x00;  // bit5 = 0
constexpr uint8_t CMDTYPE_REPLY_FRAME     = 0x20;  // bit5 = 1

/** Reserved bits [7:6] must be 0 */
constexpr uint8_t CMDTYPE_RESERVED_MASK = 0xC0;

// ---------------------------------------------------------------------------
// ENC field (section 2.2)
// ---------------------------------------------------------------------------
/** Encryption type bits [7:5] */
constexpr uint8_t ENC_TYPE_MASK     = 0xE0;
constexpr uint8_t ENC_TYPE_NONE     = 0x00;  // Unencrypted
constexpr uint8_t ENC_TYPE_AES256   = 0x20;  // AES256 (not used in current SDK)

/** Supplementary length for encryption alignment bits [4:0] */
constexpr uint8_t ENC_SUPP_LEN_MASK = 0x1F;

// ---------------------------------------------------------------------------
// Command Sets (section 2.3.1)
// ---------------------------------------------------------------------------
constexpr uint8_t CMDSET_GIMBAL  = 0x0E;  // Handheld gimbal commands
constexpr uint8_t CMDSET_CAMERA  = 0x0D;  // Third-party camera commands

// ---------------------------------------------------------------------------
// Device IDs (section 2.3.3)
// ---------------------------------------------------------------------------
constexpr uint32_t DEVICE_ID_RESERVED         = 0x00000000;
constexpr uint32_t DEVICE_ID_DJI_R_SDK        = 0x00000001;
constexpr uint32_t DEVICE_ID_REMOTE_CONTROLLER = 0x00000002;

// ---------------------------------------------------------------------------
// CAN Communication Parameters (section 3.1)
// ---------------------------------------------------------------------------
/** Baud rate used by DJI RS 2 */
constexpr uint32_t CAN_BAUD_RATE = 1000000;  // 1 Mbps

/**
 * CAN IDs when communicating with DJI RS 2
 * (from Figure 37 / 38)
 *
 * When PC / third-party device is the sender:
 *   Tx = 0x223, Rx = 0x222
 *
 * When gimbal is the sender the IDs are swapped.
 */
constexpr uint32_t CAN_ID_TX = 0x223;  // Third-party → Gimbal
constexpr uint32_t CAN_ID_RX = 0x222;  // Gimbal → Third-party

// ---------------------------------------------------------------------------
// CRC Parameters (section 3.2 / Figure 40)
// ---------------------------------------------------------------------------
// CRC-16
constexpr uint16_t CRC16_POLY   = 0x8005;
constexpr uint16_t CRC16_INIT   = 0xC55C;   // XorIn
constexpr uint16_t CRC16_XOROUT = 0x0000;
// ReflectIn = true, ReflectOut = true  (handled inside custom_crc16)

// CRC-32
constexpr uint32_t CRC32_POLY   = 0x04C11DB7;
constexpr uint32_t CRC32_INIT   = 0xC55C0000; // XorIn
constexpr uint32_t CRC32_XOROUT = 0x00000000;
// ReflectIn = true, ReflectOut = true  (handled inside custom_crc32)

// ---------------------------------------------------------------------------
// Useful offsets inside a complete packet buffer
// ---------------------------------------------------------------------------
constexpr uint8_t OFFSET_SOF        = 0;
constexpr uint8_t OFFSET_VER_LENGTH = 1;
constexpr uint8_t OFFSET_CMDTYPE    = 3;
constexpr uint8_t OFFSET_ENC        = 4;
constexpr uint8_t OFFSET_RES        = 5;
constexpr uint8_t OFFSET_SEQ        = 8;
constexpr uint8_t OFFSET_CRC16      = 10;
constexpr uint8_t OFFSET_DATA       = 12;
// CRC32 is at the end: packet_len - 4

} // namespace ronin
} // namespace dji

#endif // DJI_RONIN_CONSTANTS_H
