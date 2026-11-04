#ifndef LORA_HOME_PROTOCOL_H
#define LORA_HOME_PROTOCOL_H

#include <Arduino.h>
#include <string.h>

// ============================================================================
//  LoraHome telemetry protocol (SX1262 / RadioLib)
//
//  Wire format (little-endian, fixed size, CRC-protected):
//    offset 0 : uint8_t  magic      ('H' = 0x48)
//    offset 1 : uint8_t  version    (PROTOCOL_VERSION)
//    offset 2 : uint8_t  from_id
//    offset 3 : uint8_t  to_id
//    offset 4 : uint8_t  type       (PacketType)
//    offset 5 : uint8_t  seq        (rolling sequence number, wraps at 255)
//    offset 6 : int16_t  temp1_x100 (0xFFFF = sensor error / not present)
//    offset 8 : int16_t  temp2_x100
//    offset 10: uint16_t battery_mv  (receiver fills its own; sender sends 0)
//    offset 12: uint8_t  flags       (bit0: temp1 valid, bit1: temp2 valid)
//    offset 13: uint8_t  reserved
//    offset 14: uint16_t crc16       (CRC-CCITT/XMODES over bytes 0..13)
//  Total: 16 bytes.
//
//  Reliability layers:
//   - radio hardware CRC (packets with bad CRC are dropped by the transceiver);
//   - software CRC-16 as a second layer (catches garbage / wrong-length RX);
//   - explicit addressing: magic + version + from/to ids reject foreign pkts;
//   - duplicate suppression via (from_id, seq) pair;
//   - application-level ACK: RESPONSE echoes the REQUEST's seq, so the
//     sender can match an answer to its own request and retransmit otherwise.
// ============================================================================

namespace lorahome {

constexpr uint8_t MAGIC = 'H';
constexpr uint8_t PROTOCOL_VERSION = 0x02;
constexpr uint8_t MAX_SENSORS = 2;
constexpr uint16_t TEMP_INVALID = 0xFFFF; // sentinel for "no reading"

constexpr uint8_t FLAG_TEMP1_VALID = 0x01;
constexpr uint8_t FLAG_TEMP2_VALID = 0x02;

enum PacketType : uint8_t {
  TYPE_REQUEST = 1, // master -> slave: send me telemetry
  TYPE_RESPONSE = 2 // slave -> master: telemetry payload (ACK for request)
};

struct Packet {
  uint8_t magic;
  uint8_t version;
  uint8_t from_id;
  uint8_t to_id;
  uint8_t type;
  uint8_t seq;
  int16_t temp1; // x100 deg C
  int16_t temp2; // x100 deg C
  uint16_t battery_mv;
  uint8_t flags;
  uint8_t reserved;
  uint16_t crc;
} __attribute__((packed));

static_assert(sizeof(Packet) == 16, "Packet must be exactly 16 bytes");

inline void initPacket(Packet &p, uint8_t fromId, uint8_t toId,
                       PacketType type, uint8_t seq) {
  memset(&p, 0, sizeof(p));
  p.magic = MAGIC;
  p.version = PROTOCOL_VERSION;
  p.from_id = fromId;
  p.to_id = toId;
  p.type = static_cast<uint8_t>(type);
  p.seq = seq;
  p.temp1 = static_cast<int16_t>(TEMP_INVALID);
  p.temp2 = static_cast<int16_t>(TEMP_INVALID);
}

// CRC-16/XMODES (poly 0x1021, init 0x0000).
inline uint16_t crc16(const uint8_t *data, size_t len) {
  uint16_t crc = 0;
  for (size_t i = 0; i < len; ++i) {
    crc ^= static_cast<uint16_t>(data[i]) << 8;
    for (uint8_t bit = 0; bit < 8; ++bit) {
      crc = (crc & 0x8000) ? static_cast<uint16_t>((crc << 1) ^ 0x1021)
                           : static_cast<uint16_t>(crc << 1);
    }
  }
  return crc;
}

inline void finalize(Packet &p) {
  p.crc = 0;
  p.crc = crc16(reinterpret_cast<const uint8_t *>(&p),
                sizeof(p) - sizeof(p.crc));
}

// Full validation: length, magic, version, CRC. Returns true if packet is good.
inline bool parse(const uint8_t *buf, size_t len, Packet &out) {
  if (len != sizeof(Packet)) return false;
  memcpy(&out, buf, sizeof(Packet));
  if (out.magic != MAGIC) return false;
  if (out.version != PROTOCOL_VERSION) return false;
  const uint16_t saved = out.crc;
  const uint16_t calc =
      crc16(reinterpret_cast<const uint8_t *>(&out), offsetof(Packet, crc));
  return saved == calc;
}

// Encode temperature (deg C) into the wire field. NaN/out-of-range -> invalid.
inline int16_t encodeTempC(float t) {
  if (!(t == t)) return static_cast<int16_t>(TEMP_INVALID); // NaN guard
  const float scaled = t * 100.0f;
  if (scaled < -327.67f || scaled > 327.66f)
    return static_cast<int16_t>(TEMP_INVALID);
  return static_cast<int16_t>(lroundf(scaled));
}

} // namespace lorahome

#endif // LORA_HOME_PROTOCOL_H
