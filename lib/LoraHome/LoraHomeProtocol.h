#ifndef LORA_HOME_PROTOCOL_H
#define LORA_HOME_PROTOCOL_H

#include <Arduino.h>
#include <string.h>

#include <cstddef> // offsetof (needed by parse(); was the cause of a hard
                   // compile error in the native test build)

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

// Sentinel for "no reading". The wire field is int16_t, so the sentinel MUST
// be defined as int16_t(-1) == 0xFFFF. Defining it as uint16_t 0xFFFF and
// then static_cast<int16_t>-ing it silently turns the comparison
// "t != (int16_t)TEMP_INVALID" into "t != -1", which was the root cause of
// the receiver always showing "--.-": the sender's guard compared a valid
// encoded temperature (e.g. 2134) against the WRONG sentinel value (-1 vs
// 0xFFFF after cast), so the validity flag could never be set reliably.
constexpr int16_t TEMP_INVALID = static_cast<int16_t>(0xFFFF); // == -1

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
  p.temp1 = TEMP_INVALID;
  p.temp2 = TEMP_INVALID;
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
// CRITICAL: the range check MUST be done on the INTEGER centi-degrees value.
// The previous version compared `scaled` (x100!) against bounds written for
// degrees (-327.67f..327.66f), so EVERY real reading (e.g. 21.34 C -> scaled
// 2134 > 327.66) was rejected as "out of range" and encoded as TEMP_INVALID.
// That is why the receiver showed "--.-" even though the radio reported TX OK:
// the packet itself was perfectly valid, it just carried an invalid sentinel.
inline int16_t encodeTempC(float t) {
  if (!(t == t)) return TEMP_INVALID; // NaN guard
  const long scaled = lroundf(t * 100.0f);
  if (scaled < INT16_MIN || scaled > INT16_MAX - 1) // -1 reserved for sentinel
    return TEMP_INVALID;
  return static_cast<int16_t>(scaled);
}

// True if a wire value is a real reading (not the sentinel). NOTE: -327.68 C
// is physically impossible for DS18B20 (spec range -55..+125 C), so the
// sentinel collision is harmless in practice.
inline bool isTempValid(int16_t raw) { return raw != TEMP_INVALID; }

// Decode the wire field back to deg C (only call when isTempValid()).
inline float decodeTempC(int16_t raw) { return static_cast<float>(raw) / 100.0f; }

// Build a RESPONSE frame from up to MAX_SENSORS readings. This is the ONE
// canonical place where validity flags are set — previously the sender did it
// inline with a broken sentinel comparison and the flags never got raised,
// which made every receiver screen show "--.-" although TX reported success.
// `temps` are deg C; pass NAN (or use tempValid[] = false) for dead channels.
inline void buildResponse(Packet &p, uint8_t fromId, uint8_t toId,
                          uint8_t seq, const float temps[MAX_SENSORS],
                          const uint16_t batteryMv = 0) {
  initPacket(p, fromId, toId, TYPE_RESPONSE, seq);
  for (uint8_t i = 0; i < MAX_SENSORS; ++i) {
    const int16_t enc = encodeTempC(temps[i]);
    if (i == 0) p.temp1 = enc; else p.temp2 = enc;
    if (isTempValid(enc)) {
      p.flags |= (i == 0) ? FLAG_TEMP1_VALID : FLAG_TEMP2_VALID;
    }
  }
  p.battery_mv = batteryMv;
  finalize(p);
}

} // namespace lorahome

#endif // LORA_HOME_PROTOCOL_H
