// Native unit tests for the LoraHome protocol (pio test -e native).
#include <LoraHomeProtocol.h>
#include <unity.h>

using namespace lorahome;

void setUp(void) {}
void tearDown(void) {}

static Packet makeResponse() {
  Packet p;
  initPacket(p, 0x02, 0x01, TYPE_RESPONSE, 42);
  p.temp1 = encodeTempC(21.34f);
  p.temp2 = encodeTempC(-5.0f);
  p.flags = FLAG_TEMP1_VALID | FLAG_TEMP2_VALID;
  finalize(p);
  return p;
}

void test_packet_size(void) { TEST_ASSERT_EQUAL_UINT16(16, sizeof(Packet)); }

void test_roundtrip(void) {
  Packet tx = makeResponse();
  uint8_t buf[sizeof(Packet)];
  memcpy(buf, &tx, sizeof(buf));

  Packet rx;
  TEST_ASSERT_TRUE(parse(buf, sizeof(buf), rx));
  TEST_ASSERT_EQUAL_UINT8(0x02, rx.from_id);
  TEST_ASSERT_EQUAL_UINT8(0x01, rx.to_id);
  TEST_ASSERT_EQUAL_UINT8(TYPE_RESPONSE, rx.type);
  TEST_ASSERT_EQUAL_UINT8(42, rx.seq);
  TEST_ASSERT_EQUAL_INT16(2134, rx.temp1);
  TEST_ASSERT_EQUAL_INT16(-500, rx.temp2);
}

void test_reject_corrupted(void) {
  Packet tx = makeResponse();
  uint8_t buf[sizeof(Packet)];
  memcpy(buf, &tx, sizeof(buf));
  buf[7] ^= 0xFF; // flip a payload bit -> CRC must catch it
  Packet rx;
  TEST_ASSERT_FALSE(parse(buf, sizeof(buf), rx));
}

void test_reject_bad_length(void) {
  Packet tx = makeResponse();
  uint8_t buf[sizeof(Packet)];
  memcpy(buf, &tx, sizeof(buf));
  Packet rx;
  TEST_ASSERT_FALSE(parse(buf, sizeof(buf) - 1, rx));
}

void test_reject_foreign_magic(void) {
  uint8_t buf[sizeof(Packet)] = {};
  Packet rx;
  TEST_ASSERT_FALSE(parse(buf, sizeof(buf), rx));
}

void test_encode_invalid(void) {
  TEST_ASSERT_EQUAL_INT16((int16_t)TEMP_INVALID, encodeTempC(NAN));
  TEST_ASSERT_EQUAL_INT16((int16_t)TEMP_INVALID, encodeTempC(-1270.0f));
  TEST_ASSERT_EQUAL_INT16((int16_t)TEMP_INVALID, encodeTempC(1000.0f));
  TEST_ASSERT_EQUAL_INT16(2500, encodeTempC(25.0f));
}

void test_crc_known_vector(void) {
  // "123456789" -> CRC-16/XMODES check value 0x31C3
  const uint8_t v[] = {'1','2','3','4','5','6','7','8','9'};
  TEST_ASSERT_EQUAL_UINT16(0x31C3, crc16(v, sizeof(v)));
}

// ---- Regression: receiver must show numbers, not "--.-", for live sensors.
// Root cause of the original bug: TEMP_INVALID was uint16_t 0xFFFF while the
// wire field is int16_t; the sender's inline guard compared an encoded temp
// against the wrong sentinel value, so FLAG_TEMP*_VALID was never set even
// though TX reported success. buildResponse() is now the single canonical
// place where flags are raised — these tests pin that contract.
void test_build_response_flags_set_for_live_sensors(void) {
  const float temps[2] = {21.34f, -5.0f};
  Packet resp;
  buildResponse(resp, 0x02, 0x01, 7, temps);

  TEST_ASSERT_EQUAL_UINT8(FLAG_TEMP1_VALID | FLAG_TEMP2_VALID, resp.flags);
  TEST_ASSERT_EQUAL_INT16(2134, resp.temp1);
  TEST_ASSERT_EQUAL_INT16(-500, resp.temp2);

  // Full over-the-air simulation: serialize, parse on the other end.
  uint8_t buf[sizeof(Packet)];
  memcpy(buf, &resp, sizeof(buf));
  Packet rx;
  TEST_ASSERT_TRUE(parse(buf, sizeof(buf), rx));
  TEST_ASSERT_EQUAL_UINT8(FLAG_TEMP1_VALID | FLAG_TEMP2_VALID, rx.flags);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 21.34f, decodeTempC(rx.temp1));
  TEST_ASSERT_FLOAT_WITHIN(0.01f, -5.0f, decodeTempC(rx.temp2));
}

void test_build_response_dead_channel_no_flag(void) {
  const float temps[2] = {21.34f, NAN}; // sensor 2 disconnected
  Packet resp;
  buildResponse(resp, 0x02, 0x01, 8, temps);

  TEST_ASSERT_EQUAL_UINT8(FLAG_TEMP1_VALID, resp.flags);
  TEST_ASSERT_EQUAL_INT16(TEMP_INVALID, resp.temp2);
  TEST_ASSERT_FALSE(isTempValid(resp.temp2));
  TEST_ASSERT_TRUE(isTempValid(resp.temp1));
}

void test_temp_invalid_is_int16_sentinel(void) {
  // The sentinel MUST be usable in int16_t comparisons without casts.
  TEST_ASSERT_EQUAL_INT16(-1, TEMP_INVALID);
  TEST_ASSERT_EQUAL_INT16(TEMP_INVALID, encodeTempC(NAN));
  TEST_ASSERT_FALSE(isTempValid(encodeTempC(NAN)));
  TEST_ASSERT_TRUE(isTempValid(encodeTempC(0.0f))); // 0.0 C is a REAL reading
}

int main(int argc, char **argv) {
  UNITY_BEGIN();
  RUN_TEST(test_packet_size);
  RUN_TEST(test_roundtrip);
  RUN_TEST(test_reject_corrupted);
  RUN_TEST(test_reject_bad_length);
  RUN_TEST(test_reject_foreign_magic);
  RUN_TEST(test_encode_invalid);
  RUN_TEST(test_crc_known_vector);
  RUN_TEST(test_build_response_flags_set_for_live_sensors);
  RUN_TEST(test_build_response_dead_channel_no_flag);
  RUN_TEST(test_temp_invalid_is_int16_sentinel);
  return UNITY_END();
}
