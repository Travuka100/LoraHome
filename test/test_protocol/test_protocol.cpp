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

int main(int argc, char **argv) {
  UNITY_BEGIN();
  RUN_TEST(test_packet_size);
  RUN_TEST(test_roundtrip);
  RUN_TEST(test_reject_corrupted);
  RUN_TEST(test_reject_bad_length);
  RUN_TEST(test_reject_foreign_magic);
  RUN_TEST(test_encode_invalid);
  RUN_TEST(test_crc_known_vector);
  return UNITY_END();
}
