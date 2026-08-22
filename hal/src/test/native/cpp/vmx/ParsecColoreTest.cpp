// Copyright (c) 2026 WPILib contributors.

#include <gtest/gtest.h>

#include <array>
#include <algorithm>
#include <cstdint>
#include <mutex>
#include <string>
#include <vector>

#include "studica/Colore.h"
#include "studica/Parsec.h"
#include "studica/Transport.h"

namespace {

class MockTransport final : public studica::StudicaVendorTransport {
 public:
  studica::TransportKind Kind() const noexcept override {
    return studica::TransportKind::kUSB;
  }
  bool IsOpen() const noexcept override { return m_open; }

  studica::TransportResult Write(uint32_t, const uint8_t* data,
                                 std::size_t length,
                                 int) noexcept override {
    if (!m_open) return {studica::TransportStatus::kDisconnected, 0};
    if (!data && length != 0)
      return {studica::TransportStatus::kInvalidArgument, 0};
    if (length != 0) m_written.insert(m_written.end(), data, data + length);
    return {studica::TransportStatus::kOk, length};
  }

  studica::TransportResult Read(uint32_t, uint8_t* data, std::size_t capacity,
                                int) noexcept override {
    if (!m_open) return {studica::TransportStatus::kDisconnected, 0};
    if (!data || capacity == 0)
      return {studica::TransportStatus::kInvalidArgument, 0};
    if (m_read.empty()) return {studica::TransportStatus::kTimeout, 0};
    const auto count = std::min(capacity, m_read.size());
    std::copy_n(m_read.begin(), count, data);
    m_read.erase(m_read.begin(), m_read.begin() + count);
    return {studica::TransportStatus::kOk, count};
  }

  void Close() noexcept override { m_open = false; }

  bool m_open = true;
  std::vector<uint8_t> m_read{1, 2, 3, 4};
  std::vector<uint8_t> m_written;
};

}  // namespace

TEST(StudicaTransportTest, RegistrySeparatesCanIdsAndExclusivelyOwnsUsbPaths) {
  studica::StudicaTransportRegistry registry;
  EXPECT_TRUE(registry.ReserveCAN(0, 1, "Parsec"));
  EXPECT_FALSE(registry.ReserveCAN(0, 1, "Colore"));
  EXPECT_TRUE(registry.ReserveCAN(0, 2, "Colore"));
  registry.ReleaseCAN(0, 1);
  EXPECT_TRUE(registry.ReserveCAN(0, 1, "Parsec"));

  EXPECT_TRUE(registry.ReserveUSB("/dev/serial/by-id/parsec", "Parsec"));
  EXPECT_FALSE(registry.ReserveUSB("/dev/serial/by-id/parsec", "Colore"));
  registry.ReleaseUSB("/dev/serial/by-id/parsec");
  EXPECT_TRUE(registry.ReserveUSB("/dev/serial/by-id/parsec", "Colore"));
}

TEST(StudicaTransportTest, ContractPreservesPartialReadWriteAndTimeoutStatus) {
  MockTransport transport;
  const uint8_t tx[] = {9, 8, 7};
  auto write = transport.Write(0, tx, sizeof(tx), 50);
  EXPECT_EQ(write.status, studica::TransportStatus::kOk);
  EXPECT_EQ(write.bytes, 3U);

  uint8_t rx[2]{};
  auto first = transport.Read(0, rx, sizeof(rx), 50);
  EXPECT_EQ(first.status, studica::TransportStatus::kOk);
  EXPECT_EQ(first.bytes, 2U);
  EXPECT_EQ(rx[0], 1);
  EXPECT_EQ(rx[1], 2);
  auto second = transport.Read(0, rx, sizeof(rx), 50);
  EXPECT_EQ(second.status, studica::TransportStatus::kOk);
  EXPECT_EQ(second.bytes, 2U);
  auto timeout = transport.Read(0, rx, sizeof(rx), 50);
  EXPECT_EQ(timeout.status, studica::TransportStatus::kTimeout);
  transport.Close();
  EXPECT_EQ(transport.Read(0, rx, sizeof(rx), 50).status,
            studica::TransportStatus::kDisconnected);
  transport.m_open = true;
  EXPECT_EQ(transport.Write(0, nullptr, 1, 50).status,
            studica::TransportStatus::kInvalidArgument);
  EXPECT_EQ(transport.Write(0, nullptr, 0, 50).status,
            studica::TransportStatus::kOk);
  EXPECT_EQ(transport.Read(0, nullptr, 0, 50).status,
            studica::TransportStatus::kInvalidArgument);
}

TEST(StudicaTransportTest, DisconnectAndReconnectKeepExclusiveOwnership) {
  MockTransport transport;
  transport.Close();
  EXPECT_FALSE(transport.IsOpen());
  EXPECT_EQ(transport.Read(0, nullptr, 0, 1).status,
            studica::TransportStatus::kDisconnected);
  transport.m_open = true;
  EXPECT_TRUE(transport.IsOpen());
  uint8_t value = 0;
  EXPECT_EQ(transport.Read(0, &value, 1, 1).status,
            studica::TransportStatus::kOk);
}

TEST(StudicaParsecVendorTest, FixedSnapshotPreservesSentinelsAndMinDistance) {
  StudicaParsecHandle handle = 0;
  ASSERT_EQ(StudicaParsec_CreateCAN(3, &handle), STUDICA_PARSEC_OK);
  StudicaParsecSnapshot snapshot{};
  snapshot.structSize = sizeof(snapshot);
  snapshot.resolution = STUDICA_PARSEC_RESOLUTION_4;
  snapshot.zoneCount = 16;
  snapshot.connected = 1;
  snapshot.valid = 1;
  std::fill(std::begin(snapshot.distances), std::end(snapshot.distances),
            static_cast<int16_t>(-1));
  snapshot.distances[0] = -2;
  snapshot.distances[1] = 0;
  snapshot.distances[2] = 400;
  ASSERT_EQ(StudicaParsec_SetMockSnapshot(handle, &snapshot),
            STUDICA_PARSEC_OK);

  int16_t value = 0;
  EXPECT_EQ(StudicaParsec_GetZoneDistance(handle, 0, &value),
            STUDICA_PARSEC_OK);
  EXPECT_EQ(value, -2);
  EXPECT_EQ(StudicaParsec_GetZoneDistance(handle, 1, &value),
            STUDICA_PARSEC_OK);
  EXPECT_EQ(value, 0);
  uint8_t valid = 0;
  EXPECT_EQ(StudicaParsec_GetMinDistance(handle, &value, &valid),
            STUDICA_PARSEC_OK);
  EXPECT_TRUE(valid);
  EXPECT_EQ(value, 0);
  EXPECT_EQ(StudicaParsec_GetZoneDistance(handle, 16, &value),
            STUDICA_PARSEC_INVALID_ARGUMENT);
  StudicaParsec_Destroy(handle);
}

TEST(StudicaParsecVendorTest, SupportsEightByEightAndAllInvalidMin) {
  StudicaParsecHandle handle = 0;
  ASSERT_EQ(StudicaParsec_CreateUSB("/dev/mock-parsec", &handle),
            STUDICA_PARSEC_OK);
  StudicaParsecSnapshot snapshot{};
  snapshot.structSize = sizeof(snapshot);
  snapshot.resolution = STUDICA_PARSEC_RESOLUTION_8;
  snapshot.zoneCount = 64;
  snapshot.connected = 1;
  std::fill(std::begin(snapshot.distances), std::end(snapshot.distances),
            static_cast<int16_t>(-1));
  snapshot.distances[63] = -2;
  ASSERT_EQ(StudicaParsec_SetMockSnapshot(handle, &snapshot),
            STUDICA_PARSEC_OK);
  EXPECT_EQ(StudicaParsec_GetZoneCount(handle, &snapshot.zoneCount),
            STUDICA_PARSEC_OK);
  int16_t value = 99;
  uint8_t valid = 1;
  EXPECT_EQ(StudicaParsec_GetMinDistance(handle, &value, &valid),
            STUDICA_PARSEC_OK);
  EXPECT_FALSE(valid);
  EXPECT_EQ(value, 0);
  StudicaParsec_Destroy(handle);
}

TEST(StudicaParsecVendorTest, RejectsMalformedSnapshotAndOversizedRead) {
  StudicaParsecHandle handle = 0;
  ASSERT_EQ(StudicaParsec_CreateCAN(7, &handle), STUDICA_PARSEC_OK);
  StudicaParsecSnapshot malformed{};
  malformed.structSize = sizeof(malformed);
  malformed.resolution = STUDICA_PARSEC_RESOLUTION_4;
  malformed.zoneCount = 15;
  EXPECT_EQ(StudicaParsec_SetMockSnapshot(handle, &malformed),
            STUDICA_PARSEC_INVALID_ARGUMENT);
  int16_t distances[16]{};
  EXPECT_EQ(StudicaParsec_GetDistances(handle, distances, 15),
            STUDICA_PARSEC_BUFFER_TOO_SMALL);
  StudicaParsec_Destroy(handle);
}

TEST(StudicaColoreVendorTest, XYZRgbBrightnessAndMatchingContract) {
  StudicaColoreHandle handle = 0;
  ASSERT_EQ(StudicaColore_CreateUSB("/dev/mock-colore", &handle),
            STUDICA_COLORE_OK);
  StudicaColoreSnapshot snapshot{};
  snapshot.structSize = sizeof(snapshot);
  snapshot.x = 0.30F;
  snapshot.y = 0.40F;
  snapshot.z = 0.20F;
  snapshot.red = 0.2F;
  snapshot.green = 0.4F;
  snapshot.blue = 0.6F;
  snapshot.chromaticityX = 0.3333F;
  snapshot.chromaticityY = 0.4444F;
  snapshot.connected = 1;
  snapshot.valid = 1;
  ASSERT_EQ(StudicaColore_SetMockSnapshot(handle, &snapshot),
            STUDICA_COLORE_OK);

  float value = 0.0F;
  EXPECT_EQ(StudicaColore_GetX(handle, &value), STUDICA_COLORE_OK);
  EXPECT_FLOAT_EQ(value, 0.30F);
  EXPECT_EQ(StudicaColore_GetRed(handle, &value), STUDICA_COLORE_OK);
  EXPECT_FLOAT_EQ(value, 0.2F);
  EXPECT_EQ(StudicaColore_SetBrightness(handle, -1),
            STUDICA_COLORE_INVALID_ARGUMENT);
  EXPECT_EQ(StudicaColore_SetBrightness(handle, 100), STUDICA_COLORE_OK);
  uint8_t brightness = 0;
  EXPECT_EQ(StudicaColore_GetBrightness(handle, &brightness),
            STUDICA_COLORE_OK);
  EXPECT_EQ(brightness, 100);

  EXPECT_EQ(StudicaColore_SetReference(handle, "blue", 0.3333F, 0.4444F,
                                       0.02F),
            STUDICA_COLORE_OK);
  StudicaColoreMatchResult result{};
  EXPECT_EQ(StudicaColore_Match(handle, &result), STUDICA_COLORE_OK);
  EXPECT_STREQ(result.label, "blue");
  EXPECT_GT(result.confidence, 0.9F);
  EXPECT_EQ(StudicaColore_SetReference(handle, "unknown", 0.9F, 0.9F,
                                       0.01F),
            STUDICA_COLORE_OK);
  snapshot.chromaticityX = 0.1F;
  snapshot.chromaticityY = 0.1F;
  StudicaColore_SetMockSnapshot(handle, &snapshot);
  EXPECT_EQ(StudicaColore_Match(handle, &result), STUDICA_COLORE_OK);
  EXPECT_EQ(result.valid, 0);
  StudicaColore_Destroy(handle);
}

TEST(StudicaColoreVendorTest, NearerTightReferenceDoesNotMaskAContainingOne) {
  StudicaColoreHandle handle = 0;
  ASSERT_EQ(StudicaColore_CreateUSB("/dev/mock-colore-thresholds", &handle),
            STUDICA_COLORE_OK);
  StudicaColoreSnapshot snapshot{};
  snapshot.structSize = sizeof(snapshot);
  snapshot.connected = 1;
  snapshot.valid = 1;
  // 0.010 away from "tight", which excludes it; 0.135 away from "loose",
  // which accepts it. "tight" is the nearer of the two.
  snapshot.chromaticityX = 0.31F;
  snapshot.chromaticityY = 0.30F;
  ASSERT_EQ(StudicaColore_SetMockSnapshot(handle, &snapshot),
            STUDICA_COLORE_OK);

  ASSERT_EQ(StudicaColore_SetReference(handle, "tight", 0.30F, 0.30F, 0.005F),
            STUDICA_COLORE_OK);
  ASSERT_EQ(StudicaColore_SetReference(handle, "loose", 0.40F, 0.40F, 0.200F),
            STUDICA_COLORE_OK);

  StudicaColoreMatchResult result{};
  ASSERT_EQ(StudicaColore_Match(handle, &result), STUDICA_COLORE_OK);
  EXPECT_EQ(result.valid, 1);
  EXPECT_STREQ(result.label, "loose");
  EXPECT_GT(result.confidence, 0.0F);
  StudicaColore_Destroy(handle);
}
