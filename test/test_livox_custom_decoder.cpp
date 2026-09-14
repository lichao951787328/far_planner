#include <cstdint>
#include <vector>

#include <gtest/gtest.h>
#include <ros/serialization.h>

#include "far_planner/livox_custom_decoder.h"

namespace {

std::vector<std::uint8_t> MakePacket(const std::uint32_t point_num,
                                     const std::uint32_t array_size) {
  std_msgs::Header header;
  header.seq = 7;
  header.stamp = ros::Time(123, 456);
  header.frame_id = "livox_frame";

  const std::uint32_t point_bytes = 19U;
  const std::uint32_t size =
      ros::serialization::serializationLength(header) +
      sizeof(std::uint64_t) + sizeof(std::uint32_t) + 4U +
      sizeof(std::uint32_t) + array_size * point_bytes;
  std::vector<std::uint8_t> buffer(size);
  ros::serialization::OStream stream(buffer.data(), size);
  stream.next(header);
  const std::uint64_t timebase = 123000000456ULL;
  stream.next(timebase);
  stream.next(point_num);
  const std::uint8_t lidar_id = 2;
  stream.next(lidar_id);
  const std::uint8_t reserved = 0;
  stream.next(reserved);
  stream.next(reserved);
  stream.next(reserved);
  stream.next(array_size);
  for (std::uint32_t i = 0; i < array_size; ++i) {
    const std::uint32_t offset = i * 1000U;
    const float x = 1.0f + i;
    const float y = 2.0f + i;
    const float z = 3.0f + i;
    const std::uint8_t reflectivity = static_cast<std::uint8_t>(10U + i);
    const std::uint8_t tag = 4;
    const std::uint8_t line = 1;
    stream.next(offset);
    stream.next(x);
    stream.next(y);
    stream.next(z);
    stream.next(reflectivity);
    stream.next(tag);
    stream.next(line);
  }
  return buffer;
}

TEST(LivoxCustomDecoder, DecodesKnownWireFormat) {
  const auto buffer = MakePacket(2U, 2U);
  far_planner::LivoxCustomPacketData packet;
  std::string error;
  ASSERT_TRUE(far_planner::DecodeLivoxCustomBuffer(
      buffer.data(), buffer.size(), &packet, &error)) << error;
  EXPECT_EQ("livox_frame", packet.header.frame_id);
  EXPECT_EQ(2U, packet.point_num);
  ASSERT_EQ(2U, packet.points.size());
  EXPECT_FLOAT_EQ(1.0f, packet.points[0].x);
  EXPECT_FLOAT_EQ(3.0f, packet.points[0].z);
  EXPECT_EQ(10U, packet.points[0].reflectivity);
  EXPECT_EQ(1000U, packet.points[1].offset_time);
}

TEST(LivoxCustomDecoder, PreservesPointCountMismatchForSafeCallerPolicy) {
  const auto buffer = MakePacket(1U, 2U);
  far_planner::LivoxCustomPacketData packet;
  std::string error;
  ASSERT_TRUE(far_planner::DecodeLivoxCustomBuffer(
      buffer.data(), buffer.size(), &packet, &error)) << error;
  EXPECT_EQ(1U, packet.point_num);
  EXPECT_EQ(2U, packet.points.size());
}

TEST(LivoxCustomDecoder, RejectsTruncatedPointArray) {
  auto buffer = MakePacket(2U, 2U);
  buffer.resize(buffer.size() - 1U);
  far_planner::LivoxCustomPacketData packet;
  std::string error;
  EXPECT_FALSE(far_planner::DecodeLivoxCustomBuffer(
      buffer.data(), buffer.size(), &packet, &error));
  EXPECT_FALSE(error.empty());
}

}  // namespace

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
