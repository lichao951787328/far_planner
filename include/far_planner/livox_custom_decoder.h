#ifndef FAR_PLANNER_LIVOX_CUSTOM_DECODER_H
#define FAR_PLANNER_LIVOX_CUSTOM_DECODER_H

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include <std_msgs/Header.h>

namespace far_planner {

struct LivoxCustomPointData {
  std::uint32_t offset_time = 0;
  float x = 0.0f;
  float y = 0.0f;
  float z = 0.0f;
  std::uint8_t reflectivity = 0;
  std::uint8_t tag = 0;
  std::uint8_t line = 0;
};

struct LivoxCustomPacketData {
  std_msgs::Header header;
  std::uint64_t timebase = 0;
  std::uint32_t point_num = 0;
  std::uint8_t lidar_id = 0;
  std::vector<LivoxCustomPointData> points;
};

// Wire contract recorded by livox_ros_driver2/CustomMsg. Keeping the decoder
// behind both datatype and MD5 checks lets this package replay Livox bags
// without requiring the complete Livox driver as a build dependency.
constexpr const char* kLivoxCustomDatatype =
    "livox_ros_driver2/CustomMsg";
constexpr const char* kLivoxCustomMd5 =
    "e4d6829bdfe657cb6c21a746c86b21a6";

bool DecodeLivoxCustomBuffer(const std::uint8_t* data,
                             std::size_t size,
                             LivoxCustomPacketData* packet,
                             std::string* error);

}  // namespace far_planner

#endif
