#include "far_planner/livox_custom_decoder.h"

#include <limits>
#include <sstream>

#include <ros/serialization.h>

namespace far_planner {
namespace {

constexpr std::uint32_t kSerializedPointBytes =
    sizeof(std::uint32_t) + 3U * sizeof(float) + 3U * sizeof(std::uint8_t);
constexpr std::uint32_t kMaximumPointsPerPacket = 5000000U;

}  // namespace

bool DecodeLivoxCustomBuffer(const std::uint8_t* data,
                             const std::size_t size,
                             LivoxCustomPacketData* packet,
                             std::string* error) {
  if (packet == nullptr) {
    if (error != nullptr) *error = "output packet is null";
    return false;
  }
  packet->points.clear();
  if (data == nullptr || size == 0U ||
      size > std::numeric_limits<std::uint32_t>::max()) {
    if (error != nullptr) *error = "serialized message buffer is empty or too large";
    return false;
  }

  try {
    ros::serialization::IStream stream(
        const_cast<std::uint8_t*>(data), static_cast<std::uint32_t>(size));
    stream.next(packet->header);
    stream.next(packet->timebase);
    stream.next(packet->point_num);
    stream.next(packet->lidar_id);

    std::uint8_t reserved = 0;
    for (int i = 0; i < 3; ++i) stream.next(reserved);

    std::uint32_t array_size = 0;
    stream.next(array_size);
    if (array_size > kMaximumPointsPerPacket) {
      if (error != nullptr) *error = "Livox point array exceeds safety limit";
      return false;
    }
    if (array_size > stream.getLength() / kSerializedPointBytes) {
      if (error != nullptr) *error = "Livox point array exceeds serialized buffer";
      return false;
    }

    packet->points.resize(array_size);
    for (auto& point : packet->points) {
      stream.next(point.offset_time);
      stream.next(point.x);
      stream.next(point.y);
      stream.next(point.z);
      stream.next(point.reflectivity);
      stream.next(point.tag);
      stream.next(point.line);
    }
  } catch (const ros::serialization::StreamOverrunException&) {
    if (error != nullptr) *error = "Livox message ended before all fields were decoded";
    packet->points.clear();
    return false;
  } catch (const std::exception& ex) {
    if (error != nullptr) {
      std::ostringstream stream;
      stream << "Livox decode exception: " << ex.what();
      *error = stream.str();
    }
    packet->points.clear();
    return false;
  }
  return true;
}

}  // namespace far_planner
