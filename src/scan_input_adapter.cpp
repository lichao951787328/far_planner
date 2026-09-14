#include <algorithm>
#include <cmath>
#include <cstdint>
#include <deque>
#include <limits>
#include <map>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <geometry_msgs/PointStamped.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <ros/serialization.h>
#include <rosgraph_msgs/Clock.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Empty.h>
#include <tf/transform_listener.h>
#include <topic_tools/shape_shifter.h>
#include <XmlRpcValue.h>

#include "far_planner/livox_custom_decoder.h"

namespace far_planner {
namespace {

struct RawPoint {
  RawPoint() = default;
  RawPoint(float x_value, float y_value, float z_value,
           float intensity_value, std::uint32_t offset_value)
      : x(x_value),
        y(y_value),
        z(z_value),
        intensity(intensity_value),
        offset_ns(offset_value) {}
  float x = 0.0f;
  float y = 0.0f;
  float z = 0.0f;
  float intensity = 0.0f;
  std::uint32_t offset_ns = 0;
};

struct RawScan {
  std_msgs::Header header;
  ros::Time reference_stamp;
  ros::Time timebase;
  ros::Time latest_required_tf_stamp;
  ros::WallTime receipt_wall_time;
  std::vector<RawPoint> points;
  bool is_livox = false;
};

bool SameFrame(const std::string& lhs, const std::string& rhs) {
  const std::string left = !lhs.empty() && lhs.front() == '/' ? lhs.substr(1) : lhs;
  const std::string right = !rhs.empty() && rhs.front() == '/' ? rhs.substr(1) : rhs;
  return left == right;
}

bool HasField(const sensor_msgs::PointCloud2& cloud, const std::string& name) {
  return std::any_of(cloud.fields.begin(), cloud.fields.end(),
                     [&name](const sensor_msgs::PointField& field) {
                       return field.name == name;
                     });
}

bool ReadVector3Parameter(ros::NodeHandle* private_nh,
                          const std::string& name,
                          const std::vector<double>& fallback,
                          std::vector<double>* values) {
  XmlRpc::XmlRpcValue parameter;
  if (!private_nh->getParam(name, parameter)) {
    *values = fallback;
    return true;
  }
  if (parameter.getType() != XmlRpc::XmlRpcValue::TypeArray ||
      parameter.size() != 3) {
    ROS_ERROR("~%s must be an array with exactly three numbers", name.c_str());
    return false;
  }
  values->resize(3);
  for (int i = 0; i < 3; ++i) {
    if (parameter[i].getType() == XmlRpc::XmlRpcValue::TypeInt) {
      (*values)[i] = static_cast<int>(parameter[i]);
    } else if (parameter[i].getType() == XmlRpc::XmlRpcValue::TypeDouble) {
      (*values)[i] = static_cast<double>(parameter[i]);
    } else {
      ROS_ERROR("~%s[%d] must be numeric", name.c_str(), i);
      return false;
    }
  }
  return true;
}

ros::Time TimeFromNanoseconds(const std::uint64_t nanoseconds) {
  ros::Time stamp;
  stamp.fromNSec(nanoseconds);
  return stamp;
}

}  // namespace

class ScanInputAdapter {
 public:
  ScanInputAdapter()
      : private_nh_("~"),
        tf_listener_(ros::Duration(120.0)),
        last_published_stamp_(0),
        last_clock_(0) {
    private_nh_.param<std::string>("input_topic", input_topic_, "/raw_scan");
    private_nh_.param<std::string>("output_topic", output_topic_,
                                   "/far_scan/registered_scan");
    private_nh_.param<std::string>("origin_topic", origin_topic_,
                                   "/far_scan/registered_scan_origin");
    private_nh_.param<std::string>("world_frame", world_frame_, "map");
    private_nh_.param<std::string>("vehicle_frame", vehicle_frame_, "wuba_base");
    private_nh_.param<std::string>("lidar_frame", lidar_frame_, "livox_frame");
    private_nh_.param<std::string>("pointcloud_origin_frame",
                                   pointcloud_origin_frame_, vehicle_frame_);
    private_nh_.param<std::string>("extrinsic_source", extrinsic_source_, "params");
    private_nh_.param<bool>("deskew_enabled", deskew_enabled_, true);
    private_nh_.param<double>("deskew_time_bin", deskew_time_bin_, 0.002);
    private_nh_.param<double>("transform_timeout_wall", transform_timeout_wall_, 1.0);
    private_nh_.param<int>("pending_queue_size", pending_queue_size_, 3);
    private_nh_.param<bool>("drop_non_increasing_stamp",
                            drop_non_increasing_stamp_, true);
    private_nh_.param<bool>("remove_zero_points", remove_zero_points_, true);
    private_nh_.param<double>("zero_point_epsilon", zero_point_epsilon_, 1.0e-6);

    if (world_frame_.empty() || vehicle_frame_.empty() || lidar_frame_.empty()) {
      throw std::runtime_error("world_frame, vehicle_frame and lidar_frame must be non-empty");
    }
    if (extrinsic_source_ != "params" && extrinsic_source_ != "tf") {
      throw std::runtime_error("extrinsic_source must be 'params' or 'tf'");
    }
    deskew_time_bin_ = std::max(0.0001, deskew_time_bin_);
    transform_timeout_wall_ = std::max(0.0, transform_timeout_wall_);
    pending_queue_size_ = std::max(1, pending_queue_size_);

    std::vector<double> xyz;
    std::vector<double> rpy;
    if (!ReadVector3Parameter(&private_nh_, "extrinsic_xyz", {0.0, 0.0, 0.0},
                              &xyz) ||
        !ReadVector3Parameter(&private_nh_, "extrinsic_rpy", {0.0, 0.0, 0.0},
                              &rpy)) {
      throw std::runtime_error("invalid Livox extrinsic parameters");
    }
    tf::Quaternion rotation;
    rotation.setRPY(rpy[0], rpy[1], rpy[2]);
    vehicle_from_lidar_.setOrigin(tf::Vector3(xyz[0], xyz[1], xyz[2]));
    vehicle_from_lidar_.setRotation(rotation);

    scan_publisher_ = nh_.advertise<sensor_msgs::PointCloud2>(output_topic_, 2);
    origin_publisher_ =
        nh_.advertise<geometry_msgs::PointStamped>(origin_topic_, 2);
    input_subscriber_ = nh_.subscribe(
        input_topic_, 2, &ScanInputAdapter::InputCallback, this);
    reset_subscriber_ = nh_.subscribe(
        "/reset_visibility_graph", 1, &ScanInputAdapter::ResetCallback, this);
    clock_subscriber_ =
        nh_.subscribe("/clock", 5, &ScanInputAdapter::ClockCallback, this);
    pending_timer_ = nh_.createWallTimer(
        ros::WallDuration(0.01), &ScanInputAdapter::PendingTimerCallback, this);

    ROS_INFO("FAR scan adapter: input=%s output=%s origin=%s world=%s, "
             "PointCloud2 + %s supported",
             input_topic_.c_str(), output_topic_.c_str(), origin_topic_.c_str(),
             world_frame_.c_str(), kLivoxCustomDatatype);
    ROS_INFO("Livox registration: %s <- %s <- %s, extrinsic=%s "
             "xyz=[%.3f %.3f %.3f] rpy=[%.3f %.3f %.3f], deskew=%s bin=%.3f ms",
             world_frame_.c_str(), vehicle_frame_.c_str(), lidar_frame_.c_str(),
             extrinsic_source_.c_str(), xyz[0], xyz[1], xyz[2],
             rpy[0], rpy[1], rpy[2], deskew_enabled_ ? "on" : "off",
             deskew_time_bin_ * 1000.0);
  }

 private:
  bool DecodePointCloud2(const sensor_msgs::PointCloud2& message,
                         RawScan* scan,
                         std::string* error) const {
    if (!HasField(message, "x") || !HasField(message, "y") ||
        !HasField(message, "z")) {
      *error = "PointCloud2 is missing x/y/z";
      return false;
    }
    scan->header = message.header;
    scan->reference_stamp = message.header.stamp;
    scan->timebase = message.header.stamp;
    scan->latest_required_tf_stamp = message.header.stamp;
    scan->is_livox = false;

    if (HasField(message, "intensity")) {
      pcl::PointCloud<pcl::PointXYZI> cloud;
      pcl::fromROSMsg(message, cloud);
      scan->points.reserve(cloud.size());
      for (const auto& point : cloud.points) {
        scan->points.push_back(
            {point.x, point.y, point.z, point.intensity, 0U});
      }
    } else {
      pcl::PointCloud<pcl::PointXYZ> cloud;
      pcl::fromROSMsg(message, cloud);
      scan->points.reserve(cloud.size());
      for (const auto& point : cloud.points) {
        scan->points.push_back({point.x, point.y, point.z, 0.0f, 0U});
      }
    }
    return true;
  }

  bool DecodeLivox(const topic_tools::ShapeShifter& message,
                   RawScan* scan,
                   std::string* error) const {
    std::vector<std::uint8_t> buffer(message.size());
    ros::serialization::OStream stream(
        buffer.data(), static_cast<std::uint32_t>(buffer.size()));
    message.write(stream);

    LivoxCustomPacketData packet;
    if (!DecodeLivoxCustomBuffer(buffer.data(), buffer.size(), &packet, error)) {
      return false;
    }
    if (packet.point_num != packet.points.size()) {
      ROS_WARN_THROTTLE(
          2.0, "Livox point_num=%u differs from serialized points=%zu; using min",
          packet.point_num, packet.points.size());
    }
    const std::size_t count =
        std::min<std::size_t>(packet.point_num, packet.points.size());
    scan->header = packet.header;
    if (scan->header.frame_id.empty()) scan->header.frame_id = lidar_frame_;
    scan->timebase = packet.timebase != 0U
                         ? TimeFromNanoseconds(packet.timebase)
                         : packet.header.stamp;
    if (scan->header.stamp.isZero()) scan->header.stamp = scan->timebase;
    scan->is_livox = true;
    scan->points.reserve(count);

    std::uint32_t minimum_offset = std::numeric_limits<std::uint32_t>::max();
    std::uint32_t maximum_offset = 0U;
    for (std::size_t i = 0; i < count; ++i) {
      const auto& point = packet.points[i];
      scan->points.push_back({point.x, point.y, point.z,
                              static_cast<float>(point.reflectivity),
                              point.offset_time});
      minimum_offset = std::min(minimum_offset, point.offset_time);
      maximum_offset = std::max(maximum_offset, point.offset_time);
    }
    if (scan->points.empty()) {
      scan->reference_stamp = scan->header.stamp;
      scan->latest_required_tf_stamp = scan->header.stamp;
    } else {
      const std::uint64_t middle_offset =
          (static_cast<std::uint64_t>(minimum_offset) + maximum_offset) / 2U;
      scan->reference_stamp =
          TimeFromNanoseconds(scan->timebase.toNSec() + middle_offset);
      scan->latest_required_tf_stamp =
          TimeFromNanoseconds(scan->timebase.toNSec() + maximum_offset);
    }
    return true;
  }

  bool LookupWorldFromVehicle(const ros::Time& stamp,
                              tf::Transform* transform,
                              std::string* error) {
    if (SameFrame(world_frame_, vehicle_frame_)) {
      transform->setIdentity();
      return true;
    }
    try {
      tf::StampedTransform stamped;
      tf_listener_.lookupTransform(world_frame_, vehicle_frame_, stamp, stamped);
      *transform = stamped;
      return true;
    } catch (const tf::TransformException& ex) {
      *error = ex.what();
      return false;
    }
  }

  bool LookupWorldFromLidar(const ros::Time& stamp,
                            tf::Transform* transform,
                            std::string* error) {
    if (extrinsic_source_ == "tf") {
      if (SameFrame(world_frame_, lidar_frame_)) {
        transform->setIdentity();
        return true;
      }
      try {
        tf::StampedTransform stamped;
        tf_listener_.lookupTransform(world_frame_, lidar_frame_, stamp, stamped);
        *transform = stamped;
        return true;
      } catch (const tf::TransformException& ex) {
        *error = ex.what();
        return false;
      }
    }
    tf::Transform world_from_vehicle;
    if (!LookupWorldFromVehicle(stamp, &world_from_vehicle, error)) return false;
    *transform = world_from_vehicle * vehicle_from_lidar_;
    return true;
  }

  bool LookupWorldFromFrame(const std::string& source_frame,
                            const ros::Time& stamp,
                            tf::Transform* transform,
                            std::string* error) {
    if (SameFrame(source_frame, world_frame_)) {
      transform->setIdentity();
      return true;
    }
    if (SameFrame(source_frame, lidar_frame_)) {
      return LookupWorldFromLidar(stamp, transform, error);
    }
    if (SameFrame(source_frame, vehicle_frame_)) {
      return LookupWorldFromVehicle(stamp, transform, error);
    }
    try {
      tf::StampedTransform stamped;
      tf_listener_.lookupTransform(world_frame_, source_frame, stamp, stamped);
      *transform = stamped;
      return true;
    } catch (const tf::TransformException& ex) {
      *error = ex.what();
      return false;
    }
  }

  bool TransformScan(const RawScan& scan,
                     pcl::PointCloud<pcl::PointXYZI>* transformed,
                     geometry_msgs::PointStamped* origin,
                     std::string* error) {
    tf::Transform reference_transform;
    tf::Transform origin_transform;
    if (scan.is_livox) {
      if (!LookupWorldFromLidar(scan.reference_stamp, &reference_transform, error)) {
        return false;
      }
      origin_transform = reference_transform;
    } else {
      if (!LookupWorldFromFrame(scan.header.frame_id, scan.reference_stamp,
                                &reference_transform, error)) {
        return false;
      }
      const std::string origin_frame = SameFrame(scan.header.frame_id, world_frame_)
                                           ? pointcloud_origin_frame_
                                           : scan.header.frame_id;
      if (!LookupWorldFromFrame(origin_frame, scan.reference_stamp,
                                &origin_transform, error)) {
        return false;
      }
    }

    origin->header.frame_id = world_frame_;
    origin->header.stamp = scan.reference_stamp;
    const tf::Vector3 origin_position = origin_transform.getOrigin();
    origin->point.x = origin_position.x();
    origin->point.y = origin_position.y();
    origin->point.z = origin_position.z();

    std::map<std::uint64_t, tf::Transform> transforms;
    if (!scan.is_livox || !deskew_enabled_) {
      transforms.emplace(0U, reference_transform);
    }

    transformed->clear();
    transformed->reserve(scan.points.size());
    const std::uint64_t bin_nanoseconds =
        static_cast<std::uint64_t>(deskew_time_bin_ * 1.0e9);
    for (const auto& point : scan.points) {
      if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
          !std::isfinite(point.z)) {
        continue;
      }
      if (remove_zero_points_ &&
          std::abs(point.x) <= zero_point_epsilon_ &&
          std::abs(point.y) <= zero_point_epsilon_ &&
          std::abs(point.z) <= zero_point_epsilon_) {
        continue;
      }

      std::uint64_t key = 0U;
      if (scan.is_livox && deskew_enabled_) {
        key = point.offset_ns / bin_nanoseconds;
        if (transforms.find(key) == transforms.end()) {
          const std::uint64_t bucket_offset = key * bin_nanoseconds;
          const ros::Time bucket_stamp =
              TimeFromNanoseconds(scan.timebase.toNSec() + bucket_offset);
          tf::Transform bucket_transform;
          if (!LookupWorldFromLidar(bucket_stamp, &bucket_transform, error)) {
            transformed->clear();
            return false;
          }
          transforms.emplace(key, bucket_transform);
        }
      }
      const tf::Transform& transform = transforms.at(key);
      const tf::Vector3 mapped =
          transform * tf::Vector3(point.x, point.y, point.z);
      pcl::PointXYZI output;
      output.x = mapped.x();
      output.y = mapped.y();
      output.z = mapped.z();
      output.intensity = point.intensity;
      transformed->push_back(output);
    }
    return true;
  }

  bool IsStampAlreadyQueued(const ros::Time& stamp) const {
    return std::any_of(pending_scans_.begin(), pending_scans_.end(),
                       [&stamp](const RawScan& queued) {
                         return queued.reference_stamp == stamp;
                       });
  }

  std::string RequiredTfFrame(const RawScan& scan) const {
    if (scan.is_livox) {
      return extrinsic_source_ == "params" ? vehicle_frame_ : lidar_frame_;
    }
    if (SameFrame(scan.header.frame_id, world_frame_)) {
      return pointcloud_origin_frame_;
    }
    if (SameFrame(scan.header.frame_id, lidar_frame_) &&
        extrinsic_source_ == "params") {
      return vehicle_frame_;
    }
    return scan.header.frame_id;
  }

  bool TfTimelineAlreadyPassed(const RawScan& scan) {
    const std::string frame = RequiredTfFrame(scan);
    if (SameFrame(frame, world_frame_)) return false;
    try {
      tf::StampedTransform latest;
      tf_listener_.lookupTransform(world_frame_, frame, ros::Time(0), latest);
      return !latest.stamp_.isZero() &&
             latest.stamp_ >= scan.latest_required_tf_stamp;
    } catch (const tf::TransformException&) {
      return false;
    }
  }

  bool PublishScan(const RawScan& scan, std::string* error) {
    if (scan.reference_stamp.isZero()) {
      *error = "zero reference timestamp";
      return true;
    }
    if (drop_non_increasing_stamp_ && !last_published_stamp_.isZero() &&
        scan.reference_stamp <= last_published_stamp_) {
      ROS_WARN_THROTTLE(1.0,
                        "Drop old scan stamp %.9f; last published stamp is %.9f",
                        scan.reference_stamp.toSec(),
                        last_published_stamp_.toSec());
      return true;
    }

    pcl::PointCloud<pcl::PointXYZI> transformed;
    geometry_msgs::PointStamped origin;
    if (!TransformScan(scan, &transformed, &origin, error)) return false;
    if (transformed.empty()) {
      ROS_WARN_THROTTLE(1.0, "Drop scan because no valid points remain");
      return true;
    }

    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(transformed, output);
    output.header.frame_id = world_frame_;
    output.header.stamp = scan.reference_stamp;

    // Publish origin first. FAR also buffers both topics by exact timestamp,
    // so transport scheduling cannot pair an origin with the wrong scan.
    origin_publisher_.publish(origin);
    scan_publisher_.publish(output);
    last_published_stamp_ = scan.reference_stamp;
    ++published_scans_;
    if (published_scans_ % 50U == 0U) {
      ROS_INFO("FAR scan adapter published %zu scans; latest type=%s points=%zu "
               "stamp=%.9f",
               published_scans_, scan.is_livox ? "Livox CustomMsg" : "PointCloud2",
               transformed.size(), scan.reference_stamp.toSec());
    }
    return true;
  }

  void ProcessPending() {
    while (!pending_scans_.empty()) {
      std::string error;
      if (PublishScan(pending_scans_.front(), &error)) {
        pending_scans_.pop_front();
        continue;
      }
      if (TfTimelineAlreadyPassed(pending_scans_.front())) {
        ROS_WARN("Drop scan stamp %.9f because the recorded TF timeline has "
                 "already passed its required time: %s",
                 pending_scans_.front().reference_stamp.toSec(),
                 error.c_str());
        pending_scans_.pop_front();
        ++dropped_scans_;
        continue;
      }
      const double waiting =
          (ros::WallTime::now() - pending_scans_.front().receipt_wall_time).toSec();
      if (waiting >= transform_timeout_wall_) {
        ROS_WARN("Drop scan stamp %.9f after %.3f s wall-time TF wait: %s",
                 pending_scans_.front().reference_stamp.toSec(), waiting,
                 error.c_str());
        pending_scans_.pop_front();
        ++dropped_scans_;
        continue;
      }
      break;
    }
  }

  void InputCallback(const topic_tools::ShapeShifter::ConstPtr& message) {
    RawScan scan;
    scan.receipt_wall_time = ros::WallTime::now();
    std::string error;

    if (message->getDataType() == "sensor_msgs/PointCloud2") {
      if (message->getMD5Sum() !=
          ros::message_traits::MD5Sum<sensor_msgs::PointCloud2>::value()) {
        ROS_ERROR_THROTTLE(1.0, "Reject PointCloud2 with unexpected MD5 %s",
                           message->getMD5Sum().c_str());
        return;
      }
      const auto cloud = message->instantiate<sensor_msgs::PointCloud2>();
      if (!cloud || !DecodePointCloud2(*cloud, &scan, &error)) {
        ROS_ERROR_THROTTLE(1.0, "PointCloud2 decode failed: %s", error.c_str());
        return;
      }
    } else if (message->getDataType() == kLivoxCustomDatatype) {
      if (message->getMD5Sum() != kLivoxCustomMd5) {
        ROS_ERROR_THROTTLE(1.0,
                           "Reject %s with unsupported MD5 %s (expected %s)",
                           kLivoxCustomDatatype, message->getMD5Sum().c_str(),
                           kLivoxCustomMd5);
        return;
      }
      if (!DecodeLivox(*message, &scan, &error)) {
        ROS_ERROR_THROTTLE(1.0, "Livox CustomMsg decode failed: %s",
                           error.c_str());
        return;
      }
    } else {
      ROS_ERROR_THROTTLE(1.0, "Unsupported raw scan type: %s",
                         message->getDataType().c_str());
      return;
    }

    if (scan.reference_stamp.isZero()) {
      ROS_WARN_THROTTLE(1.0, "Drop raw scan with zero timestamp");
      return;
    }
    if (drop_non_increasing_stamp_ &&
        ((!last_published_stamp_.isZero() &&
          scan.reference_stamp <= last_published_stamp_) ||
         IsStampAlreadyQueued(scan.reference_stamp))) {
      ROS_WARN_THROTTLE(1.0, "Drop duplicate/old raw scan stamp %.9f",
                        scan.reference_stamp.toSec());
      return;
    }

    if (pending_scans_.size() >=
        static_cast<std::size_t>(pending_queue_size_)) {
      ROS_WARN("TF pending queue full; drop oldest scan stamp %.9f",
               pending_scans_.front().reference_stamp.toSec());
      pending_scans_.pop_front();
      ++dropped_scans_;
    }
    pending_scans_.push_back(std::move(scan));
    ProcessPending();
  }

  void ResetState(const char* reason) {
    pending_scans_.clear();
    last_published_stamp_ = ros::Time(0);
    ROS_WARN("FAR scan adapter timestamp state reset: %s", reason);
  }

  void ResetCallback(const std_msgs::EmptyConstPtr&) {
    ResetState("visibility graph reset");
  }

  void ClockCallback(const rosgraph_msgs::ClockConstPtr& message) {
    if (!last_clock_.isZero() && message->clock < last_clock_) {
      ResetState("/clock moved backwards");
    }
    last_clock_ = message->clock;
  }

  void PendingTimerCallback(const ros::WallTimerEvent&) { ProcessPending(); }

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  tf::TransformListener tf_listener_;
  ros::Subscriber input_subscriber_;
  ros::Subscriber reset_subscriber_;
  ros::Subscriber clock_subscriber_;
  ros::Publisher scan_publisher_;
  ros::Publisher origin_publisher_;
  ros::WallTimer pending_timer_;

  std::string input_topic_;
  std::string output_topic_;
  std::string origin_topic_;
  std::string world_frame_;
  std::string vehicle_frame_;
  std::string lidar_frame_;
  std::string pointcloud_origin_frame_;
  std::string extrinsic_source_;
  bool deskew_enabled_ = true;
  bool drop_non_increasing_stamp_ = true;
  bool remove_zero_points_ = true;
  double deskew_time_bin_ = 0.002;
  double transform_timeout_wall_ = 1.0;
  double zero_point_epsilon_ = 1.0e-6;
  int pending_queue_size_ = 3;
  tf::Transform vehicle_from_lidar_;

  std::deque<RawScan> pending_scans_;
  ros::Time last_published_stamp_;
  ros::Time last_clock_;
  std::size_t published_scans_ = 0U;
  std::size_t dropped_scans_ = 0U;
};

}  // namespace far_planner

int main(int argc, char** argv) {
  ros::init(argc, argv, "far_scan_input_adapter");
  try {
    far_planner::ScanInputAdapter adapter;
    ros::spin();
  } catch (const std::exception& ex) {
    ROS_FATAL("FAR scan adapter initialization failed: %s", ex.what());
    return 1;
  }
  return 0;
}
