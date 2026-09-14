#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <tf/transform_listener.h>
#include <XmlRpcValue.h>

#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

namespace {

enum class SemanticRole {
  kTerrain,
  kStaticObstacle,
  kDynamicObstacle,
  kIgnore
};

struct BinaryPoint {
  float x;
  float y;
  float z;
  float intensity;
  std::uint8_t static_obstacle;
};

std::string NormalizeFrameId(const std::string& frame_id) {
  const std::size_t first = frame_id.find_first_not_of('/');
  return first == std::string::npos ? std::string() : frame_id.substr(first);
}

bool HasField(const sensor_msgs::PointCloud2& cloud, const std::string& name) {
  for (const auto& field : cloud.fields) {
    if (field.name == name) return true;
  }
  return false;
}

bool XmlBool(const XmlRpc::XmlRpcValue& value, const std::string& context) {
  if (value.getType() == XmlRpc::XmlRpcValue::TypeBoolean) {
    return static_cast<bool>(value);
  }
  if (value.getType() == XmlRpc::XmlRpcValue::TypeInt) {
    return static_cast<int>(value) != 0;
  }
  throw std::runtime_error(context + " must be bool");
}

SemanticRole ParseRole(const std::string& role) {
  if (role == "terrain") return SemanticRole::kTerrain;
  if (role == "static_obstacle") return SemanticRole::kStaticObstacle;
  if (role == "dynamic_obstacle") return SemanticRole::kDynamicObstacle;
  if (role == "ignore") return SemanticRole::kIgnore;
  throw std::runtime_error("unsupported semantic role: " + role);
}

class SemanticVoxelToFarTerrain {
 public:
  SemanticVoxelToFarTerrain() : private_nh_("~") {
    private_nh_.param<std::string>(
        "input_topic", input_topic_,
        "/local_3d_semantic_voxel_map/voxel_cloud");
    private_nh_.param<std::string>("output_topic", output_topic_,
                                   "/far_binary/terrain_map");
    private_nh_.param<std::string>("world_frame", world_frame_, "map");
    private_nh_.param<double>("tf_timeout", tf_timeout_, 0.2);
    private_nh_.param<double>("obstacle_threshold", obstacle_threshold_, 0.75);
    private_nh_.param<int>("minimum_observations", minimum_observations_, 1);
    private_nh_.param<bool>("missing_cost_is_obstacle",
                            missing_cost_is_obstacle_, true);
    private_nh_.param<bool>("publish_debug_clouds", publish_debug_clouds_, true);
    private_nh_.param<std::string>("cost_field", cost_field_, "traversability");

    if (!(obstacle_threshold_ >= 0.0 && obstacle_threshold_ <= 1.0)) {
      throw std::runtime_error("~obstacle_threshold must be in [0, 1]");
    }
    if (minimum_observations_ < 0) {
      throw std::runtime_error("~minimum_observations must be non-negative");
    }
    if (NormalizeFrameId(world_frame_).empty()) {
      throw std::runtime_error("~world_frame must not be empty");
    }

    LoadSemanticSchema();

    terrain_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(output_topic_, 1);
    if (publish_debug_clouds_) {
      free_debug_pub_ = private_nh_.advertise<sensor_msgs::PointCloud2>(
          "free_debug", 1);
      obstacle_debug_pub_ = private_nh_.advertise<sensor_msgs::PointCloud2>(
          "obstacle_debug", 1);
      protected_static_debug_pub_ =
          private_nh_.advertise<sensor_msgs::PointCloud2>(
              "protected_static_debug", 1);
    }
    cloud_sub_ = nh_.subscribe(input_topic_, 1,
                               &SemanticVoxelToFarTerrain::CloudCallback, this);

    ROS_INFO("semantic_voxel_to_far_terrain: %s -> %s, frame=%s, threshold=%.3f",
             input_topic_.c_str(), output_topic_.c_str(), world_frame_.c_str(),
             obstacle_threshold_);
  }

 private:
  void LoadSemanticSchema() {
    XmlRpc::XmlRpcValue schema;
    if (!private_nh_.getParam("semantic_schema", schema) &&
        !nh_.getParam("/semantic_schema", schema)) {
      throw std::runtime_error(
          "semantic schema is missing; load config/five_class_semantic_schema.yaml");
    }
    if (schema.getType() != XmlRpc::XmlRpcValue::TypeStruct ||
        !schema.hasMember("classes")) {
      throw std::runtime_error("semantic_schema/classes is missing");
    }

    XmlRpc::XmlRpcValue& classes = schema["classes"];
    if (classes.getType() != XmlRpc::XmlRpcValue::TypeArray) {
      throw std::runtime_error("semantic_schema/classes must be an array");
    }
    for (int i = 0; i < classes.size(); ++i) {
      XmlRpc::XmlRpcValue& item = classes[i];
      if (item.getType() != XmlRpc::XmlRpcValue::TypeStruct ||
          !item.hasMember("label") || !item.hasMember("role")) {
        throw std::runtime_error(
            "each semantic class must contain label and role");
      }
      if (item["label"].getType() != XmlRpc::XmlRpcValue::TypeInt ||
          item["role"].getType() != XmlRpc::XmlRpcValue::TypeString) {
        throw std::runtime_error("semantic class label/role has invalid type");
      }
      const int signed_label = static_cast<int>(item["label"]);
      if (signed_label < 0) {
        throw std::runtime_error("semantic class label must be non-negative");
      }
      const std::uint32_t label = static_cast<std::uint32_t>(signed_label);
      const std::string role = static_cast<std::string>(item["role"]);
      const SemanticRole parsed_role = ParseRole(role);
      if (!roles_.emplace(label, parsed_role).second) {
        throw std::runtime_error("duplicate semantic class label");
      }
      bool far_static_protection =
          parsed_role == SemanticRole::kStaticObstacle;
      if (item.hasMember("far_static_protection")) {
        far_static_protection = XmlBool(
            item["far_static_protection"], "far_static_protection");
      }
      if (far_static_protection &&
          parsed_role != SemanticRole::kStaticObstacle) {
        throw std::runtime_error(
            "far_static_protection=true requires role=static_obstacle");
      }
      far_static_protection_[label] = far_static_protection;
      if (item.hasMember("geometry_only") &&
          XmlBool(item["geometry_only"], "geometry_only")) {
        geometry_only_[label] = true;
      }
    }
    if (roles_.empty()) {
      throw std::runtime_error("semantic schema contains no classes");
    }
  }

  bool IsHardObstacle(std::uint32_t label) const {
    // A geometry-only label (five-class label 0) never contributes semantic
    // obstacle evidence; its final traversability cost decides the result.
    if (geometry_only_.count(label) != 0u) return false;
    const auto found = roles_.find(label);
    if (found == roles_.end()) return false;
    return found->second == SemanticRole::kStaticObstacle ||
           found->second == SemanticRole::kDynamicObstacle;
  }

  bool IsProtectedStaticObstacle(std::uint32_t label) const {
    if (geometry_only_.count(label) != 0u) return false;
    const auto found = far_static_protection_.find(label);
    return found != far_static_protection_.end() && found->second;
  }

  sensor_msgs::PointCloud2 MakeCloud(const std_msgs::Header& source_header,
                                     const std::vector<BinaryPoint>& points) const {
    sensor_msgs::PointCloud2 output;
    output.header = source_header;
    output.header.frame_id = world_frame_;
    output.height = 1u;
    output.is_dense = true;
    sensor_msgs::PointCloud2Modifier modifier(output);
    // Keep intensity binary for stock FAR compatibility.  The extra field is
    // consumed only by the semantic-aware FAR build and travels in the same
    // message/stamp, avoiding a second topic that would need synchronization.
    modifier.setPointCloud2Fields(
        5, "x", 1, sensor_msgs::PointField::FLOAT32,
        "y", 1, sensor_msgs::PointField::FLOAT32,
        "z", 1, sensor_msgs::PointField::FLOAT32,
        "intensity", 1, sensor_msgs::PointField::FLOAT32,
        "static_obstacle", 1, sensor_msgs::PointField::UINT8);
    modifier.resize(points.size());

    sensor_msgs::PointCloud2Iterator<float> x(output, "x");
    sensor_msgs::PointCloud2Iterator<float> y(output, "y");
    sensor_msgs::PointCloud2Iterator<float> z(output, "z");
    sensor_msgs::PointCloud2Iterator<float> intensity(output, "intensity");
    sensor_msgs::PointCloud2Iterator<std::uint8_t> static_obstacle(
        output, "static_obstacle");
    for (const auto& point : points) {
      *x = point.x;
      *y = point.y;
      *z = point.z;
      *intensity = point.intensity;
      *static_obstacle = point.static_obstacle;
      ++x;
      ++y;
      ++z;
      ++intensity;
      ++static_obstacle;
    }
    return output;
  }

  void CloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud) {
    const std::vector<std::string> required_fields = {
        "x", "y", "z", "label", cost_field_, "observations"};
    for (const auto& field : required_fields) {
      if (!HasField(*cloud, field)) {
        ROS_ERROR_THROTTLE(1.0,
                           "semantic_voxel_to_far_terrain: missing field '%s'",
                           field.c_str());
        return;
      }
    }
    if (cloud->header.stamp.isZero()) {
      ROS_ERROR_THROTTLE(1.0,
                         "semantic_voxel_to_far_terrain: input stamp is zero");
      return;
    }
    if (NormalizeFrameId(cloud->header.frame_id).empty()) {
      ROS_ERROR_THROTTLE(1.0,
                         "semantic_voxel_to_far_terrain: input frame is empty");
      return;
    }

    const bool transform_needed =
        NormalizeFrameId(cloud->header.frame_id) != NormalizeFrameId(world_frame_);
    tf::StampedTransform cloud_to_world;
    if (transform_needed) {
      try {
        tf_listener_.waitForTransform(world_frame_, cloud->header.frame_id,
                                      cloud->header.stamp,
                                      ros::Duration(tf_timeout_));
        tf_listener_.lookupTransform(world_frame_, cloud->header.frame_id,
                                     cloud->header.stamp, cloud_to_world);
      } catch (const tf::TransformException& ex) {
        ROS_ERROR_THROTTLE(
            1.0, "semantic_voxel_to_far_terrain: timestamped TF failed: %s",
            ex.what());
        return;
      }
    }

    std::vector<BinaryPoint> all_points;
    std::vector<BinaryPoint> free_points;
    std::vector<BinaryPoint> obstacle_points;
    std::vector<BinaryPoint> protected_static_points;
    const std::size_t point_count =
        static_cast<std::size_t>(cloud->width) * cloud->height;
    all_points.reserve(point_count);
    free_points.reserve(point_count);
    obstacle_points.reserve(point_count);
    protected_static_points.reserve(point_count);

    try {
      sensor_msgs::PointCloud2ConstIterator<float> x(*cloud, "x");
      sensor_msgs::PointCloud2ConstIterator<float> y(*cloud, "y");
      sensor_msgs::PointCloud2ConstIterator<float> z(*cloud, "z");
      sensor_msgs::PointCloud2ConstIterator<std::uint32_t> label(*cloud, "label");
      sensor_msgs::PointCloud2ConstIterator<float> cost(*cloud, cost_field_);
      sensor_msgs::PointCloud2ConstIterator<std::uint32_t> observations(
          *cloud, "observations");

      for (; x != x.end(); ++x, ++y, ++z, ++label, ++cost, ++observations) {
        if (!std::isfinite(*x) || !std::isfinite(*y) || !std::isfinite(*z)) {
          continue;
        }
        if (*observations < static_cast<std::uint32_t>(minimum_observations_)) {
          continue;
        }

        const bool protected_static = IsProtectedStaticObstacle(*label);
        bool obstacle = IsHardObstacle(*label);
        if (!obstacle) {
          obstacle = std::isfinite(*cost) ?
              static_cast<double>(*cost) >= obstacle_threshold_ :
              missing_cost_is_obstacle_;
        }

        tf::Vector3 position(*x, *y, *z);
        if (transform_needed) position = cloud_to_world * position;
        BinaryPoint point{
            static_cast<float>(position.x()),
            static_cast<float>(position.y()),
            static_cast<float>(position.z()),
            obstacle ? 1.0f : 0.0f,
            protected_static ? std::uint8_t{1} : std::uint8_t{0}};
        all_points.push_back(point);
        if (obstacle) {
          obstacle_points.push_back(point);
          if (protected_static) protected_static_points.push_back(point);
        } else {
          free_points.push_back(point);
        }
      }
    } catch (const std::runtime_error& ex) {
      ROS_ERROR_THROTTLE(1.0,
                         "semantic_voxel_to_far_terrain: field decode failed: %s",
                         ex.what());
      return;
    }

    terrain_pub_.publish(MakeCloud(cloud->header, all_points));
    if (publish_debug_clouds_) {
      free_debug_pub_.publish(MakeCloud(cloud->header, free_points));
      obstacle_debug_pub_.publish(MakeCloud(cloud->header, obstacle_points));
      protected_static_debug_pub_.publish(
          MakeCloud(cloud->header, protected_static_points));
    }
  }

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Subscriber cloud_sub_;
  ros::Publisher terrain_pub_;
  ros::Publisher free_debug_pub_;
  ros::Publisher obstacle_debug_pub_;
  ros::Publisher protected_static_debug_pub_;
  tf::TransformListener tf_listener_;

  std::string input_topic_;
  std::string output_topic_;
  std::string world_frame_;
  std::string cost_field_;
  double tf_timeout_;
  double obstacle_threshold_;
  int minimum_observations_;
  bool missing_cost_is_obstacle_;
  bool publish_debug_clouds_;
  std::unordered_map<std::uint32_t, SemanticRole> roles_;
  std::unordered_map<std::uint32_t, bool> far_static_protection_;
  std::unordered_map<std::uint32_t, bool> geometry_only_;
};

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "semantic_voxel_to_far_terrain");
  try {
    SemanticVoxelToFarTerrain node;
    ros::spin();
  } catch (const std::exception& ex) {
    ROS_FATAL("semantic_voxel_to_far_terrain: startup failed: %s", ex.what());
    return 1;
  }
  return 0;
}
