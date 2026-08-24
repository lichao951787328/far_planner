#include <geometry_msgs/PointStamped.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>

#include <string>

namespace {

std::string NormalizeFrame(const std::string& frame) {
  return !frame.empty() && frame.front() == '/' ? frame.substr(1) : frame;
}

class PointStampedTfAdapter {
 public:
  PointStampedTfAdapter() : private_nh_("~") {
    private_nh_.param<std::string>("input_topic", input_topic_, "/way_point");
    private_nh_.param<std::string>("output_topic", output_topic_,
                                   "/way_point_map");
    private_nh_.param<std::string>("target_frame", target_frame_, "map");
    private_nh_.param<double>("transform_timeout", transform_timeout_, 0.2);

    publisher_ = nh_.advertise<geometry_msgs::PointStamped>(output_topic_, 5);
    subscriber_ = nh_.subscribe(
        input_topic_, 5, &PointStampedTfAdapter::PointCallback, this,
        ros::TransportHints().tcpNoDelay());
    ROS_INFO("point_stamped_tf_adapter: %s -> %s, target frame '%s'",
             input_topic_.c_str(), output_topic_.c_str(),
             target_frame_.c_str());
  }

 private:
  void PointCallback(const geometry_msgs::PointStampedConstPtr& input) {
    if (!input) return;
    if (input->header.frame_id.empty()) {
      ROS_ERROR_THROTTLE(
          1.0,
          "point_stamped_tf_adapter: input point has an empty frame_id; dropping it.");
      return;
    }

    if (NormalizeFrame(input->header.frame_id) ==
        NormalizeFrame(target_frame_)) {
      geometry_msgs::PointStamped output = *input;
      output.header.frame_id = target_frame_;
      publisher_.publish(output);
      return;
    }

    geometry_msgs::PointStamped output;
    try {
      const ros::Time transform_time = input->header.stamp;
      if (!tf_listener_.waitForTransform(
              target_frame_, input->header.frame_id, transform_time,
              ros::Duration(transform_timeout_))) {
        ROS_WARN_THROTTLE(
            1.0,
            "point_stamped_tf_adapter: transform unavailable (%s <- %s) at %.6f; dropping waypoint.",
            target_frame_.c_str(), input->header.frame_id.c_str(),
            transform_time.toSec());
        return;
      }
      tf_listener_.transformPoint(target_frame_, *input, output);
      output.header.frame_id = target_frame_;
      publisher_.publish(output);
    } catch (const tf::TransformException& exception) {
      ROS_WARN_THROTTLE(
          1.0,
          "point_stamped_tf_adapter: failed to transform waypoint (%s <- %s): %s",
          target_frame_.c_str(), input->header.frame_id.c_str(),
          exception.what());
    }
  }

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Subscriber subscriber_;
  ros::Publisher publisher_;
  tf::TransformListener tf_listener_;
  std::string input_topic_;
  std::string output_topic_;
  std::string target_frame_;
  double transform_timeout_ = 0.2;
};

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "point_stamped_tf_adapter");
  PointStampedTfAdapter adapter;
  ros::spin();
  return 0;
}
