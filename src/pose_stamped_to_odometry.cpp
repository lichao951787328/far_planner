#include <cmath>
#include <string>

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

namespace {

double NormalizeAngle(double angle) {
  while (angle > M_PI) angle -= 2.0 * M_PI;
  while (angle < -M_PI) angle += 2.0 * M_PI;
  return angle;
}

std::string NormalizeFrame(const std::string& frame) {
  return !frame.empty() && frame.front() == '/' ? frame.substr(1) : frame;
}

class PoseStampedToOdometry {
 public:
  PoseStampedToOdometry() : private_nh_("~") {
    private_nh_.param<std::string>("input_pose_topic", input_pose_topic_,
                                   "/model/go2_semantic/pose");
    private_nh_.param<std::string>("output_odom_topic", output_odom_topic_,
                                   "/odom_world");
    private_nh_.param<std::string>("world_frame", world_frame_, "world");
    private_nh_.param<std::string>("child_frame", child_frame_, "base_link");
    private_nh_.param<bool>("publish_tf", publish_tf_, false);
    private_nh_.param<bool>("require_matching_input_frame",
                            require_matching_input_frame_, false);
    private_nh_.param<double>("maximum_twist_interval",
                              maximum_twist_interval_, 0.5);

    odom_pub_ = nh_.advertise<nav_msgs::Odometry>(output_odom_topic_, 20);
    pose_sub_ = nh_.subscribe(input_pose_topic_, 100,
                              &PoseStampedToOdometry::PoseCallback, this);
    ROS_INFO("pose_stamped_to_odometry: %s -> %s, frame %s -> %s",
             input_pose_topic_.c_str(), output_odom_topic_.c_str(),
             world_frame_.c_str(), child_frame_.c_str());
  }

 private:
  void PoseCallback(const geometry_msgs::PoseStampedConstPtr& pose) {
    if (!pose) return;
    if (!pose->header.frame_id.empty() &&
        NormalizeFrame(pose->header.frame_id) != NormalizeFrame(world_frame_)) {
      if (require_matching_input_frame_) {
        ROS_ERROR_THROTTLE(
            1.0,
            "pose_stamped_to_odometry: input frame '%s' does not match configured world frame '%s'.",
            pose->header.frame_id.c_str(), world_frame_.c_str());
        return;
      }
      ROS_WARN_THROTTLE(
          5.0,
          "pose_stamped_to_odometry: relabeling numeric pose frame '%s' as '%s'; ensure both frames represent the same Gazebo world.",
          pose->header.frame_id.c_str(), world_frame_.c_str());
    }

    nav_msgs::Odometry odom;
    odom.header.stamp = pose->header.stamp.isZero()
                            ? ros::Time::now()
                            : pose->header.stamp;
    odom.header.frame_id = world_frame_;
    odom.child_frame_id = child_frame_;
    odom.pose.pose = pose->pose;

    const double yaw = tf::getYaw(pose->pose.orientation);
    if (has_previous_pose_) {
      const double dt = (odom.header.stamp - previous_stamp_).toSec();
      if (dt > 0.0 &&
          (maximum_twist_interval_ <= 0.0 ||
           dt <= maximum_twist_interval_)) {
        const double vx_world =
            (pose->pose.position.x - previous_pose_.position.x) / dt;
        const double vy_world =
            (pose->pose.position.y - previous_pose_.position.y) / dt;
        odom.twist.twist.linear.x =
            std::cos(yaw) * vx_world + std::sin(yaw) * vy_world;
        odom.twist.twist.linear.y =
            -std::sin(yaw) * vx_world + std::cos(yaw) * vy_world;
        odom.twist.twist.linear.z =
            (pose->pose.position.z - previous_pose_.position.z) / dt;
        odom.twist.twist.angular.z =
            NormalizeAngle(yaw - previous_yaw_) / dt;
      }
    }

    odom_pub_.publish(odom);
    if (publish_tf_) {
      tf::Transform transform;
      tf::poseMsgToTF(pose->pose, transform);
      tf_broadcaster_.sendTransform(tf::StampedTransform(
          transform, odom.header.stamp, world_frame_, child_frame_));
    }

    previous_pose_ = pose->pose;
    previous_stamp_ = odom.header.stamp;
    previous_yaw_ = yaw;
    has_previous_pose_ = true;
  }

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Subscriber pose_sub_;
  ros::Publisher odom_pub_;
  tf::TransformBroadcaster tf_broadcaster_;
  std::string input_pose_topic_;
  std::string output_odom_topic_;
  std::string world_frame_;
  std::string child_frame_;
  bool publish_tf_ = false;
  bool require_matching_input_frame_ = false;
  double maximum_twist_interval_ = 0.5;
  bool has_previous_pose_ = false;
  geometry_msgs::Pose previous_pose_;
  ros::Time previous_stamp_;
  double previous_yaw_ = 0.0;
};

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "pose_stamped_to_odometry");
  PoseStampedToOdometry converter;
  ros::spin();
  return 0;
}
