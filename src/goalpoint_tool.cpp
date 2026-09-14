#include "far_planner/goalpoint_tool.h"

#include <geometry_msgs/PointStamped.h>
#include <pluginlib/class_list_macros.hpp>
#include <sensor_msgs/Joy.h>

#include <rviz/display_context.h>
#include <rviz/properties/bool_property.h>
#include <rviz/properties/ros_topic_property.h>
#include <rviz/properties/string_property.h>

namespace far_planner {

GoalPointTool::GoalPointTool()
    : goal_topic_property_(nullptr),
      odom_topic_property_(nullptr),
      joy_topic_property_(nullptr),
      goal_frame_property_(nullptr),
      publish_joy_property_(nullptr),
      vehicle_z_(0.0) {
  shortcut_key_ = 'w';

  goal_topic_property_ = new rviz::RosTopicProperty(
      "Goal Topic", "/goal_point", "geometry_msgs/PointStamped",
      "PointStamped topic consumed by FAR Planner.", getPropertyContainer(),
      SLOT(updateTopics()), this);
  odom_topic_property_ = new rviz::RosTopicProperty(
      "Odometry Topic", "/fusion_localization", "nav_msgs/Odometry",
      "Odometry used only to copy the vehicle height into the selected goal.",
      getPropertyContainer(), SLOT(updateTopics()), this);
  goal_frame_property_ = new rviz::StringProperty(
      "Goal Frame", "map",
      "Frame placed in the PointStamped header. Leave empty to use RViz Fixed Frame.",
      getPropertyContainer());
  publish_joy_property_ = new rviz::BoolProperty(
      "Publish Joy Start", true,
      "Also publish the original FAR start command after selecting a goal.",
      getPropertyContainer(), SLOT(updateTopics()), this);
  joy_topic_property_ = new rviz::RosTopicProperty(
      "Joy Topic", "/joy", "sensor_msgs/Joy",
      "Topic for the optional original FAR start command.",
      getPropertyContainer(), SLOT(updateTopics()), this);
}

void GoalPointTool::onInitialize() {
  rviz::PoseTool::onInitialize();
  setName("Goalpoint");
  updateTopics();
}

void GoalPointTool::updateTopics() {
  odom_sub_.shutdown();
  goal_pub_.shutdown();
  joy_pub_.shutdown();

  const std::string odom_topic = odom_topic_property_->getTopicStd();
  const std::string goal_topic = goal_topic_property_->getTopicStd();
  if (!odom_topic.empty()) {
    odom_sub_ = nh_.subscribe(odom_topic, 5, &GoalPointTool::odomHandler, this);
  }
  if (!goal_topic.empty()) {
    goal_pub_ = nh_.advertise<geometry_msgs::PointStamped>(goal_topic, 5);
  }
  if (publish_joy_property_->getBool()) {
    const std::string joy_topic = joy_topic_property_->getTopicStd();
    if (!joy_topic.empty()) {
      joy_pub_ = nh_.advertise<sensor_msgs::Joy>(joy_topic, 5);
    }
  }
}

void GoalPointTool::odomHandler(const nav_msgs::Odometry::ConstPtr& odom) {
  vehicle_z_ = odom->pose.pose.position.z;
}

void GoalPointTool::onPoseSet(double x, double y, double /*theta*/) {
  if (!goal_pub_) {
    ROS_ERROR_THROTTLE(1.0, "GoalPointTool: Goal Topic is empty");
    return;
  }

  std::string goal_frame = goal_frame_property_->getStdString();
  if (goal_frame.empty() && context_ != nullptr) {
    goal_frame = context_->getFixedFrame().toStdString();
  }

  geometry_msgs::PointStamped goal;
  goal.header.stamp = ros::Time::now();
  goal.header.frame_id = goal_frame;
  goal.point.x = x;
  goal.point.y = y;
  goal.point.z = vehicle_z_;

  // Two publications preserve the behavior of the original FAR plugin and
  // make a single click robust to a just-created subscriber connection.
  goal_pub_.publish(goal);
  ros::WallDuration(0.01).sleep();
  goal_pub_.publish(goal);

  if (publish_joy_property_->getBool() && joy_pub_) {
    sensor_msgs::Joy joy;
    joy.header.stamp = ros::Time::now();
    joy.header.frame_id = "goalpoint_tool";
    joy.axes = {0.0f, 0.0f, -1.0f, 0.0f, 1.0f, 1.0f, 0.0f, 0.0f};
    joy.buttons = {0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0};
    joy_pub_.publish(joy);
  }
}

}  // namespace far_planner

PLUGINLIB_EXPORT_CLASS(far_planner::GoalPointTool, rviz::Tool)
