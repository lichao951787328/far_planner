#include "far_planner/goalpoint_tool.h"

#include <cmath>
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
      publish_joy_property_(nullptr) {
  shortcut_key_ = 'w';

  goal_topic_property_ = new rviz::RosTopicProperty(
      "Goal Topic", "/goal_point", "geometry_msgs/PointStamped",
      "PointStamped topic consumed by FAR Planner.", getPropertyContainer(),
      SLOT(updateTopics()), this);
  odom_topic_property_ = new rviz::RosTopicProperty(
      "Odometry Topic", "/fusion_localization", "nav_msgs/Odometry",
      "Odometry used to set the goal height in the RViz Fixed Frame.",
      getPropertyContainer(), SLOT(updateTopics()), this);
  goal_frame_property_ = new rviz::StringProperty(
      "Goal Frame", "map",
      "Optional output frame. The clicked point is transformed from RViz Fixed Frame; "
      "leave empty to publish in Fixed Frame.",
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
  latest_odom_ = odom;
}

void GoalPointTool::onPoseSet(double x, double y, double /*theta*/) {
  if (!goal_pub_) {
    ROS_ERROR_THROTTLE(1.0, "GoalPointTool: Goal Topic is empty");
    return;
  }

  const std::string fixed_frame =
      context_ != nullptr ? context_->getFixedFrame().toStdString() : "";
  if (fixed_frame.empty()) {
    ROS_ERROR_THROTTLE(1.0, "GoalPointTool: RViz Fixed Frame is empty");
    return;
  }
  const std::string configured_goal_frame = goal_frame_property_->getStdString();
  const std::string goal_frame = configured_goal_frame.empty()
                                     ? fixed_frame : configured_goal_frame;

  geometry_msgs::PointStamped goal;
  goal.header.stamp = ros::Time::now();
  // Some recorded bags publish /clock in the record-time epoch while odometry
  // and TF retain their acquisition-time epoch. In that case, use the most
  // recent odometry acquisition stamp as the TF reference rather than asking
  // for a transform millions of seconds outside the recorded TF buffer.
  if (latest_odom_ != nullptr && !latest_odom_->header.stamp.isZero() &&
      !goal.header.stamp.isZero() &&
      std::abs((goal.header.stamp - latest_odom_->header.stamp).toSec()) > 60.0) {
    ROS_WARN_THROTTLE(
        2.0, "GoalPointTool: /clock and odometry use different time domains; "
             "using latest odometry stamp for goal TF");
    goal.header.stamp = latest_odom_->header.stamp;
  }
  goal.header.frame_id = fixed_frame;
  goal.point.x = x;
  goal.point.y = y;
  goal.point.z = 0.0;

  if (latest_odom_ != nullptr && !latest_odom_->header.frame_id.empty()) {
    geometry_msgs::PointStamped vehicle;
    vehicle.header = latest_odom_->header;
    vehicle.point = latest_odom_->pose.pose.position;
    if (vehicle.header.frame_id == fixed_frame) {
      goal.point.z = vehicle.point.z;
    } else {
      try {
        geometry_msgs::PointStamped vehicle_in_fixed;
        tf_listener_.transformPoint(fixed_frame, vehicle, vehicle_in_fixed);
        goal.point.z = vehicle_in_fixed.point.z;
      } catch (const tf::TransformException& ex) {
        ROS_WARN_THROTTLE(
            1.0, "GoalPointTool: odometry height TF unavailable; using z=0: %s",
            ex.what());
      }
    }
  }

  // PoseTool's x/y are in RViz Fixed Frame. Never change only the frame label:
  // an explicit Goal Frame requires an actual transform at the goal stamp.
  if (goal_frame != fixed_frame) {
    try {
      geometry_msgs::PointStamped transformed_goal;
      tf_listener_.waitForTransform(goal_frame, fixed_frame,
                                    goal.header.stamp, ros::Duration(0.2));
      tf_listener_.transformPoint(goal_frame, goal, transformed_goal);
      goal = transformed_goal;
    } catch (const tf::TransformException& ex) {
      ROS_ERROR_THROTTLE(
          1.0, "GoalPointTool: cannot transform click from '%s' to '%s': %s",
          fixed_frame.c_str(), goal_frame.c_str(), ex.what());
      return;
    }
  }

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
