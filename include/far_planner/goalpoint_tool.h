#ifndef FAR_PLANNER_GOALPOINT_TOOL_H
#define FAR_PLANNER_GOALPOINT_TOOL_H

#include <ros/ros.h>

#include <nav_msgs/Odometry.h>
#include <rviz/default_plugin/tools/pose_tool.h>
#include <tf/transform_listener.h>

namespace rviz {
class BoolProperty;
class RosTopicProperty;
class StringProperty;
}  // namespace rviz

namespace far_planner {

class GoalPointTool : public rviz::PoseTool {
  Q_OBJECT

 public:
  GoalPointTool();
  ~GoalPointTool() override = default;

  void onInitialize() override;

 protected:
  void onPoseSet(double x, double y, double theta) override;

 private Q_SLOTS:
  void updateTopics();

 private:
  void odomHandler(const nav_msgs::Odometry::ConstPtr& odom);

  ros::NodeHandle nh_;
  ros::Subscriber odom_sub_;
  ros::Publisher goal_pub_;
  ros::Publisher joy_pub_;
  tf::TransformListener tf_listener_;
  nav_msgs::Odometry::ConstPtr latest_odom_;

  rviz::RosTopicProperty* goal_topic_property_;
  rviz::RosTopicProperty* odom_topic_property_;
  rviz::RosTopicProperty* joy_topic_property_;
  rviz::StringProperty* goal_frame_property_;
  rviz::BoolProperty* publish_joy_property_;

};

}  // namespace far_planner

#endif  // FAR_PLANNER_GOALPOINT_TOOL_H
