#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>

#include <string>

namespace {

class PoseStampedToPointStamped {
public:
  PoseStampedToPointStamped() : private_nh_("~") {
    private_nh_.param<std::string>("input_goal_topic", input_topic_,
                                   "/move_base_simple/goal");
    private_nh_.param<std::string>("output_goal_topic", output_topic_,
                                   "/goal_point");
    private_nh_.param<std::string>("default_frame", default_frame_, "world");

    publisher_ = nh_.advertise<geometry_msgs::PointStamped>(output_topic_, 1);
    subscriber_ = nh_.subscribe(input_topic_, 1,
                                &PoseStampedToPointStamped::GoalCallback, this);
    ROS_INFO_STREAM("PoseStamped goal adapter: " << input_topic_ << " -> "
                    << output_topic_);
  }

private:
  void GoalCallback(const geometry_msgs::PoseStampedConstPtr& input) {
    if (!input) return;
    geometry_msgs::PointStamped output;
    output.header = input->header;
    if (output.header.frame_id.empty()) output.header.frame_id = default_frame_;
    if (output.header.stamp.isZero()) output.header.stamp = ros::Time::now();
    output.point = input->pose.position;
    publisher_.publish(output);
  }

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Subscriber subscriber_;
  ros::Publisher publisher_;
  std::string input_topic_;
  std::string output_topic_;
  std::string default_frame_;
};

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "pose_stamped_to_point_stamped");
  PoseStampedToPointStamped adapter;
  ros::spin();
  return 0;
}
