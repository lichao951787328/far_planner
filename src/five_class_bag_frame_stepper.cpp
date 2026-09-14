#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <nav_msgs/Odometry.h>
#include <rosgraph_msgs/Clock.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Header.h>
#include <std_srvs/Trigger.h>
#include <tf2_msgs/TFMessage.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <algorithm>
#include <chrono>
#include <condition_variable>
#include <fstream>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

namespace {

std::vector<std::string> TopicAliases(const std::string& topic) {
    if (topic.empty()) return {};
    if (topic.front() == '/') return {topic, topic.substr(1)};
    return {topic, "/" + topic};
}

class FiveClassBagFrameStepper {
public:
    FiveClassBagFrameStepper()
        : private_nh_("~"), tf_listener_(tf_buffer_) {
        private_nh_.param<std::string>("bag", bag_path_, std::string());
        private_nh_.param<double>("bag_start", bag_start_, 0.0);
        private_nh_.param<std::string>("cloud_topic", cloud_topic_,
                                       "/grids_points");
        private_nh_.param<std::string>("odom_topic", odom_topic_,
                                       "/fusion_localization");
        private_nh_.param<std::string>("tf_topic", tf_topic_, "/tf");
        private_nh_.param<std::string>(
            "voxel_ack_topic", voxel_ack_topic_,
            "/local_3d_semantic_voxel_map/voxel_cloud");
        private_nh_.param<std::string>("far_ack_topic", far_ack_topic_,
                                       "/far_debug/frame_done");
        private_nh_.param<std::string>("world_frame", world_frame_, "map");
        private_nh_.param<double>("tf_pre_roll", tf_pre_roll_, 2.0);
        private_nh_.param<double>("tf_lookahead", tf_lookahead_, 0.25);
        private_nh_.param<double>("subscriber_timeout",
                                  subscriber_timeout_, 10.0);
        private_nh_.param<double>("processing_timeout",
                                  processing_timeout_, 30.0);
        if (bag_path_.empty()) {
            throw std::runtime_error("~bag is required");
        }

        // rosbag::Bag::open() also throws when the path is wrong, but its
        // exception is easy to miss among roslaunch's required-node shutdown
        // messages. Fail first with the fully expanded path so the launch
        // argument that needs correcting is immediately visible.
        std::ifstream bag_file(bag_path_, std::ios::in | std::ios::binary);
        if (!bag_file.good()) {
            throw std::runtime_error(
                "cannot read bag '" + bag_path_ +
                "'; pass bag:=/absolute/path/to/file.bag or fix the launch default");
        }
        bag_file.close();

        bag_.open(bag_path_, rosbag::bagmode::Read);
        rosbag::View full_view(bag_);
        const ros::Time bag_begin = full_view.getBeginTime();
        const ros::Time requested_start =
            bag_begin + ros::Duration(std::max(0.0, bag_start_));
        const ros::Time dependency_start = std::max(
            bag_begin,
            requested_start - ros::Duration(std::max(0.0, tf_pre_roll_)));

        cloud_view_.reset(new rosbag::View(
            bag_, rosbag::TopicQuery(TopicAliases(cloud_topic_)),
            requested_start));
        odom_view_.reset(new rosbag::View(
            bag_, rosbag::TopicQuery(TopicAliases(odom_topic_)),
            dependency_start));
        tf_view_.reset(new rosbag::View(
            bag_, rosbag::TopicQuery(TopicAliases(tf_topic_)),
            dependency_start));
        static_tf_view_.reset(new rosbag::View(
            bag_, rosbag::TopicQuery(TopicAliases("/tf_static"))));

        cloud_iterator_.reset(
            new rosbag::View::iterator(cloud_view_->begin()));
        odom_iterator_.reset(
            new rosbag::View::iterator(odom_view_->begin()));
        tf_iterator_.reset(
            new rosbag::View::iterator(tf_view_->begin()));

        cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(
            cloud_topic_, 1);
        odom_pub_ = nh_.advertise<nav_msgs::Odometry>(odom_topic_, 20);
        tf_pub_ = nh_.advertise<tf2_msgs::TFMessage>(tf_topic_, 100);
        // Match the production bag launch: recorded static transforms are
        // visible for inspection but never contaminate the live map_start tree.
        recorded_static_tf_pub_ = nh_.advertise<tf2_msgs::TFMessage>(
            "/recorded_tf_static", 10, true);
        clock_pub_ = nh_.advertise<rosgraph_msgs::Clock>("/clock", 10, true);

        voxel_sub_ = nh_.subscribe(
            voxel_ack_topic_, 10,
            &FiveClassBagFrameStepper::VoxelCallback, this);
        far_sub_ = nh_.subscribe(
            far_ack_topic_, 10,
            &FiveClassBagFrameStepper::FarCallback, this);
        next_service_ = private_nh_.advertiseService(
            "next", &FiveClassBagFrameStepper::Next, this);
        skip_service_ = private_nh_.advertiseService(
            "skip", &FiveClassBagFrameStepper::Skip, this);
        status_service_ = private_nh_.advertiseService(
            "status", &FiveClassBagFrameStepper::Status, this);

        PublishRecordedStaticTransforms();
        SetStatus("READY: call ~next for the first /grids_points frame");
        ROS_INFO("Five-class bag frame stepper opened %s at +%.3f s; "
                 "one ~next call publishes exactly one cloud after TF validation.",
                 bag_path_.c_str(), bag_start_);
    }

    ~FiveClassBagFrameStepper() {
        bag_.close();
    }

private:
    bool LoadPendingCloud(std::string* error) {
        if (pending_cloud_) return true;
        while (*cloud_iterator_ != cloud_view_->end()) {
            const rosbag::MessageInstance instance = **cloud_iterator_;
            ++(*cloud_iterator_);
            sensor_msgs::PointCloud2::ConstPtr cloud =
                instance.instantiate<sensor_msgs::PointCloud2>();
            if (!cloud) continue;
            pending_cloud_ = cloud;
            pending_record_time_ = instance.getTime();
            ++source_frame_index_;
            return true;
        }
        if (error) *error = "end of bag";
        return false;
    }

    void PublishClock(const ros::Time& time) {
        rosgraph_msgs::Clock clock;
        clock.clock = time;
        clock_pub_.publish(clock);
    }

    void PublishDependencies(const ros::Time& record_time) {
        const ros::Time tf_limit =
            record_time + ros::Duration(std::max(0.0, tf_lookahead_));
        std::vector<tf2_msgs::TFMessage> pending_tf_messages;
        while (*tf_iterator_ != tf_view_->end()) {
            const rosbag::MessageInstance instance = **tf_iterator_;
            if (instance.getTime() > tf_limit) break;
            ++(*tf_iterator_);
            tf2_msgs::TFMessage::ConstPtr message =
                instance.instantiate<tf2_msgs::TFMessage>();
            if (message) pending_tf_messages.push_back(*message);
        }

        // Some bags start with a cloud slightly before their first dynamic
        // TF sample (the 2026-08-13 bag is 100 ms early). Normal rosbag play
        // silently drops that cloud. A frame stepper must not: bootstrap each
        // previously unseen child frame from its nearest look-ahead sample,
        // stamped at the pending cloud time. This happens only once per child;
        // all later frames use the original recorded TF timestamps.
        std::unordered_map<std::string, geometry_msgs::TransformStamped>
            bootstrap_candidates;
        for (const tf2_msgs::TFMessage& message : pending_tf_messages) {
            for (const geometry_msgs::TransformStamped& transform :
                 message.transforms) {
                const std::string& child = transform.child_frame_id;
                const auto earliest = earliest_tf_stamp_.find(child);
                if (earliest == earliest_tf_stamp_.end() ||
                    transform.header.stamp < earliest->second) {
                    earliest_tf_stamp_[child] = transform.header.stamp;
                }
                if (transform.header.stamp <= pending_cloud_->header.stamp) {
                    continue;
                }
                const auto candidate = bootstrap_candidates.find(child);
                if (candidate == bootstrap_candidates.end() ||
                    transform.header.stamp <
                        candidate->second.header.stamp) {
                    bootstrap_candidates[child] = transform;
                }
            }
        }
        tf2_msgs::TFMessage bootstrap;
        for (const auto& item : bootstrap_candidates) {
            const auto earliest = earliest_tf_stamp_.find(item.first);
            if (earliest != earliest_tf_stamp_.end() &&
                earliest->second > pending_cloud_->header.stamp) {
                geometry_msgs::TransformStamped transform = item.second;
                transform.header.stamp = pending_cloud_->header.stamp;
                bootstrap.transforms.push_back(transform);
                earliest_tf_stamp_[item.first] = transform.header.stamp;
            }
        }
        if (!bootstrap.transforms.empty()) {
            tf_pub_.publish(bootstrap);
            ROS_WARN("bag_stepper: bootstrapped %zu TF child frame(s) at "
                     "the first cloud stamp; nearest future sample is used.",
                     bootstrap.transforms.size());
        }
        for (const tf2_msgs::TFMessage& message : pending_tf_messages) {
            tf_pub_.publish(message);
        }
        while (*odom_iterator_ != odom_view_->end()) {
            const rosbag::MessageInstance instance = **odom_iterator_;
            if (instance.getTime() > record_time) break;
            ++(*odom_iterator_);
            nav_msgs::Odometry::ConstPtr message =
                instance.instantiate<nav_msgs::Odometry>();
            if (!message) continue;
            latest_odom_ = message;
        }
    }

    bool WaitForSubscribers(std::string* error) {
        const ros::WallTime deadline =
            ros::WallTime::now() + ros::WallDuration(subscriber_timeout_);
        ros::WallRate rate(50.0);
        while (ros::ok() && ros::WallTime::now() < deadline) {
            if (cloud_pub_.getNumSubscribers() > 0 &&
                tf_pub_.getNumSubscribers() > 0 &&
                odom_pub_.getNumSubscribers() > 0) {
                return true;
            }
            rate.sleep();
        }
        if (error) {
            std::ostringstream stream;
            stream << "subscriber timeout cloud/tf/odom="
                   << cloud_pub_.getNumSubscribers() << "/"
                   << tf_pub_.getNumSubscribers() << "/"
                   << odom_pub_.getNumSubscribers();
            *error = stream.str();
        }
        return false;
    }

    bool WaitForFrameTransform(std::string* error) {
        const ros::WallTime deadline =
            ros::WallTime::now() + ros::WallDuration(2.0);
        ros::WallRate rate(100.0);
        while (ros::ok() && ros::WallTime::now() < deadline) {
            if (tf_buffer_.canTransform(
                    world_frame_, pending_cloud_->header.frame_id,
                    pending_cloud_->header.stamp)) {
                try {
                    tf_buffer_.lookupTransform(
                        world_frame_, pending_cloud_->header.frame_id,
                        pending_cloud_->header.stamp);
                    return true;
                } catch (const tf2::TransformException&) {
                }
            }
            rate.sleep();
        }
        if (error) {
            std::ostringstream stream;
            stream << "TF_MISSING " << world_frame_ << " <- "
                   << pending_cloud_->header.frame_id << " at "
                   << pending_cloud_->header.stamp.toSec();
            *error = stream.str();
        }
        return false;
    }

    bool WaitForAck(bool wait_for_far, std::string* error) {
        std::unique_lock<std::mutex> lock(ack_mutex_);
        const auto deadline = std::chrono::steady_clock::now() +
            std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                std::chrono::duration<double>(processing_timeout_));
        const bool received = ack_condition_.wait_until(
            lock, deadline, [&]() {
                return wait_for_far ? far_received_ : voxel_received_;
            });
        if (!received && error) {
            *error = wait_for_far ? "timeout waiting for FAR frame_done"
                                  : "timeout waiting for voxel_cloud";
        }
        return received;
    }

    bool Next(std_srvs::Trigger::Request&,
              std_srvs::Trigger::Response& response) {
        std::unique_lock<std::mutex> step_lock(step_mutex_, std::try_to_lock);
        if (!step_lock.owns_lock()) {
            response.success = false;
            response.message = "BUSY: the previous frame is still processing";
            return true;
        }

        std::string error;
        if (!LoadPendingCloud(&error)) {
            SetStatus("DONE: " + error);
            response.success = false;
            response.message = status_;
            return true;
        }
        if (!WaitForSubscribers(&error)) {
            SetStatus("WAIT_SUBSCRIBERS: " + error);
            response.success = false;
            response.message = status_;
            return true;
        }

        PublishClock(pending_record_time_);
        PublishDependencies(pending_record_time_);
        ros::WallDuration(0.05).sleep();
        if (!WaitForFrameTransform(&error)) {
            SetStatus(error + "; frame remains pending, call ~next to retry or ~skip");
            response.success = false;
            response.message = status_;
            return true;
        }

        {
            std::lock_guard<std::mutex> lock(ack_mutex_);
            expected_stamp_ = pending_cloud_->header.stamp;
            voxel_received_ = false;
            far_received_ = false;
        }
        SetStatus("WAIT_VOXEL");
        cloud_pub_.publish(*pending_cloud_);
        if (!WaitForAck(false, &error)) {
            SetStatus("WAIT_VOXEL: " + error);
            response.success = false;
            response.message = status_;
            return true;
        }

        // The first usable cloud creates map_start. Never publish odometry
        // before voxel completion: FAR's odom callback waits for map_start,
        // and that ROS-time timeout cannot advance while stepped /clock is
        // paused. Publishing it here also keeps later frames transactional.
        if (latest_odom_) odom_pub_.publish(*latest_odom_);

        SetStatus("WAIT_FAR");
        if (!WaitForAck(true, &error)) {
            SetStatus("WAIT_FAR: " + error);
            response.success = false;
            response.message = status_;
            return true;
        }

        std::ostringstream stream;
        stream << "READY frame=" << source_frame_index_
               << " bag_time=" << std::fixed << pending_record_time_.toSec()
               << " cloud_stamp=" << expected_stamp_.toSec()
               << " voxel=PROCESSED far=PROCESSED";
        SetStatus(stream.str());
        pending_cloud_.reset();
        response.success = true;
        response.message = status_;
        return true;
    }

    bool Skip(std_srvs::Trigger::Request&,
              std_srvs::Trigger::Response& response) {
        std::lock_guard<std::mutex> step_lock(step_mutex_);
        std::string error;
        if (!LoadPendingCloud(&error)) {
            response.success = false;
            response.message = "DONE: " + error;
            return true;
        }
        std::ostringstream stream;
        stream << "SKIPPED frame=" << source_frame_index_
               << " stamp=" << pending_cloud_->header.stamp.toSec();
        pending_cloud_.reset();
        SetStatus(stream.str());
        response.success = true;
        response.message = status_;
        return true;
    }

    bool Status(std_srvs::Trigger::Request&,
                std_srvs::Trigger::Response& response) {
        std::lock_guard<std::mutex> lock(status_mutex_);
        response.success = true;
        response.message = status_;
        return true;
    }

    void VoxelCallback(const sensor_msgs::PointCloud2ConstPtr& message) {
        std::lock_guard<std::mutex> lock(ack_mutex_);
        if (!expected_stamp_.isZero() &&
            message->header.stamp == expected_stamp_) {
            voxel_received_ = true;
            ack_condition_.notify_all();
        }
    }

    void FarCallback(const std_msgs::HeaderConstPtr& message) {
        std::lock_guard<std::mutex> lock(ack_mutex_);
        if (!expected_stamp_.isZero() &&
            message->stamp == expected_stamp_) {
            far_received_ = true;
            ack_condition_.notify_all();
        }
    }

    void SetStatus(const std::string& status) {
        std::lock_guard<std::mutex> lock(status_mutex_);
        status_ = status;
        ROS_INFO_STREAM("bag_stepper: " << status_);
    }

    void PublishRecordedStaticTransforms() {
        for (const rosbag::MessageInstance& instance : *static_tf_view_) {
            tf2_msgs::TFMessage::ConstPtr message =
                instance.instantiate<tf2_msgs::TFMessage>();
            if (message) recorded_static_tf_pub_.publish(*message);
        }
    }

    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    rosbag::Bag bag_;
    std::unique_ptr<rosbag::View> cloud_view_, odom_view_, tf_view_;
    std::unique_ptr<rosbag::View> static_tf_view_;
    std::unique_ptr<rosbag::View::iterator> cloud_iterator_;
    std::unique_ptr<rosbag::View::iterator> odom_iterator_;
    std::unique_ptr<rosbag::View::iterator> tf_iterator_;

    ros::Publisher cloud_pub_, odom_pub_, tf_pub_, recorded_static_tf_pub_;
    ros::Publisher clock_pub_;
    ros::Subscriber voxel_sub_, far_sub_;
    ros::ServiceServer next_service_, skip_service_, status_service_;

    tf2_ros::Buffer tf_buffer_{ros::Duration(30.0)};
    tf2_ros::TransformListener tf_listener_;

    std::string bag_path_, cloud_topic_, odom_topic_, tf_topic_;
    std::string voxel_ack_topic_, far_ack_topic_, world_frame_;
    double bag_start_ = 0.0;
    double tf_pre_roll_ = 2.0;
    double tf_lookahead_ = 0.25;
    double subscriber_timeout_ = 10.0;
    double processing_timeout_ = 30.0;

    sensor_msgs::PointCloud2::ConstPtr pending_cloud_;
    nav_msgs::Odometry::ConstPtr latest_odom_;
    ros::Time pending_record_time_;
    ros::Time expected_stamp_;
    std::size_t source_frame_index_ = 0;
    std::unordered_map<std::string, ros::Time> earliest_tf_stamp_;

    std::mutex step_mutex_;
    std::mutex ack_mutex_;
    std::condition_variable ack_condition_;
    bool voxel_received_ = false;
    bool far_received_ = false;

    std::mutex status_mutex_;
    std::string status_;
};

}  // namespace

int main(int argc, char** argv) {
    ros::init(argc, argv, "five_class_bag_frame_stepper");
    try {
        FiveClassBagFrameStepper stepper;
        ros::AsyncSpinner spinner(4);
        spinner.start();
        ros::waitForShutdown();
    } catch (const std::exception& exception) {
        ROS_FATAL("five_class_bag_frame_stepper: %s", exception.what());
        return 1;
    }
    return 0;
}
