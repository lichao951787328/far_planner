#!/usr/bin/env python3
"""Replay one recorded FAR goal after checking the recorded start pose."""

import argparse
import csv
import math
import os
import sys
import time

import rospy
import rospkg
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry


def load_record(path, sequence):
    with open(path, newline="") as stream:
        rows = list(csv.DictReader(stream))
    if not rows:
        raise RuntimeError("goal record contains no selections")
    if sequence is None:
        return rows[-1]
    for row in rows:
        if int(row["sequence"]) == sequence:
            return row
    raise RuntimeError("sequence {} was not found".format(sequence))


def main():
    default_file = os.path.join(
        rospkg.RosPack().get_path("far_planner"),
        "logs", "goal_selections.csv")
    parser = argparse.ArgumentParser()
    parser.add_argument("--file", default=default_file)
    parser.add_argument("--sequence", type=int)
    parser.add_argument("--odom-topic", default="/odom_world")
    parser.add_argument("--goal-topic", default="/goal_point")
    parser.add_argument("--start-tolerance", type=float, default=0.75)
    parser.add_argument("--force", action="store_true",
                        help="publish even when current and recorded starts differ")
    args = parser.parse_args(rospy.myargv()[1:])

    try:
        record = load_record(args.file, args.sequence)
    except (OSError, ValueError, RuntimeError) as error:
        print("replay_goal_selection: {}".format(error), file=sys.stderr)
        return 2

    rospy.init_node("replay_goal_selection", anonymous=True)
    try:
        odom = rospy.wait_for_message(args.odom_topic, Odometry, timeout=5.0)
    except rospy.ROSException as error:
        rospy.logerr("Cannot read current odometry: %s", error)
        return 3

    recorded_start = (
        float(record["start_x"]), float(record["start_y"]),
        float(record["start_z"]))
    current = odom.pose.pose.position
    start_error = math.sqrt(
        (current.x - recorded_start[0]) ** 2 +
        (current.y - recorded_start[1]) ** 2 +
        (current.z - recorded_start[2]) ** 2)
    if start_error > args.start_tolerance and not args.force:
        rospy.logerr(
            "Recorded start differs by %.3f m (limit %.3f m). "
            "Move/reset the robot to (%.3f, %.3f, %.3f), or use --force.",
            start_error, args.start_tolerance, *recorded_start)
        return 4


    publisher = rospy.Publisher(args.goal_topic, PointStamped,
                                queue_size=1, latch=True)
    deadline = time.monotonic() + 2.0
    while publisher.get_num_connections() == 0 and time.monotonic() < deadline:
        rospy.sleep(0.05)
    message = PointStamped()
    message.header.stamp = rospy.Time.now()
    message.header.frame_id = record["world_frame"]
    message.point.x = float(record["goal_x"])
    message.point.y = float(record["goal_y"])
    message.point.z = float(record["goal_z"])
    publisher.publish(message)
    rospy.sleep(0.25)
    rospy.loginfo(
        "Replayed sequence %s selected at %s: start error %.3f m, "
        "goal=(%.3f, %.3f, %.3f).",
        record["sequence"], record["system_time"], start_error,
        message.point.x, message.point.y, message.point.z)
    return 0


if __name__ == "__main__":
    sys.exit(main())
