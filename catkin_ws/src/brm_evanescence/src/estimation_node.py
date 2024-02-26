#! /usr/bin/env python3
import argparse
import sys

import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from apriltag_ros.msg import AprilTagDetectionArray

from typing import NamedTuple

class Observation:
    id: int
    body_from_tag: TransformStamped


def compute_observations(detections, tf_buffer):
    for detection in detections:
        rospy.loginfo(detection)
        camera_frame = detection.pose.header.frame_id
        body_from_camera = tf_buffer.lookup_transform(
                'body', camera_frame, rospy.Time.now(), rospy.Duration(1.0))

        id = detection.id[0]

        camera_from_tag = detection.pose.pose.pose

        rospy.loginfo('*' * 20)
        rospy.loginfo(id)
        rospy.loginfo(body_from_camera)
        rospy.loginfo(camera_from_tag)


def tag_callback(detection_msg, tf_buffer):
    observations = []
    observations.append(
            compute_observations(detection_msg.detections, tf_buffer))

        


def main():
    rospy.init_node('observation_node')

    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)
    tag_detection_subscribers = []
    for camera in ['frontleft', 'frontright', 'left', 'back', 'right']:
        topic_name = f"/spot/apriltag/{camera}/tag_detections"
        tag_detection_subscribers.append(
                rospy.Subscriber(
                    topic_name, AprilTagDetectionArray, tag_callback,
                    callback_args=tf_buffer))

    try:
        rospy.spin()
    except KeyboardInterrupt:
        ...

    rospy.loginfo("finished")


if __name__ == '__main__':
    main()
