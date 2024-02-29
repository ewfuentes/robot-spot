#! /usr/bin/env python3
from __future__ import annotations

import argparse
import sys
import math

import rospy
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import TransformStamped
from apriltag_ros.msg import AprilTagDetectionArray

from experimental.beacon_sim import ekf_slam_python as esp
from experimental.beacon_sim.generate_observations_python import BeaconObservation 
from common.time import robot_time_python as rtp
from common.liegroups import se2_python as se2

from typing import NamedTuple
import numpy as np
np.set_printoptions(linewidth=200)



def compute_observations(detections, tf_buffer):
    observations = []
    for detection in detections:
        stamped_pose = detection.pose
        camera_frame = stamped_pose.header.frame_id

        body_from_camera = tf_buffer.lookup_transform(
                'body', camera_frame, rospy.Time.now(), rospy.Duration(1.0))

        id = detection.id[0]
        stamp = stamped_pose.header.stamp
        time_of_validity = rtp.RobotTimestamp() + rtp.as_duration(stamp.secs) + rtp.as_duration(stamp.nsecs / 1e9)

        camera_from_tag = detection.pose.pose

        body_from_tag = tf2_geometry_msgs.do_transform_pose(camera_from_tag, body_from_camera)

        range_m_sq = body_from_tag.pose.position.x**2 + body_from_tag.pose.position.y**2
        range_m = math.sqrt(range_m_sq)
        bearing_rad = math.atan2(body_from_tag.pose.position.y, body_from_tag.pose.position.x)

        observations.append((time_of_validity, BeaconObservation(id, range_m, bearing_rad)))
    return observations


def tag_callback(detection_msg, tf_buffer, ekf):
    ekf_config = esp.EkfSlamConfig(
        max_num_beacons=2,
        initial_beacon_uncertainty_m=100.0,
        along_track_process_noise_m_per_rt_meter=0.1,
        cross_track_process_noise_m_per_rt_meter=0.01,
        pos_process_noise_m_per_rt_s=0.01,
        heading_process_noise_rad_per_rt_meter=0.001,
        heading_process_noise_rad_per_rt_s=0.00001,
        beacon_pos_process_noise_m_per_rt_s=1.0,
        range_measurement_noise_m=0.05,
        bearing_measurement_noise_rad=0.005,
        on_map_load_position_uncertainty_m=0.1,
        on_map_load_heading_uncertainty_rad=0.01,
    )

    observations = compute_observations(detection_msg.detections, tf_buffer)
    if len(observations) == 0:
        return


    observations = sorted(observations, key=lambda x: x[0])

    for obs in observations:
        if ekf is None:
            ekf = esp.EkfSlam(ekf_config, obs[0])

        ekf.predict(obs[0], se2.SE2(0.0))
        ekf.update([obs[1]])

    rospy.loginfo('*' * 30)
    rospy.loginfo('estimate:')
    rospy.loginfo(ekf.estimate().mean)

        


def main():
    rospy.init_node('observation_node')

    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)
    tag_detection_subscribers = []

    ekf = None

    for camera in ['frontleft', 'frontright', 'left', 'back', 'right']:
        topic_name = f"/spot/apriltag/{camera}/tag_detections"
        tag_detection_subscribers.append(
                rospy.Subscriber(
                    topic_name, AprilTagDetectionArray, lambda data: tag_callback(data, tf_buffer, ekf)
                    ))

    try:
        rospy.spin()
    except KeyboardInterrupt:
        ...

    rospy.loginfo("finished")


if __name__ == '__main__':
    main()
