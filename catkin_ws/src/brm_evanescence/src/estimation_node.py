#! /usr/bin/env python3
from __future__ import annotations

import math

import rospy
import tf2_ros
import tf2_msgs
import tf2_geometry_msgs
import geometry_msgs
from apriltag_ros.msg import AprilTagDetectionArray
import threading
import datetime

from experimental.beacon_sim import ekf_slam_python as esp
from experimental.beacon_sim.generate_observations_python import BeaconObservation
from common.time import robot_time_python as rtp
from common.liegroups import se2_python as se2
from scipy.spatial.transform import Rotation

import numpy as np
np.set_printoptions(linewidth=200)

def ros_time_from_robot_time(timestamp: rtp.RobotTimestamp):
    one_second = datetime.timedelta(seconds=1)
    one_microsecond = datetime.timedelta(microseconds=1)
    past_seconds = timestamp.time_since_epoch() // one_second
    past_us = (timestamp.time_since_epoch() % one_second) // one_microsecond
    return rospy.Time(past_seconds, past_us * 1000)


def robot_se2_from_stamped_transform(b_from_a_msg):
    origin = tf2_geometry_msgs.PointStamped()
    unit_x = tf2_geometry_msgs.PointStamped()
    unit_x.point.x = 1.0
    origin_in_b = tf2_geometry_msgs.do_transform_point(origin, b_from_a_msg)
    x_in_b = tf2_geometry_msgs.do_transform_point(unit_x, b_from_a_msg)

    theta = math.atan2(x_in_b.point.y, x_in_b.point.x)
    return se2.SE2(theta, np.array([origin_in_b.point.x, origin_in_b.point.y]))

def stamped_transform_from_se2(b_from_a, b, a, ros_time):
    out = geometry_msgs.msg.TransformStamped()
    out.header.stamp.secs = ros_time.secs
    out.header.stamp.nsecs = ros_time.nsecs
    out.header.frame_id = b
    out.child_frame_id = a

    out.transform.translation.x = b_from_a.translation()[0]
    out.transform.translation.y = b_from_a.translation()[1]
    out.transform.translation.z = 0.0

    b_from_a_rot = Rotation.from_rotvec(np.array([0, 0, b_from_a.so2().log()]))
    b_from_a_quat = b_from_a_rot.as_quat()
    
    out.transform.rotation.x = b_from_a_quat[0]
    out.transform.rotation.y = b_from_a_quat[1]
    out.transform.rotation.z = b_from_a_quat[2]
    out.transform.rotation.w = b_from_a_quat[3]
    return out

def stamped_transform_from_point2(pt_in_b, b, a, ros_time):
    out = geometry_msgs.msg.TransformStamped()
    out.header.stamp.secs = ros_time.secs
    out.header.stamp.nsecs = ros_time.nsecs
    out.header.frame_id = b
    out.child_frame_id = a

    out.transform.translation.x = pt_in_b[0]
    out.transform.translation.y = pt_in_b[1]
    out.transform.translation.z = 0.0

    out.transform.rotation.x = 0.0
    out.transform.rotation.y = 0.0
    out.transform.rotation.z = 0.0
    out.transform.rotation.w = 1.0
    return out


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


def tag_callback(detection_msg, tf_buffer, ekf, ekf_lock):
    observations = compute_observations(detection_msg.detections, tf_buffer)
    if len(observations) == 0:
        return

    observations = sorted(observations, key=lambda x: x[0])

    with ekf_lock:
        for obs in observations:

            time, beacon_obs = obs
            ekf_tov = ekf.estimate.time_of_validity
            dt_us = (time - ekf_tov) / datetime.timedelta(microseconds=1)
            dt_days = (time - ekf_tov) / datetime.timedelta(days=1)

            if dt_us < -1e5:
                rospy.loginfo(f'received stale observations ekf tov: {ekf_tov} obs: {obs}')
                continue

            if dt_days > 1.0:
                # If there is a very large jump, just update the time of validity, this has
                # the effect of not adding any noise on the next observation
                ekf.estimate.time_of_validity = time

            if dt_us > 0.0:
                past_time = ros_time_from_robot_time(ekf.estimate.time_of_validity)
                obs_time = ros_time_from_robot_time(time)

                timeout_s = rospy.Duration(0.1)
                odom_from_past_robot = tf_buffer.lookup_transform('odom', 'body', past_time, timeout_s)
                odom_from_obs_robot = tf_buffer.lookup_transform('odom', 'body', obs_time, timeout_s)

                odom_from_past_robot = robot_se2_from_stamped_transform(odom_from_past_robot)
                odom_from_obs_robot = robot_se2_from_stamped_transform(odom_from_obs_robot)

                past_robot_from_obs_robot = odom_from_past_robot.inverse() * odom_from_obs_robot

                ekf.predict(time, past_robot_from_obs_robot)

            ekf.update([beacon_obs])


def timer_callback(timer_event, tf_buffer, ekf, ekf_lock, ros_time_from_bag_time, tf_publisher):
    with ekf_lock:
        # Publish TF transforms
        # is_offset_set = (ros_time_from_bag_time.secs != 0
        #                  or ros_time_from_bag_time.nsecs != 0)
        # if (not is_offset_set and
        #         ekf.estimate.time_of_validity != rtp.RobotTimestamp()):
        #     bag_time = ros_time_from_robot_time(ekf.estimate.time_of_validity)
        #     ros_time_from_bag_time = timer_event.current_expected - bag_time
        #     is_offset_set = True

        rospy.loginfo(f"timer: {timer_event.current_expected} ekf_time: {ekf.estimate.time_of_validity} offset: {ros_time_from_bag_time}")
        if ekf.estimate.time_of_validity == rtp.RobotTimestamp():
            return
            

        publish_time = ros_time_from_robot_time(ekf.estimate.time_of_validity) + ros_time_from_bag_time

        tfs = []
        tfs.append(stamped_transform_from_se2(
            ekf.estimate.local_from_robot(), 'map', 'body', publish_time))

        for beacon_id in ekf.estimate.beacon_ids:
            tfs.append(stamped_transform_from_point2(
                ekf.estimate.beacon_in_local(beacon_id),
                'map', f'map_tag_{beacon_id}', publish_time))

        tfm = tf2_msgs.msg.TFMessage(tfs)
        tf_publisher.publish(tfm)
        rospy.loginfo(f'Published {len(tfs)} transforms')


def main():
    rospy.init_node('observation_node')

    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    tf_publisher = rospy.Publisher('/tf', tf2_msgs.msg.TFMessage, queue_size=16)
    tag_detection_subscribers = []

    ekf_config = esp.EkfSlamConfig(
        max_num_beacons=10,
        initial_beacon_uncertainty_m=100.0,
        along_track_process_noise_m_per_rt_meter=0.1,
        cross_track_process_noise_m_per_rt_meter=0.01,
        pos_process_noise_m_per_rt_s=0.01,
        heading_process_noise_rad_per_rt_meter=0.001,
        heading_process_noise_rad_per_rt_s=0.00001,
        beacon_pos_process_noise_m_per_rt_s=0.001,
        range_measurement_noise_m=0.05,
        bearing_measurement_noise_rad=0.005,
        on_map_load_position_uncertainty_m=0.1,
        on_map_load_heading_uncertainty_rad=0.01,
    )

    ekf = esp.EkfSlam(ekf_config, rtp.RobotTimestamp())
    ekf_lock = threading.Lock()

    ros_time_from_bag_time = rospy.Duration(0)

    for camera in ['frontleft', 'frontright', 'left', 'back', 'right']:
        topic_name = f"/spot/apriltag/{camera}/tag_detections"
        tag_detection_subscribers.append(
                rospy.Subscriber(
                    topic_name,
                    AprilTagDetectionArray,
                    lambda data: tag_callback(data, tf_buffer, ekf, ekf_lock)
                ))

    rospy.Timer(rospy.Duration(0.02),
                lambda data: timer_callback(data, tf_buffer, ekf, ekf_lock, ros_time_from_bag_time, tf_publisher))

    try:
        rospy.spin()
    except KeyboardInterrupt:
        ...

    rospy.loginfo("finished")


if __name__ == '__main__':
    main()
