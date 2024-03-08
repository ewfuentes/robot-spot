#! /usr/bin/env python3
from __future__ import annotations

import math

import queue
import rospy
import tf2_ros
import tf2_msgs
import tf2_geometry_msgs
import std_msgs
import geometry_msgs.msg
from apriltag_ros.msg import AprilTagDetectionArray
import visualization_msgs.msg as viz
import threading
import datetime
import json
import sys

from experimental.beacon_sim import ekf_slam_python as esp
from experimental.beacon_sim.generate_observations_python import BeaconObservation
from common.time import robot_time_python as rtp
from common.liegroups import se2_python as se2
from scipy.spatial.transform import Rotation
from dataclasses import dataclass, field

import numpy as np

np.set_printoptions(linewidth=200)

BODY_FRAME = "flat_body"
MAP_FRAME = "map"


class ObservationsQueue:
    """
    This class is meant to handle the synchronization of tag detections across
    all cameras. All tag detections are accumulated in a priority queue sorted
    by publish time. Items can only be pulled from the queue once all cameras
    have published a detection message
    """

    @dataclass(order=True)
    class PrioritizedItem:
        receive_time: rtp.RobotTimestamp
        item: AprilTagDetectionArray = field(compare=False)

    def __init__(self, camera_list):
        self._lock = threading.RLock()
        self._queue = queue.PriorityQueue()
        self._last_update_time_from_camera = {
            c: rtp.RobotTimestamp() for c in camera_list
        }

    def insert(self, detections):
        with self._lock:
            detection_time = robot_time_from_ros_time(detections.header.stamp)
            camera_name = detections.header.frame_id
            self._last_update_time_from_camera[camera_name] = detection_time
            self._queue.put(self.PrioritizedItem(detection_time, detections))

    def are_samples_available(self):
        with self._lock:
            oldest_update_time = min(self._last_update_time_from_camera.values())
            return (
                not self._queue.empty()
                and oldest_update_time >= self._queue.queue[0].receive_time
            )

    def pop(self):
        with self._lock:
            if not self.are_samples_available():
                return None
            return self._queue.get().item


def robot_time_from_ros_time(stamp: ros.Time):
    return (
        rtp.RobotTimestamp()
        + rtp.as_duration(stamp.secs)
        + rtp.as_duration(stamp.nsecs / 1e9)
    )


def ros_time_from_robot_time(timestamp: rtp.RobotTimestamp):
    one_second = datetime.timedelta(seconds=1)
    one_microsecond = datetime.timedelta(microseconds=1)
    past_seconds = timestamp.time_since_epoch() // one_second
    past_us = (timestamp.time_since_epoch() % one_second) // one_microsecond
    return rospy.Time(past_seconds, past_us * 1000)


def robot_se2_from_stamped_transform(b_from_a_msg):
    origin = np.zeros(3)
    unit_x = np.zeros(3)
    unit_x[0] = 1.0

    b_from_a_rot = Rotation.from_quat(
        [
            b_from_a_msg.transform.rotation.x,
            b_from_a_msg.transform.rotation.y,
            b_from_a_msg.transform.rotation.z,
            b_from_a_msg.transform.rotation.w,
        ]
    )

    b_from_a_trans = np.array(
        [
            b_from_a_msg.transform.translation.x,
            b_from_a_msg.transform.translation.y,
            b_from_a_msg.transform.translation.z,
        ]
    )

    origin_in_b = b_from_a_rot.apply(origin) + b_from_a_trans
    x_in_b = b_from_a_rot.apply(unit_x) + b_from_a_trans

    theta = math.atan2(x_in_b[1] - origin_in_b[1], x_in_b[0] - origin_in_b[0])
    return se2.SE2(theta, np.array([origin_in_b[0], origin_in_b[1]]))


def stamped_transform_from_se2(parent_from_child, parent, child, ros_time):
    out = geometry_msgs.msg.TransformStamped()
    out.header.stamp.secs = ros_time.secs
    out.header.stamp.nsecs = ros_time.nsecs
    out.header.frame_id = parent
    out.child_frame_id = child

    out.transform.translation.x = parent_from_child.translation()[0]
    out.transform.translation.y = parent_from_child.translation()[1]
    out.transform.translation.z = 0.0

    b_from_a_rot = Rotation.from_rotvec(np.array([0, 0, parent_from_child.so2().log()]))
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

        id = detection.id[0]
        stamp = stamped_pose.header.stamp
        time_of_validity = robot_time_from_ros_time(stamp)

        body_from_camera = tf_buffer.lookup_transform(BODY_FRAME, camera_frame, stamp)

        camera_from_tag = detection.pose.pose

        body_from_tag = tf2_geometry_msgs.do_transform_pose(
            camera_from_tag, body_from_camera
        )

        range_m_sq = (
            body_from_tag.pose.position.x ** 2 + body_from_tag.pose.position.y ** 2
        )
        range_m = math.sqrt(range_m_sq)
        bearing_rad = math.atan2(
            body_from_tag.pose.position.y, body_from_tag.pose.position.x
        )

        observations.append(
            (time_of_validity, BeaconObservation(id, range_m, bearing_rad))
        )
    return observations


#  Note that the EKF should be locked when calling this function
def perform_process_update(ekf, tf_buffer, update_time):

    dt = update_time - ekf.estimate.time_of_validity
    dt_days = dt / datetime.timedelta(days=1)
    if dt_days > 1.0:
        ekf.estimate.time_of_validity = update_time - rtp.as_duration(0.01)

    ros_past_time = ros_time_from_robot_time(ekf.estimate.time_of_validity)

    # try:
    timeout_s = rospy.Duration(1.0)
    ros_update_time = ros_time_from_robot_time(update_time)
    odom_from_past_robot = tf_buffer.lookup_transform(
        "odom", BODY_FRAME, ros_past_time, timeout_s
    )
    odom_from_new_robot = tf_buffer.lookup_transform(
        "odom", BODY_FRAME, ros_update_time, timeout_s
    )

    odom_from_past_robot = robot_se2_from_stamped_transform(odom_from_past_robot)
    odom_from_new_robot = robot_se2_from_stamped_transform(odom_from_new_robot)
    past_robot_from_new_robot = odom_from_past_robot.inverse() * odom_from_new_robot

    ekf.predict(update_time, past_robot_from_new_robot)


def create_debug_message(observations):
    obs_dict = {
        "observations": [
            {
                "time_of_validity": str(obs[0]),
                "id": obs[1].maybe_id,
                "range_m": obs[1].maybe_range_m,
                "bearing_rad": obs[1].maybe_bearing_rad,
            }
            for obs in observations
        ]
    }

    out = std_msgs.msg.String(data=json.dumps(obs_dict))
    return out


def create_obs_viz(observations, camera_name, ekf_tov):
    viz_marker = viz.MarkerArray()

    unobserved_beacon_ids = list(range(10))
    for obs_and_time in observations:
        ...
        marker = viz.Marker()
        obs_tov = ros_time_from_robot_time(obs_and_time[0])
        obs = obs_and_time[1]
        unobserved_beacon_ids.remove(obs.maybe_id)
        marker_header = std_msgs.msg.Header()
        marker_header.stamp = obs_tov
        marker_header.frame_id = BODY_FRAME
        marker.header = marker_header

        marker.ns = f"obs_{camera_name}"
        marker.id = obs.maybe_id
        marker.type = viz.Marker.SPHERE
        marker.action = viz.Marker.ADD

        marker.pose.position.x = obs.maybe_range_m * np.cos(obs.maybe_bearing_rad)
        marker.pose.position.y = obs.maybe_range_m * np.sin(obs.maybe_bearing_rad)
        marker.pose.position.z = 0.0

        marker.pose.orientation.w = 1.0

        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1

        is_old = obs_and_time[0] - ekf_tov < -rtp.as_duration(0.5)

        marker.color.a = 1.0
        marker.color.r = 1.0 if is_old else 0.25
        marker.color.g = 0.25
        marker.color.b = 0.25 if is_old else 1.0
        viz_marker.markers.append(marker)

    for id in unobserved_beacon_ids:
        marker = viz.Marker()
        marker.header.frame_id = BODY_FRAME
        marker.ns = f"obs_{camera_name}"
        marker.id = id
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1

        marker.action = viz.Marker.DELETE
        viz_marker.markers.append(marker)
    return viz_marker


def create_tf_msg(ekf):
    if ekf.estimate.time_of_validity == rtp.RobotTimestamp():
        return None

    publish_time = ros_time_from_robot_time(ekf.estimate.time_of_validity)

    tfs = []
    for beacon_id in ekf.estimate.beacon_ids:
        tfs.append(
            stamped_transform_from_point2(
                ekf.estimate.beacon_in_local(beacon_id),
                MAP_FRAME,
                f"map_tag_{beacon_id}",
                publish_time,
            )
        )

    tfs.append(
        stamped_transform_from_se2(
            ekf.estimate.local_from_robot().inverse(),
            BODY_FRAME,
            MAP_FRAME,
            publish_time,
        )
    )

    return tf2_msgs.msg.TFMessage(tfs)


def create_viz_msg(ekf):
    if ekf.estimate.time_of_validity == rtp.RobotTimestamp():
        return None

    publish_time = ros_time_from_robot_time(ekf.estimate.time_of_validity)
    viz_msg = viz.MarkerArray()

    marker_header = std_msgs.msg.Header()
    marker_header.stamp.secs = publish_time.secs
    marker_header.stamp.nsecs = publish_time.nsecs
    marker_header.frame_id = MAP_FRAME

    # Landmark viz
    for beacon_id in ekf.estimate.beacon_ids:
        marker = viz.Marker()
        marker.header = marker_header
        marker.ns = "tag"
        marker.id = beacon_id

        marker.type = viz.Marker.LINE_STRIP
        marker.action = viz.Marker.ADD
        marker.scale.x = 0.01
        marker.pose.orientation.w = 1.0

        beacon_in_local = ekf.estimate.beacon_in_local(beacon_id)
        beacon_cov = ekf.estimate.beacon_cov(beacon_id)

        cov_root = np.linalg.cholesky(beacon_cov)

        theta = np.arange(0, 2 * np.pi, 0.1)
        np.append(theta, 0.01)
        xs = 2 * np.cos(theta)
        ys = 2 * np.sin(theta)

        pts = np.vstack([xs, ys])

        tx_pts = cov_root @ pts

        for i in range(tx_pts.shape[1]):
            x, y = tx_pts[:, i]
            marker.points.append(
                geometry_msgs.msg.Point(
                    x=x + beacon_in_local[0], y=y + beacon_in_local[1]
                )
            )
            marker.colors.append(std_msgs.msg.ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0))

        viz_msg.markers.append(marker)

    # Robot viz
    marker = viz.Marker()
    marker.header = marker_header
    marker.ns = "robot"
    marker.id = 0

    marker.type = viz.Marker.LINE_STRIP
    marker.action = viz.Marker.ADD
    marker.scale.x = 0.01
    marker.pose.orientation.w = 1.0

    local_from_robot = ekf.estimate.local_from_robot()
    robot_cov = ekf.estimate.robot_cov()

    cov_root = np.linalg.cholesky(robot_cov + np.identity(3) * 1e-6)

    theta = np.arange(0, 2 * np.pi, 0.1)
    np.append(theta, 0.01)
    xs = 2 * np.cos(theta)
    ys = 2 * np.sin(theta)

    pts = np.vstack([xs, ys, np.zeros_like(xs)])

    tx_pts_in_robot = cov_root @ pts
    tx_pts_in_local = local_from_robot @ tx_pts_in_robot[:2, :]

    for i in range(tx_pts_in_local.shape[1]):
        x, y = tx_pts_in_local[:, i]

        marker.points.append(geometry_msgs.msg.Point(x=x, y=y))
        marker.colors.append(std_msgs.msg.ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0))

    viz_msg.markers.append(marker)

    return viz_msg


def observation_callback(detections, observations_queue, queue_lock):
    with queue_lock:
        observations_queue.append(detections)


def tick_estimator(
    timer_event,
    observations_queue,
    ekf,
    tf_buffer,
    tf_publisher,
    viz_publisher,
    debug_publisher,
):
    # Collect all available observations
    observations = []
    while observations_queue.are_samples_available():
        detections_msg = observations_queue.pop()
        new_detections = compute_observations(detections_msg.detections, tf_buffer)

        # Update the detections visualization
        new_detections = sorted(new_detections, key=lambda x: x[0])
        viz_publisher.publish(
            create_obs_viz(
                new_detections,
                detections_msg.header.frame_id,
                ekf.estimate.time_of_validity,
            )
        )
        observations.extend(new_detections)

    # Sort them in order
    observations = sorted(observations, key=lambda x: x[0])

    # Update the filter with each observation
    for time, obs in observations:
        perform_process_update(ekf, tf_buffer, time)
        ekf.update([obs])

    # publish the visualization
    viz_msg = create_viz_msg(ekf)
    if viz_msg is not None:
        viz_publisher.publish(viz_msg)

    debug_str = create_debug_message(observations)
    debug_publisher.publish(debug_str)

    if len(observations) == 0:
        return

    # publish the estimate
    tf_message = create_tf_msg(ekf)
    if tf_message is not None:
        tf_publisher.publish(tf_message)


def main():
    rospy.init_node("observation_node")

    tf_buffer = tf2_ros.Buffer(rospy.Duration(60.0))
    tf_listener = tf2_ros.TransformListener(tf_buffer)
    tf_publisher = rospy.Publisher("/tf", tf2_msgs.msg.TFMessage, queue_size=16)
    viz_publisher = rospy.Publisher("/estimate_markers", viz.MarkerArray, queue_size=16)
    debug_publisher = rospy.Publisher(
        "/estimate_debug", std_msgs.msg.String, queue_size=16
    )
    tag_detection_subscribers = []

    ekf_config = esp.EkfSlamConfig(
        max_num_beacons=10,
        initial_beacon_uncertainty_m=100.0,
        along_track_process_noise_m_per_rt_meter=0.02,
        cross_track_process_noise_m_per_rt_meter=0.02,
        pos_process_noise_m_per_rt_s=0.0001,
        heading_process_noise_rad_per_rt_meter=0.0001,
        heading_process_noise_rad_per_rt_s=0.00001,
        beacon_pos_process_noise_m_per_rt_s=0.001,
        range_measurement_noise_m=0.1,
        bearing_measurement_noise_rad=0.02,
        on_map_load_position_uncertainty_m=0.1,
        on_map_load_heading_uncertainty_rad=0.01,
    )

    camera_list = ["frontleft", "frontright", "left", "back", "right"]

    observations_queue = ObservationsQueue([f"{c}_fisheye" for c in camera_list])
    ekf = esp.EkfSlam(ekf_config, rtp.RobotTimestamp())

    for camera in camera_list:
        topic_name = f"/spot/apriltag/{camera}/tag_detections"
        tag_detection_subscribers.append(
            rospy.Subscriber(
                topic_name,
                AprilTagDetectionArray,
                lambda data: observations_queue.insert(data),
            )
        )

    rospy.Timer(
        rospy.Duration(0.05),
        lambda timer_event: tick_estimator(
            timer_event,
            observations_queue,
            ekf,
            tf_buffer,
            tf_publisher,
            viz_publisher,
            debug_publisher,
        ),
    )

    try:
        rospy.spin()
    except KeyboardInterrupt:
        ...


if __name__ == "__main__":
    main()
