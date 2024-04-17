
import argparse
import datetime
from pathlib import Path
import math
import numpy as np
from scipy.spatial.transform import Rotation

from common.liegroups import se2_python as se2
from common.time import robot_time_python as rtp
import planning.probabilistic_road_map_python as prmp
from experimental.beacon_sim.generate_observations_python import BeaconObservation
from experimental.beacon_sim import ekf_slam_python as esp

import rospy
import rosbag
import tf2_ros
import tf2_geometry_msgs

import warnings
warnings.filterwarnings("ignore")

BODY_FRAME = "flat_body"

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


def robot_time_from_ros_time(stamp: rospy.Time):
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


def compute_observations(detections, tf_buffer):
    observations = []
    timeout = rospy.Duration(0)
    for detection in detections:
        stamped_pose = detection.pose
        camera_frame = stamped_pose.header.frame_id

        id = detection.id[0]
        stamp = stamped_pose.header.stamp
        time_of_validity = robot_time_from_ros_time(stamp)

        body_from_camera = tf_buffer.lookup_transform(BODY_FRAME, camera_frame, rospy.Time(0), timeout)

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


def read_observations(bag, buffer):
    camera_list = ['frontleft', 'frontright', 'left', 'back', 'right']
    topic_list = [
        f'/spot/apriltag/{camera}/tag_detections' for camera in camera_list
    ]

    out = []
    for topic, msg, t in bag.read_messages(topics=topic_list):
        out.extend(compute_observations(msg.detections, buffer))

    return sorted(out, key=lambda x: x[0])


def read_odometry(bag):
    buffer = tf2_ros.Buffer(rospy.Duration(3600.0))
    DEFAULT_AUTHORITY = ''
    for topic, msg, t in bag.read_messages(topics=['/tf', '/tf_static']):
        for transform in msg.transforms:
            if topic == '/tf_static':
                buffer.set_transform_static(transform, DEFAULT_AUTHORITY)
            elif transform.child_frame_id in ['flat_body', 'odom']:
                buffer.set_transform(transform, DEFAULT_AUTHORITY)
    return buffer


def create_ekf(map_file: Path):
    ekf_config = esp.EkfSlamConfig(
        max_num_beacons=40,
        initial_beacon_uncertainty_m=100.0,
        along_track_process_noise_m_per_rt_meter=0.02,
        cross_track_process_noise_m_per_rt_meter=0.02,
        pos_process_noise_m_per_rt_s=0.0001,
        heading_process_noise_rad_per_rt_meter=0.0001,
        heading_process_noise_rad_per_rt_s=0.00001,
        beacon_pos_process_noise_m_per_rt_s=0.001,
        range_measurement_noise_m=0.1,
        bearing_measurement_noise_rad=0.02,
        on_map_load_position_uncertainty_m=1000.0,
        on_map_load_heading_uncertainty_rad=6.0,
    )

    ekf = esp.EkfSlam(ekf_config, rtp.RobotTimestamp())
    ekf.load_map(str(map_file))
    return ekf


def run_estimator(tf_buffer, observations, ekf):
    timeout_s = rospy.Duration(0)
    for i, (time, obs) in enumerate(observations):
        ros_update_time = ros_time_from_robot_time(time)

        if (not tf_buffer.can_transform('odom', BODY_FRAME, ros_update_time)
                and ekf.estimate.time_of_validity == rtp.RobotTimestamp()):
            continue

        if ekf.estimate.time_of_validity == rtp.RobotTimestamp():
            # When uninitialized, set the time to the first observation for which we can find
            # a transform to the odometry frame
            ekf.estimate.time_of_validity = time

        # Process Update
        ros_past_time = ros_time_from_robot_time(ekf.estimate.time_of_validity)
        try:
            odom_from_past_robot = tf_buffer.lookup_transform(
                "odom", BODY_FRAME, ros_past_time, timeout_s
            )
            odom_from_new_robot = tf_buffer.lookup_transform(
                "odom", BODY_FRAME, ros_update_time, timeout_s
            )
        except Exception as e:
            print(e)
            print(time, obs)
            continue

        odom_from_past_robot = robot_se2_from_stamped_transform(odom_from_past_robot)
        odom_from_new_robot = robot_se2_from_stamped_transform(odom_from_new_robot)
        past_robot_from_new_robot = odom_from_past_robot.inverse() * odom_from_new_robot

        ekf.predict(time, past_robot_from_new_robot)

        # Measurement Update
        ekf.update([obs])
    return ekf


def compute_goal_error(map_from_robot, goal_in_map):
    goal_in_robot = map_from_robot.inverse() * goal_in_map
    return goal_in_robot


def main(bag_file: Path, map_file: Path, road_map: Path, goal_node_id: int):
    bag = rosbag.Bag(bag_file)
    tf_buffer = read_odometry(bag)
    observations = read_observations(bag, tf_buffer)

    ekf = create_ekf(map_file)

    ekf = run_estimator(tf_buffer, observations, ekf)

    road_map = prmp.RoadMap.from_proto_string(road_map.read_bytes())
    goal_in_map = road_map.point(goal_node_id)

    error = compute_goal_error(ekf.estimate.local_from_robot(), goal_in_map)
    print(error)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--bag_file', required=True)
    parser.add_argument('--map_file', required=True)
    parser.add_argument('--road_map_file', required=True)
    parser.add_argument('--goal_node_id', required=True, type=int)

    args = parser.parse_args()

    main(Path(args.bag_file), Path(args.map_file), Path(args.road_map_file), args.goal_node_id)
