
from __future__ import annotations

import threading

import spot_msgs.msg
import rospy
import tf2_ros
import visualization_msgs.msg as viz
import geometry_msgs
import actionlib
import time
import numpy as np

from brm_evanescence.srv import CreatePlanResponse, CreatePlan, CreatePlanRequest
from brm_evanescence.srv import ExecutePlanResponse, ExecutePlan
from brm_evanescence.msg import Map

import common.liegroups.se2_python as se2
import experimental.beacon_sim.beacon_potential_python as bpp
import planning.probabilistic_road_map_python as prmp
import experimental.beacon_sim.belief_road_map_planner_python as brmp
import experimental.beacon_sim.ekf_slam_python as esp
from scipy.spatial.transform import Rotation as R

from typing import NamedTuple

BODY_FRAME = "flat_body"
MAP_FRAME = "map"


class Plan(NamedTuple):
    time_of_validity: rospy.Time
    nodes: list[int]


EKF_CONFIG = esp.EkfSlamConfig(
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

MAX_SENSOR_RANGE_M = 2.5
TIMEOUT = None
MAX_NUM_TRANSFORMS = 1000
BELIEF_ROADMAP_OPTIONS = brmp.BeliefRoadMapOptions(
    max_sensor_range_m=MAX_SENSOR_RANGE_M,
    uncertainty_tolerance=None,
    max_num_edge_transforms=MAX_NUM_TRANSFORMS,
    timeout=TIMEOUT
)

NUM_WORLD_SAMPLES = 100
SEED = 0
EXPECTED_BELIEF_ROADMAP_OPTIONS = brmp.ExpectedBeliefRoadMapOptions(
    NUM_WORLD_SAMPLES, SEED, BELIEF_ROADMAP_OPTIONS
)

MAX_NUM_COMPONENTS = 100
AGGREGATION_METHOD = brmp.LandmarkBeliefRoadMapOptions.ExpectedDeterminant()
# AGGREGATION_METHOD = brmp.LandmarkBeliefRoadMapOptions.ValueAtRiskDeterminant(0.95)
SAMPLED_BELIEF = brmp.LandmarkBeliefRoadMapOptions.SampledBeliefOptions(
    MAX_NUM_COMPONENTS,
    SEED,
)
LANDMARK_BELIEF_ROADMAP_OPTIONS = brmp.LandmarkBeliefRoadMapOptions(
    MAX_SENSOR_RANGE_M,
    AGGREGATION_METHOD,
    SAMPLED_BELIEF,
    TIMEOUT,
)


def run_optimistic_brm_planner(road_map, potential, ekf):
    potential = bpp.BeaconPotential()
    result = brmp.compute_belief_road_map_plan(
        road_map, ekf, potential, BELIEF_ROADMAP_OPTIONS
    )
    return result.nodes


def run_expected_brm_planner(road_map, potential, ekf):
    result = brmp.compute_expected_belief_road_map_plan(
        road_map, ekf, potential, EXPECTED_BELIEF_ROADMAP_OPTIONS
    )
    return result.nodes


def run_landmark_brm_planner(road_map, potential, ekf):
    result = brmp.compute_landmark_belief_road_map_plan(
        road_map, ekf, potential, LANDMARK_BELIEF_ROADMAP_OPTIONS
    )
    return result.nodes


def robot_se2_from_stamped_transform(b_from_a_msg):
    origin = np.zeros(3)
    unit_x = np.zeros(3)
    unit_x[0] = 1.0

    b_from_a_rot = R.from_quat(
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

    theta = np.arctan2(x_in_b[1] - origin_in_b[1], x_in_b[0] - origin_in_b[0])
    return se2.SE2(theta, np.array([origin_in_b[0], origin_in_b[1]]))


class PlanningNode:
    def __init__(self):
        rospy.init_node("planning_node")

        rospy.Subscriber("/map", Map, self.map_callback)
        self._viz_publisher = rospy.Publisher("/plan_visualization", viz.MarkerArray, queue_size=16)
        rospy.Service(
            f"{rospy.get_name()}/create_plan", CreatePlan, self.handle_plan_request
        )

        rospy.Service(
            f"{rospy.get_name()}/execute_plan", ExecutePlan, self.handle_plan_execution_request
        )
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)

        self._traj_client = actionlib.SimpleActionClient('/spot/trajectory', spot_msgs.msg.TrajectoryAction)

        self._map_lock = threading.Lock()
        self._map = None

        self._plan = None
        self._road_map = None

    def map_callback(self, data):
        with self._map_lock:
            self._map = data

    def visualize_road_map(self, road_map):
        marker_out = viz.MarkerArray()

        pt_idxs = list(range(len(road_map.points())))
        if road_map.has_start_goal():
            pt_idxs += [road_map.START_IDX, road_map.GOAL_IDX]

        # Visualize the nodes
        for pt_idx in pt_idxs:
            marker = viz.Marker()
            pt = road_map.point(pt_idx)
            marker.header.frame_id = "map"

            marker.ns = "road_map_nodes"
            marker.id = pt_idx
            marker.type = viz.Marker.CUBE
            marker.action = viz.Marker.ADD
            marker.pose.position.x = pt[0]
            marker.pose.position.y = pt[1]
            marker.pose.orientation.w = 1.0

            marker.scale.x = 0.1
            marker.scale.y = 0.1
            marker.scale.z = 0.1

            marker.color.r = 0.1
            marker.color.g = 0.6
            marker.color.b = 0.6
            marker.color.a = 1.0

            if pt_idx == road_map.START_IDX:
                marker.color.r = 0.6
                marker.color.g = 0.1
                marker.color.b = 0.6
                marker.color.a = 1.0

            elif pt_idx == road_map.GOAL_IDX:
                marker.color.r = 0.1
                marker.color.g = 0.1
                marker.color.b = 0.6
                marker.color.a = 1.0

            marker_out.markers.append(marker)

            marker = viz.Marker()
            pt = road_map.point(pt_idx)
            marker.header.frame_id = "map"

            marker.ns = "road_map_nodes_labels"
            marker.id = pt_idx
            marker.type = viz.Marker.TEXT_VIEW_FACING
            marker.action = viz.Marker.ADD
            marker.pose.position.x = pt[0] + 0.15
            marker.pose.position.y = pt[1]
            marker.pose.orientation.w = 1.0
            TEXT_HEIGHT_M = 0.1
            marker.scale.z = TEXT_HEIGHT_M
            marker.color.r = 0.8
            marker.color.g = 0.8
            marker.color.b = 0.8
            marker.color.a = 1.0

            marker.text = f'{pt_idx}'

            if pt_idx == road_map.START_IDX:
                marker.text = 'Start'
                marker.pose.position.z = TEXT_HEIGHT_M

            elif pt_idx == road_map.GOAL_IDX:
                marker.text = 'Goal'
                marker.pose.position.z = TEXT_HEIGHT_M

            marker_out.markers.append(marker)

        # Visualize the edges
        edge_marker = viz.Marker()
        edge_marker.header.frame_id = "map"
        edge_marker.ns = "road_map_edges"
        edge_marker.id = 0
        edge_marker.type = viz.Marker.LINE_LIST
        edge_marker.action = viz.Marker.ADD
        edge_marker.pose.orientation.w = 1.0

        edge_marker.scale.x = 0.01
        edge_marker.color.r = 0.6
        edge_marker.color.g = 0.1
        edge_marker.color.b = 0.1
        edge_marker.color.a = 1.0

        for pt_idx in pt_idxs:
            pt_loc = road_map.point(pt_idx)
            for neighbor_idx, neighbor_loc in road_map.neighbors(pt_idx):
                if neighbor_idx < pt_idx:
                    continue
                edge_marker.points.append(
                    geometry_msgs.msg.Point(x=pt_loc[0], y=pt_loc[1], z=0.0)
                )
                edge_marker.points.append(
                    geometry_msgs.msg.Point(x=neighbor_loc[0], y=neighbor_loc[1], z=0.0)
                )
        marker_out.markers.append(edge_marker)

        self._viz_publisher.publish(marker_out)

    def visualize_plan(self, road_map, plan):
        plan_marker = viz.Marker()
        plan_marker.header.frame_id = "map"
        plan_marker.ns = "plan_edges"
        plan_marker.id = 0
        plan_marker.type = viz.Marker.LINE_STRIP
        plan_marker.action = viz.Marker.ADD
        plan_marker.pose.position.z = 0.1
        plan_marker.pose.orientation.w = 1.0

        plan_marker.scale.x = 0.02
        plan_marker.color.r = 0.1
        plan_marker.color.g = 0.7
        plan_marker.color.b = 0.1
        plan_marker.color.a = 1.0

        for pt_idx in plan:
            pt_loc = road_map.point(pt_idx)
            plan_marker.points.append(
                geometry_msgs.msg.Point(x=pt_loc[0], y=pt_loc[1], z=0.0)
            )

        self._viz_publisher.publish(viz.MarkerArray([plan_marker]))

    def handle_plan_request(self, req):
        rospy.loginfo(req)

        # Get the latest estimate
        with self._map_lock:
            if self._map is None or self._map.header.stamp == rospy.Time():
                return CreatePlanResponse(success=False)

            ekf_estimate = esp.EkfSlamEstimate.from_proto_string(
                self._map.estimate_proto
            )
        ekf = esp.EkfSlam(EKF_CONFIG, ekf_estimate.time_of_validity)
        ekf.estimate = ekf_estimate

        # Load the beacon potential
        with open(req.beacon_potential_path, "rb") as file_in:
            beacon_potential = bpp.BeaconPotential.from_proto_string(file_in.read())

        # Load the road map
        with open(req.road_map_path, "rb") as file_in:
            road_map = prmp.RoadMap.from_proto_string(file_in.read())

        goal_loc = road_map.point(req.goal_node)
        start_loc = ekf.estimate.local_from_robot().translation()

        CONNECTION_RADIUS_M = 2
        road_map.add_start_goal(
            prmp.StartGoalPair(start_loc, goal_loc, CONNECTION_RADIUS_M)
        )

        # call the appropriate planner
        if req.planning_method == CreatePlanRequest.OPTIMISTIC_BRM:
            plan = run_optimistic_brm_planner(road_map, beacon_potential, ekf)
        elif req.planning_method == CreatePlanRequest.EXPECTED_BRM:
            plan = run_expected_brm_planner(road_map, beacon_potential, ekf)
        elif req.planning_method == CreatePlanRequest.LANDMARK_BRM:
            plan = run_landmark_brm_planner(road_map, beacon_potential, ekf)
        else:
            rospy.loginfo(
                f"Unknown Planning Method: {req.planning_method} not in [{CreatePlanRequest.OPTIMISTIC_BRM}, {CreatePlanRequest.EXPECTED_BRM}, {CreatePlanRequest.LANDMARK_BRM}]"
            )
            return CreatePlanResponse(success=False)

        rospy.loginfo(f"Plan: {plan}")
        self._road_map = road_map
        self._plan = Plan(
            time_of_validity=rospy.Time.now(),
            nodes=plan
        )

        self.visualize_road_map(self._road_map)
        self.visualize_plan(self._road_map, plan)

        return CreatePlanResponse(success=True)

    def handle_plan_execution_request(self, req):
        if self._plan is None:
            return ExecutePlanResponse(success=False)

        trajectory = spot_msgs.msg.TrajectoryGoal()
        trajectory.precise_positioning = True
        trajectory.duration.data = rospy.Duration(20.0)
        trajectory.target_pose.header.frame_id = BODY_FRAME

        timeout = rospy.Duration(1.0)
        for node_id in self._plan.nodes[1:]:
            # Turn towards the next point
            nowish = rospy.Time.now() - rospy.Duration(1.0)
            map_from_robot = None
            while True:
                try:
                    map_from_robot_ros = self._tf_buffer.lookup_transform(
                            MAP_FRAME, BODY_FRAME, nowish, timeout)
                    map_from_robot = robot_se2_from_stamped_transform(map_from_robot_ros)
                    break
                except Exception as e:
                    rospy.loginfo(e)
                    continue


            goal_in_map = self._road_map.point(node_id)
            goal_in_body = map_from_robot.inverse() @ goal_in_map
            if np.linalg.norm(goal_in_body) < 0.5:
                # We are near the next node, so we skip it
                continue
            theta = np.arctan2(goal_in_body[1], goal_in_body[0])
            robot_from_goal_facing_robot = R.from_rotvec(np.array([0, 0, theta]))
            robot_from_goal_facing_robot_quat = robot_from_goal_facing_robot.as_quat()

            trajectory.target_pose.pose.position.x = 0.0
            trajectory.target_pose.pose.position.y = 0.0
            trajectory.target_pose.pose.position.z = 0.0
            trajectory.target_pose.pose.orientation.x = robot_from_goal_facing_robot_quat[0]
            trajectory.target_pose.pose.orientation.y = robot_from_goal_facing_robot_quat[1]
            trajectory.target_pose.pose.orientation.z = robot_from_goal_facing_robot_quat[2]
            trajectory.target_pose.pose.orientation.w = robot_from_goal_facing_robot_quat[3]
            
            self._traj_client.send_goal(trajectory)
            result = self._traj_client.wait_for_result(rospy.Duration(10.0))
            time.sleep(2.0)

            # Move towards the goal
            nowish = rospy.Time.now() - rospy.Duration(1.0)
            while True:
                try:
                    map_from_robot_ros = self._tf_buffer.lookup_transform(
                            MAP_FRAME, BODY_FRAME, nowish, timeout)
                    map_from_robot = robot_se2_from_stamped_transform(map_from_robot_ros)
                    break
                except Exception as e:
                    rospy.loginfo(e)
                    continue
            goal_in_body = map_from_robot.inverse() @ goal_in_map


            trajectory.target_pose.pose.position.x = goal_in_body[0]
            trajectory.target_pose.pose.position.y = goal_in_body[1]
            trajectory.target_pose.pose.position.z = 0.0
            trajectory.target_pose.pose.orientation.x = 0.0
            trajectory.target_pose.pose.orientation.y = 0.0
            trajectory.target_pose.pose.orientation.z = 0.0
            trajectory.target_pose.pose.orientation.w = 1.0
            
            self._traj_client.send_goal(trajectory)
            result = self._traj_client.wait_for_result(rospy.Duration(10.0))
            time.sleep(2.0)


        return ExecutePlanResponse(success=True)



if __name__ == "__main__":
    planning_node = PlanningNode()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        ...
