
import threading


import rospy
import tf2_ros

from brm_evanescence.srv import CreatePlanResponse, CreatePlan, CreatePlanRequest
from brm_evanescence.msg import Map 

import experimental.beacon_sim.beacon_potential_python as bpp
import planning.probabilistic_road_map_python as prmp
import experimental.beacon_sim.belief_road_map_planner_python as brmp
import experimental.beacon_sim.ekf_slam_python as esp

def run_optimistic_brm_planner(road_map, potential, local_from_robot):
    ...

def run_expected_brm_planner(road_map, potential, local_from_robot):
    ...

def run_landmark_brm_planner(road_map, potential, local_from_robot):
    ...

class PlanningNode():
    def __init__(self):
        rospy.init_node('planning_node')
        
        rospy.Subscriber('/map', Map, self.map_callback)
        rospy.Service(f'{rospy.get_name()}/create_plan', CreatePlan, self.handle_plan_request)

        self._map_lock = threading.Lock()
        self._map = None

    def map_callback(self, data):
        with self._map_lock:
            self._map = data



    def handle_plan_request(self, request):
        rospy.loginfo(request)
        
        # Get the latest estimate
        with self._map_lock:
            ekf_estimate = esp.EkfSlamEstimate.from_proto_string(self._map.estimate_proto)
        rospy.loginfo(ekf_estimate)

        # Load the beacon potential
        with open(request.beacon_potential_path, 'rb') as file_in:
            beacon_potential = bpp.BeaconPotential(file_in.read())

        # Load the road map
        with open(request.road_map_path, 'rb') as file_in:
            road_map = prmp.RoadMap(file_in.read())

        # call the appropriate planner
        if req.planning_method == CreatePlanRequest.OPTIMISTIC_BRM:
            run_optimistic_brm_planner(road_map, beacon_potential, local_from_robot)
        elif req.planning_method == CreatePlanRequest.EXPECTED_BRM:
            run_expected_brm_planner(road_map, beacon_potential, local_from_robot)
        elif req.planning_method == CreatePlanRequest.LANDMARK_BRM:
            run_landmark_brm_planner(road_map, beacon_potential, local_from_robot)
        else:
            rospy.logerror(f'Unknown Planning Method: {request.planning_method}')
            return CreatePlanResponse(success=False)
        

        return CreatePlanResponse(success=True)


if __name__ == "__main__":
    planning_node = PlanningNode()
    
    try:
        rospy.spin()
    except KeyboardInterrupt:
        ...


