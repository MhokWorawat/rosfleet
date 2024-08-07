#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from nav_msgs.srv import GetPlan, GetPlanRequest
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
import math

class FleetManagement:
    def __init__(self):
        rospy.init_node('fleet_management', anonymous=True)

        self.agv_status = ["Not connect", "Not connect", "Not connect", "Not connect"]

        self.agv_poses = {
            'AGV01': (0, 0, 0, 0, 0, 0, 0),
            'AGV02': (0, 0, 0, 0, 0, 0, 0),
            'AGV03': (0, 0, 0, 0, 0, 0, 0),
            'AGV04': (0, 0, 0, 0, 0, 0, 0)
        }
        self.station_coordinates = {
            'A': (2.500, -5.500, 0.000, 0.000, 0.000, 0.707, 0.707),
            'B': (7.000, -1.500, 0.000, 0.000, 0.000, 0.000, 1.000),
            'C': (13.500, -9.300, 0.000, 0.000, 0.000, 0.000, 1.000),
            'D': (7.500, -10.500, 0.000, 0.000, 0.000, -0.707, 0.707),
            'E': (20.000, -10.500, 0.000, 0.000, 0.000, 0.000, 1.000),
            'G': (18.000, -21.000, 0.000, 0.000, 0.000, 1.000, 0.000),
            'Park1': (1.000, -10.000, 0.000, 0.000, 0.000, 0.000, 1.000),
            'Park2': (17.750, -7.250, 0.000, 0.000, 0.000, -0.707, 0.707),
            'Park3': (1.000, -1.000, 0.000, 0.000, 0.000, 0.000, 1.000),
            'Park4': (25.000, -10.000, 0.000, 0.000, 0.000, -1.000, 0.000)
        }

        # ROS Publishers
        self.agv_status_pub = rospy.Publisher('/agv_status', String, queue_size=10, latch=True)
        self.task_pub = rospy.Publisher('/task', String, queue_size=10)

        # ROS Subscribers
        rospy.Subscriber('/agv01/amcl_pose', PoseWithCovarianceStamped, self.agv01_pose_callback)
        rospy.Subscriber('/agv02/amcl_pose', PoseWithCovarianceStamped, self.agv02_pose_callback)
        rospy.Subscriber('/agv03/amcl_pose', PoseWithCovarianceStamped, self.agv03_pose_callback)
        rospy.Subscriber('/agv04/amcl_pose', PoseWithCovarianceStamped, self.agv04_pose_callback)
        rospy.Subscriber('/request_parking', String, self.ros_subscribe_request_park)
        rospy.Subscriber('/agv_status', String, self.ros_subscribe_agv_status)
        rospy.Subscriber('/mission', String, self.ros_subscribe_mission)

        self.parking_status_update = ParkingStatusUpdate(self)

    def agv01_pose_callback(self, msg):
        self.agv_poses['AGV01'] = (msg.pose.pose.position.x, 
                                   msg.pose.pose.position.y,
                                   msg.pose.pose.position.z,
                                   msg.pose.pose.orientation.x,
                                   msg.pose.pose.orientation.y,
                                   msg.pose.pose.orientation.z,
                                   msg.pose.pose.orientation.w)

    def agv02_pose_callback(self, msg):
        self.agv_poses['AGV02'] = (msg.pose.pose.position.x, 
                                   msg.pose.pose.position.y,
                                   msg.pose.pose.position.z,
                                   msg.pose.pose.orientation.x,
                                   msg.pose.pose.orientation.y,
                                   msg.pose.pose.orientation.z,
                                   msg.pose.pose.orientation.w)

    def agv03_pose_callback(self, msg):
        self.agv_poses['AGV03'] = (msg.pose.pose.position.x, 
                                   msg.pose.pose.position.y,
                                   msg.pose.pose.position.z,
                                   msg.pose.pose.orientation.x,
                                   msg.pose.pose.orientation.y,
                                   msg.pose.pose.orientation.z,
                                   msg.pose.pose.orientation.w)

    def agv04_pose_callback(self, msg):
        self.agv_poses['AGV04'] = (msg.pose.pose.position.x, 
                                   msg.pose.pose.position.y,
                                   msg.pose.pose.position.z,
                                   msg.pose.pose.orientation.x,
                                   msg.pose.pose.orientation.y,
                                   msg.pose.pose.orientation.z,
                                   msg.pose.pose.orientation.w)
        
    def ros_subscribe_agv_status(self, msg):
        rospy.loginfo(f"Fleet Received AGV status: {msg.data}")
        status_list = msg.data.split(', ')
        if len(status_list) == 4:
            self.agv_status = status_list

    def ros_subscribe_mission(self, msg):
        rospy.loginfo(f"Received mission: {msg.data}")
        mission = msg.data.split(',')
        if len(mission) == 3:
            selected_agv = mission[0].strip()
            station_first = mission[1].strip()
            station_last = mission[2].strip()
            self.select_agv_for_task(selected_agv, station_first, station_last)
        else:
            rospy.loginfo("Invalid mission format received.")

    def select_agv_for_task(self, selected_agv, station_first, station_last):
        available_agvs = []
        for i, status in enumerate(self.agv_status):
            if status == "Available":
                available_agvs.append(f'AGV{i + 1:02}')

        rospy.loginfo(f"Available AGVs: {available_agvs}")
        if available_agvs:
            if station_first in self.station_coordinates:
                station_first_coords = self.station_coordinates[station_first]
            else:
                rospy.loginfo(f"Station {station_first} coordinates not found.")
                return

            if selected_agv == "Automatic":
                if len(available_agvs) == 1:
                    closest_agv = available_agvs[0]
                    rospy.loginfo(f"Only one AGV available: {closest_agv}")
                else:
                    closest_agv, distances = self.get_closest_agv(available_agvs, station_first_coords)
                    rospy.loginfo(f"Closest :{closest_agv} distances: {distances}")
            else:
                closest_agv = selected_agv

            self.agv_status[int(closest_agv[-2:]) - 1] = "Go to First Station"
            self.publish_agv_status()
            self.publish_task(closest_agv, station_first, station_last)
        else:
            rospy.loginfo("No AGVs available for the task.")

    def get_closest_agv(self, available_agvs, target_coords):
        min_distance = float('inf')
        closest_agv = None
        distances = {}

        for agv in available_agvs:
            start_pose = self.create_pose_stamped(self.agv_poses[agv])
            goal_pose = self.create_pose_stamped(target_coords)
            path_distance = self.calculate_path_length(start_pose, goal_pose, agv.lower())

            distances[agv] = path_distance
            rospy.loginfo(f"{agv} -> {target_coords}: {path_distance} meters")
            if path_distance < min_distance:
                min_distance = path_distance
                closest_agv = agv

        return closest_agv, distances
        
    def ros_subscribe_request_park(self, msg):
        rospy.loginfo(f"{msg.data} : Requested Parking,")
        agv_request = msg.data.upper()

        try:
            agv_index = int(agv_request[-2:]) - 1
        except ValueError:
            rospy.loginfo(f"Invalid AGV name: {agv_request}")
            return

        if self.agv_status[agv_index] == "Go to Last Station":
            self.select_park_for_agv(agv_request)
        else:
            rospy.loginfo(f"{agv_request} requested parking, but the status is not 'Go to Last Station'")

    def select_park_for_agv(self, agv_request):
        empty_parks = [f'Park{i + 1:01}' for i, status in enumerate(self.parking_status_update.station_status.values()) if status == "Empty"]

        rospy.loginfo(f"Empty Parks: {empty_parks}")
        if empty_parks:
            agv_coords = self.agv_poses[agv_request]
            agv_namespace = agv_request.lower()
            if len(empty_parks) == 1:
                closest_park = empty_parks[0]
                rospy.loginfo(f"Only one empty park available: {closest_park}")
            else:
                closest_park, distances = self.get_closest_park(empty_parks, agv_coords, agv_namespace)
                rospy.loginfo(f"Closest : {closest_park} distances: {distances}")

            self.agv_status[int(agv_request[-2:]) - 1] = "Available"
            self.publish_agv_status()
        else:
            rospy.loginfo("No empty parks available.")

    def get_closest_park(self, stations, target_coords, agv_namespace):
        min_distance = float('inf')
        closest_station = None
        distances = {}

        for station in stations:
            start_pose = self.create_pose_stamped(target_coords)
            goal_pose = self.create_pose_stamped(self.station_coordinates[station])
            path_distance = self.calculate_path_length(start_pose, goal_pose, agv_namespace)

            distances[station] = path_distance
            rospy.loginfo(f"{station} -> {target_coords}: {path_distance} meters")
            if path_distance < min_distance:
                min_distance = path_distance
                closest_station = station

        return closest_station, distances

    
    def create_pose_stamped(self, pose):
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = "map"
        pose_stamped.header.stamp = rospy.Time.now()
        pose_stamped.pose.position.x = pose[0]
        pose_stamped.pose.position.y = pose[1]
        pose_stamped.pose.position.z = pose[2]
        pose_stamped.pose.orientation.x = pose[3]
        pose_stamped.pose.orientation.y = pose[4]
        pose_stamped.pose.orientation.z = pose[5]
        pose_stamped.pose.orientation.w = pose[6]
        return pose_stamped

    def calculate_path_length(self, start, goal, namespace):
        service_name = f'/{namespace}/move_base/make_plan'
        rospy.wait_for_service(service_name)
        try:
            get_plan = rospy.ServiceProxy(service_name, GetPlan)
            req = GetPlanRequest()
            req.start = start
            req.goal = goal
            req.tolerance = 0.0
            resp = get_plan(req)
            rospy.loginfo(f"Service call successful, received plan with {len(resp.plan.poses)} poses")

            path_length = 0.0
            prev_pose = None
            for pose in resp.plan.poses:
                if prev_pose:
                    dx = pose.pose.position.x - prev_pose.pose.position.x
                    dy = pose.pose.position.y - prev_pose.pose.position.y
                    path_length += math.sqrt(dx*dx + dy*dy)
                prev_pose = pose
            return path_length
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
            return float('inf')
        
    def publish_agv_status(self):
        agv_status_info = ', '.join(self.agv_status)
        self.agv_status_pub.publish(agv_status_info)
        rospy.loginfo(f"Publish AGV status: {agv_status_info}")

    def publish_task(self, closest_agv, station_first, station_last):
        task_info = f"{closest_agv}, {station_first}, {station_last}"
        self.task_pub.publish(task_info)
        rospy.loginfo(f"Task assigned to {closest_agv}: {station_first} -> {station_last}")

class ParkingStatusUpdate:
    def __init__(self, fleet_management):
        self.fleet_management = fleet_management

        self.station_status = {
            'Park1': "Empty",
            'Park2': "Empty",
            'Park3': "Empty",
            'Park4': "Empty"
        }
        self.bounding_boxes = {
            'A': (1.0, 1.0),
            'B': (1.0, 1.0),
            'C': (1.0, 1.0),
            'D': (1.0, 1.0),
            'E': (1.0, 1.0),
            'G': (1.0, 1.0),
            'Park1': (2.0, 2.5),
            'Park2': (2.0, 2.5),
            'Park3': (2.0, 1.5),
            'Park4': (2.0, 2.5)
        }

        self.parking_status_pub = rospy.Publisher('/parking_status', String, queue_size=10, latch=True)
        rospy.Timer(rospy.Duration(1), self.update_station_status)

    def update_station_status(self, event):
        agv_poses = self.fleet_management.agv_poses
        station_coordinates = self.fleet_management.station_coordinates
        updated_status = {station: "Empty" for station in ['Park1', 'Park2', 'Park3', 'Park4']}

        for agv, agv_pose in agv_poses.items():
            agv_index = int(agv[-2:]) - 1
            if self.fleet_management.agv_status[agv_index] != "Not connect":
                for station, station_pose in station_coordinates.items():
                    if self.is_within_bounds(agv_pose, station_pose, self.bounding_boxes[station]):
                        updated_status[station] = "Occupied"

        self.station_status = updated_status
        self.publish_station_status()

    def is_within_bounds(self, agv_pose, station_pose, bounding_box):
        x_min = station_pose[0] - bounding_box[0] / 2
        x_max = station_pose[0] + bounding_box[0] / 2
        y_min = station_pose[1] - bounding_box[1] / 2
        y_max = station_pose[1] + bounding_box[1] / 2

        return x_min <= agv_pose[0] <= x_max and y_min <= agv_pose[1] <= y_max

    def publish_station_status(self):
        station_status_info = ', '.join([self.station_status[f'Park{i+1}'] for i in range(4)])
        self.parking_status_pub.publish(station_status_info)

if __name__ == '__main__':
    try:
        fleet_management = FleetManagement()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass