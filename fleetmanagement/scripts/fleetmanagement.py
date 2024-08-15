#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from std_msgs.msg import Int32
from nav_msgs.srv import GetPlan, GetPlanRequest
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from detection_msgs.msg import BoundingBoxes
from geometry_msgs.msg import Twist
from threading import Timer
import math

class FleetManagement:
    def __init__(self):
        rospy.init_node('fleet_management', anonymous=True)

        self.agv_status = ["Not connect", "Not connect", "Not connect", "Not connect"]
        self.mission_counter = 0
        self.agv_poses = {
            'AGV01': (0, 0, 0, 0, 0, 0, 0),
            'AGV02': (0, 0, 0, 0, 0, 0, 0),
            'AGV03': (0, 0, 0, 0, 0, 0, 0),
            'AGV04': (0, 0, 0, 0, 0, 0, 0)
        }
        self.detections = {
            'AGV01': [],
            'AGV02': [],
            'AGV03': [],
            'AGV04': []
        }
        self.block_update_station = {
            'P1': [],
            'P2': [],
            'P3': [],
            'P4': []
        }
        self.station_coordinates = {
            'A': (2.900, -6.500, 0.000, 0.000, 0.000, 0.707, 0.707),
            'B': (7.950, -1.000, 0.000, 0.000, 0.000, 0.000, 1.000),
            'C': (14.950, -9.200, 0.000, 0.000, 0.000, 0.000, 1.000),
            'D': (6.050, -10.00, 0.000, 0.000, 0.000, -0.707, 0.707),
            'E': (19.600, -10.300, 0.000, 0.000, 0.000, 0.000, 1.000),
            'G': (16.600, -20.850, 0.000, 0.000, 0.000, 1.000, 0.000),
            'P1': (1.050, -9.700, 0.000, 0.000, 0.000, 0.000, 1.000),
            'P2': (17.900, -7.800, 0.000, 0.000, 0.000, -0.707, 0.707),
            'P3': (1.050, -1.000, 0.000, 0.000, 0.000, 0.000, 1.000),
            'P4': (24.700, -9.750, 0.000, 0.000, 0.000, 1.000, 0.000)
        }
        self.stop_publishing = {}
        self.cmd_vel_pubs = {}

        # ROS Publishers
        self.mission_counter_pub = rospy.Publisher('/counter', Int32, queue_size=10, latch=True)
        self.agv_status_pub = rospy.Publisher('/agv_status', String, queue_size=10, latch=True)
        self.object_detection_pub = rospy.Publisher('/object_detection', String, queue_size=10)
        self.closest_parking_pub = rospy.Publisher('/closest_parking', String, queue_size=10)
        self.task_pub = rospy.Publisher('/task', String, queue_size=10)
        self.cmd_vel_pub = {
            'AGV01': rospy.Publisher(f'/agv01/cmd_vel', Twist, queue_size=10),
            'AGV02': rospy.Publisher(f'/agv02/cmd_vel', Twist, queue_size=10),
            'AGV03': rospy.Publisher(f'/agv03/cmd_vel', Twist, queue_size=10),
            'AGV04': rospy.Publisher(f'/agv04/cmd_vel', Twist, queue_size=10)
        }

        # ROS Subscribers
        rospy.Subscriber('/agv01/amcl_pose', PoseWithCovarianceStamped, self.agv01_pose_callback)
        rospy.Subscriber('/agv02/amcl_pose', PoseWithCovarianceStamped, self.agv02_pose_callback)
        rospy.Subscriber('/agv03/amcl_pose', PoseWithCovarianceStamped, self.agv03_pose_callback)
        rospy.Subscriber('/agv04/amcl_pose', PoseWithCovarianceStamped, self.agv04_pose_callback)
        rospy.Subscriber('/request_parking', String, self.ros_subscribe_request_park)
        rospy.Subscriber('/agv_status', String, self.ros_subscribe_agv_status)
        rospy.Subscriber('/mission', String, self.ros_subscribe_mission)
        self.detection_subscribers = {
            'AGV01': rospy.Subscriber('/agv01/yolov5/detections', BoundingBoxes, self.ros_subscribe_detection, 'AGV01'),
            'AGV02': rospy.Subscriber('/agv02/yolov5/detections', BoundingBoxes, self.ros_subscribe_detection, 'AGV02'),
            'AGV03': rospy.Subscriber('/agv03/yolov5/detections', BoundingBoxes, self.ros_subscribe_detection, 'AGV03'),
            'AGV04': rospy.Subscriber('/agv04/yolov5/detections', BoundingBoxes, self.ros_subscribe_detection, 'AGV04')
        }

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

            if station_first not in self.station_coordinates or station_last not in self.station_coordinates:
                rospy.logerr(f"Invalid station names: {station_first}, {station_last}")
                return
            
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
        self.mission_counter += 1
        self.mission_counter_pub.publish(self.mission_counter)
        rospy.loginfo(f"Mission Counter {self.mission_counter}")

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
        empty_parks = [f'P{i + 1:01}' for i, status in enumerate(self.parking_status_update.station_status.values()) if status == "Empty"]

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

            self.block_update_station[closest_park] = agv_request
            self.publish_closest_parking(agv_request, closest_park)
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
        
        max_retries = 3
        retry_delay = 2
        attempts = 0

        while attempts < max_retries:
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
                attempts += 1
                rospy.logerr(f"Service call failed: {e}, attempt {attempts} of {max_retries}")
                if attempts < max_retries:
                    rospy.sleep(retry_delay)
                else:
                    rospy.logerr(f"Failed to call service {service_name} after {max_retries} attempts. Returning infinity.")
                    return float('inf')
        
    def publish_agv_status(self):
        agv_status_info = ', '.join(self.agv_status)
        self.agv_status_pub.publish(agv_status_info)
        rospy.loginfo(f"Publish AGV status: {agv_status_info}")

    def publish_task(self, closest_agv, station_first, station_last):
        task_info = f"{closest_agv}, {station_first}, {station_last}"
        self.task_pub.publish(task_info)
        rospy.loginfo(f"Task assigned to {closest_agv}: {station_first} -> {station_last}")

    def publish_closest_parking(self, agv_request, closest_park):
        closest_parking_info = f"{agv_request}, {closest_park}"
        self.closest_parking_pub.publish(closest_parking_info)
        rospy.loginfo(f"Closest Parking for {agv_request}: {closest_park}")

    def ros_subscribe_detection(self, msg, agv_name):
        self.detections[agv_name] = [box for box in msg.bounding_boxes if box.Class in ["people", "box"]]
        if self.detections[agv_name]:
            rospy.Timer(rospy.Duration(2), self.publish_object_detection)
        else:
            self.detections.pop(agv_name, None)
        
    def publish_object_detection(self, event=None):
        if not self.detections:
            rospy.loginfo("No detections available.")
            return

        detections_msg = ""
        for agv_name, boxes in self.detections.items():
            people_detections = [box for box in boxes if box.Class == "people"]
            box_detections = [box for box in boxes if box.Class == "box"]

            detections_msg += f"{agv_name} : "
            detection_classes = [box.Class for box in people_detections + box_detections]
            if detection_classes:
                detections_msg += ", ".join(detection_classes)
            detections_msg += " | "
        
        if detections_msg.endswith(" | "):
            detections_msg = detections_msg[:-3]
        
        self.object_detection_pub.publish(detections_msg.strip())

    def check_parking_reservations(self, event):
        for station, agv_list in self.block_update_station.items():
            if agv_list:
                for agv in agv_list:
                    agv_pose = self.agv_poses[agv]
                    station_pose = self.station_coordinates[station]

                    if self.is_close_to_parking(agv_pose, station_pose):
                        rospy.loginfo(f"{agv} has reached {station}. Clearing reservation.")
                        self.block_update_station[station].remove(agv)

    def is_close_to_parking(self, agv_pose, station_pose):
        distance = math.sqrt((agv_pose[0] - station_pose[0])**2 + (agv_pose[1] - station_pose[1])**2)
        return distance <= 0.2

    def run(self):
        rospy.spin()

class ParkingStatusUpdate:
    def __init__(self, fleet_management):
        self.fleet_management = fleet_management

        self.station_status = {
            'P1': "Empty",
            'P2': "Empty",
            'P3': "Empty",
            'P4': "Empty"
        }
        self.bounding_boxes = {
            'A': (0.6, 0.8),
            'B': (0.8, 0.6),
            'C': (0.8, 0.6),
            'D': (0.8, 0.6),
            'E': (0.8, 0.6),
            'G': (0.8, 0.6),
            'P1': (2.0, 2.5),
            'P2': (1.75, 2.95),
            'P3': (2.0, 1.95),
            'P4': (1.85, 2.5)
        }

        self.parking_status_pub = rospy.Publisher('/parking_status', String, queue_size=10, latch=True)
        rospy.Timer(rospy.Duration(1), self.update_station_status)

    def update_station_status(self, event):
        agv_poses = self.fleet_management.agv_poses
        station_coordinates = self.fleet_management.station_coordinates
        updated_status = {station: "Empty" for station in ['P1', 'P2', 'P3', 'P4']}

        for station in ['P1', 'P2', 'P3', 'P4']:
            if self.fleet_management.block_update_station[station]:
                updated_status[station] = "Occupied"
                continue

            for agv, agv_pose in agv_poses.items():
                agv_index = int(agv[-2:]) - 1
                if self.fleet_management.agv_status[agv_index] != "Not connect":
                    station_pose = station_coordinates[station]
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
        station_status_info = ', '.join([self.station_status[f'P{i+1}'] for i in range(4)])
        self.parking_status_pub.publish(station_status_info)

if __name__ == '__main__':
    try:
        fleet_management = FleetManagement()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass