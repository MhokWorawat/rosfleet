#!/usr/bin/env python3

import rospy
from std_msgs.msg import String, Float32, Bool
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
import math
import json
import os

class AGV_management:
    def __init__(self, agv_name):
        rospy.init_node(f'{agv_name}_management', anonymous=True)

        self.agv_name = agv_name
        self.agv_status = ["Not connect", "Not connect", "Not connect", "Not connect"]
        self.agv_pose = (0, 0, 0, 0, 0, 0, 0)
        self.agv_velocity = {
            'x': 0.0,
            'y': 0.0,
            'th': 0.0
        }
        self.estimated_time = 0.0
        self.station_coordinates = {
            'A': (2.900, -6.500, 0.000, 0.000, 0.000, 0.707, 0.707),
            'B': (7.950, -1.000, 0.000, 0.000, 0.000, 0.000, 1.000),
            'C': (14.950, -9.450, 0.000, 0.000, 0.000, 1.000, 0.000),
            'D': (6.750, -10.550, 0.000, 0.000, 0.000, 0.000, 1.000),
            'E': (19.600, -10.300, 0.000, 0.000, 0.000, 1.000, 0.000),
            'G': (16.600, -20.850, 0.000, 0.000, 0.000, 1.000, 0.000),
            'P1': (1.050, -9.700, 0.000, 0.000, 0.000, 0.000, 1.000),
            'P2': (17.900, -7.800, 0.000, 0.000, 0.000, -0.707, 0.707),
            'P3': (0.850, -1.000, 0.000, 0.000, 0.000, 0.000, 1.000),
            'P4': (24.700, -9.750, 0.000, 0.000, 0.000, 1.000, 0.000)
        }

        self.first_station = None
        self.last_station = None

        # Initialize timers
        self.stop_publishing_timer = None
        self.stop_duration_timer = None

        # ROS Publishers
        self.request_parking_pub = rospy.Publisher('/request_parking', String, queue_size=10)
        self.agv_status_pub = rospy.Publisher('/agv_status', String, queue_size=10)
        self.estimated_time_pub = rospy.Publisher(f'/{self.agv_name.lower()}/estimated_time', Float32, queue_size=10)
        self.goal_pub = rospy.Publisher(f"/{self.agv_name.lower()}/move_base_simple/goal", PoseStamped, queue_size=10)
        self.stop_cmd_pub = rospy.Publisher(f"/{self.agv_name.lower()}/cmd_vel", Twist, queue_size=10)
        self.buzzer_pub = rospy.Publisher(f"/{self.agv_name.lower()}/buzzer", Bool, queue_size=10)

        # ROS Subscribers
        rospy.Subscriber(f'/{self.agv_name.lower()}/amcl_pose', PoseWithCovarianceStamped, self.agv_pose_callback)
        rospy.Subscriber(f'/{self.agv_name.lower()}/Velocity', Twist, self.ros_subscribe_agv_velocity)
        rospy.Subscriber('/object_detection', String, self.ros_subscribe_object_detection)
        rospy.Subscriber('/agv_status', String, self.ros_subscribe_agv_status)
        rospy.Subscriber('/closest_parking', String, self.closest_parking_callback)
        rospy.Subscriber('/task', String, self.ros_subscribe_task)

        # Initialize sequence number
        self.seq = 0

    def agv_pose_callback(self, msg):
        rospy.logdebug(f"Received {self.agv_name} pose: {msg}")
        self.agv_pose = (msg.pose.pose.position.x, 
                         msg.pose.pose.position.y,
                         msg.pose.pose.position.z,
                         msg.pose.pose.orientation.x,
                         msg.pose.pose.orientation.y,
                         msg.pose.pose.orientation.z,
                         msg.pose.pose.orientation.w)
    
    def ros_subscribe_agv_velocity(self, msg):
        self.agv_velocity['x'] = msg.linear.x
        self.agv_velocity['y'] = msg.linear.y
        self.agv_velocity['th'] = msg.angular.z

    def ros_subscribe_agv_status(self, msg):
        rospy.loginfo(f"{self.agv_name} Received AGV status: {msg.data}")
        status_list = msg.data.split(', ')
        if len(status_list) == 4:
            self.agv_status = status_list

    def ros_subscribe_task(self, msg):
        rospy.loginfo(f"{self.agv_name} Received task: {msg.data}")
        task_data = msg.data.split(', ')
        if len(task_data) != 3:
            rospy.logwarn(f"Invalid task data received: {msg.data}")
            return
        if task_data[0] != self.agv_name:
            return
        agv_name, first_station, last_station = task_data
        rospy.loginfo(f"Parsed task data -> AGV: {agv_name}, First Station: {first_station}, Last Station: {last_station}")
        self.handle_task(first_station, last_station)

    # def handle_task(self, first_station, last_station):
    #     rospy.loginfo(f"Handling task for {self.agv_name} from {first_station} to {last_station}")
    #     self.first_station = first_station
    #     self.last_station = last_station
    #     agv_index = int(self.agv_name[-2:]) - 1

    #     if self.agv_status[agv_index] == "Go to First Station":
    #         target_coords = self.station_coordinates.get(first_station)
    #         if target_coords is None:
    #             rospy.logwarn(f"Station {first_station} coordinates not found for {self.agv_name}")
    #             return

    #         rospy.loginfo(f"Sending {self.agv_name} to first station: {first_station} with coordinates {target_coords}")
    #         self.send_goal(target_coords)
    #         self.wait_for_arrival(target_coords)
            
    #         self.agv_status[agv_index] = "Go to Last Station"
    #         self.publish_agv_status()
    #         rospy.sleep(3)
    #         return

            # target_coords = self.station_coordinates.get(last_station)
            # if target_coords is None:
            #     rospy.logwarn(f"Station {last_station} coordinates not found for {self.agv_name}")
            #     return

            # rospy.loginfo(f"Sending {self.agv_name} to last station: {last_station} with coordinates {target_coords}")
            # self.send_goal(target_coords)
            # self.wait_for_arrival(target_coords)
            # rospy.sleep(3)
            # self.request_parking_pub.publish(self.agv_name)
        
        # rospy.sleep(1)

    def handle_task(self, first_station, last_station):
        rospy.loginfo(f"Handling task for {self.agv_name} from {first_station} to {last_station}")
        self.first_station = first_station
        self.last_station = last_station
        agv_index = int(self.agv_name[-2:]) - 1

        if self.agv_status[agv_index] == "Go to First Station":
            rospy.logdebug(f"Loading path from {self.agv_name} to {first_station}")
            target_coords = self.station_coordinates.get(first_station)
            rospy.logdebug(f"Sending {self.agv_name} to first station: {first_station} with coordinates {target_coords}")
            self.send_goal(target_coords)
            self.wait_for_arrival(target_coords)
            self.agv_status[agv_index] = "Go to Last Station"
            self.publish_agv_status()
            rospy.sleep(5)

        if self.agv_status[agv_index] == "Go to Last Station":
            rospy.logdebug(f"Loading path from {first_station} to {last_station}")
            target_coords = self.station_coordinates.get(last_station)
            rospy.logdebug(f"Sending {self.agv_name} to last station: {last_station} with coordinates {target_coords}")
            self.send_goal(target_coords)
            self.wait_for_arrival(target_coords)
            rospy.sleep(5)
            self.request_parking_pub.publish(self.agv_name)

    def closest_parking_callback(self, msg):
        if msg.data.startswith(self.agv_name):
            rospy.loginfo(f"{self.agv_name} Received closest parking info: {msg.data}")
            agv_name, closest_parking = msg.data.split(', ')
            if agv_name == self.agv_name:
                target_coords = self.station_coordinates.get(closest_parking)
                rospy.loginfo(f"Sending {self.agv_name} to parking at {closest_parking} with coordinates {target_coords}")
                self.send_goal(target_coords)
                self.wait_for_arrival(target_coords)

    def ros_subscribe_object_detection(self, msg):
        rospy.loginfo(f"{self.agv_name} Received object detection data: {msg.data}")
        detection_data = [item.strip() for item in msg.data.split('|')]

        obstacle = []
        for data in detection_data:
            parts = data.split(":")
            if len(parts) == 2:
                agv = parts[0].strip()
                detected_object = parts[1].strip()
                obstacle.append((agv, detected_object))

        for agv, detected_object in obstacle:
            if agv == self.agv_name:
                if detected_object in ["box", "people"]:
                    rospy.loginfo(f"{self.agv_name} Detected {detected_object}. Stopping...")
                    self.buzzer_pub.publish(True)
                else:
                    self.buzzer_pub.publish(False)

    def wait_for_arrival(self, target_coords):
        rospy.loginfo(f"Waiting for {self.agv_name} to arrive at target {target_coords}")
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            current_coords = self.agv_pose
            current_velocity = math.sqrt((self.agv_velocity['x'] ** 2) + (self.agv_velocity['y'] ** 2) + (self.agv_velocity['th'] ** 2))
            distance = math.sqrt((current_coords[0] - target_coords[0]) ** 2 + (current_coords[1] - target_coords[1]) ** 2)
            
            if current_velocity > 0:
                self.estimated_time = distance / current_velocity
            else:
                self.estimated_time = self.estimated_time

            self.estimated_time_pub.publish(self.estimated_time)

            if distance < 0.2:
                rospy.loginfo(f"{self.agv_name} arrived at target.")
                self.estimated_time = 0.0
                self.estimated_time_pub.publish(self.estimated_time)
                break
            
            rate.sleep()

    # def publish_stop_command(self, duration=None):
    #     stop_cmd = Twist()
    #     stop_cmd.linear.x = 0.0
    #     stop_cmd.linear.y = 0.0
    #     stop_cmd.angular.z = 0.0
        
    #     if duration:
    #         rospy.loginfo(f"Publishing stop command for {self.agv_name} for {duration} seconds")
            
    #         def stop_publishing_callback(event):
    #             self.stop_cmd_pub.publish(stop_cmd)
    #             rospy.loginfo(f"Continuing to publish stop command for {self.agv_name}.")
    #             rospy.loginfo(f"{self.agv_name} >> stop")

    #         self.stop_publishing_timer = rospy.Timer(rospy.Duration(0.1), stop_publishing_callback)
    #         rospy.sleep(duration)
    #         self.stop_publishing_timer.shutdown()
    #         rospy.loginfo(f"Stopped publishing stop command for {self.agv_name} after {duration} seconds")
    #     else:
    #         def stop_publishing_callback(event):
    #             self.stop_cmd_pub.publish(stop_cmd)

    #         self.stop_publishing_timer = rospy.Timer(rospy.Duration(0.1), stop_publishing_callback)

    def publish_agv_status(self):
        status_msg = ", ".join(self.agv_status)
        rospy.loginfo(f"Publishing {self.agv_name} status: {status_msg}")
        self.agv_status_pub.publish(status_msg)

    def send_goal(self, target_coords):
        if not self.goal_pub.get_num_connections():
            rospy.logwarn(f"{self.agv_name} No subscribers connected to /move_base_simple/goal topic.")
            return

        goal = PoseStamped()
        goal.header.seq = self.seq
        goal.header.stamp = rospy.Time.now()
        goal.header.frame_id = "map"
        goal.pose.position.x, goal.pose.position.y, goal.pose.position.z = target_coords[:3]
        goal.pose.orientation.x, goal.pose.orientation.y, goal.pose.orientation.z, goal.pose.orientation.w = target_coords[3:]
        
        rospy.loginfo(f"{self.agv_name} Sending goal: {target_coords}")
        self.goal_pub.publish(goal)
        self.seq += 1

    # def load_and_send_path(self, first_station, last_station):
    #     data_path_file_name = f"{first_station}to{last_station}.json"
    #     path_file = os.path.dirname(__file__)
    #     pathtofile_data_path = os.path.join(path_file, f"../data_paths/{data_path_file_name}")
    #     rospy.loginfo(f"Loading path from file {data_path_file_name}")

    #     try:
    #         with open(pathtofile_data_path, 'r') as file:
    #             path_data = json.load(file)
    #     except FileNotFoundError:
    #         rospy.logwarn(f"Path file {data_path_file_name} not found.")
    #         return
    #     except json.JSONDecodeError as e:
    #         rospy.logwarn(f"Error loading JSON file {data_path_file_name}: {e}")
    #         return

    #     # Initialize action client
    #     client = actionlib.SimpleActionClient(f"/{self.agv_name.lower()}/move_base", MoveBaseAction)
    #     client.wait_for_server()

    #     for pose_dict in path_data:
    #         # Create a goal from each waypoint
    #         goal = MoveBaseGoal()
    #         goal.target_pose.header.stamp = rospy.Time.now()
    #         goal.target_pose.header.frame_id = 'map'

    #         goal.target_pose.pose.position.x = pose_dict['position']['x']
    #         goal.target_pose.pose.position.y = pose_dict['position']['y']
    #         goal.target_pose.pose.position.z = pose_dict['position']['z']
    #         goal.target_pose.pose.orientation.x = pose_dict['orientation']['x']
    #         goal.target_pose.pose.orientation.y = pose_dict['orientation']['y']
    #         goal.target_pose.pose.orientation.z = pose_dict['orientation']['z']
    #         goal.target_pose.pose.orientation.w = pose_dict['orientation']['w']

    #         rospy.loginfo(f"Sending waypoint: {goal}")
    #         client.send_goal(goal)

    #         # Wait for the result
    #         client.wait_for_result()
    #         result = client.get_result()

    #         if result:
    #             rospy.loginfo(f"Reached waypoint: {pose_dict['position']}")
    #         else:
    #             rospy.logwarn(f"Failed to reach waypoint: {pose_dict['position']}")
    #             break
    
    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        agv_management = AGV_management()
        agv_management.run()
    except rospy.ROSInterruptException:
        pass
