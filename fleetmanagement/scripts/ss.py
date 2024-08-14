#!/usr/bin/env python3

import rospy
from std_msgs.msg import String, Float32
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Path
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
            'C': (14.950, -9.200, 0.000, 0.000, 0.000, 0.000, 1.000),
            'D': (6.050, -10.250, 0.000, 0.000, 0.000, -0.707, 0.707),
            'E': (19.600, -10.300, 0.000, 0.000, 0.000, 0.000, 1.000),
            'G': (16.600, -20.850, 0.000, 0.000, 0.000, 1.000, 0.000),
            'P1': (1.050, -9.700, 0.000, 0.000, 0.000, 0.000, 1.000),
            'P2': (17.900, -7.800, 0.000, 0.000, 0.000, -0.707, 0.707),
            'P3': (1.050, -1.000, 0.000, 0.000, 0.000, 0.000, 1.000),
            'P4': (24.700, -9.750, 0.000, 0.000, 0.000, 1.000, 0.000)
        }

        # ROS Publishers
        self.request_parking_pub = rospy.Publisher('/request_parking', String, queue_size=10, latch=True)
        self.agv_status_pub = rospy.Publisher('/agv_status', String, queue_size=10, latch=True)
        self.estimated_time_pub = rospy.Publisher(f'/{self.agv_name.lower()}/estimated_time', Float32, queue_size=10, latch=True)

        # ROS Subscribers
        rospy.Subscriber(f'/{self.agv_name.lower()}/amcl_pose', PoseWithCovarianceStamped, self.agv_pose_callback)
        rospy.Subscriber(f'/{self.agv_name.lower()}/Velocity', Float32, self.ros_subscribe_agv_velocity)
        rospy.Subscriber('/agv_status', String, self.ros_subscribe_agv_status)
        rospy.Subscriber('/closest_parking', String, self.closest_parking_callback)
        rospy.Subscriber('/task', String, self.ros_subscribe_task)

        # Initialize sequence number
        self.seq = 0

    def run(self):
        rospy.spin()

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
        # Assuming Float32 message contains a single float value for linear velocity
        self.agv_velocity['x'] = msg.linear.x.data
        self.agv_velocity['y'] = msg.linear.y.data
        self.agv_velocity['th'] = msg.angular.z.data

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
        rospy.logdebug(f"Parsed task data - AGV: {agv_name}, First Station: {first_station}, Last Station: {last_station}")
        self.handle_task(first_station, last_station)

    def handle_task(self, first_station, last_station):
        rospy.loginfo(f"Handling task for {self.agv_name} from {first_station} to {last_station}")

        if self.agv_status[int(self.agv_name[-2:]) - 1] == "Go to First Station":
            rospy.logdebug(f"Loading path from {self.agv_name} to {first_station}")
            target_coords = self.station_coordinates.get(first_station)
            if target_coords is None:
                rospy.logwarn(f"Invalid first station: {first_station}")
                return
            rospy.logdebug(f"Sending {self.agv_name} to first station: {first_station} with coordinates {target_coords}")
            self.send_goal(target_coords)
            self.wait_for_arrival(target_coords)

            rospy.sleep(5)
            self.agv_status[int(self.agv_name[-2:]) - 1] = "Go to Last Station"
            self.publish_agv_status()

        if self.agv_status[int(self.agv_name[-2:]) - 1] == "Go to Last Station":
            rospy.logdebug(f"Loading path from {first_station} to {last_station}")
            self.load_and_send_path(first_station, last_station)
            target_coords = self.station_coordinates.get(last_station)
            if target_coords is None:
                rospy.logwarn(f"Invalid last station: {last_station}")
                return
            rospy.logdebug(f"Sending {self.agv_name} to last station: {last_station} with coordinates {target_coords}")
            self.send_goal(target_coords)
            self.wait_for_arrival(target_coords)

            rospy.sleep(5)
            self.request_parking_pub.publish(self.agv_name)

    def send_goal(self, target_coords):
        goal_topic = f"/{self.agv_name.lower()}/move_base_simple/goal"
        goal_msg = PoseStamped()
        goal_msg.header.frame_id = "map"
        goal_msg.pose.position.x = target_coords[0]
        goal_msg.pose.position.y = target_coords[1]
        goal_msg.pose.position.z = target_coords[2]
        goal_msg.pose.orientation.x = target_coords[3]
        goal_msg.pose.orientation.y = target_coords[4]
        goal_msg.pose.orientation.z = target_coords[5]
        goal_msg.pose.orientation.w = target_coords[6]

        rospy.loginfo(f"Publishing goal for {self.agv_name} on topic {goal_topic} with message {goal_msg}")
        pub = rospy.Publisher(goal_topic, PoseStamped, queue_size=10)
        
        while pub.get_num_connections() == 0:
            rospy.loginfo(f"Waiting for subscribers to connect to {goal_topic}")
            rospy.sleep(0.1)

        pub.publish(goal_msg)
        rospy.loginfo(f"Goal published for {self.agv_name} on topic {goal_topic}")

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

    def load_and_send_path(self, first_station, last_station):
        data_path_file_name = f"{first_station}to{last_station}.json"
        path_file = os.path.dirname(__file__)
        pathtofile_data_path = os.path.join(path_file, f"../data_paths/{data_path_file_name}")
        rospy.logdebug(f"Loading path from file {data_path_file_name}")

        try:
            with open(pathtofile_data_path, 'r') as file:
                path_data = json.load(file)
        except FileNotFoundError:
            rospy.logwarn(f"Path file {data_path_file_name} not found.")
            return
        except json.JSONDecodeError as e:
            rospy.logwarn(f"Error loading JSON file {data_path_file_name}: {e}")
            return

        path_msg = Path()
        path_msg.header.seq = self.seq
        path_msg.header.stamp = rospy.Time.now()
        path_msg.header.frame_id = f'{self.agv_name.lower()}_odom'

        path_msg.poses = []
        for pose_dict in path_data:
            pose_stamped = PoseStamped()
            pose_stamped.header.seq = self.seq
            pose_stamped.header.stamp = rospy.Time.now()
            pose_stamped.header.frame_id = f'{self.agv_name.lower()}_odom'

            pose_stamped.pose.position.x = pose_dict['position']['x']
            pose_stamped.pose.position.y = pose_dict['position']['y']
            pose_stamped.pose.position.z = pose_dict['position']['z']
            pose_stamped.pose.orientation.x = pose_dict['orientation']['x']
            pose_stamped.pose.orientation.y = pose_dict['orientation']['y']
            pose_stamped.pose.orientation.z = pose_dict['orientation']['z']
            pose_stamped.pose.orientation.w = pose_dict['orientation']['w']
            path_msg.poses.append(pose_stamped)

            self.seq += 1

        path_topic = f"/{self.agv_name.lower()}/move_base/TrajectoryPlannerROS/global_plan"

        if not hasattr(self, 'pub'):
            self.pub = rospy.Publisher(path_topic, Path, queue_size=10)
            rospy.loginfo(f"Publisher created for topic {path_topic}")

        self.pub.publish(path_msg)
        rospy.loginfo("Path data published successfully.")

    def closest_parking_callback(self, msg):
        rospy.loginfo(f"Received closest parking info: {msg.data}")
        agv_name, closest_park = msg.data.split(', ')
        if agv_name != self.agv_name:
            return
        self.agv_status[int(self.agv_name[-2:]) - 1] = "Available"
        self.publish_agv_status()

        target_coords = self.station_coordinates.get(closest_park)
        if target_coords is None:
            rospy.logwarn(f"Invalid parking location: {closest_park}")
            return
        rospy.logdebug(f"Sending {self.agv_name} to closest parking: {closest_park} with coordinates {target_coords}")
        self.send_goal(target_coords)
        
    def publish_agv_status(self):
        agv_status_info = ', '.join(self.agv_status)
        self.agv_status_pub.publish(agv_status_info)
        rospy.loginfo(f"Published AGV status: {agv_status_info}")

if __name__ == '__main__':
    try:
        AGV_management = AGV_management()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
