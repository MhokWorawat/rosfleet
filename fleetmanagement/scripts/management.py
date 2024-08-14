#!/usr/bin/env python3

import rospy
from std_msgs.msg import String, Float32, Bool
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Path
from threading import Timer
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

        # Initialize stop publishing timer
        self.stop_publishing_timer = None

        # ROS Publishers
        self.request_parking_pub = rospy.Publisher('/request_parking', String, queue_size=10, latch=True)
        self.agv_status_pub = rospy.Publisher('/agv_status', String, queue_size=10, latch=True)
        self.estimated_time_pub = rospy.Publisher(f'/{self.agv_name.lower()}/estimated_time', Float32, queue_size=10, latch=True)
        self.path_pub = rospy.Publisher(f"/{self.agv_name.lower()}/move_base/TrajectoryPlannerROS/global_plan", Path, queue_size=10)
        self.stop_cmd_pub = rospy.Publisher(f"/{self.agv_name.lower()}/cmd_vel", Twist, queue_size=10, latch=True)
        self.buzzer_pub = rospy.Publisher(f"/{self.agv_name.lower()}/buzzer", Bool, queue_size=10, latch=True)

        # ROS Subscribers
        rospy.Subscriber(f'/{self.agv_name.lower()}/amcl_pose', PoseWithCovarianceStamped, self.agv_pose_callback)
        rospy.Subscriber(f'/{self.agv_name.lower()}/Velocity', Twist, self.ros_subscribe_agv_velocity)
        rospy.Subscriber('/object_detection', String, self.ros_subscribe_object_detection)
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
        rospy.logdebug(f"Parsed task data -> AGV: {agv_name}, First Station: {first_station}, Last Station: {last_station}")
        self.handle_task(first_station, last_station)

    def handle_task(self, first_station, last_station):
        rospy.loginfo(f"Handling task for {self.agv_name} from {first_station} to {last_station}")

        if self.agv_status[int(self.agv_name[-2:]) - 1] == "Go to First Station":
            rospy.logdebug(f"Loading path from {self.agv_name} to {first_station}")
            target_coords = self.station_coordinates.get(first_station)
            rospy.logdebug(f"Sending {self.agv_name} to first station: {first_station} with coordinates {target_coords}")
            # self.load_and_send_path(first_station, last_station)
            self.send_goal(target_coords)
            self.wait_for_arrival(target_coords)
            self.publish_stop_command()
            rospy.sleep(10)
            self.agv_status[int(self.agv_name[-2:]) - 1] = "Go to Last Station"
            self.publish_agv_status()

        if self.agv_status[int(self.agv_name[-2:]) - 1] == "Go to Last Station":
            rospy.logdebug(f"Loading path from {first_station} to {last_station}")
            target_coords = self.station_coordinates.get(last_station)
            rospy.logdebug(f"Sending {self.agv_name} to last station: {last_station} with coordinates {target_coords}")
            self.send_goal(target_coords)
            self.wait_for_arrival(target_coords)
            self.publish_stop_command()
            rospy.sleep(10)
            self.request_parking_pub.publish(self.agv_name)

    def closest_parking_callback(self, msg):
        if msg.data.startswith(self.agv_name):
            rospy.loginfo(f"{self.agv_name} Received closest parking info: {msg.data}")
            agv_name, closest_parking = msg.data.split(', ')
            target_coords = self.station_coordinates.get(closest_parking)
            rospy.logdebug(f"Sending {self.agv_name} to parking at {closest_parking} with coordinates {target_coords}")
            self.send_goal(target_coords)
            self.wait_for_arrival(target_coords)

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
        path_msg.header.frame_id = 'map'

        path_msg.poses = []
        for pose_dict in path_data:
            pose_stamped = PoseStamped()
            pose_stamped.header.seq = self.seq
            pose_stamped.header.stamp = rospy.Time.now()
            pose_stamped.header.frame_id = 'map'

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

    def publish_stop_command(self):
        rospy.loginfo(f"Stopping {self.agv_name}")
        stop_cmd = Twist()
        stop_cmd.linear.x = 0.0
        stop_cmd.linear.y = 0.0
        stop_cmd.linear.z = 0.0
        stop_cmd.angular.x = 0.0
        stop_cmd.angular.y = 0.0
        stop_cmd.angular.z = 0.0
        self.stop_cmd_pub.publish(stop_cmd)
        
        if self.stop_publishing_timer is not None and self.stop_publishing_timer.is_alive():
            self.stop_publishing_timer.cancel()

        self.stop_publishing_timer = Timer(2.0, self.publish_stop_command)
        self.stop_publishing_timer.start()

    def publish_agv_status(self):
        agv_status_str = ', '.join(self.agv_status)
        rospy.loginfo(f"Publishing AGV status: {agv_status_str}")
        self.agv_status_pub.publish(agv_status_str)

    def ros_subscribe_object_detection(self, msg):
        agv_entries = msg.data.split('|')
        for entry in agv_entries:
            entry = entry.strip()
            if ':' in entry:
                agv_name, detected_objects = entry.split(':', 1)
                agv_name = agv_name.strip()
                detected_objects = detected_objects.strip()

                if agv_name == self.agv_name:
                    if detected_objects:
                        rospy.loginfo(f"{self.agv_name} detected objects: {detected_objects}")
                        self.publish_stop_command()
                        self.buzzer_pub.publish(True)
                    else:
                        self.buzzer_pub.publish(False)

if __name__ == '__main__':
    try:
        agv_management = AGV_management()
        agv_management.run()
    except rospy.ROSInterruptException:
        pass
