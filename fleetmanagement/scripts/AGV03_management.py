#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
import math
import yaml

class AGV03_management:
    def __init__(self):
        rospy.init_node('AGV03_management', anonymous=True)

        self.agv_name = 'AGV03'
        self.agv_status = ["Not connect", "Not connect", "Not connect", "Not connect"]
        self.agv_pose = (0, 0, 0, 0, 0, 0, 0)
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
        self.request_parking_pub = rospy.Publisher('/request_parking', String, queue_size=10, latch=True)
        self.agv_status_pub = rospy.Publisher('/agv_status', String, queue_size=10, latch=True)

        # ROS Subscribers
        rospy.Subscriber(f'/{self.agv_name.lower()}/amcl_pose', PoseWithCovarianceStamped, self.agv_pose_callback)
        rospy.Subscriber('/agv_status', String, self.ros_subscribe_agv_status)
        rospy.Subscriber('/closest_parking', String, self.closest_parking_callback)
        rospy.Subscriber('/task', String, self.ros_subscribe_task)

    def ros_subscribe_task(self, msg):
        rospy.loginfo(f"{self.agv_name} Received task: {msg.data}")
        task_data = msg.data.split(', ')
        if len(task_data) != 3:
            return
        if task_data[0] != self.agv_name:  
            return
        agv_name, first_station, last_station = task_data
        rospy.logdebug(f"Parsed task data - AGV: {agv_name}, First Station: {first_station}, Last Station: {last_station}")
        self.handle_task(first_station, last_station)

    def agv_pose_callback(self, msg):
        rospy.logdebug(f"Received {self.agv_name} pose: {msg}")
        self.agv_pose = (msg.pose.pose.position.x, 
                         msg.pose.pose.position.y,
                         msg.pose.pose.position.z,
                         msg.pose.pose.orientation.x,
                         msg.pose.pose.orientation.y,
                         msg.pose.pose.orientation.z,
                         msg.pose.pose.orientation.w)
    
    def ros_subscribe_agv_status(self, msg):
        rospy.loginfo(f"{self.agv_name} Received AGV status: {msg.data}")
        status_list = msg.data.split(', ')
        if len(status_list) == 4:
            self.agv_status = status_list

    def handle_task(self, first_station, last_station):
        rospy.loginfo(f"Handling task for {self.agv_name} from {first_station} to {last_station}")

        if self.agv_status[int(self.agv_name[-2:]) - 1] == "Go to First Station":
            rospy.logdebug(f"Loading path from {self.agv_name} to {first_station}")
            target_coords = self.station_coordinates[first_station]
            rospy.logdebug(f"Sending {self.agv_name} to first station: {first_station} with coordinates {target_coords}")
            self.send_goal(target_coords)
            self.wait_for_arrival(target_coords)

            rospy.sleep(5)
            self.agv_status[int(self.agv_name[-2:]) - 1] = "Go to Last Station"
            self.publish_agv_status()

        if self.agv_status[int(self.agv_name[-2:]) - 1] == "Go to Last Station":
            rospy.logdebug(f"Loading path from {first_station} to {last_station}")
            self.load_and_send_path(first_station, last_station)
            target_coords = self.station_coordinates[last_station]
            rospy.logdebug(f"Sending {self.agv_name} to last station: {last_station} with coordinates {target_coords}")
            self.send_goal(target_coords)
            self.wait_for_arrival(target_coords)

            self.request_parking_pub.publish(self.agv_name)
            self.agv_status[int(self.agv_name[-2:]) - 1] = "Available"
            self.publish_agv_status()

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
        while not rospy.is_shutdown():
            current_coords = self.agv_pose
            distance = math.sqrt((current_coords[0] - target_coords[0]) ** 2 +
                                 (current_coords[1] - target_coords[1]) ** 2)
            if distance < 0.1:
                rospy.loginfo(f"{self.agv_name} arrived at target.")
                break
            rospy.sleep(1)

    def load_and_send_path(self, first_station, last_station):
        path_file = f"{first_station}to{last_station}.yaml"
        rospy.logdebug(f"Loading path from file {path_file}")

        # Ensure file exists before trying to load it
        try:
            with open(path_file, 'r') as file:
                path_data = yaml.safe_load(file)
        except FileNotFoundError:
            rospy.logwarn(f"Path file {path_file} not found.")
            return
        except yaml.YAMLError as e:
            rospy.logwarn(f"Error loading YAML file {path_file}: {e}")
            return

        path_topic = f"/{self.agv_name.lower()}/move_base/TrajectoryPlannerROS/global_plan"
        pub = rospy.Publisher(path_topic, PoseStamped, queue_size=10)

        for point in path_data['path']:
            path_msg = PoseStamped()
            path_msg.pose.position.x = point['x']
            path_msg.pose.position.y = point['y']
            path_msg.pose.position.z = point['z']
            path_msg.pose.orientation.x = point['qx']
            path_msg.pose.orientation.y = point['qy']
            path_msg.pose.orientation.z = point['qz']
            path_msg.pose.orientation.w = point['qw']
            pub.publish(path_msg)
            rospy.sleep(0.1)

    def closest_parking_callback(self, msg):
        rospy.loginfo(f"Received closest parking info: {msg.data}")
        agv_name, closest_park = msg.data.split(', ')
        if agv_name != self.agv_name:
            return
        
        target_coords = self.station_coordinates.get(closest_park)

        if target_coords is None:
            rospy.logwarn(f"Invalid parking location: {closest_park}")
            return

        rospy.logdebug(f"Sending {self.agv_name} to closest parking: {closest_park} with coordinates {target_coords}")
        self.send_goal(target_coords)
        self.agv_status[int(self.agv_name[-2:]) - 1] = "Available"
        self.publish_agv_status()
    
    def publish_agv_status(self):
        agv_status_info = ', '.join(self.agv_status)
        self.agv_status_pub.publish(agv_status_info)
        rospy.loginfo(f"Publish AGV status: {agv_status_info}")

if __name__ == '__main__':
    try:
        AGV03_management = AGV03_management()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
