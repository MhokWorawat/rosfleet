#!/usr/bin/env python3

import rospy
import yaml
from std_msgs.msg import String
from geometry_msgs.msg import PoseWithCovarianceStamped
import os

class FleetManagement:
    def __init__(self):
        rospy.init_node('fleet_management', anonymous=True)

        self.distance_AGV01 = 10
        self.distance_AGV02 = 20
        self.distance_AGV03 = 30
        self.distance_AGV04 = 40

        self.agv_status = ["Not connect", "Not connect", "Not connect", "Not connect"]

        # ROS Publishers
        self.agv_status_pub = rospy.Publisher('/agv_status', String, queue_size=4, latch=True)
        self.task_pub = rospy.Publisher('/task', String, queue_size=1)
        self.agv01Emergen_pub = rospy.Publisher('/agv01/Emergen', String, queue_size=1, latch=True)
        self.agv02Emergen_pub = rospy.Publisher('/agv02/Emergen', String, queue_size=1, latch=True)
        self.agv03Emergen_pub = rospy.Publisher('/agv03/Emergen', String, queue_size=1, latch=True)
        self.agv04Emergen_pub = rospy.Publisher('/agv04/Emergen', String, queue_size=1, latch=True)

        # ROS Subscribers
        rospy.Subscriber('/agv_status', String, self.ros_subscrib_agv_status)
        rospy.Subscriber('/mission', String, self.ros_subscrib_mission)
        rospy.Subscriber('/task', String, self.ros_subscrib_task)

        # Set start positions
        self.set_start_positions()

        self.agv01Emergen_pub.publish("on")
        self.agv02Emergen_pub.publish("off")
        self.agv03Emergen_pub.publish("off")
        self.agv04Emergen_pub.publish("off")

    def set_start_positions(self):
        position_atstart = {
            'AGV01': (-3.530, -2.181, 0.000, 0.000, 0.000, 0.707, 0.707),
            'AGV02': (1.603, -2.326, 0.000, 0.000, 0.000, 0.704, 0.710),
            'AGV03': (-2.646, 2.552, 0.000, 0.000, 0.000, 0.003, 1.000),
            'AGV04': (1.564, 3.617, 0.000, 0.000, 0.000, -0.705, 0.709),
        }

        for agv_name, position in position_atstart.items():
            pub = rospy.Publisher(f'/{agv_name}/initialpose', PoseWithCovarianceStamped, queue_size=1)
            rospy.sleep(0.1)

            pose = PoseWithCovarianceStamped()
            pose.header.frame_id = "map"
            pose.pose.pose.position.x = position[0]
            pose.pose.pose.position.y = position[1]
            pose.pose.pose.position.z = position[2]
            pose.pose.pose.orientation.x = position[3]
            pose.pose.pose.orientation.y = position[4]
            pose.pose.pose.orientation.z = position[5]
            pose.pose.pose.orientation.w = position[6]

            rospy.loginfo(f"Setting initial pose for {agv_name} to {position}")
            pub.publish(pose)

    def select_agv_for_task(self, station_first, station_last):
        available_agvs = []
        for i, status in enumerate(self.agv_status):
            if status == "Available":
                available_agvs.append(i+1)

        rospy.loginfo(f"Available AGVs: {available_agvs}")

        if available_agvs:
            # Find the closest AGV
            closest_agv = min(available_agvs, key=lambda agv: getattr(self, f'distance_AGV{agv:02}'))
            rospy.loginfo(f"Closest AGV: AGV{closest_agv}")

            # Send the task
            task_info = f"AGV{closest_agv:02}, {station_first}, {station_last}"
            self.task_pub.publish(task_info)
            rospy.loginfo(f"Task assigned to AGV{closest_agv:02}: {station_first} -> {station_last}")

            # Update AGV status to "Executing mission"
            self.agv_status[closest_agv - 1] = "Executing mission"
            self.publish_agv_status()
        else:
            rospy.loginfo("No AGVs available for the task.")

    def ros_subscrib_agv_status(self, msg):
        rospy.loginfo(f"Received AGV status: {msg.data}")
        status_list = msg.data.split(', ')
        if len(status_list) == 4:
            self.agv_status = status_list

    def ros_subscrib_mission(self, msg):
        rospy.loginfo(f"Received mission: {msg.data}")
        mission = msg.data.split(', ')
        if len(mission) != 2:
            mission = msg.data.split(',')

        if len(mission) == 2:
            station_first = mission[0].strip()
            station_last = mission[1].strip()
            self.select_agv_for_task( station_first, station_last)
        else:
            rospy.loginfo("Invalid mission format received.")

    def load_yaml_file(self, data_path):
        try:
            with open(data_path, 'r') as file:
                data = yaml.safe_load(file)
            return data
        except FileNotFoundError:
            rospy.logwarn(f"YAML file not found: {data_path}")
            return None

    def ros_subscrib_task(self, msg):
        rospy.loginfo(f"Received task: {msg.data}")
        task_data = msg.data.strip("()").split(", ")
        agv = task_data[0]
        start_station = task_data[1]
        end_station = task_data[2]

        data_path_file_name = f"{start_station}to{end_station}.yaml"
        pathtofile_data_path = f"/home/mark/rosfleet/src/fleetmanagement/data_path/{data_path_file_name}"

        map_data = self.load_yaml_file(pathtofile_data_path)
        if map_data:
            self.ros_publish_to_navigation(agv, map_data)
        else:
            rospy.logwarn(f"Failed to load map data for task: {msg.data}")

    def ros_publish_to_navigation(self, agv, map_data):
        agv_padded = agv.replace("AGV", "AGV0") if len(agv) == 4 else agv
        nav_topic = f"/{agv_padded}/navigation"
        rospy.loginfo(f"Sending map data to {nav_topic}")
        # nav_goal_msg = NavGoal()
        # nav_goal_msg.map_data = map_data
        # pub = rospy.Publisher(nav_topic, NavGoal, queue_size=10)
        # pub.publish(nav_goal_msg)

    def publish_agv_status(self):
        agv_status_info = ', '.join(self.agv_status)
        rospy.loginfo(f"Publishing AGV status: {agv_status_info}")
        self.agv_status_pub.publish(agv_status_info)

if __name__ == '__main__':
    try:
        fleet_management = FleetManagement()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
