#!/usr/bin/env python3

import rospy
import yaml
from std_msgs.msg import String
from nav_msgs.srv import GetPlan
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
import os

class FleetManagement:
    def __init__(self):
        rospy.init_node('fleet_management', anonymous=True)

        self.agv_status = ["Not connect", "Not connect", "Not connect", "Not connect"]

        self.agv_poses = {
            'AGV01': (0, 0),
            'AGV02': (0, 0),
            'AGV03': (0, 0),
            'AGV04': (0, 0)
        }

        self.station_coordinates = {
            'A': (-2.793, -3.458),
            'B': (0.907, -3.544),
            'C': (0.198, -0.259),
            'D': (1.644, 1.393),
            'E': (-3.492, 1.483),
            'G': (0.163, 3.282),
            'P1': (1.0, -10.0),
            'P2': (17.75, -7.25),
            'P3': (1.0, -1.0),
            'P4': (25.0, -10.0)
        }

        # ROS Publishers
        self.agv_status_pub = rospy.Publisher('/agv_status', String, queue_size=10, latch=True)
        self.task_pub = rospy.Publisher('/task', String, queue_size=10)

        # ROS Subscribers
        rospy.Subscriber('/agv01/amcl_pose', PoseWithCovarianceStamped, self.agv01_pose_callback)
        rospy.Subscriber('/agv02/amcl_pose', PoseWithCovarianceStamped, self.agv02_pose_callback)
        rospy.Subscriber('/agv03/amcl_pose', PoseWithCovarianceStamped, self.agv03_pose_callback)
        rospy.Subscriber('/agv04/amcl_pose', PoseWithCovarianceStamped, self.agv04_pose_callback)
        rospy.Subscriber('/agv_status', String, self.ros_subscrib_agv_status)
        rospy.Subscriber('/mission', String, self.ros_subscrib_mission)

    def agv01_pose_callback(self, msg):
        self.agv_poses['AGV01'] = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    def agv02_pose_callback(self, msg):
        self.agv_poses['AGV02'] = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    def agv03_pose_callback(self, msg):
        self.agv_poses['AGV03'] = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    def agv04_pose_callback(self, msg):
        self.agv_poses['AGV04'] = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    def ros_subscrib_agv_status(self, msg):
        rospy.loginfo(f"Received AGV status: {msg.data}")
        status_list = msg.data.split(', ')
        if len(status_list) == 4:
            self.agv_status = status_list

    def ros_subscrib_mission(self, msg):
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
            if selected_agv == "Automatic":
                if len(available_agvs) == 1:
                    closest_agv = available_agvs[0]
                    rospy.loginfo(f"Only one AGV available: {closest_agv}")
                else:
                    if station_first in self.station_coordinates:
                        station_first_coords = self.station_coordinates[station_first]
                    else:
                        rospy.loginfo(f"Station {station_first} coordinates not found.")
                        return

                    # Calculate the distance for each AGV
                    min_distance = float('inf')
                    closest_agv = None

                    for agv in available_agvs:
                        agv_position = self.agv_poses[agv]
                        distance = self.calculate_path_distance(agv_position, station_first_coords)
                        if distance < min_distance:
                            min_distance = distance
                            closest_agv = agv

                    if closest_agv is not None:
                        rospy.loginfo(f"Closest AGV: {closest_agv}")
                    else:
                        rospy.loginfo("No AGVs available for the task.")
                        return
            else:
                closest_agv = selected_agv
                rospy.loginfo(f"Selected AGV: {closest_agv}")

            # Send the task
            task_info = f"{closest_agv}, {station_first}, {station_last}"
            self.task_pub.publish(task_info)
            rospy.loginfo(f"Task assigned to {closest_agv}: {station_first} -> {station_last}")

            # Update AGV status to "Executing mission"
            self.agv_status[int(closest_agv[-2:]) - 1] = "Executing mission"
            self.publish_agv_status()
        else:
            rospy.loginfo("No AGVs available for the task.")

    def calculate_path_distance(self, start, goal):
        rospy.wait_for_service('/move_base/make_plan')
        try:
            get_plan = rospy.ServiceProxy('/move_base/make_plan', GetPlan)
            
            start_pose = PoseStamped()
            start_pose.header.frame_id = "map"
            start_pose.pose.position.x = start[0]
            start_pose.pose.position.y = start[1]
            start_pose.pose.orientation.w = 1.0
            
            goal_pose = PoseStamped()
            goal_pose.header.frame_id = "map"
            goal_pose.pose.position.x = goal[0]
            goal_pose.pose.position.y = goal[1]
            goal_pose.pose.orientation.w = 1.0
            
            tolerance = 0.05
            
            plan = get_plan(start_pose, goal_pose, tolerance)
            
            distance = sum(self.euclidean_distance(pose.pose.position, plan.plan.poses[i + 1].pose.position)
                           for i, pose in enumerate(plan.plan.poses[:-1]))
            
            return distance
        except rospy.ServiceException as e:
            rospy.loginfo(f"Service call failed: {e}")
            return float('inf')

    def euclidean_distance(self, pos1, pos2):
        return ((pos1.x - pos2.x) ** 2 + (pos1.y - pos2.y) ** 2) ** 0.5

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
        path_file = os.path.dirname(__file__)
        pathtofile_data_path = os.path.join(path_file, 'fleetmanagement', 'data_path', data_path_file_name)

        map_data = self.load_yaml_file(pathtofile_data_path)
        if map_data:
            self.ros_publish_path_to_navigation(agv, map_data)
        else:
            rospy.logwarn(f"Failed to load map data for task: {msg.data}")

    def ros_publish_path_to_navigation(self, agv, map_data):
        agv_padded = f"agv{int(agv.replace('AGV', '')):02d}"
        nav_topic = f"/{agv_padded}/move_base/TrajectoryPlannerROS/global_plan"
        rospy.loginfo(f"Sending map data to {nav_topic}")

        path_msg = Path()
        path_msg.header.frame_id = "map"
        
        path_msg.poses = []
        for pose_dict in map_data.get('poses', []):
            pose_stamped = PoseStamped()
            pose_stamped.header = pose_dict['header']
            pose_stamped.pose.position.x = pose_dict['pose']['position']['x']
            pose_stamped.pose.position.y = pose_dict['pose']['position']['y']
            pose_stamped.pose.position.z = pose_dict['pose']['position']['z']
            pose_stamped.pose.orientation.x = pose_dict['pose']['orientation']['x']
            pose_stamped.pose.orientation.y = pose_dict['pose']['orientation']['y']
            pose_stamped.pose.orientation.z = pose_dict['pose']['orientation']['z']
            pose_stamped.pose.orientation.w = pose_dict['pose']['orientation']['w']
            path_msg.poses.append(pose_stamped)
        
        pub = rospy.Publisher(nav_topic, Path, queue_size=10)
        pub.publish(path_msg)

    def publish_agv_status(self):
        agv_status_info = ', '.join(self.agv_status)
        rospy.loginfo(f"Publish  AGV status: {agv_status_info}")
        self.agv_status_pub.publish(agv_status_info)

if __name__ == '__main__':
    try:
        fleet_management = FleetManagement()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
