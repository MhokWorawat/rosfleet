#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import pandas as pd
import os

class TimeTracker:
    def __init__(self):
        rospy.init_node('time_tracker', anonymous=True)

        self.mission_start_time = None
        self.task_start_time = None
        self.agv_start_time = None
        self.task_msg = None
        self.time_data = []

        self.agv_velocity = {
            'x': 0.0,
            'y': 0.0,
            'th': 0.0
        }

        self.mission_sub = rospy.Subscriber('/mission', String, self.ros_subscribe_mission, queue_size=100)
        self.task_sub = rospy.Subscriber('/task', String, self.ros_subscribe_task, queue_size=100)
        self.agv01_velocity_sub = rospy.Subscriber('/agv01/Velocity', Twist, self.ros_subscribe_agv_velocity, queue_size=100)

        rospy.on_shutdown(self.shutdown_hook)

        rospy.loginfo("TimeTracker node initialized")

    def ros_subscribe_mission(self, msg):
        self.mission_start_time = rospy.get_rostime()
        rospy.loginfo(f"Mission message received at {self.mission_start_time.to_sec()} seconds")

    def ros_subscribe_task(self, msg):
        if self.mission_start_time:
            self.task_start_time = rospy.get_rostime()
            self.task_msg = msg.data
            rospy.loginfo(f"Task message received at {self.task_start_time.to_sec()} seconds")
    
    def ros_subscribe_agv_velocity(self, msg):
        self.agv_velocity['x'] = msg.linear.x
        self.agv_velocity['y'] = msg.linear.y
        self.agv_velocity['th'] = msg.angular.z

        if self.agv_velocity['x'] != 0 or self.agv_velocity['y'] != 0 or self.agv_velocity['th'] != 0:
            self.agv_start_time = rospy.get_rostime()
            self.calculate_time_difference()

    def calculate_time_difference(self):
        if self.mission_start_time is not None and self.task_start_time is not None:
            time_difference_mission_to_task = (self.task_start_time - self.mission_start_time).to_sec()
            rospy.loginfo(f"Time difference Mission to Task : {time_difference_mission_to_task} seconds")

        if self.task_start_time is not None and self.agv_start_time is not None:
            time_difference_task_to_agv = (self.agv_start_time - self.task_start_time).to_sec()
            rospy.loginfo(f"Time difference Task to AGV     : {time_difference_task_to_agv} seconds")

        if self.mission_start_time is not None and self.task_start_time is not None and self.agv_start_time is not None:
            self.time_data.append({
                'Mission': self.task_msg,
                'Processing Time (s)': time_difference_mission_to_task,
                'Responding Time (s)': time_difference_task_to_agv
            })

            self.mission_start_time = None
            self.task_start_time = None
            self.agv_start_time = None
            self.task_msg = None
            

    def save_to_csv(self):
        df = pd.DataFrame(self.time_data)
        file_path = '/home/mark/Desktop/Expiriment3/timedata.csv'
        file_exists = os.path.isfile(file_path)

        if file_exists:
            df.to_csv(file_path, mode='a', header=False, index=False)
        else:
            df.to_csv(file_path, mode='w', header=True, index=False)
        
        rospy.loginfo("Data saved to CSV file.")

    def shutdown_hook(self):
        self.save_to_csv()
        rospy.loginfo("Node shutting down. Data saved.")

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        time_tracker = TimeTracker()
        time_tracker.run()
    except rospy.ROSInterruptException:
        pass
