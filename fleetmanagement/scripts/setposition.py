import rospy
from geometry_msgs.msg import Pose

def send_pose(start_station, target_station):
    rospy.init_node('set_position', anonymous=True)
    
    start_pose_pub = rospy.Publisher('start_pose', Pose, queue_size=10)
    target_pose_pub = rospy.Publisher('target_pose', Pose, queue_size=10)
    
    start_pose = Pose()
    target_pose = Pose()
    
    stations = {
        'A': (-2.793, -3.458, 0.000, 0.000, 0.000, 0.000, 1.000),
        'B': (0.907, -3.544, 0.000, 0.000, 0.000, 0.000, 1.000),
        'C': (0.198, -0.259, 0.000, -0.000, 0.000, 1.000, -0.000),
        'D': (1.644, 1.393, 0.000, 0.000, 0.000, 0.705, 0.709),
        'E': (-3.492, 1.483, 0.000, 0.000, 0.000, 0.705, 0.709),
        'G': (0.163, 3.282, 0.000, 0.000, 0.000, -1.000, 0.004),
        'P1': (-3.530, -2.181, 0.000, 0.000, 0.000, 0.707, 0.707),
        'P2': (1.603, -2.326, 0.000, 0.000, 0.000, 0.704, 0.710),
        'P3': (-2.646, 2.552, 0.000, 0.000, 0.000, 0.003, 1.000),
        'P4': (1.564, 3.617, 0.000, 0.000, 0.000, -0.705, 0.709),
        'P5': (1.564, 3.617, 0.000, 0.000, 0.000, -0.705, 0.709)
    }

    if start_station in stations and target_station in stations:
        start_position = stations[start_station]
        target_position = stations[target_station]

        # Set start pose
        start_pose.position.x, start_pose.position.y, start_pose.position.z = start_position[:3]
        start_pose.orientation.x, start_pose.orientation.y, start_pose.orientation.z, start_pose.orientation.w = start_position[3:]

        # Set target pose
        target_pose.position.x, target_pose.position.y, target_pose.position.z = target_position[:3]
        target_pose.orientation.x, target_pose.orientation.y, target_pose.orientation.z, target_pose.orientation.w = target_position[3:]
        
        rospy.sleep(1)
        start_pose_pub.publish(start_pose)
        rospy.loginfo("Published start pose: (%f, %f)", start_pose.position.x, start_pose.position.y)
        target_pose_pub.publish(target_pose)
        rospy.loginfo("Published target pose: (%f, %f)", target_pose.position.x, target_pose.position.y)
    else:
        rospy.logerr("Invalid station selected. Please select valid stations.")

if __name__ == '__main__':
    try:
        start_station = input("Enter the start station (A, B, C, D, E, G, P1, P2, P3, P4, P5): ").strip().upper()
        target_station = input("Enter the target station (A, B, C, D, E, G, P1, P2, P3, P4, P5): ").strip().upper()
        send_pose(start_station, target_station)
    except rospy.ROSInterruptException:
        pass
