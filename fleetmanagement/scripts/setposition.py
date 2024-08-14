import rospy
import json
import numpy as np
import matplotlib.pyplot as plt
from geometry_msgs.msg import Pose
from nav_msgs.msg import Path
from tf.transformations import euler_from_quaternion

def save_processed_path_to_json(filename, path_data):
    file_path = f"/home/mark/rosfleet/src/fleetmanagement/data_paths/{filename}"
    
    with open(file_path, 'w') as json_file:
        json.dump(path_data, json_file, indent=4)
    
    rospy.loginfo(f"Processed path data saved to {file_path}")
    
    # Plot the path data after saving the JSON file
    plot_path_graph(path_data, filename)

def plot_path_graph(path_data, filename):
    x_coords = [pose['position']['x'] for pose in path_data]
    y_coords = [pose['position']['y'] for pose in path_data]

    plt.figure(figsize=(10, 6))
    title_name = filename.replace('.json', '')
    plt.plot(x_coords, y_coords, marker='o', linestyle='-', color='red')
    plt.title(f'Path from {title_name}')
    plt.xlabel('X Position')
    plt.ylabel('Y Position')
    plt.grid(True)
    
    graph_file_path = f"/home/mark/rosfleet/src/fleetmanagement/graph_paths/{filename.replace('.json', '.png')}"
    plt.savefig(graph_file_path)
    plt.close()
    
    rospy.loginfo(f"Path graph saved to {graph_file_path}")

def nav_path_callback(msg, start_station, target_station):
    processed_data = []
    
    for i in range(len(msg.poses) - 1):
        current_pose = msg.poses[i].pose
        next_pose = msg.poses[i + 1].pose
        
        processed_data.append({
            'position': {
                'x': float(current_pose.position.x),
                'y': float(current_pose.position.y),
                'z': float(current_pose.position.z)
            },
            'orientation': {
                'x': float(current_pose.orientation.x), 
                'y': float(current_pose.orientation.y), 
                'z': float(current_pose.orientation.z), 
                'w': float(current_pose.orientation.w)
            }
        })

    output_filename = f"{start_station}to{target_station}.json"
    save_processed_path_to_json(output_filename, processed_data)

def send_pose(start_station, target_station):
    start_pose_pub = rospy.Publisher('start_pose', Pose, queue_size=10)
    target_pose_pub = rospy.Publisher('target_pose', Pose, queue_size=10)
    
    station_coordinates = {
        'A': (2.900, -6.500, 0.000, 0.000, 0.000, 0.707, 0.707),
        'B': (7.950, -1.000, 0.000, 0.000, 0.000, 0.000, 1.000),
        'C': (14.950, -9.200, 0.000, 0.000, 0.000, 0.000, 1.000),
        'D': (6.050, -10.2500, 0.000, 0.000, 0.000, -0.707, 0.707),
        'E': (19.600, -10.300, 0.000, 0.000, 0.000, 0.000, 1.000),
        'G': (16.600, -20.850, 0.000, 0.000, 0.000, 1.000, 0.000),
        'P1': (1.050, -9.700, 0.000, 0.000, 0.000, 0.000, 1.000),
        'P2': (17.850, -7.600, 0.000, 0.000, 0.000, -0.707, 0.707),
        'P3': (1.050, -1.000, 0.000, 0.000, 0.000, 0.000, 1.000),
        'P4': (24.700, -9.750, 0.000, 0.000, 0.000, -1.000, 0.000)
    }

    if start_station in station_coordinates and target_station in station_coordinates:
        start_position = station_coordinates[start_station]
        target_position = station_coordinates[target_station]

        start_pose = Pose()
        start_pose.position.x, start_pose.position.y, start_pose.position.z = start_position[:3]
        start_pose.orientation.x, start_pose.orientation.y, start_pose.orientation.z, start_pose.orientation.w = start_position[3:]

        target_pose = Pose()
        target_pose.position.x, target_pose.position.y, target_pose.position.z = target_position[:3]
        target_pose.orientation.x, target_pose.orientation.y, target_pose.orientation.z, target_pose.orientation.w = target_position[3:]

        rospy.sleep(1)
        start_pose_pub.publish(start_pose)
        rospy.loginfo(f"Published start pose: {start_station}")
        target_pose_pub.publish(target_pose)
        rospy.loginfo(f"Published target pose: {target_station}")

        rospy.Subscriber('/nav_path', Path, lambda msg: nav_path_callback(msg, start_station, target_station))
        rospy.loginfo(f"Waiting for path data to process and save as {start_station}to{target_station}.json")
    else:
        rospy.logerr("Invalid station selected. Please select a valid station.")

if __name__ == '__main__':
    try:
        rospy.init_node('set_position', anonymous=True)
        
        start_station = input("Enter the start station (A, B, C, D, E, G, P1, P2, P3, P4): ").strip().upper()
        target_station = input("Enter the target station (A, B, C, D, E, G, P1, P2, P3, P4): ").strip().upper()
        send_pose(start_station, target_station)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
