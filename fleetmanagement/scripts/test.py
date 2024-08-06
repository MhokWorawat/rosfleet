import tkinter as tk
from PIL import Image, ImageTk
import os
import rospy
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseWithCovarianceStamped
import numpy as np

class MiniMapUI:
    def __init__(self, parent, fleetGUI):
        self.parent = parent
        self.map_canvas = tk.Canvas(parent, width=880, height=700, highlightthickness=0)
        self.map_canvas.place(x=431, y=150)

        self.minimap_canvas = tk.Canvas(parent, width=880, height=700, highlightthickness=0)
        self.minimap_canvas.place(x=431, y=150)

        rospy.Subscriber('/map', OccupancyGrid, self.map_callback)
        rospy.Subscriber('/agv01/amcl_pose', PoseWithCovarianceStamped, self.pose_callback_agv01)
        rospy.Subscriber('/agv02/amcl_pose', PoseWithCovarianceStamped, self.pose_callback_agv02)
        rospy.Subscriber('/agv03/amcl_pose', PoseWithCovarianceStamped, self.pose_callback_agv03)
        rospy.Subscriber('/agv04/amcl_pose', PoseWithCovarianceStamped, self.pose_callback_agv04)

        self.fleetGUI = fleetGUI
        self.map_data = None
        self.robot_poses = {'AGV01': None, 'AGV02': None, 'AGV03': None, 'AGV04': None}
        self.latest_msgs = {'AGV01': None, 'AGV02': None, 'AGV03': None, 'AGV04': None}

        self.new_width = 0
        self.new_height = 0
        self.x_center = 0
        self.y_center = 0
        self.update_interval = 500

        self.load_map_image()
        self.load_agv_images()
        self.parent.after(self.update_interval, self.update_poses)

    def load_map_image(self):
        path_file = os.path.dirname(__file__)
        map_image_path = os.path.join(path_file, "../image/minimap.png")
        self.minimap_image = ImageTk.PhotoImage(Image.open(map_image_path))
        self.minimap_canvas.create_image(0, 0, anchor=tk.NW, image=self.map_image, tags="map_image")
        self.minimap_canvas.tag_lower("map_image")

    def load_agv_images(self):
        path_file = os.path.dirname(__file__)
        self.agv_images = {
            'AGV01': ImageTk.PhotoImage(Image.open(os.path.join(path_file, "../image/agv01.png"))),
            'AGV02': ImageTk.PhotoImage(Image.open(os.path.join(path_file, "../image/agv02.png"))),
            'AGV03': ImageTk.PhotoImage(Image.open(os.path.join(path_file, "../image/agv03.png"))),
            'AGV04': ImageTk.PhotoImage(Image.open(os.path.join(path_file, "../image/agv04.png")))
        }

    def map_callback(self, data):
        self.map_data = data
        self.draw_map()
        self.draw_robots()

    def pose_callback_agv01(self, data):
        self.latest_msgs['AGV01'] = data

    def pose_callback_agv02(self, data):
        self.latest_msgs['AGV02'] = data

    def pose_callback_agv03(self, data):
        self.latest_msgs['AGV03'] = data

    def pose_callback_agv04(self, data):
        self.latest_msgs['AGV04'] = data

    def update_poses(self):
        if self.map_data:
            if self.latest_msgs['AGV01']:
                self.robot_poses['AGV01'] = self.latest_msgs['AGV01'].pose.pose
            if self.latest_msgs['AGV02']:
                self.robot_poses['AGV02'] = self.latest_msgs['AGV02'].pose.pose
            if self.latest_msgs['AGV03']:
                self.robot_poses['AGV03'] = self.latest_msgs['AGV03'].pose.pose
            if self.latest_msgs['AGV04']:
                self.robot_poses['AGV04'] = self.latest_msgs['AGV04'].pose.pose
            self.draw_robots()

        self.parent.after(self.update_interval, self.update_poses)

    def draw_map(self):
        if self.map_data:
            self.map_canvas.delete("map_image")
            width = self.map_data.info.width
            height = self.map_data.info.height
            map_image = np.array(self.map_data.data).reshape((height, width))

            map_image = np.flip(map_image, axis=0)

            map_image_colored = np.zeros((height, width, 3), dtype=np.uint8)
            map_image_colored[map_image == 0] = [255, 255, 255]
            map_image_colored[map_image == 100] = [0, 0, 0]
            map_image_colored[map_image == -1] = [207, 207, 207]

            map_image_pil = Image.fromarray(map_image_colored)

            aspect_ratio = min(880 / width, 700 / height)
            self.new_width = int(width * aspect_ratio)
            self.new_height = int(height * aspect_ratio)

            map_image_pil = map_image_pil.resize((self.new_width, self.new_height), Image.LANCZOS)
            self.map_photo = ImageTk.PhotoImage(image=map_image_pil)

            self.x_center = (880 - self.new_width) // 2
            self.y_center = (700 - self.new_height) // 2

            self.map_canvas.create_image(self.x_center, self.y_center, anchor=tk.NW, image=self.map_photo, tags="map_image")
            self.map_canvas.image = self.map_photo

            # Keep minimap image at the bottom
            self.map_canvas.tag_lower("map_image")

    def draw_robots(self):
        if self.map_data:
            resolution = self.map_data.info.resolution
            origin_x = self.map_data.info.origin.position.x
            origin_y = self.map_data.info.origin.position.y

            aspect_ratio = min(880 / self.map_data.info.width, 700 / self.map_data.info.height)

            agv_status = self.fleetGUI.get_agv_status()

            for i, (robot_id, pose) in enumerate(self.robot_poses.items()):
                status = agv_status[i] if i < len(agv_status) else "Not connect"
                if status != "Available":
                    self.map_canvas.delete(robot_id)
                    continue

                if pose is None:
                    self.map_canvas.delete(robot_id)
                    continue

                x = (pose.position.x - origin_x) / resolution
                y = (pose.position.y - origin_y) / resolution
                robot_x = self.x_center + int(x * aspect_ratio)
                robot_y = self.y_center + int((self.map_data.info.height - y) * aspect_ratio)

                self.map_canvas.delete(robot_id)

                if robot_id in self.agv_images:
                    self.map_canvas.create_image(robot_x, robot_y, image=self.agv_images[robot_id], anchor=tk.CENTER, tags=robot_id)
                    self.map_canvas.tag_raise(robot_id)
                else:
                    rospy.logwarn(f"Pose for {robot_id} is None, skipping drawing for this robot.")

if __name__ == "__main__":
    root = tk.Tk()
    app = FleetManagementUI(root)
    root.mainloop()
