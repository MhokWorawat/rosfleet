import tkinter as tk
import rospy
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseWithCovarianceStamped
import numpy as np
from PIL import Image, ImageTk
import os

class MiniMapUI:
    def __init__(self, parent):
        self.map_canvas = tk.Canvas(parent, width=880, height=700, highlightthickness=0)
        self.map_canvas.place(x=431, y=150)

        rospy.Subscriber('/map', OccupancyGrid, self.map_callback)
        rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.pose_callback)

        self.map_data = None
        self.robot_pose = None

        self.new_width = 0
        self.new_height = 0
        self.x_center = 0
        self.y_center = 0

    def map_callback(self, data):
        self.map_data = data
        self.draw_map()

    def pose_callback(self, data):
        self.robot_pose = data.pose.pose
        if self.map_data:
            self.draw_robot()

    def draw_map(self):
        if self.map_data:
            # Remove only the existing map image and draw it again
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

        # Draw the robot after the map
        self.draw_robot()

    def draw_robot(self):
        if self.robot_pose and self.map_data:
            resolution = self.map_data.info.resolution
            origin_x = self.map_data.info.origin.position.x
            origin_y = self.map_data.info.origin.position.y

            x = (self.robot_pose.position.x - origin_x) / resolution
            y = (self.robot_pose.position.y - origin_y) / resolution

            print(f"Robot position in pixels: x={x}, y={y}")

            aspect_ratio = min(880 / self.map_data.info.width, 700 / self.map_data.info.height)
            robot_x = self.x_center + int(x * aspect_ratio)
            robot_y = self.y_center + int((self.map_data.info.height - y) * aspect_ratio)

            print(f"Robot position on canvas: x={robot_x}, y={robot_y}")

            # Delete the existing robot drawing
            self.map_canvas.delete("robot")

            # Draw the robot's position
            self.map_canvas.create_oval(robot_x - 10, robot_y - 10, robot_x + 10, robot_y + 10, fill="red", tags="robot")
            self.map_canvas.create_text(robot_x, robot_y, text="1", fill="black", font=('Arial', 12, 'bold'), tags="robot")
            self.map_canvas.tag_raise("robot")  # Ensure robot is on top layer

if __name__ == "__main__":
    rospy.init_node('minimap_ui', anonymous=True)
    root = tk.Tk()
    root.geometry("1280x960")
    app = MiniMapUI(root)
    root.mainloop()
