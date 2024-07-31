#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseWithCovarianceStamped
from detection_msgs.msg import BoundingBoxes
from tkinter import font as tkFont
from tkinter import ttk
from tkinter import TclError
from PIL import Image, ImageTk
from functools import partial
import tkinter as tk
import numpy as np
import subprocess
import threading
import signal
import math
import time
import os

class FleetManagementUI:
    def __init__(self, root):
        rospy.init_node('gui_fleet_management', anonymous=True)
        self.process = subprocess.Popen(["roslaunch", "navigation", "map.launch"])

        self.root = root
        self.root.title("Fleet Management")
        self.root.geometry("1846x1042")

        self.root.protocol("WM_DELETE_WINDOW", self.shutdown_ui)

        # Setup environment UI
        self.load_environment()
        self.setup_background()
        root.iconphoto(False, self.Icon)

        # Add MiniMapUI
        self.minimap = MiniMapUI(self.root)

        # Initialize data
        self.buttons = []
        self.labels = []
        self.current_images = [0, 0, 0, 0]
        self.processes = [None, None, None, None]
        self.battery_of_agv = [100, 100, 100, 100]
        self.dropdown_vars = []
        self.recent_missions = []
        self.station = ["A", "B", "C", "D", "E", "G"]
        self.agv_status = ["Not connect", "Not connect", "Not connect", "Not connect"]
        self.tasks = {
            'AGV01': {'first': ' ', 'last': ' '},
            'AGV02': {'first': ' ', 'last': ' '},
            'AGV03': {'first': ' ', 'last': ' '},
            'AGV04': {'first': ' ', 'last': ' '}
        }
        self.emergency_status = {
            'AGV01': False,
            'AGV02': False,
            'AGV03': False,
            'AGV04': False
        }

        # ros_Publisher
        self.agv_status_pub = rospy.Publisher('/agv_status', String, queue_size=4, latch=True)
        self.mission_pub = rospy.Publisher('/mission', String, queue_size=6, latch=True)
        self.ros_publish_agv_status(self.agv_status)

        # ros_Subscriber
        self.task_subscriber = None
        rospy.Subscriber('/task', String, self.ros_subscribe_task)
        self.emergency_subscribers = {
            'AGV01': rospy.Subscriber('/agv01/Emergen', String, self.ros_subscribe_emergency, 'AGV01'),
            'AGV02': rospy.Subscriber('/agv02/Emergen', String, self.ros_subscribe_emergency, 'AGV02'),
            'AGV03': rospy.Subscriber('/agv03/Emergen', String, self.ros_subscribe_emergency, 'AGV03'),
            'AGV04': rospy.Subscriber('/agv04/Emergen', String, self.ros_subscribe_emergency, 'AGV04')
        }
        self.detection_subscribers = {
            'AGV01': rospy.Subscriber('/agv01/yolov5/detections', BoundingBoxes, self.ros_subscribe_detection, 'AGV01'),
            'AGV02': rospy.Subscriber('/agv02/yolov5/detections', BoundingBoxes, self.ros_subscribe_detection, 'AGV02'),
            'AGV03': rospy.Subscriber('/agv03/yolov5/detections', BoundingBoxes, self.ros_subscribe_detection, 'AGV03'),
            'AGV04': rospy.Subscriber('/agv04/yolov5/detections', BoundingBoxes, self.ros_subscribe_detection, 'AGV04')
        }

        # Create widget
        self.create_launch_buttons()
        self.create_dropdown()
        self.create_confirm_button()

        # Update UI with initial data
        self.update_dropdowns()
        self.update_tasks()
        self.update_battery()
        self.periodic_battery_update()
        self.update_emergency_status()

        self.detection_images = []
        self.detection_x = 436
        self.detection_y = 870
    
    def shutdown_ui(self):
        try:
            # Kill the launched process
            if self.process:
                self.process.terminate()
                self.process.wait()
            nodes = subprocess.check_output(['rosnode', 'list']).decode().split()

            # Kill all nodes except /rosout
            for node in nodes:
                if node != '/rosout':
                    os.system(f'rosnode kill {node}')
                
            rospy.loginfo("UI and related nodes have been shut down.")
        except Exception as e:
            rospy.logerr(f"Error shutting down UI: {e}")
        finally:
            self.root.destroy()

    def load_environment(self):
        path_file = os.path.dirname(__file__)
        self.font_name = os.path.join(path_file, "../font/Prompt/Prompt-Regular.ttf")
        self.bg_image = Image.open(os.path.join(path_file, "../image/background.png"))
        self.Icon = ImageTk.PhotoImage(Image.open(os.path.join(path_file, "../image/Icon.png")))
        self.buttonSSHon = ImageTk.PhotoImage(Image.open(os.path.join(path_file, "../image/button_on.png")))
        self.buttonSSHoff = ImageTk.PhotoImage(Image.open(os.path.join(path_file, "../image/button_off.png")))
        self.buttonConfirm = ImageTk.PhotoImage(Image.open(os.path.join(path_file, "../image/button_confirm.png")))
        self.buttonConfirmOn = ImageTk.PhotoImage(Image.open(os.path.join(path_file, "../image/button_confirm_On.png")))
        self.imgEmergency = ImageTk.PhotoImage(Image.open(os.path.join(path_file, "../image/emergency.png")))
        self.imgObstacle_box = ImageTk.PhotoImage(Image.open(os.path.join(path_file, "../image/obstacle_box.png")))
        self.imgObstacle_human = ImageTk.PhotoImage(Image.open(os.path.join(path_file, "../image/obstacle_human.png")))
        self.imgBattery_0 = ImageTk.PhotoImage(Image.open(os.path.join(path_file, "../image/battery0%.png")))
        self.imgBattery_25 = ImageTk.PhotoImage(Image.open(os.path.join(path_file, "../image/battery25%.png")))
        self.imgBattery_50 = ImageTk.PhotoImage(Image.open(os.path.join(path_file, "../image/battery50%.png")))
        self.imgBattery_75 = ImageTk.PhotoImage(Image.open(os.path.join(path_file, "../image/battery75%.png")))
        self.imgBattery_100 = ImageTk.PhotoImage(Image.open(os.path.join(path_file, "../image/battery100%.png")))

    def setup_background(self):
        self.bg_image = self.bg_image.resize((1846, 1042), Image.LANCZOS)
        self.bg_photo = ImageTk.PhotoImage(self.bg_image)

        self.canvas = tk.Canvas(self.root, width=1846, height=1042)
        self.canvas.pack(fill="both", expand=True)
        self.canvas.create_image(0, 0, image=self.bg_photo, anchor="nw")

    def get_font(self, size, weight="normal"):
        return tkFont.Font(family=self.font_name, size=size, weight=weight)

    def draw_text(self, x, y, text, size, color, weight, tag):
        custom_font = self.get_font(size, weight)
        self.canvas.create_text(x, y, text=text, font=custom_font, fill=color, anchor="nw", tags=tag)

    def create_launch_buttons(self):
        self.create_launch_button(0, 60, 128)
        self.create_launch_button(1, 60, 342)
        self.create_launch_button(2, 60, 556)
        self.create_launch_button(3, 60, 769)

    def create_launch_button(self, index, x, y):
        button = tk.Label(self.root, image=self.buttonSSHoff, borderwidth=0, highlightthickness=0, bg="#D9D9D9")
        button.place(x=x, y=y)
        button.bind("<Button-1>", partial(self.launch_button, index))
        self.buttons.append(button)

    def launch_button(self, AGV_index, event):
        if self.current_images[AGV_index] == 0:
            threading.Thread(target=self.start_launch, args=(AGV_index,)).start()
        else:
            threading.Thread(target=self.stop_launch, args=(AGV_index,)).start()

    def start_launch(self, AGV_index):
        commands = [
            "roslaunch navigation agv01_navigation.launch",
            "roslaunch navigation agv02_navigation.launch",
            "roslaunch navigation agv03_navigation.launch",
            "roslaunch navigation agv04_navigation.launch"
        ]
        try:
            process = subprocess.Popen(commands[AGV_index], shell=True, preexec_fn=os.setsid)
            # Give it a moment to start
            time.sleep(5)
            
            # Check if the process is running
            if process.poll() is None:
                self.processes[AGV_index] = process
                self.current_images[AGV_index] = 1
                self.buttons[AGV_index].config(image=self.buttonSSHon)
                self.agv_status[AGV_index] = "Available"
                self.ros_publish_agv_status(self.agv_status)
            else:
                # Process failed to start, clean up
                self.show_custom_message("Connection Error", f"Failed to connect to AGV{AGV_index + 1}")
                process.terminate()
                self.processes[AGV_index] = None
                self.current_images[AGV_index] = 0
                self.buttons[AGV_index].config(image=self.buttonSSHoff)
                self.agv_status[AGV_index] = "Not connect"
                self.ros_publish_agv_status(self.agv_status)

        except Exception as e:
            print(f"Exception occurred while launching AGV{AGV_index + 1}: {e}")

            self.show_custom_message("Connection Error", f"Failed to connect to AGV{AGV_index + 1}")

            self.processes[AGV_index] = None
            self.current_images[AGV_index] = 0
            self.buttons[AGV_index].config(image=self.buttonSSHoff)
            self.agv_status[AGV_index] = "Not connect"
            self.ros_publish_agv_status(self.agv_status)

    def stop_launch(self, AGV_index):
        process = self.processes[AGV_index]
        if process:
            os.killpg(os.getpgid(process.pid), signal.SIGTERM)
            time.sleep(2)
            if process.poll() is None:
                os.killpg(os.getpgid(process.pid), signal.SIGKILL)
                time.sleep(1)
                if process.poll() is None:
                    print(f"Process for AGV{AGV_index + 1} may still be running.")
            else:
                print(f"Process for AGV{AGV_index + 1} terminated with SIGTERM.")

            self.processes[AGV_index] = None

            # Ensure all related ROS nodes are killed
            node_names = subprocess.check_output(['rosnode', 'list']).decode('utf-8').split('\n')
            for node in node_names:
                if f"agv{AGV_index + 1}" in node:
                    try:
                        subprocess.check_call(['rosnode', 'kill', node])
                    except subprocess.CalledProcessError:
                        pass

            # Kill any remaining processes related to the AGV
            try:
                subprocess.check_call(f"pkill -f agv{AGV_index + 1}", shell=True)
            except subprocess.CalledProcessError:
                pass

            # Run rosnode cleanup to remove stale nodes
            subprocess.check_call('echo y | rosnode cleanup', shell=True)
            time.sleep(1)

            # Verify rosout is still running
            node_names = subprocess.check_output(['rosnode', 'list']).decode('utf-8').split('\n')
            if '/rosout' not in node_names:
                print("Warning: rosout is not running.")

        self.current_images[AGV_index] = 0
        self.buttons[AGV_index].config(image=self.buttonSSHoff)
        self.agv_status[AGV_index] = "Not connect"
        self.ros_publish_agv_status(self.agv_status)

    def create_dropdown(self):
        self.setup_dropdown(1558, 266)
        self.setup_dropdown(1558, 341)

    def setup_dropdown(self, x, y):
        style = self.dropdown_style()
        custom_font = self.get_font(14)

        variable = tk.StringVar(self.root)
        variable.set("Select")
        variable.trace("w", self.update_dropdowns)

        combobox = ttk.Combobox(self.root, textvariable=variable, font=custom_font, style='Custom.TCombobox', state='readonly', justify='center')
        combobox['values'] = self.station
        combobox.set("Select")
        combobox.place(x=x, y=y, width=205, height=37)
        combobox.option_add('*TCombobox*Listbox.font', custom_font)
        combobox.option_add('*TCombobox*Listbox.justify', 'center')

        self.dropdown_vars.append(variable)
        self.labels.append((variable, combobox))

    def dropdown_style(self):
        style = ttk.Style()
        # style.theme_use("clam")
        style.configure('Custom.TCombobox',
                        background='white',
                        fieldbackground='white',
                        foreground='black',
                        selectbackground='white',
                        selectforeground='black',
                        relief='flat',
                        arrowsize=20,
                        borderwidth=0,
                        focusthickness=0,
                        highlightthickness=0)
        style.map('Custom.TCombobox',
                background=[('active', 'white')],
                fieldbackground=[('readonly', 'white')],
                selectbackground=[('readonly', 'white')],
                selectforeground=[('readonly', 'black')],
                highlightcolor=[('focus', 'white')],
                highlightbackground=[('focus', 'white')],
                highlightthickness=[('focus', 0)],
                bordercolor=[('focus', 'white')],
                relief=[('focus', 'flat')])

        style.layout('Custom.TCombobox', [
            ('Combobox.field', {'expand': '1', 'sticky': 'nswe', 'children': [
            ('Combobox.padding', {'expand': '1', 'sticky': 'nswe', 'children': [
            ('Combobox.textarea', {'sticky': 'nswe'}),
            ('Combobox.downarrow', {'side': 'right', 'sticky': 'e'})
            ]})
            ]})
        ])
        return style

    def update_dropdowns(self, *args):
        selected_values = {var.get() for var in self.dropdown_vars if var.get() != "Select"}
        for var, combobox in self.labels:
            current_value = var.get()
            available_options = [option for option in self.station if option not in selected_values or option == current_value]
            combobox['values'] = available_options
            if current_value not in available_options:
                var.set("Select")

    def update_dropdown_state(self):
        available_AGVs = [agv for agv in range(4) if self.agv_status[agv] == "Available"]
        state = 'disabled' if not available_AGVs else 'readonly'

        for var, combobox in self.labels:
            combobox.config(state=state)
            if state == 'disabled':
                var.set('Select')

    def create_confirm_button(self):
        self.confirm_button = tk.Label(self.root, image=self.buttonConfirm, borderwidth=0, highlightthickness=0, bg="#ececec")
        self.confirm_button.place(x=1444, y=460)
        self.confirm_button.bind("<Enter>", lambda event: self.confirm_button.config(image=self.buttonConfirmOn))
        self.confirm_button.bind("<Leave>", lambda event: self.confirm_button.config(image=self.buttonConfirm))
        self.confirm_button.bind("<Button-1>", self.on_confirm_button_click)

    def on_confirm_button_click(self, event):
        first_station = self.dropdown_vars[0].get()
        last_station = self.dropdown_vars[1].get()

        available_AGVs = [
            (0, self.tasks['AGV01']['first'], self.tasks['AGV01']['last']),
            (1, self.tasks['AGV02']['first'], self.tasks['AGV02']['last']),
            (2, self.tasks['AGV03']['first'], self.tasks['AGV03']['last']),
            (3, self.tasks['AGV04']['first'], self.tasks['AGV04']['last'])
        ]
        available_AGVs = [agv for agv in available_AGVs if self.agv_status[agv[0]] == "Available"]

        if not available_AGVs:
            self.show_custom_message("Info", "No available AGVs. All AGVs have tasks assigned.")
            return
        else:
            
            if first_station == "Select" or last_station == "Select":
                self.show_custom_message("Error", "Please select both stations.")
                return
            else:
                # Publisher agv available
                mission_info = (first_station, last_station)
                self.ros_publish_mission(mission_info)

        def on_task_update(task_data):
            agv_id, task_first_station, task_last_station = task_data.data.split(", ")
            agv_index = int(agv_id[-1]) - 1

            # Update the AGV's tasks
            self.tasks[f'AGV0{agv_index + 1}']['first'] = task_first_station
            self.tasks[f'AGV0{agv_index + 1}']['last'] = task_last_station

            # Update recent missions
            self.recent_missions.insert(0, (task_first_station, task_last_station, f'AGV0{agv_index + 1}'))
            if len(self.recent_missions) > 4:
                self.recent_missions.pop()

            # Reset dropdowns to default "Select"
            for combobox in self.labels:
                combobox[1].set("Select")

            self.update_dropdowns()
            self.update_tasks()
            self.update_recent_missions(agv_index + 1)

        self.ros_subscribe_task(on_task_update)

    def update_tasks(self):
        coords = {
            'AGV01': {'first': (282, 235), 'last': (370, 235)},
            'AGV02': {'first': (282, 449), 'last': (370, 449)},
            'AGV03': {'first': (282, 663), 'last': (370, 663)},
            'AGV04': {'first': (282, 877), 'last': (370, 877)}
        }

        self.canvas.delete("tasks")

        for agv, task in self.tasks.items():
            first_station = task['first']
            last_station = task['last']
            self.draw_text(coords[agv]['first'][0], coords[agv]['first'][1], first_station, size=13, color="black", weight="normal", tag="tasks")
            self.draw_text(coords[agv]['last'][0], coords[agv]['last'][1], last_station, size=13, color="black", weight="normal", tag="tasks")

    def update_status(self):
        coords = {
            'AGV01': (228, 198),
            'AGV02': (228, 412),
            'AGV03': (228, 626),
            'AGV04': (228, 839)
        }
        agv_number = ['AGV01', 'AGV02', 'AGV03', 'AGV04']
        for agv_index in range(4):
            agv = agv_number[agv_index]
            status = self.agv_status[agv_index]
            x, y = coords[agv]
            self.canvas.delete(f"{agv}_status")
            self.draw_text(x, y, status, size=13, color="black", weight="normal", tag=f"{agv}_status")

    def update_recent_missions(self, recent_agv):
        self.canvas.delete("recent_missions")

        for i, (first, last, agv) in enumerate(self.recent_missions):
            self.draw_text(1477, 627 + (i * 25), f"Station {first} to Station {last}", size=12, color="black", weight="normal", tag="recent_missions")
            self.draw_text(1698, 627 + (i * 25), agv, size=12, color="black", weight="normal", tag="recent_missions")

        self.draw_text(1698, 627, f'AGV0{recent_agv}', size=12, color="black", weight="normal", tag="recent_missions")
    
    def shift_detections(self):
        self.detection_images = [(x + 220, y, img, agv_id) for (x, y, img, agv_id) in self.detection_images]

    def update_detections(self):
        self.root.after(100, self.draw_detections)

    def draw_detections(self):
        self.minimap.map_canvas.delete("detection")
        
        for (x, y, img, agv_id) in self.detection_images:
            self.minimap.map_canvas.create_image(x, y, image=img, anchor=tk.NW, tags="detection")
            self.minimap.map_canvas.create_text(x + 144, y + 15, text=agv_id, fill="black", font=('Arial', 12, 'bold'), tags="detection")
        
        self.minimap.map_canvas.tag_raise("detection")

    def update_emergency_status(self):
        coords = {
            'AGV01': (393, 304),
            'AGV02': (393, 518),
            'AGV03': (393, 732),
            'AGV04': (393, 945)
        }
        self.canvas.delete("emergency")

        for agv_name, position in coords.items():
            if self.emergency_status[agv_name]:
                self.canvas.create_image(position[0], position[1], image=self.imgEmergency, anchor=tk.CENTER, tag="emergency")

        self.canvas.after(100, self.update_emergency_status)
    
    def update_battery(self):
        self.canvas.delete("Battery_AGV01")
        self.canvas.delete("Battery_AGV02")
        self.canvas.delete("Battery_AGV03")
        self.canvas.delete("Battery_AGV04")

        self.update_battery_status(0, 331, 128, self.battery_of_agv[0])
        self.update_battery_status(1, 331, 342, self.battery_of_agv[1])
        self.update_battery_status(2, 331, 556, self.battery_of_agv[2])
        self.update_battery_status(3, 331, 769, self.battery_of_agv[3])

    def update_battery_status(self, index, x, y, battery_level):
        if battery_level == 100:
            battery_image = self.imgBattery_100
        elif 50 < battery_level <= 75:
            battery_image = self.imgBattery_75
        elif 25 < battery_level <= 50:
            battery_image = self.imgBattery_50
        elif 0 < battery_level <= 25:
            battery_image = self.imgBattery_25
        else:
            battery_image = self.imgBattery_0

        self.canvas.create_image(x, y, image=battery_image, anchor="nw", tags=f"Battery_AGV0{index + 1}")

    def periodic_battery_update(self):
        self.update_battery()
        self.root.after(100, self.periodic_battery_update)

    def show_custom_message(self, title, message):
        top = tk.Toplevel(self.root)
        top.title(title)
        top.geometry("300x150")

        custom_font = self.get_font(12)

        message_label = tk.Label(top, text=message, font=custom_font, wraplength=280)
        message_label.pack(pady=20, padx=20)

        ok_button = tk.Button(top, text="OK", command=top.destroy, font=custom_font)
        ok_button.pack(pady=10, padx=10)

        top.update_idletasks()
        try:
            top.grab_set()
        except TclError:
            top.grab_release()
            top.grab_set()
    
    def ros_subscribe_detection(self, data, agv_id):
        detection_image = None
        for bbox in data.bounding_boxes:
            if bbox.Class == "people":
                detection_image = self.imgObstacle_human
            elif bbox.Class == "box":
                detection_image = self.imgObstacle_box

            if detection_image:
                self.shift_detections()
                self.detection_images.append((self.detection_x, self.detection_y, detection_image, agv_id))
                self.detection_x += 220
                self.update_detections()
    
    def ros_subscribe_emergency(self, msg, agv_name):
        if msg.data == 'on':
            self.emergency_status[agv_name] = True
        elif msg.data == 'off':
            self.emergency_status[agv_name] = False

    def ros_subscribe_agv_status(self, msg):
        self.agv_status = msg.data.split(', ')
        for i, status in enumerate(self.agv_status):
            agv_name = f'AGV0{i+1}'
            if status == 'Not connect':
                # Unsubscribe from emergency topic if status is "Not connect"
                if self.emergency_subscribers.get(agv_name):
                    self.emergency_subscribers[agv_name].unregister()
                    self.emergency_subscribers[agv_name] = None
            else:
                # Subscribe to emergency topic if status is not "Not connect" and not already subscribed
                if not self.emergency_subscribers.get(agv_name):
                    self.emergency_subscribers[agv_name] = rospy.Subscriber(f'/{agv_name}/Emergen', 
                        String, self.ros_subscribe_emergency, agv_name)
        self.update_status()
        self.update_dropdown_state()

    def ros_subscribe_task(self, callback):
        if self.task_subscriber is None:
            self.task_subscriber = rospy.Subscriber('/task', String, callback)

    def ros_publish_agv_status(self, status):
        agv_status_info = ', '.join(status)
        rospy.loginfo(agv_status_info)
        self.agv_status_pub.publish(agv_status_info)

        for i, stat in enumerate(status):
            self.agv_status[i] = stat

    def ros_publish_mission(self, mission_info):
        first, last = mission_info
        mission_info_str = f"{first},{last}"
        rospy.loginfo(f"mission : ({mission_info_str})")
        self.mission_pub.publish(mission_info_str)

class MiniMapUI:
    def __init__(self, parent):
        self.map_canvas = tk.Canvas(parent, width=880, height=700, highlightthickness=0)
        self.map_canvas.place(x=431, y=150)

        rospy.Subscriber('/map', OccupancyGrid, self.map_callback)
        rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.pose_callback_agv01)
        rospy.Subscriber('/agv02/amcl_pose', PoseWithCovarianceStamped, self.pose_callback_agv02)
        rospy.Subscriber('/agv03/amcl_pose', PoseWithCovarianceStamped, self.pose_callback_agv03)
        rospy.Subscriber('/agv04/amcl_pose', PoseWithCovarianceStamped, self.pose_callback_agv04)

        self.map_data = None
        self.robot_poses = {}

        self.new_width = 0
        self.new_height = 0
        self.x_center = 0
        self.y_center = 0

        self.load_agv_images()

    def load_agv_images(self):
        path_file = os.path.dirname(__file__)
        self.agv_images = {
            'agv01': ImageTk.PhotoImage(Image.open(os.path.join(path_file, "../image/agv01.png"))),
            'agv02': ImageTk.PhotoImage(Image.open(os.path.join(path_file, "../image/agv02.png"))),
            'agv03': ImageTk.PhotoImage(Image.open(os.path.join(path_file, "../image/agv03.png"))),
            'agv04': ImageTk.PhotoImage(Image.open(os.path.join(path_file, "../image/agv04.png")))
        }

    def map_callback(self, data):
        self.map_data = data
        self.draw_map()

    def pose_callback_agv01(self, data):
        self.robot_poses['agv01'] = data.pose.pose
        if self.map_data:
            self.draw_robots()

    def pose_callback_agv02(self, data):
        self.robot_poses['agv02'] = data.pose.pose
        if self.map_data:
            self.draw_robots()

    def pose_callback_agv03(self, data):
        self.robot_poses['agv03'] = data.pose.pose
        if self.map_data:
            self.draw_robots()

    def pose_callback_agv04(self, data):
        self.robot_poses['agv04'] = data.pose.pose
        if self.map_data:
            self.draw_robots()

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

        # Draw the robots after the map
        self.draw_robots()

    def draw_robots(self):
        if self.map_data:
            resolution = self.map_data.info.resolution
            origin_x = self.map_data.info.origin.position.x
            origin_y = self.map_data.info.origin.position.y

            aspect_ratio = min(880 / self.map_data.info.width, 700 / self.map_data.info.height)

            for robot_id, pose in self.robot_poses.items():
                x = (pose.position.x - origin_x) / resolution
                y = (pose.position.y - origin_y) / resolution

                robot_x = self.x_center + int(x * aspect_ratio)
                robot_y = self.y_center + int((self.map_data.info.height - y) * aspect_ratio)

                # Remove existing robot image
                self.map_canvas.delete(robot_id)

                # Draw the robot's image
                if robot_id in self.agv_images:
                    self.map_canvas.create_image(robot_x, robot_y, image=self.agv_images[robot_id], anchor=tk.CENTER, tags=robot_id)
                    self.map_canvas.tag_raise(robot_id)  # Ensure robot is on top layer

if __name__ == "__main__":
    root = tk.Tk()
    app = FleetManagementUI(root)
    root.mainloop()