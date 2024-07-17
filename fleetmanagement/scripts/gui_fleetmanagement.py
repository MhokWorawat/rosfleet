#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
import tkinter as tk
from tkinter import font as tkFont
from tkinter import ttk
from tkinter import TclError
from PIL import Image, ImageTk
from functools import partial
import paramiko
import threading
import time
import os

class FleetManagementUI:
    def __init__(self, root):
        rospy.init_node('gui_fleet_management', anonymous=True)

        self.root = root
        self.root.title("Fleet Management")
        self.root.geometry("1846x1042")

        # Setup environment UI
        self.load_environment()
        self.setup_background()

        # Initialize data
        self.buttons = []
        self.labels = []
        self.current_images = [2, 2, 2, 2]
        self.ssh_ips = ["192.168.0.110", "192.168.0.120", "192.168.0.130", "192.168.0.140"]
        self.ssh_users = ["pi01", "pi02", "pi03", "pi04"]
        self.ssh_clients = [None, None, None, None]
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

        # ROS
        self.ros_setup_source = "source /opt/ros/noetic/setup.bash && export ROS_MASTER_URI=http://192.168.0.200:11311 && export ROS_HOSTNAME={0} && export ROS_IP={0}"
        self.battery_AGV01 = 100
        self.battery_AGV02 = 100
        self.battery_AGV03 = 100
        self.battery_AGV04 = 100

        # ros_Publisher
        self.agv_status_pub = rospy.Publisher('/agv_status', String, queue_size=4, latch=True)
        self.mission_pub = rospy.Publisher('/mission', String, queue_size=6, latch=True)
        self.ros_publish_agv_status(self.agv_status)

        # ros_Subscriber
        self.task_subscriber = None 
        rospy.Subscriber('/agv_status', String, self.ros_subscrib_agv_status)
        rospy.Subscriber('/task', String, self.ros_subscrib_task)
        rospy.Subscriber('/agv01/Emergen', String, self.ros_subscrib_emergency, 'AGV01')
        rospy.Subscriber('/agv02/Emergen', String, self.ros_subscrib_emergency, 'AGV02')
        rospy.Subscriber('/agv03/Emergen', String, self.ros_subscrib_emergency, 'AGV03')
        rospy.Subscriber('/agv04/Emergen', String, self.ros_subscrib_emergency, 'AGV04')

        # Create widget
        self.create_ssh_buttons()
        self.create_dropdown()
        self.create_confirm_button()

        # Update UI with initial data
        self.update_dropdowns()
        self.update_tasks()
        self.update_battery()
        self.periodic_battery_update()
        self.update_emergency_status()

    def load_environment(self):
        path_file = os.path.dirname(__file__)
        self.font_name = os.path.join(path_file, "../font/Prompt/Prompt-Regular.ttf")
        self.bg_image = Image.open(os.path.join(path_file, "../image/background.png"))
        self.buttonSSHon = ImageTk.PhotoImage(Image.open(os.path.join(path_file, "../image/button_on.png")))
        self.buttonSSHoff = ImageTk.PhotoImage(Image.open(os.path.join(path_file, "../image/button_off.png")))
        self.buttonConfirm = ImageTk.PhotoImage(Image.open(os.path.join(path_file, "../image/button_confirm.png")))
        self.buttonConfirmOn = ImageTk.PhotoImage(Image.open(os.path.join(path_file, "../image/button_confirm_On.png")))
        self.imgEmergency = ImageTk.PhotoImage(Image.open(os.path.join(path_file, "../image/emergency.png")))
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

    def create_ssh_buttons(self):
        self.create_ssh_button(0, 60, 128)
        self.create_ssh_button(1, 60, 342)
        self.create_ssh_button(2, 60, 556)
        self.create_ssh_button(3, 60, 769)

    def create_ssh_button(self, index, x, y):
        button = tk.Label(self.root, image=self.buttonSSHoff, borderwidth=0, highlightthickness=0, bg="#D9D9D9")
        button.place(x=x, y=y)
        button.bind("<Button-1>", partial(self.toggle_button, index))
        self.buttons.append(button)

    def toggle_button(self, AGV_index, event):
        if self.current_images[AGV_index] == 2:
            threading.Thread(target=self.connect_ssh_client, args=(AGV_index,)).start()
        else:
            threading.Thread(target=self.disconnect_ssh_client, args=(AGV_index,)).start()

    def connect_ssh_client(self, AGV_index):
        ip = self.ssh_ips[AGV_index]
        user = self.ssh_users[AGV_index]
        password = "password"

        print(f"Attempting SSH connection to {user}@{ip}")

        client = paramiko.SSHClient()
        client.set_missing_host_key_policy(paramiko.AutoAddPolicy())

        if self.on_ssh_connect(AGV_index, user, ip, 10):
            self.buttons[AGV_index].config(image=self.buttonSSHon)
            self.current_images[AGV_index] = 1
            self.ROS_commands(AGV_index)
            self.agv_status[AGV_index] = "Available"
            self.ros_publish_agv_status(self.agv_status)
            print(f"SSH connection to {user}@{ip} successful")
        else:
            self.agv_status[AGV_index] = "Not connect"
            self.ros_publish_agv_status(self.agv_status)
            print(f"SSH connection to AGV{AGV_index + 1} failed")
            self.show_custom_message("Connection Error", f"Failed to connect to {user}@{ip}")

    def disconnect_ssh_client(self, AGV_index):
        ip = self.ssh_ips[AGV_index]
        user = self.ssh_users[AGV_index]
        client = self.ssh_clients[AGV_index]

        if client:
            try:
                # Setup ROS environment variables
                client.exec_command(self.ros_setup_source.format(self.ssh_ips[AGV_index]))
                full_command = f"{self.ros_setup_source} && sudo reboot"
                stdin, stdout, stderr = client.exec_command(full_command)
                
                # Print output and error for debugging
                out = stdout.read().decode()
                err = stderr.read().decode()
                print(f"stdout: {out}")
                print(f"stderr: {err}")

                stdout.channel.recv_exit_status()  # Wait for command to complete
                print(f"Executed reboot command for {user}@{ip}: sudo reboot")

                # Wait for a bit to ensure command execution
                time.sleep(1)

                # Disconnect from SSH
                self.on_ssh_disconnect(AGV_index)
                self.buttons[AGV_index].config(image=self.buttonSSHoff)
                self.current_images[AGV_index] = 2
                self.agv_status[AGV_index] = "Not connect"
                self.ros_publish_agv_status(self.agv_status)
                print(f"Disconnected from {user}@{ip}")
            except Exception as e:
                print(f"Failed to disconnect from {user}@{ip}: {e}")
        else:
            print(f"No SSH client found for {user}@{ip}")

    def on_ssh_connect(self, AGV_index, user, ip, timeout):
        client = paramiko.SSHClient()
        client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        try:
            client.connect(ip, username=user, password='password', timeout=timeout)
            self.ssh_clients[AGV_index] = client
            return True
        except Exception as e:
            print(f"Connection to {user}@{ip} failed: {e}")
            return False

    def on_ssh_disconnect(self, AGV_index):
        client = self.ssh_clients[AGV_index]
        if client:
            client.close()
            self.ssh_clients[AGV_index] = None
            print(f"SSH client closed for {self.ssh_users[AGV_index]}@{self.ssh_ips[AGV_index]}")
    
    def ROS_commands(self, AGV_index):
        client = self.ssh_clients[AGV_index]
        if client:
            # Setup ROS environment variables
            client.exec_command(self.ros_setup_source.format(self.ssh_ips[AGV_index]))
            
            if AGV_index == 0:   #-----AGV01
                rosrun_cmds = [
                    "screen -dmS rosrun rosserial_python serial_node.py /dev/ttyACM0"
                ]
            elif AGV_index == 1: #-----AGV02
                rosrun_cmds = [
                    "screen -dmS rosrun rosserial_python serial_node.py /dev/ttyACM0"
                ]
            elif AGV_index == 2: #-----AGV03
                rosrun_cmds = [
                    "screen -dmS rosrun rosserial_python serial_node.py /dev/ttyACM0"
                ]
            elif AGV_index == 3: #-----AGV04
                rosrun_cmds = [
                    "screen -dmS rosrun rosserial_python serial_node.py /dev/ttyACM0",
                    "screen -dmS lidar roslaunch rplidar_ros rplidar_a1.launch"
                ]

            for cmd in rosrun_cmds:
                full_command = f"{self.ros_setup_source.format(self.ssh_ips[AGV_index])} && {cmd}"
                stdin, stdout, stderr = client.exec_command(full_command)
                print(stdout.read().decode())
                print(stderr.read().decode())
        else:
            print(f"SSH client is not connected for {self.ssh_users[AGV_index]}@{self.ssh_ips[AGV_index]}")

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

        self.ros_subscrib_task(on_task_update)

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

        self.update_battery_status(0, 331, 128, self.battery_AGV01)
        self.update_battery_status(1, 331, 342, self.battery_AGV02)
        self.update_battery_status(2, 331, 556, self.battery_AGV03)
        self.update_battery_status(3, 331, 769, self.battery_AGV04)

    def update_battery_status(self, index, x, y, battery_level):
        if 75 < battery_level <= 100:
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
        self.battery_AGV01 = max(0, self.battery_AGV01 - 1)
        self.battery_AGV02 = max(0, self.battery_AGV02 - 1)
        self.battery_AGV03 = max(0, self.battery_AGV03 - 1)
        self.battery_AGV04 = max(0, self.battery_AGV04 - 1)
        self.update_battery()
        self.root.after(3000, self.periodic_battery_update)

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
    
    def ros_subscrib_emergency(self, msg, agv_name):
        if msg.data == 'on':
            self.emergency_status[agv_name] = True
        elif msg.data == 'off':
            self.emergency_status[agv_name] = False

    def ros_subscrib_agv_status(self, msg):
        self.agv_status = msg.data.split(', ')
        self.update_status()
        self.update_dropdown_state()

    def ros_subscrib_task(self, callback):
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
        rospy.loginfo(mission_info_str)
        self.mission_pub.publish(mission_info_str)

if __name__ == "__main__":
    root = tk.Tk()
    app = FleetManagementUI(root)
    root.mainloop()