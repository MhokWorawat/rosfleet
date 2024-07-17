#!/usr/bin/env python3

import rospy
import tkinter as tk
from std_msgs.msg import Float32

class GUI:
    def __init__(self):
        self.nodeName = 'GUI'

        self.topicVeloX = 'agv01/LinearX'
        self.topicVeloY = 'agv01/LinearY'
        self.topicVeloZ = 'agv01/AngularZ'

        self.topicSpeedL = 'agv01/SpeedL'
        self.topicSpeedR = 'agv01/SpeedR'

        self.topicEncL = 'agv01/EncL'
        self.topicEncR = 'agv01/EncR'

        self.VeloX, self.VeloY, self.VeloZ, self.SpeedL, self.SpeedR, self.EncoderL, self.EncoderR = 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0

        self.root = tk.Tk()
        self.root.title("Display Data")

        self.label_width = 20
        self.label_height = 1
        self.label_spacing = 10

        labelA_title = tk.Label(self.root, text="Linear Velocity X :", width=self.label_width, anchor="e")
        labelB_title = tk.Label(self.root, text="Linear Velocity Y :", width=self.label_width, anchor="e")
        labelC_title = tk.Label(self.root, text="Angular Velocity Z :", width=self.label_width, anchor="e")
        labelD_title = tk.Label(self.root, text="speed L :", width=self.label_width, anchor="e")
        labelE_title = tk.Label(self.root, text="speed R :", width=self.label_width, anchor="e")
        labelF_title = tk.Label(self.root, text="encoder L :", width=self.label_width, anchor="e")
        labelG_title = tk.Label(self.root, text="encoder R :", width=self.label_width, anchor="e")

        labelunitA_title = tk.Label(self.root, text="m/s", width=self.label_width, anchor="w")
        labelunitB_title = tk.Label(self.root, text="m/s", width=self.label_width, anchor="w")
        labelunitC_title = tk.Label(self.root, text="m/s", width=self.label_width, anchor="w")
        labelunitD_title = tk.Label(self.root, text="m/s", width=self.label_width, anchor="w")
        labelunitE_title = tk.Label(self.root, text="m/s", width=self.label_width, anchor="w")
        labelunitF_title = tk.Label(self.root, text="/100ms", width=self.label_width, anchor="w")
        labelunitG_title = tk.Label(self.root, text="/100ms", width=self.label_width, anchor="w")

        self.labelA_value = tk.Label(self.root, text=str(self.VeloX), width=self.label_width, height=self.label_height, bg="white", anchor="w")
        self.labelB_value = tk.Label(self.root, text=str(self.VeloY), width=self.label_width, height=self.label_height, bg="white", anchor="w")
        self.labelC_value = tk.Label(self.root, text=str(self.VeloZ), width=self.label_width, height=self.label_height, bg="white", anchor="w")
        self.labelD_value = tk.Label(self.root, text=str(self.SpeedL), width=self.label_width, height=self.label_height, bg="white", anchor="w")
        self.labelE_value = tk.Label(self.root, text=str(self.SpeedR), width=self.label_width, height=self.label_height, bg="white", anchor="w")
        self.labelF_value = tk.Label(self.root, text=str(self.EncoderL), width=self.label_width, height=self.label_height, bg="white", anchor="w")
        self.labelG_value = tk.Label(self.root, text=str(self.EncoderR), width=self.label_width, height=self.label_height, bg="white", anchor="w")
   
        labelA_title.grid(row=0, column=0, sticky="e", padx=5, pady=5)
        labelB_title.grid(row=1, column=0, sticky="e", padx=5, pady=5)
        labelC_title.grid(row=2, column=0, sticky="e", padx=5, pady=5)
        labelD_title.grid(row=3, column=0, sticky="e", padx=5, pady=5)
        labelE_title.grid(row=4, column=0, sticky="e", padx=5, pady=5)
        labelF_title.grid(row=5, column=0, sticky="e", padx=5, pady=5)
        labelG_title.grid(row=6, column=0, sticky="e", padx=5, pady=5)

        labelunitA_title.grid(row=0, column=2, sticky="w", padx=5, pady=5)
        labelunitB_title.grid(row=1, column=2, sticky="w", padx=5, pady=5)
        labelunitC_title.grid(row=2, column=2, sticky="w", padx=5, pady=5)
        labelunitD_title.grid(row=3, column=2, sticky="w", padx=5, pady=5)
        labelunitE_title.grid(row=4, column=2, sticky="w", padx=5, pady=5)
        labelunitF_title.grid(row=5, column=2, sticky="w", padx=5, pady=5)
        labelunitG_title.grid(row=6, column=2, sticky="w", padx=5, pady=5)

        self.labelA_value.grid(row=0, column=1, sticky="w", padx=(0, self.label_spacing), pady=5)
        self.labelB_value.grid(row=1, column=1, sticky="w", padx=(0, self.label_spacing), pady=5)
        self.labelC_value.grid(row=2, column=1, sticky="w", padx=(0, self.label_spacing), pady=5)
        self.labelD_value.grid(row=3, column=1, sticky="w", padx=(0, self.label_spacing), pady=5)
        self.labelE_value.grid(row=4, column=1, sticky="w", padx=(0, self.label_spacing), pady=5)
        self.labelF_value.grid(row=5, column=1, sticky="w", padx=(0, self.label_spacing), pady=5)
        self.labelG_value.grid(row=6, column=1, sticky="w", padx=(0, self.label_spacing), pady=5)

        rospy.init_node(self.nodeName, anonymous=True)

        rospy.Subscriber(self.topicVeloX, Float32, self.callBackVelocityX)
        rospy.Subscriber(self.topicVeloY, Float32, self.callBackVelocityY)
        rospy.Subscriber(self.topicVeloZ, Float32, self.callBackVelocityZ)

        rospy.Subscriber(self.topicSpeedL, Float32, self.callBackSpeedL)
        rospy.Subscriber(self.topicSpeedR, Float32, self.callBackSpeedR)

        rospy.Subscriber(self.topicEncL, Float32, self.callBackEncL)
        rospy.Subscriber(self.topicEncR, Float32, self.callBackEncR)

        self.root.mainloop()

    def callBackVelocityX(self, message):
        self.VeloX = message.data
        self.labelA_value.config(text=str(self.VeloX))

    def callBackVelocityY(self, message):
        self.VeloY = message.data
        self.labelB_value.config(text=str(self.VeloY))

    def callBackVelocityZ(self, message):
        self.VeloZ = message.data
        self.labelC_value.config(text=str(self.VeloZ))

    def callBackSpeedL(self, message):
        self.SpeedL = message.data
        self.labelD_value.config(text=str(self.SpeedL))

    def callBackSpeedR(self, message):
        self.SpeedR = message.data
        self.labelE_value.config(text=str(self.SpeedR))

    def callBackEncL(self, message):
        self.EncoderL = message.data
        self.labelF_value.config(text=str(self.EncoderL))

    def callBackEncR(self, message):
        self.EncoderR = message.data
        self.labelG_value.config(text=str(self.EncoderR))

if __name__ == '__main__':
    gui = GUI()
    rospy.spin()
