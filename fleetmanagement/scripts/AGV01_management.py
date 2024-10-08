#!/usr/bin/env python3

import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__)))

import rospy
from management import AGV_management

class AGV01:
    def __init__(self):
        self.agv_name = 'AGV01'
        self.management = AGV_management(self.agv_name)

if __name__ == '__main__':
    try:
        agv = AGV01()
        agv.management.run()
    except rospy.ROSInterruptException:
        pass
