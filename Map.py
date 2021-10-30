import rospy
import math
import numpy
class map:
    def __init__(self):
        arr=numpy.zeros((100,100))
        self.vel_pub=rospy.Publisher('/drive_wheel/command',Float32,queue_size=10)