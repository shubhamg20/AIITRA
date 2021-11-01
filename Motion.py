import rospy
import math
import numpy as np
from geometry_msgs.msg import Twist

class Motion:
    def _init_(self,index):
        topic='/robot'+index+'/cmd_vel'
        self.publisher_velocity=rospy.Publisher('topic',Twist,queue_size=10)
    # def Rotate(self):
    #     vel_msg = Twist()
    #     vel_msg.linear.x=0
    #     vel_msg.linear.y=0
    #     vel_msg.linear.z=0
    #     vel_msg.angular.x=0
    #     vel_msg.angular.y=0
    #     vel_msg.angular.z=wz
    #     self.publisher_velocity.publish(vel_msg)
    def Translate(self,Vx,Vy):
        vel_msg=Twist()
        vel_msg.linear.x=Vx
        vel_msg.linear.y=Vy
        vel_msg.linear.z=0
        vel_msg.angular.x=0
        vel_msg.angular.y=0
        vel_msg.angular.z=0
        self.publisher_velocity.publish(vel_msg)