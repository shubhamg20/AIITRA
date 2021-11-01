#!/usr/bin/env python
import rospy
import time
from geometry_msgs.msg import Twist

def move():
    # Starts a new node
    rospy.init_node('bot_cleaner', anonymous=False)
    velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    vel_msg = Twist()
    i=0
    while(i<5):
        #Receiveing the user's input
        print("Let's move your robot")
        speed = input("Input your speed:")
        distance = input("Type your distance:")
        isForward = input("Foward?: ")#True or False

        #Checking if the movement is forward or backwards
        if(isForward):
            vel_msg.linear.x = abs(speed)
        else:
            vel_msg.linear.x = -abs(speed)
        #Since we are moving just in x-axis
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z = 0

        

            #Setting the current time for distance calculus
        t0 =time.time()
        current_distance = 0

            #Loop to move the turtle in an specified distance
        while(current_distance < distance):
                #Publish the velocity
            velocity_publisher.publish(vel_msg)
            #Takes actual time to velocity calculus
            t1=time.time()
                    #Calculates distancePoseStamped
            current_distance= speed*(t1-t0)
                #After the loop, stops the robot
        vel_msg.linear.x = 0
        #Force the robot to stop
        velocity_publisher.publish(vel_msg)
        i=i+1

if __name__ == '__main__':
    try:
        #Testing our function

        move()
    except rospy.ROSInterruptException: pass