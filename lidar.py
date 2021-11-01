#!/usr/bin/env python
import math
from tf.transformations import euler_from_quaternion
import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import AIITRA
class Lidar():
    def get_laser_data(self,data):              
        self.bot_ranges=data.ranges
    # def get_rotation (self,msg):
    #     orientation_msg = msg.pose.pose.orientation
    #     orientation_list = [orientation_msg.x, orientation_msg.y, orientation_msg.z, orientation_msg.w]
    #     (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
    #     self.yaw=yaw                                                        #check yaw

    def get_exit_cell_cordis(self):
        exit_gridcells=[]
        (Fx,Fy)=[self.main_bot_cord.x+(self.range)*math.cos(self.yaw),self.main_bot_cord.y+(self.range)*math.sin(self.yaw)]
        (Fx,Fy)=[int(2*Fx),int(2*Fy)]
        (Bx,By)=[int(2*self.main_bot_cord.x),int(2*self.main_bot_cord.y)]
        # if (Bx-Fx,By-Fy)==(0,1):
        #     exit_gridcells=[[Bx+1,By],[Bx+1,By+1],[Bx,By+1],[Bx-1,By-1],[Bx-1,By]]                  #change the order here
        # if (Bx-Fx,By-Fy)==(1,1):
        #     exit_gridcells=[[Bx+1,By],[Bx+1,By+1],[Bx,By+1],[Bx-1,By+1],[Bx+1,By-1]]            
        # if (Bx-Fx,By-Fy)==(1,0):
        #     exit_gridcells=([Bx+1,By],[Bx+1,By+1],[Bx,By+1],[Bx,By-1],[Bx+1,By-1])            
        # if (Bx-Fx,By-Fy)==(1,-1):
        #     exit_gridcells=([Bx+1,By],[Bx+1,By+1],[Bx,By-1],[Bx-1,By-1],[Bx+1,By-1])            
        # if (Bx-Fx,By-Fy)==(0,-1):
        #     exit_gridcells=[[Bx-1,By],[Bx-1,By-1],[Bx,By-1],[Bx+1,By+1],[Bx+1,By]]            
        # if (Bx-Fx,By-Fy)==(-1,-1):
        #     exit_gridcells=[[Bx-1,By],[Bx-1,By-1],[Bx,By-1],[Bx+1,By-1],[Bx-1,By+1]]          
        # if (Bx-Fx,By-Fy)==(-1,0):
        #     exit_gridcells=([Bx-1,By],[Bx-1,By-1],[Bx,By-1],[Bx,By+1],[Bx-1,By+1])   
        # if (Bx-Fx,By-Fy)==(-1,1):
        #     exit_gridcells=([Bx-1,By],[Bx-1,By-1],[Bx,By+1],[Bx+1,By+1],[Bx-1,By+1])
        exit_gridcells=[[Bx+1,By],[Bx+1,By+1],[Bx,By+1],[Bx-1,By-1],[Bx-1,By],[Bx,By-1],[Bx-1,By+1],[Bx+1,By-1]]
        self.exit_gridcells=exit_gridcells
        return self.exit_gridcells

    def get_exit_cell_indexes(self):         # detect obstacles grid then give permone to them 
        self.exit_list=[]
        self.indexes_with_obs=[]
        if (self.bot_ranges[0]>.8 and self.bot_ranges[1]>.8) and self.bot_ranges[2]>.8:
            self.exit_list.append(1)
        if (self.bot_ranges[6]>.8 and self.bot_ranges[7]>.8) and self.bot_ranges[8]>.8:
            self.exit_list.append(2)    
        if (self.bot_ranges[12]>.8 and self.bot_ranges[13]>.8) and self.bot_ranges[14]>.8:
            self.exit_list.append(3)
        if (self.bot_ranges[18]>.8 and self.bot_ranges[19]>.8) and self.bot_ranges[20]>.8:
            self.exit_list.append(4)
        if (self.bot_ranges[22]>.8 and self.bot_ranges[23]>.8) and self.bot_ranges[24]>.8:
            self.exit_list.append(5)
        if (self.bot_ranges[22]>.8 and self.bot_ranges[23]>.8) and self.bot_ranges[24]>.8:
            self.exit_list.append(6)
        if (self.bot_ranges[22]>.8 and self.bot_ranges[23]>.8) and self.bot_ranges[24]>.8:
            self.exit_list.append(7)
        if (self.bot_ranges[22]>.8 and self.bot_ranges[23]>.8) and self.bot_ranges[24]>.8:
            self.exit_list.append(8)            
        self.n=len(self.exit_list)
        for i in [1,2,3,4,5,6,7,8]:
            if i not in self.exit_list:
                self.indexes_with_obs.append(i)
            
    def get_final_exit_cordis(self,indexes_list):
        final_exit_cells=[]
        exit_gridcells=self.get_exit_cell_cordis()   #total exit cells coordis   always 5
        for x in indexes_list:  # after filtering cells indexes [0,2,4]
            final_exit_cells.append(exit_gridcells[x])
        return final_exit_cells    

    def find_unexplored(self):
        unexplored_grids=[]
        final_exit_cordis=self.get_final_exit_cordis(self.exit_list) 
        for grid in final_exit_cordis:
            if self.map.map[grid[0]][grid[1]]==0:
                unexplored_grids.append(grid)
            else:
                pass
        return unexplored_grids                             

    def __init__(self,index,main_bot_cord):
        self.map=AIITRA.Map()
        self.index=index
        self.range=0.7
        self.main_bot_cord=main_bot_cord
        self.bot_ranges=[]
        rospy.Subscriber('/robot' + self.index +'/scan',LaserScan,self.get_laser_data)
        # rospy.Subscriber ('/robot'+self.index+'odom', Odometry,self.get_rotation)
        if self.bot_ranges:
           self.get_exit_loctns()
           self.get_exit_cell_locns()
           self.get_unknown_cells()
        else:
            pass



