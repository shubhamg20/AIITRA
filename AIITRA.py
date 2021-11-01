#!/usr/bin/env python
import rospy 
import numpy as np 
import math                                                                        #correction in magnitude of rejection vector     
from std_msgs.msg import Float32 
from nav_msgs.msg import Odometry 
import lidar
from Motion import Motion 
from random import randrange 

class Map:
    def __init__(self):
        self.map=np.zeros((100,100),Float32)

    def chng_map_values(self,cell_locns,value):
        for cell in cell_locns:
            self.map[cell[0]][cell[1]]=self.map[cell[0]][cell[1]]+value

class Area_coverage:
    def give_pheromone(self,cell_locns,value):
            map=Map()
            map.chng_map_values(cell_locns,value)

    def get_unit_vtr(self,v):
        vtr= (v.astype(float)/ np.sqrt(np.sum(v**2)))
        return vtr

    def get_grid_state(self):
        return [int(2*self.main_bot_cord[0]),int(2*self.main_bot_cord[1])]

    def get_bot_cords(self,data):
            if self.index==self.main_index:
                self.main_bot_cord=[data.pose.pose.position.x,data.pose.pose.position.y]
            else:
                self.bot_cords_arr.append([data.pose.pose.position.x,data.pose.pose.position.y])      
            
    def get_bot_vtrs_arr(self):
        bot_vtrs_arr=[]
        cords_arr=self.bot_cords_arr
        for cords in cords_arr: 
            arr=np.array(cords[0]-self.main_bot_cord[0],cords[1]-self.main_bot_cord[1])
            bot_vtrs_arr.append(arr)

    def get_normal_vtrs(self,vtrs,m):
        i=0
        for vtr in vtrs:
            unit_vtr=self.get_unit_vtr(vtr)
            vtr=m[i]*vtr
            t=vtr[0]
            vtr[0]=vtr[1]
            vtr[1]=t
            i=i+1
        return vtrs

    def __init__(self):
        r = rospy.Rate(50)  
        self.vel_pub=rospy.Publisher('/drive_wheel/command',Float32,queue_size=10)
        rospy.init_node('/cleaning_bot',anonymous=True)
        self.threshold_bool=[1,1,1,1]
        self.exit_locn=[[],[],[],[]]
        while not rospy.is_shutdown():
            for i in [1,2,3,4]:
                self.main_index=i
                self.lidar_info=lidar.Lidar(self.main_index,self.main_bot_cord)                              
                # self.threshold_rotate=1
                self.bot_cords_arr=[]
                self.bot_vtrs=[]
                for j in [1,2,3,4]:
                    topic='/robot'+j+'/odom'
                    rospy.Subscriber(topic,Odometry,self.get_bot_cords)                            
                curr_bot_grid=self.get_grid_state()
                threshold_x=self.main_bot_cord[0]>curr_bot_grid[0]+(.15) and self.main_bot_cord[0]>curr_bot_grid[0]+1-(.15)
                threshold_y=self.main_bot_cord[1]>curr_bot_grid[1]+(.15) and self.main_bot_cord[1]>curr_bot_grid[1]+1-(.15)
                if (threshold_x and threshold_y)and (self.threshold_bool[self.main_index]):
                    # self.threshold_yaw=self.lidar_info.yaw
                    self.threshold_bool[i]=0
                    cordis_with_obs=self.lidar_info.get_final_exit_cordis(self.lidar_info.indexes_with_obs)
                    self.give_pheromone(cordis_with_obs,-2)
                    UE_locns=self.lidar_info.find_unexplored()
                    if len(UE_locns)==0:
                        if len(cordis_with_obs)==5:
                            self.give_pheromone(curr_bot_grid,-2)                               #not in our given map
                            break  #backtrack and rotate random angle
                        else:
                            self.give_pheromone(curr_bot_grid,-.02)
                            E_locns=[]
                            E_obst_locns=self.lidar_info.get_final_exit_cordis()
                            for grid in E_obst_locns:
                                if grid not in cordis_with_obs:
                                    E_locns.append(grid)
                            pheromone_max=-900
                            map=Map().map
                            grid_max=[]
                            for grid in E_locns:
                                phermone=map[grid[0]][grid[1]]
                                if phermone>pheromone_max:
                                    phermone_max=phermone
                                    grid_max=grid
                            self.exit_locn[self.main_index-1]=grid_max
                            
                            #go to that grid                

                    elif len(UE_locns)==1:
                        self.give_pheromone(curr_bot_grid,-.02)
                        self.exit_locn[self.main_index-1]=UE_locns[0]
                        #go there
                    else:
                        self.give_pheromone(curr_bot_grid,-.02+len(UE_locns))
                        rejection_vtr=self.get_rejection_vtr()
                        self.exit_locn[self.main_index-1]=self.get_closest_exit_locn(rejection_vtr)
                        #go towards this dir
                else:
                    threshold_exit_x=self.main_bot_cord[0]>self.exit_locn[self.main_index-1][0]+(.15) and self.main_bot_cord[0]>self.exit_locn[self.main_index-1][0]+1-(.15)
                    threshold_exit_y=self.main_bot_cord[1]>self.exit_locn[self.main_index-1][1]+(.15) and self.main_bot_cord[1]>self.exit_locn[self.main_index-1][1]+1-(.15)
                    if ( threshold_exit_y and threshold_exit_x ):
                        # self.threshold_rotate=0
                        self.threshold_bool[self.main_index]=1
                    else:
                        EL_mid=(2*self.exit_locn[self.main_index][0]+0.5)/2
                        BC=self.main_bot_cord
                        vel_X=EL_mid[0]-BC[0]
                        vel_Y=-EL_mid[1]-BC[1]
                        Motion.translate(vel_X,vel_Y)
            r.sleep()

    def magnitude_of_vtrs(vector_array): 
        m=[]
        for vector in vector_array:
            k=20                                                                     # find k constant
            RJ_value=k/(math.sqrt(sum(pow(element, 2) for element in vector)))
            if RJ_value>1000:
                RJ_value=1000
            if RJ_value>-1000:
                RJ_value=-1000
            m.append(RJ_value)
        return m 

    def tie_breaker(self,min_vtr):
        counter1=0,counter2=0,index=0
        bot_cell=[int(2*self.main_bot_cord[0]),int(2*self.main_bot_cord[1])]
        for vtr in min_vtr:
            i=1
            while i!=11:
                if  index==0 and map[bot_cell[0]+i*vtr[0]][bot_cell[1]+i*vtr[1]]==0:
                    counter1=counter1+1

                if  index==0 and map[bot_cell[0]+i*vtr[0]][bot_cell[1]+i*vtr[1]]==0:
                    counter2=counter2+1
                i=i+1             
            index=index+1
        if counter1>counter2:
            return min_vtr[0]

        elif counter1<counter2:
            return min_vtr[1]

        else:
            index=randrange(0, 2)
            return min_vtr[index]    

    def get_closest_exit_locn(self,RJ_vtr):
        unexplo_grids=self.lidar_info.find_unexplored()
        grid_to_bot_vtr=[]
        bot=self.main_bot_cord
        angle_min=580
        min_vtr=[]
        for grid in unexplo_grids:
            vector_1=np.array(grid[0]-bot[0],grid[1]-bot[1])
            vector_2=RJ_vtr
            unit_vector_1 = vector_1 / np.linalg.norm(vector_1)
            unit_vector_2 = vector_2 / np.linalg.norm(vector_2)
            dot_product = np.dot(unit_vector_1, unit_vector_2)
            angle = np.arccos(dot_product)
            if angle<=angle_min:
                if math.degrees(angle)-angle_min<math.degrees(5) and math.degrees(angle)-angle_min>-1*math.degrees(5): 
                    min_vtr.append(unit_vector_1)
                angle_min=angle
        if len(min_vtr==1):
           # publish vel/command and rotate/command
            pass
        else:
            min_vtr=self.tie_breaker_RJ(min_vtr)
        return min_vtr

    def get_rejection_vtr(self):
        self.bot_vtrs_arr=self.get_bot_vtrs_arr()
        m=self.magnitude_of_vtrs()
        nrml_vtrs=self.get_normal_vtrs(self.bot_vtrs_arr,m)
        rejection_vtr=[]
        for vtr in nrml_vtrs:
            rejection_vtr=rejection_vtr+vtr
        return rejection_vtr


if  __name__==__main__:
        Area_coverage()