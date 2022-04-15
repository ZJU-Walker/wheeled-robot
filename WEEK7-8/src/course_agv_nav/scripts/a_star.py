from calendar import c
from math import floor
import numpy as np
from heapq import *

#map=np.array()


class AStarPlanner():
    def __init__(self,dt,radius):
        self.dt=dt
        self.radius=radius
    
    def planning(self,obs_x,obs_y,start_x,start_y,target_x,target_y,minx,miny,maxx,maxy):

        #合并障碍物的xy信息
        obs=np.vstack([obs_x,obs_y]).T
        print(obs)

        #create map
        map_length=100
        map_width=100
        pixel_length=(maxy-miny)/map_length
        pixel_width=(maxx-minx)/map_width
        map=np.zeros([map_length,map_length])
        map=map.reshape(-1) #一维化

        #a是像素化矩阵，矩阵中每个点寸这个像素的中心点对应的原始坐标，
        a=np.zeros([map_width,map_length,2])
        for i in np.arange(map_width):
            for j in np.arange(map_length):
                a[i][j][0]=(i+0.5)*pixel_width+minx
                a[i][j][1]=(j+0.5)*pixel_length+miny
        a=a.reshape(-1,2)#a变为i一列坐标

        #对于每个障碍物（圆），在map中标出障碍物点
        for o in obs:
            dist=np.hypot(o[0]-a[:,0],o[1]-a[:,1]) #每个像素点到当前o圆心的距离，构成的列表
            map[dist<self.radius]=1
        map=map.reshape(map_width,map_length)
        print(map)
        
        #地图建立完毕，开始A*算法

        #给出像素描述的起始和终点坐标
        start_pixel_x=floor((start_x-minx)/pixel_width)
        start_pixel_y=floor((start_y-miny)/pixel_length)
        target_pixel_x=floor((target_x-minx)/pixel_width)
        target_pixel_y=floor((target_y-miny)/pixel_length)


        #设置Table和list并初始化
        Infinity=1000000
        NotAVertex=(-1,-1)
        #vertex = np.zeros([map_width,map_length])
        dist_from_start = np.zeros([map_width,map_length])
        dist_to_target = np.zeros([map_width,map_length])
        known = np.zeros([map_width,map_length])
        path = np.zeros([map_width,map_length,2])#多一维，存x和y

        for i in range(map_width):
            for j in range(map_length):
                dist_from_start[i][j]=Infinity
                dist_to_target[i][j]=np.hypot(i-target_pixel_x, j-target_pixel_y)
                known[i][j]=0
                path[i][j]=NotAVertex

        dist_from_start[start_pixel_x][start_pixel_y]=0
        known[start_pixel_x][start_pixel_y]=1

        list=[(0,start_pixel_x,start_pixel_y),]
        heapify(list)

        #寻路
        while(list is not np.empty):
            current=heappop(list)
            known[current[1]][current[2]]=1
            if(current[1]==target_pixel_x and current[2]==target_pixel_y): #target found 
                break
            #遍历周围的点
            for i in [current[1]-1,current[1],current[1]+1]:
                for j in [current[2]-1,current[2],current[2]+1]:
                    if(0<=i<map_width and 0<=j<map_length):#位于地图内
                        if(not known[i][j] and not map[i][j] ): #未知，不是障碍物，
                            if(dist_from_start[i][j] > dist_from_start[current[1]][current[2]]+np.hypot(i-current[1],j-current[2])):#距离可以被更新
                                dist_from_start[i][j]=dist_from_start[current[1]][current[2]]+np.hypot(i-current[1],j-current[2])#更新距离
                                path[i][j][0]=current[1]
                                path[i][j][1]=current[2]#更新路径
                                node=(dist_from_start[i][j]+dist_to_target[i][j],i,j) #设置要插入heap的结点
                                heappush(list,node)
        print(current)
        print(target_pixel_x)
        print(target_pixel_y)
        #print(path[:][:][0])
        #print(path[1])

        #确定路径
        path_list_x=[]
        path_list_y=[]
        current_x=int(current[1])
        current_y=int(current[2])
        while(not(current_x==start_pixel_x and current_y==start_pixel_y)):
            path_list_x.append(current_x)
            path_list_y.append(current_y)
            former_x=int(path[current_x][current_y][0])
            former_y=int(path[current_x][current_y][1])
            current_x=former_x
            current_y=former_y
        path_list_x.append(start_pixel_x)
        path_list_y.append(start_pixel_y)
        # path_list_x.reverse()
        # path_list_y.reverse()

        #还原回原图坐标
        path_list_x_r=[]
        path_list_y_r=[]

        for item in path_list_x:
            path_list_x_r.append((float(item)+0.5)*pixel_width+minx)
        for item in path_list_y:
            path_list_y_r.append((float(item)+0.5)*pixel_length+miny)

        #print("path_x:",path_list_x_r)
        # print("path_y:",path_list_y)
        return path_list_x_r,path_list_y_r




        





        




   


