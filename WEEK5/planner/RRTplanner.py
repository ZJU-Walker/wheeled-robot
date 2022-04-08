import numpy as np
import random
import math

# 节点
class Node:

    def __init__(self, x, y, parent = None):
        self.parent = parent
        self.x = x
        self.y = y



class RRTPlanner:

    def __init__(self, radius):
        self.radius = radius

    def planning(self, obs_x, obs_y, start_x, start_y, target_x, target_y, min_x, min_y, max_x, max_y):
        self.obs_x = obs_x
        self.obs_y = obs_y
        self.start_x = start_x
        self.start_y = start_y
        self.target_x = target_x
        self.target_y = target_y
        self.min_x = min_x
        self.min_y = min_y
        self.max_x = max_x
        self.max_y = max_y
        self.Iter = 5000
        self.path = None

        self.start = Node(self.start_x, self.start_y, None)
        self.target = Node(self.target_x, self.target_y, None)

        self.node_list = [self.start] # 存储路径
        self.step = 1 # 步长
        self.obs = np.array([obs_x,obs_y]).T #转成二维数组obs坐标
        self.px = self.start_x
        self.py = self.start_y

        # 计算地图宽度长度,用于生成随机点
        self.map_x = self.max_x-self.min_x
        self.map_y = self.max_y-self.min_y

        for i in range(self.Iter):
            # 随机生成新的点
            new_point = self.rand_new_point()
            # 获取最近的节点
            nearest_point = self.get_nearest_point(new_point)
            # 获取两点之间的角度
            theta = self.get_theta(nearest_point, new_point)
            # 获取新的点
            new_point = self.get_new_point(nearest_point, theta, self.step)
            # 检查障碍物是否与点相交
            if self.check_obs_collision(nearest_point, new_point):
                new_point.parent = nearest_point
                self.node_list.append(new_point)

                if self.reach_goal(new_point):
                # 找到路径
                    self.path = self.get_path()
                    path1 = np.array(self.path)
                    self.px = np.array(path1[:,0])
                    self.py = np.array(path1[:,1])
                    #pxrrt = [1,2,3,4,5]
                    #pyrrt = [1,2,3,4,5]
                    return self.px, self.py
                


    def rand_new_point(self):
        # 随机生成新的点
        new_point = Node(random.uniform(self.min_x, self.max_x), random.uniform(self.min_y, self.max_y), None)
        return new_point
        

    def get_nearest_point(self, point):
        # 获取最近的节点
        min_dist = float("inf")
        nearest_point = Node(0, 0, None)
        for i in range(len(self.node_list)):
            node = self.node_list[i]
            dist = np.sqrt((node.x - point.x)**2 + (node.y - point.y)**2)
            if dist < min_dist:
                min_dist = dist
                nearest_point = node
        return nearest_point
    

    def get_theta(self, point1, point2):
        # 获取两点之间的角度
        theta = math.atan2(point2.y-point1.y, point2.x-point1.x)
        return theta

    
    def get_new_point(self, point, theta, step):
        # 获取新的点
        new_point = Node(point.x + step*math.cos(theta), point.y + step*math.sin(theta), None)
        # new_point.parent = point
        return new_point


    def check_obs_collision(self, point1, point2):
        # 检查障碍物是否与点相交
        for (ox,oy) in self.obs:
            dis = self.get_dis(point1, point2, ox, oy)
            if dis < self.radius:
                return False
        return True


    def get_dis(self, point1, point2, ox, oy):
        # 获取障碍物中心点与生成线段的距离
        if point1 == point2:
            dis = math.sqrt((point1.x - ox)**2 + (point1.y - oy)**2)
        else:
            dis = abs((point2.y-point1.y)/(point2.x-point1.x)*ox-oy-(point2.y-point1.y)/(point2.x-point1.x)*point1.x+point1.y)/math.sqrt(((point2.y-point1.y)/(point2.x-point1.x))**2+1)
        return dis

    
    def reach_goal(self, point):
        # 判断是否到达目标点
        if self.get_dis(point, point, self.target.x, self.target.y) < self.radius:
            if self.check_obs_collision(point, self.target):
                self.target.parent = point
                return True
        return False


    def get_path(self):
        pathrrt = [[self.target.x, self.target.y]]
        node = self.target
        while node.parent is not None:
            node = node.parent
            pathrrt.append([node.x, node.y])
        # pathrrt.append([self.start.x, self.start.y])
        return pathrrt