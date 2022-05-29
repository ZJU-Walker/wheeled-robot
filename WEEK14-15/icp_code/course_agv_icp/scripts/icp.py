#!/usr/bin/env python

import rospy
import numpy as np
import math

# structure of the nearest neighbor 
class NeighBor:
    def __init__(self):
        self.distances = []
        self.src_indices = []
        self.tar_indices = []

class ICP:
    def __init__(self):
        # max iterations
        self.max_iter = rospy.get_param('/icp/max_iter',10)
        # distance threshold for filter the matching points
        self.dis_th = rospy.get_param('/icp/dis_th',3)
        # tolerance to stop icp
        self.tolerance = rospy.get_param('/icp/tolerance',0)
        # min match
        self.min_match = rospy.get_param('/icp/min_match',2)
    
    # ICP process function
    # Waiting for Implementation 
    # return: T = (R, t), where T is 2*3, R is 2*2 and t is 2*1
    def process(self,tar_pc,src_pc):
        # ...find nearest匹配
        # ... 
        pass
        
    # find the nearest points & filter
    # return: neighbors of src and tar
    def findNearest(self,src,tar):
        # src当前
        # tar上一
        src_new = []
        tar_new = []
        if (src.shape[1] > tar.shape[1]):#src列数多于tar列数
            min = 999999999
            for i in range(tar.shape[1]):
                for j in range(src.shape[1]):

                    if (min > self.calcDist(tar[:2,i],src[:2,j])):
                        min = self.calcDist(tar[:2,i],src[:2,j])
                        src_ind = j
                src_new = np.append(src[:,src_ind],1)
            return src_new,tar
        if (tar.shape[1] > src.shape[1]):
            min = 999999999
            for i in range(src.shape[1]):
                for j in range(tar.shape[1]):                   
                    if (min > self.calcDist(src[:2,i],tar[:2,j])):
                        min = self.calcDist(src[:2,i],tar[:2,j])
                        tar_ind = j
                tar_new = np.append(tar[:,tar_ind],1)
            return src,tar_new

    # Waiting for Implementation 
    # return: T = (R, t), where T is 2*3, R is 2*2 and t is 2*1
    def getTransform(self,src,tar):
        # ...
        # ... 
        pass

    def calcDist(self,a,b):
        dx = a[0] - b[0]
        dy = a[1] - b[1]
        return math.hypot(dx,dy)
