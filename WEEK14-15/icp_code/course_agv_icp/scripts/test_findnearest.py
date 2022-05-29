from re import S
import numpy as np
import math
def findNearest(src,tar):
        # src当前
        # tar上一
        src = np.array(src[:2,:])
        tar = np.array(tar[:2,:])
        src_nan = ~np.isnan(src)
        tar_nan = ~np.isnan(tar)
        for i in range(src.shape[1]):
            if (src_nan[1,i] == False):
                src = np.delete(src,i,1)
        for i in range(tar.shape[1]):
            if (tar_nan[1,i] == False):
                tar = np.delete(tar,i,1)
        # src_new = []
        # tar_new = []
        src_new = np.array([[0],[0]])
        tar_new = np.array([[0],[0]])
        if (src.shape[1] >= tar.shape[1]):#src列数多于tar列数
            for i in range(tar.shape[1]):
                min = 999999999
                for j in range(src.shape[1]):
                    if (min > calcDist(tar[:,i],src[:,j])):
                        min = calcDist(tar[:,i],src[:,j])
                        src_ind = j
                print(src[:,src_ind].reshape(2,1))
                src_new = np.append(src_new,src[:,src_ind].reshape(2,1),axis = 1)
            src_new = np.delete(src_new,0,1)
            return src_new,tar
        if (tar.shape[1] > src.shape[1]):
            for i in range(src.shape[1]):
                min = 999999999
                for j in range(tar.shape[1]):                   
                    if (min > calcDist(src[:,i],tar[:,j])):
                        min = calcDist(src[:,i],tar[:,j])
                        tar_ind = j

                tar_new = np.append(tar[:,tar_ind].T,1)
            tar_new = np.delete(tar_new,0,1)
            return src,tar_new
def calcDist(a,b):
        dx = a[0] - b[0]
        dy = a[1] - b[1]
        return math.hypot(dx,dy)

if __name__ == '__main__':
    src = np.array([[np.nan,np.nan,1],[0,0,1],[1,1,1],[2,200,1],[100,2,1],[2,4,1]]).T
    tar = np.array([[1,1,1],[0,0,1],[2,3,1]]).T
    src_new,tar_new = findNearest(src,tar)
    print(src)
    print(src_new)

