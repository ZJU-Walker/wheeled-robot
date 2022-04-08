import threading
import time
import math
import matplotlib
import matplotlib.pyplot as plt
import numpy as np

from planner import RRTplanner
from localplanner import dwa

plt.rcParams["figure.figsize"] = [12.0,12.0]
plt.rcParams["figure.autolayout"] = True
plt.rcParams['keymap.save'].remove('s')

def transformation_matrix(x, y, theta):
    return np.array([
        [np.cos(theta), -np.sin(theta), x],
        [np.sin(theta), np.cos(theta), y],
        [0, 0, 1]
    ])

class DWAConfig:
    robot_radius = 0.3
    def __init__(self,obs_radius):
        self.obs_radius = obs_radius
        self.dt = 0.1  # [s] Time tick for motion prediction

        self.max_speed = 1.5  # [m/s]
        self.min_speed = -1.5  # [m/s]
        self.max_accel = 3  # [m/ss]
        self.v_reso = self.max_accel*self.dt/5.0  # [m/s]

        self.max_yawrate = 360.0 * math.pi / 180.0  # [rad/s]
        self.max_dyawrate = 360.0 * math.pi / 180.0  # [rad/ss]
        self.yawrate_reso = self.max_dyawrate*self.dt/6.0  # [rad/s]

        self.predict_time = 1.5  # [s]

        self.to_goal_cost_gain = 1.5
        self.speed_cost_gain = 0.2
        self.obstacle_cost_gain = 1.0

        self.tracking_dist = self.predict_time*self.max_speed
        self.arrive_dist = 0.1

class Playground:
    planning_obs_radius = 0.5
    def __init__(self,planner=None,vplanner=None):
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.vx = 0.0
        self.vw = 0.0
        self.x_traj = []
        self.y_traj = []

        self.dwaconfig = DWAConfig(self.planning_obs_radius)
        self.dt = self.dwaconfig.dt

        self.fig, self.ax = plt.subplots()

        self.fig.canvas.mpl_connect('button_press_event', self.on_mousepress)
        self.fig.canvas.mpl_connect('key_press_event', self.on_press)
        self.fig.canvas.mpl_connect('motion_notify_event',self.on_mousemove)
        self.NEED_EXIT = False

        ############################################

        self.planning_obs = np.empty(shape=(0,2))
        
        self.planning_path = np.empty(shape=(0,2))

        self.planning_target = None

        self.planner = planner
        self.vplanner = vplanner

        self.vplanner_midpos_index = None

        #####################################
        self.temp_obs = [0,0]
    def run(self):
        while True:
            if self.NEED_EXIT:
                plt.close("all")
                break
            self.vplanner_midpos_index = self.check_path()
            all_u = []
            all_traj = []
            best_traj = None
            if self.vplanner_midpos_index >= 0:
                midpos = self.planning_path[self.vplanner_midpos_index]
                [self.vx,self.vw],best_traj,all_traj,all_u = dwa.plan([self.x,self.y,self.theta,self.vx,self.vw],self.dwaconfig,midpos,self.planning_obs)
            else:
                self.vx,self.vw = 0.0,0.0
            
            dx,dy,dw = self.vx*self.dt, 0, self.vw*self.dt
            T = transformation_matrix(self.x, self.y, self.theta)
            p = np.matmul(T,np.array([dx,dy,1]))
            self.x = p[0]
            self.y = p[1]
            self.theta += dw
            self.x_traj.append(self.x)
            self.y_traj.append(self.y)

            plt.cla()
            self.__draw(best_traj,all_traj,all_u)
    def check_path(self):
        if self.planning_path is None or self.planning_path.shape[0] == 0:
            return -1
        
        if self.vplanner_midpos_index is not None and self.vplanner_midpos_index >=0:
            midindex = self.vplanner_midpos_index
            while True:
                midpos = self.planning_path[midindex]
                dist = np.hypot(self.x-midpos[0],self.y-midpos[1])
                if dist > self.dwaconfig.tracking_dist:
                    break
                if midindex + 1 == self.planning_path.shape[0]:
                    return midindex
                midindex += 1
            return midindex
        else:
            return 0
    def add_obs(self,x,y):
        self.planning_obs = np.append(self.planning_obs,[[x,y]],axis=0)
    def add_obss(self,xs,ys):
        self.planning_obs = np.append(self.planning_obs,np.vstack([xs,ys]).T,axis=0)
    def __draw(self,best_traj,all_traj,all_value):

        p1_i = np.array([0.5, 0, 1]).T
        p2_i = np.array([-0.5, 0.25, 1]).T
        p3_i = np.array([-0.5, -0.25, 1]).T

        T = transformation_matrix(self.x, self.y, self.theta)
        p1 = np.matmul(T, p1_i)
        p2 = np.matmul(T, p2_i)
        p3 = np.matmul(T, p3_i)

        plt.plot([p1[0], p2[0], p3[0], p1[0]], [p1[1], p2[1], p3[1], p1[1]], 'k-')

        if self.planning_target is not None:
            self.ax.plot(self.planning_target[0],self.planning_target[1],"r*",markersize=20)

        if len(all_traj) > 0:
            all_value = np.array(all_value,dtype=float)
            all_value = (all_value-all_value.min())/(all_value.max()-all_value.min())
            for i,traj in enumerate(all_traj):
                color = plt.cm.jet(all_value[i])
                self.ax.plot(traj[:,0],traj[:,1],".",color=color,markersize=1)
                self.ax.plot(traj[-1,0],traj[-1,1],"+",color=color,markersize=3)
        if best_traj is not None:
            self.ax.plot(best_traj[:,0],best_traj[:,1],color="green",linewidth=3)
        if self.planning_path is not None:
            self.ax.plot(self.planning_path[:,0], self.planning_path[:,1], 'b--')
            if self.vplanner_midpos_index is not None and self.vplanner_midpos_index >= 0:
                midpos = self.planning_path[self.vplanner_midpos_index]
                self.ax.plot(midpos[0],midpos[1],"g*",markersize=20)
        if len(self.x_traj) > 0:
            plt.plot(self.x_traj, self.y_traj, 'g-')
        for obs in self.planning_obs:
            self.ax.add_artist(plt.Circle((obs[0],obs[1]), self.planning_obs_radius,fill=False))

        self.ax.set_xlim(-5, 5)
        self.ax.set_ylim(-5, 5)

        plt.pause(0.01)

    def on_mousepress(self,event):
        if not event.dblclick:
            if event.button == 1:
                self.x,self.y = event.xdata,event.ydata
            if event.button == 3:
                self.planning_target = np.array([event.xdata,event.ydata])
            if event.button == 2:
                self.add_obs(event.xdata,event.ydata)
                self.temp_obs = [event.xdata,event.ydata]
    def on_mousemove(self,event):
        if hasattr(event,"button") and event.button == 2:
            dx = event.xdata-self.temp_obs[0]
            dy = event.ydata-self.temp_obs[1]
            if np.hypot(dx,dy) > self.planning_obs_radius*0.8:
                self.temp_obs = [event.xdata,event.ydata]
                self.add_obs(*self.temp_obs)
    def on_press(self,event):
        if(event.key == 'escape'):
            self.set_exit()
        if(event.key == ' '):
            if self.planning_target is not None and self.planner is not None:
                print("do planning...")
                self.planning_path = None
                self.x_traj,self.y_traj = [],[]
                self.vplanner_midpos_index = None
                px,py = planner.planning(self.planning_obs[:,0],self.planning_obs[:,1],self.x,self.y,self.planning_target[0],self.planning_target[1],-10,-10,10,10)
                px,py = px.tolist(),py.tolist()
                px.reverse(),py.reverse()
                self.planning_path = np.vstack([px,py]).T
                print("pathLength : ",self.planning_path.shape[0])

    def set_exit(self):
        self.NEED_EXIT = True

if __name__ == "__main__":
    # planner = None
    # planner = AStarPlanner(0.2,Playground.planning_obs_radius+DWAConfig.robot_radius)
    planner = RRTplanner.RRTPlanner(0.2)
    # planner = RRTPlanner(0.2,Playground.planning_obs_radius+DWAConfig.robot_radius)
    pg = Playground(planner)
    pg.run()