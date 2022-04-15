#!/usr/bin/python3

import numpy as np
import math


        
class dwa:
    def __init__(self):
        self.init = None
            

    def motion_model(self, information, velocity_arr, dt):
        information[0] += velocity_arr[0] * dt * math.cos(information[2])
        information[1] += velocity_arr[0] * dt * math.sin(information[2])
        information[2] += velocity_arr[1] * dt
        information[3] = velocity_arr[0]
        information[4] = velocity_arr[1]

        return information

    def velocity_generate(self, information, config):
        vinfo = [config.min_speed, config.max_speed, -config.max_yawrate, config.max_yawrate]

        vmove = [information[3] - config.max_accel * config.dt, 
                 information[3] + config.max_accel * config.dt,
                 information[4] - config.max_dyawrate * config.dt,
                 information[4] + config.max_dyawrate * config.dt]

        vw = [max(vinfo[0], vmove[0]), min(vinfo[1], vmove[1]), 
              max(vinfo[2], vmove[2]), min(vinfo[3], vmove[3])]

        return vw        

    def traj_calculate(self, information, config, velocity_arr):
        ctraj = np.array(information)
        new = np.array(information)
        time = 0

        while time <= config.predict_time:
            new = self.motion_model(new, velocity_arr, config.dt)
            ctraj = np.vstack((ctraj, new))
            time += config.dt

        return ctraj

    ## 评估函数，三个，加入权重
    def calculate_obstacle_cost(self, traj, obs, config):
        min_r = float("Inf")

        for i in range(0, len(traj[:, 1]), 2):
            for j in range(len(obs[:, 0])):
                ox = obs[j, 0]
                oy = obs[j, 1]
                dx = traj[i, 0] - ox
                dy = traj[i, 1] - oy

                r = math.sqrt(dx ** 2 + dy ** 2)
                if (r <= config.robot_radius + config.obs_radius):
                    return float("Inf")

                if min_r >= r:
                    min_r = r
                
        return 1.0 / min_r
    
    def calculate_goal_cost(self, traj, goal):
        cost = math.sqrt((traj[-1, 0] - goal[0]) ** 2 + (traj[-1, 1] - goal[1]) ** 2)

        return cost

    def calculate_velocity_cost(self, traj, config):
        cost = config.max_speed - traj[-1, 3]
        return cost

    def plan(self, information, config, midpos, obs):
        # information:[x, y, theta, vx, vw]
        vw = self.velocity_generate(information, config)
        best_ctraj = np.array(information)
        min_evaluation = 10000.0

        for v in np.arange(vw[0], vw[1], config.v_reso):
            for w in np.arange(vw[2], vw[3], config.yawrate_reso):
                ctraj = self.traj_calculate(information, config, [v, w])
                goal_score = config.to_goal_cost_gain * self.calculate_goal_cost(ctraj, midpos)
                vel_score = config.speed_cost_gain * self.calculate_velocity_cost(ctraj, config)
                traj_score = config.obstacle_cost_gain * self.calculate_obstacle_cost(ctraj, obs, config)

                ctraj_score = goal_score + vel_score + traj_score

                if min_evaluation >= ctraj_score:
                    min_evaluation = ctraj_score
                    u = np.array([v, w])
                    best_ctraj = ctraj

        return u[0], u[1]