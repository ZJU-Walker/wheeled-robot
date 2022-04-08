import math
import numpy as np
'''
now_condition: [self.x,self.y,self.theta,self.vx,self.vw]
dwaconfig: DWA config
midpos: [x,y]
planning_obs: [[x,y],[x,y],...]
'''
    ############################################
    

'''
定义机器人运动模型
返回坐标(x,y),偏移角theta,速度v,角速度w
'''
def motion(x, u, dt):
    

    x[2] += u[1] * dt # v=v+a*t
    x[0] += u[0] * math.cos(x[2]) * dt # x=x+v*cos(theta)
    x[1] += u[0] * math.sin(x[2]) * dt # y=y+v*sin(theta)
    x[3] = u[0]
    x[4] = u[1]

    return x


'''
计算动态窗口
'''
def calc_dynamic_window(x, config):
    """
    calculation dynamic window based on current state x
    """

    # Dynamic window from robot specification
    Vs = [config.min_speed, config.max_speed,
          -config.max_yawrate, config.max_yawrate]

    # Dynamic window from motion model
    Vd = [x[3] - config.max_accel * config.dt,
          x[3] + config.max_accel * config.dt,
          x[4] - config.max_dyawrate * config.dt,
          x[4] + config.max_dyawrate * config.dt]

    #  [v_min, v_max, yaw_rate_min, yaw_rate_max]
    dw = [max(Vs[0], Vd[0]), min(Vs[1], Vd[1]),
          max(Vs[2], Vd[2]), min(Vs[3], Vd[3])]

    return dw


'''
采样轨迹
'''
# evaluate all trajectory with sampled input in dynamic window
# for v in np.arange(dw[0], dw[1], config.v_resolution):
#     for y in np.arange(dw[2], dw[3], config.yaw_rate_resolution):
#         trajectory = predict_trajectory(x_init, v, y, config)
def predict_trajectory(x_init, v, y, config):
    """
    predict trajectory with an input
    """

    x = np.array(x_init)
    trajectory = np.array(x)
    time = 0
    while time <= config.predict_time:
        x = motion(x, [v, y], config.dt)
        trajectory = np.vstack((trajectory, x))
        time += config.dt

    return trajectory


'''
朝向目标点 ：保证机器人朝目标点运动
距离目标点评价函数
'''
def calc_to_goal_cost(trajectory, goal):
    """
        calc to goal cost with angle difference
    """

    dx = goal[0] - trajectory[-1, 0]
    dy = goal[1] - trajectory[-1, 1]
    error_angle = math.atan2(dy, dx)
    cost_angle = error_angle - trajectory[-1, 2]
    cost = abs(math.atan2(math.sin(cost_angle), math.cos(cost_angle)))

    return cost


'''
速度cost
速度cost之间是根据得出的轨迹的速度和最大速度做个差值再乘一个速度cost的系数得出来的
'''
def calc_speed_cost(config, trajectory):
    speed_cost = config.speed_cost_gain * (config.max_speed - trajectory[-1, 3])
    return speed_cost


'''
远离障碍物 ：保证机器人避开障碍物，安全不碰撞
轨迹距离障碍物的评价函数
'''
def calc_obstacle_cost(trajectory, obstacle, config):
    """
    calc obstacle cost inf: collision
    """
    MinDistance = float('Inf')          #初始化时候机器人周围无障碍物所以最小距离设为无穷
    for i in range(len(trajectory)):           #对每一个位置点循环
        for j in range(len(obstacle)):  #对每一个障碍物循环
            Current_Distance = math.sqrt((trajectory[i,0]-obstacle[j,0])**2+(trajectory[i,1]-obstacle[j,1])**2)  #求出每个点和每个障碍物距离
            if Current_Distance < config.robot_radius:            #如果小于机器人自身的半径那肯定撞到障碍物了返回的评价值自然为无穷
                return float('Inf')
            if Current_Distance < MinDistance:
                MinDistance=Current_Distance         #得到点和障碍物距离的最小

    return 1/MinDistance


def check_if_goaled(x, goal, config):
    """
    check if robot reached goal
    """
    distance = math.sqrt((x[0] - goal[0])**2 + (x[1] - goal[1])**2)
    if distance <= config.robot_radius:
        return True
    else:
        return False




def dwa_core(X, u, config, goal, obstacles):
    dw = calc_dynamic_window(X, config)   
    best_traj = np.array(X)
    min_score = 100000
    for v in np.arange(dw[0], dw[1], config.v_reso):
        for y in np.arange(dw[2], dw[3], config.yawrate_reso):
            trajectory = predict_trajectory(X, v, y, config)
            to_goal_cost = config.to_goal_cost_gain * calc_to_goal_cost(trajectory, goal)
            speed_cost = config.speed_cost_gain * (config.max_speed - trajectory[-1, 3])
            ob_cost = config.obstacle_cost_gain * calc_obstacle_cost(trajectory, obstacles, config)
            score = to_goal_cost + speed_cost + ob_cost
            if min_score >= score:                    #得出最优评分和轨迹
                min_score = score
                u = np.array([v, y])
                best_traj = trajectory
    return best_traj, u


def plan(now_condition, config, midpos, planning_obs):
    x = now_condition
    u = np.array([now_condition[3], now_condition[4]])
    goal = midpos
    all_traj = np.array(x)
    all_u = u
    while check_if_goaled(x, goal, config):
        best_traj, u = dwa_core(x, u, config, goal, planning_obs)
        x = motion(x, u, config.dt)
        all_traj = np.vstack((all_traj,x))  
        all_u = np.vstack(all_u, u)
    return u, best_traj, all_traj, all_u
    #trajectory = predict_trajectory(now_condition, )
    # calc cost
    #to_goal_cost = config.to_goal_cost_gain * calc_to_goal_cost(trajectory, goal)
    #speed_cost = config.speed_cost_gain * (config.max_speed - trajectory[-1, 3])
    #ob_cost = config.obstacle_cost_gain * calc_obstacle_cost(trajectory, ob, config)