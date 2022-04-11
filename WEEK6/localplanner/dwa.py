import math
import numpy as np

    


def motion(x, u, dt):
    '''
    定义机器人运动模型
    返回坐标(x,y),偏移角theta,速度v,角速度w
    x: [self.x, self.y, self.theta, self.vx, self.vw]
       0       1        2        3        4
    '''
    x[2] += u[1] * dt # theta = theta + w*t
    x[0] += u[0] * math.cos(x[2]) * dt # x = x + v*cos(theta)
    x[1] += u[0] * math.sin(x[2]) * dt # y = y + v*sin(theta)
    x[3] = u[0] # v = v
    x[4] = u[1] # w = w
    #print("running motion")
    return x



def calc_dynamic_window(x, config):
    '''
    计算动态窗口
    '''
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
    print("running calc_dynamic_window")
    return dw


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


def calc_to_goal_cost(trajectory, goal):
    '''
    朝向目标点 ：保证机器人朝目标点运动
    距离目标点评价函数
    '''
    return math.sqrt((trajectory[-1,0]-goal[0])**2+(trajectory[-1,1]-goal[1])**2)



def calc_speed_cost(config, trajectory):
    '''
    速度cost
    速度cost之间是根据得出的轨迹的速度和最大速度做个差值再乘一个速度cost的系数得出来的
    '''
    speed_cost = config.speed_cost_gain * (config.max_speed - trajectory[-1, 3])
    #print("running calc_speed_cost")
    return speed_cost



def calc_obstacle_cost(trajectory, obstacle, config):
    '''
    远离障碍物 ：保证机器人避开障碍物，安全不碰撞
    轨迹距离障碍物的评价函数
    '''
    MinDistance = float('Inf')          #初始化时候机器人周围无障碍物所以最小距离设为无穷
    for i in range(len(trajectory)):           #对每一个位置点循环
        for j in range(len(obstacle)):  #对每一个障碍物循环
            Current_Distance = math.sqrt((trajectory[i,0]-obstacle[j,0])**2+(trajectory[i,1]-obstacle[j,1])**2)  #求出每个点和每个障碍物距离
            if Current_Distance < config.robot_radius:            #如果小于机器人自身的半径那肯定撞到障碍物了返回的评价值自然为无穷
                return float('Inf')
            if Current_Distance < MinDistance:
                MinDistance=Current_Distance         #得到点和障碍物距离的最小
    #print("running calc_obstacle_cost")
    return 1/MinDistance



def plan(information, config, goal, obs):

    dw = calc_dynamic_window(information, config) #动态窗口
    for v in range(dw[0], dw[1], config.v_reso):
        for y in range(dw[2], dw[3], config.yawrate_reso):

            trajectory = predict_trajectory(information, v, y, config)
            to_goal_cost = config.to_goal_cost_gain * calc_to_goal_cost(trajectory, goal)
            speed_cost = config.speed_cost_gain * (config.max_speed - trajectory[-1, 3])
            ob_cost = config.obstacle_cost_gain * calc_obstacle_cost(trajectory, obs, config)
            min_cost = float("inf")
            all_cost = to_goal_cost + speed_cost + ob_cost
            if min_cost >= all_cost:
                min_cost = all_cost
                best_u = [v, y]

    return best_u[0], best_u[1]

    



