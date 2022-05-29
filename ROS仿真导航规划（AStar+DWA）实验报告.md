# ROS仿真导航规划（AStar+DWA）实验报告

姓名：王柯

学号：3200103911

(WEEK7-8的仿真还没验收，视频和代码已经钉钉发给助教了，麻烦助教验收一下)

## WEEK5: AStar Path Planning

### 算法描述

AStar算法是一种很常用的路径查找和图形遍历算法。它有较好的性能和准确度。算法通过下面这个函数来计算每个节点的优先级：
$$
f(n)=g(n)+h(n)
$$
其中：

- f(n) 是节点n的综合优先级。当我们选择下一个要遍历的节点时，我们总会选取综合优先级最高（值最小）的节点。
- g(n) 是节点n距离起点的代价。
- h(n) 是节点n距离终点的预计代价。

这里使用地图上的点对起点和终点的欧式距离作为g(n), h(n)。

算法伪代码如下：

```
Algorithm: AStar Path Planning
add start node "s" to openlist
loop
if openlist is empty
	return
else
	remove the node "n" with the lowest f(n) from the openlist
	mark "n" as expanded
	if n == goal
		find a path 
		return
	else
		for all unexpanded neighbors "m" of node "n"
        	if dist(m,s) > dist(m,n) + dist(n,s)
        		dist(m,s) = dist(m,n) + dist(n,s)
        		add m to openlist
        end
end
```

### 遇到问题

在编写pathplanning过程中，首先需要实现地图的栅格化，将地图分为很多小方块，并将障碍物信息反映到小方块上。因此，首先我先将地图分成小方格，具体数目由分辨率决定，然后取出每个方格的中心点坐标，作为这个方格的坐标，然后计算每个方格中心点距离障碍物的距离，如果距离小于障碍物半径，则视作这个方格不可到达，由此，完成地图的栅格化，后续操作将只对这个处理后的地图进行。

### 实现效果

![AStar_Result](/home/wangke/github/wheeled-robot/WEEK5/AStar_Result.png)



## WEEK6: DWA

### 算法描述

**dynamic window approach**(DWA)，通过对速度进行采样，在一定的速度窗口中，取一系列v,w的值，根据机器人运动模型，假设他们将维持这个速度运动一段时间，则不同的（v,w）会得到不同的运动半径，有些会与障碍物碰撞，有些不会，因此我们定义一个评估函数：
$$
cost = \alpha \times obs\_cost + \beta \times vel\_cost + \gamma \times heading
$$
其中，obs_cost为距离障碍物的评价指标，vel_cost为速度评价指标，heading·为距离目标点的评价指标。在这里，obs_cost为距离周围障碍物最小距离倒数，vel_cost为当前采样速度与最大速度之差，heading为距离目标点距离。dwa在运行时不断对速度进行取样，每次都选择cost最小的（v,w），返回（v,w）。

### 遇到问题

- 利用路径规划得到的path的list中是终点到起点，因此一开始dwa的目标点就设在了终点，与预期不符。
- 函数接口问题，接口设置不对，程序运行失败。
- 系数设置不当，无法实现有效避障。

### 实现效果

![dwa_result](/home/wangke/Desktop/WEEK6-DWA/dwa_result.png)



## WEEK7-8: gazebo+rviz+AStar+dwa

在前几周的基础上，在仿真中实现路径规划+局部路径规划实现自主导航

### 遇到问题

问题集中在localpanner中，即dwa算法

- 评价函数三项系数设置不合理，导致仿真模拟时小车速度过快无法及时减速，或速度过慢。
- 设置最大速度与加速度的值不合理，小车无法及时减速或改变方向。

### 实现效果

![Screenshot from 2022-04-29 15-36-33](/home/wangke/Pictures/Screenshot from 2022-04-29 15-36-33.png)









