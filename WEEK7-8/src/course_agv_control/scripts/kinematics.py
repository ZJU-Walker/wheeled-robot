#!/usr/bin/env python3
import rospy
import geometry_msgs.msg
import std_msgs.msg
from std_msgs.msg import Float64

class Kinematics:
    
    left_wheel_vel = 0.0
    right_wheel_vel = 0.0

    r = 0.08
    l = 0.2

    def __init__(self):
        self.sub = rospy.Subscriber('/course_agv/velocity',geometry_msgs.msg.Twist,self.callback)
        self.pubL = rospy.Publisher('/course_agv/left_wheel_velocity_controller/command',Float64, queue_size=10)
        self.pubR = rospy.Publisher('/course_agv/right_wheel_velocity_controller/command',Float64, queue_size=10) 
        # call rospy.Subscriber() and rospy.Publisher() to define receiver and publisher
        # type: geometry_msgs.msg.Twist, Float64, Float64
        #pass

    # input: 
    #   data: 
    #       (data.linear.x, data.angular.z)
    # call self.kinematics(vx, vw) to compute wheel velocities

    def callback(self, data):
        self.left_wheel_vel,self.right_wheel_vel = self.kinematics(data.linear.x,data.angular.z)
        # Compute self.left_wheel_vel and self.right_wheel_vel according to input
        pass

    # input: vx and vw of the robot
    # output: wheel velocities of two wheels
    def kinematics(self,vx,vw):
        left_wheel_vel = (2*vx-self.l*vw)/2
        right_wheel_vel = (self.l*vw+2*vx)/2
        return left_wheel_vel,right_wheel_vel
        pass

    # Call publisher to publish wheel velocities
    def publish(self):
        self.pubL.publish(self.left_wheel_vel)
        self.pubR.publish(self.right_wheel_vel)
        pass

def main():
    node_name = "course_agv_kinematics"
    print("node : ",node_name)
    try:
        rospy.init_node(node_name)
        k = Kinematics()
        rate = rospy.Rate(rospy.get_param('~publish_rate',200))
        while not rospy.is_shutdown():
            k.publish()
            rate.sleep()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
