#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64

if __name__ == "__main__":

    rospy.init_node("sendJointsGzNode")
    topic1 = '/robot/joint1_position_controller/command'
    topic2 = '/robot/joint2_position_controller/command'
    topic3 = '/robot/joint3_position_controller/command'
    topic4 = '/robot/joint4_position_controller/command'
    topic4n = '/robot/joint4n_position_controller/command'
    topic5 = '/robot/joint5_position_controller/command'
    topic6 = '/robot/joint6_position_controller/command'

    pub1 = rospy.Publisher(topic1, Float64, queue_size=10, latch=True)
    pub2 = rospy.Publisher(topic2, Float64, queue_size=10, latch=True)
    pub3 = rospy.Publisher(topic3, Float64, queue_size=10, latch=True)
    pub4 = rospy.Publisher(topic4, Float64, queue_size=10, latch=True)
    pub4n = rospy.Publisher(topic4n, Float64, queue_size=10, latch=True)
    pub5 = rospy.Publisher(topic5, Float64, queue_size=10, latch=True)
    pub6 = rospy.Publisher(topic6, Float64, queue_size=10, latch=True)
    
    j1 = Float64()
    j2 = Float64()
    j3 = Float64()
    j4 = Float64()
    j4n = Float64()
    j5 = Float64()
    j6 = Float64()

    j1.data = 0.5
    j2.data = 0.5
    j3.data = 0.5
    j4.data = 0.5
    j4n.data = 0.5
    j5.data = 0.5
    j6.data = 0.5


    pub1.publish(j1)
    pub2.publish(j2)
    pub3.publish(j3)
    pub4.publish(j4)
    pub4n.publish(j4n)
    pub5.publish(j5)
    pub6.publish(j6)

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        rate.sleep()