#!/usr/bin/env python
import rospy
from sensor_msgs.msg import JointState

from markers import *
from irb4400_functions import *

if __name__ == '__main__':

  rospy.init_node("jointsNode")
  pub = rospy.Publisher('joint_states', JointState, queue_size=1)
  bmarker = BallMarker(color['GREEN'])

  # Joint names
  jnames = ("joint_1", "joint_2", "joint_3","joint_4", "joint_4n", "joint_5", "joint_6")
  # Joint Configuration
  q = [1,0.3,-0.5,-1,0.1,1.8,4.2]

  # End effector with respect to the base
  T = fkine_irb(q)

  print( np.round(T, 3) )
  bmarker.position(T)

  # Object (message) whose type is JointState
  jstate = JointState()
  # Set values to the message
  jstate.header.stamp = rospy.Time.now()
  jstate.name = jnames
  # Add the head joint value (with value 0) to the joints
  jstate.position = q

  # Loop rate (in Hz)
  rate = rospy.Rate(20)
  # Continuous execution loop
  while not rospy.is_shutdown():
    # Current time (needed for ROS)
    jstate.header.stamp = rospy.Time.now()
    # Publish the message
    pub.publish(jstate)
    bmarker.publish()
    # Wait for the next iteration
    rate.sleep()