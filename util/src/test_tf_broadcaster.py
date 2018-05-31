#!/usr/bin/env python

# Mabel Zhang
# 13 Feb 2018
#
# Example usage of tf_broadcaster.py
#
# Usage
#   $ rosrun util tf_broadcaster.py
#   $ rosrun util test_tf_broadcaster.py
#   Now you should see tf frame continuously broadcasted, if you run
#   $ rosrun tf tf_echo <from_frame> <to_frame>
#
#   (optional) To see the tf frame visually
#   $ rosrun rviz rviz
#

# ROS
import rospy
import tf

# My packages
from util_msgs.msg import Transform


def main ():

  rospy.init_node ('test_tf_broadcaster', anonymous=True)

  # Choosing a good queue_size: http://wiki.ros.org/rospy/Overview/Publishers%20and%20Subscribers#Choosing_a_good_queue_size
  tf_pub = rospy.Publisher ('/tf_broadcaster/transform', Transform,
    queue_size=1)


  print ('test_tf_broadcaster running...')


  tf_msg = Transform ()

  tf_msg.parent_frame = 'world'
  tf_msg.child_frame = 'test_frame'

  tf_msg.pose.position.x = 0
  tf_msg.pose.position.y = 0
  tf_msg.pose.position.z = 1

  tf_msg.pose.orientation.x = 0
  tf_msg.pose.orientation.y = 0
  tf_msg.pose.orientation.z = 0
  tf_msg.pose.orientation.w = 1


  wait_rate = rospy.Rate (10)

  # Publish just once, and you will see that tf_broadcaster.py will broadcast
  #   continuously, even when this node doesn't publish it continuously - this
  #   is the whole point of tf_broadcaster.py, so that the node doesn't need to
  #   have a ROS loop, it can just do one call and a frame will be published
  #   all the time. This is useful for grasping goal poses.
  for i in range (5):
    tf_pub.publish (tf_msg)
    rospy.sleep (.5)


if __name__ == '__main__':

  # This enables Ctrl+C kill at any time
  try:
    main ()
  except rospy.exceptions.ROSInterruptException, err:
    pass

