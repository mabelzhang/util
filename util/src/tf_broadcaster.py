#!/usr/bin/env python

# Mabel Zhang
# 10 Nov 2015
#
# Broadcast a given tf frame non-stop
#
# Usage:
#   $ rosrun util tf_broadcaster.py
#   To run multiple instances of this node, run this from launch file and remap
#     the topic to different ones.
#   Publish the desired transform that is to be constantly broadcasted, to
#     the rostopic that tfBroadcaster subscribes to.
#

# ROS
import rospy
import tf

# My packages
from util_msgs.msg import Transform


class tfBroadcaster:

  def __init__ (self):

    self.bc = tf.TransformBroadcaster ()

    rospy.Subscriber ('/tf_broadcaster/transform', Transform,
      self.transformCB)

    self.transform = None


  def transformCB (self, msg):

    print ('tf_broadcaster received tranform from %s to %s' % (
      msg.parent_frame, msg.child_frame))

    self.transform = msg


  def broadcast (self):

    if not self.transform:
      return


    # http://mirror.umd.edu/roswiki/doc/diamondback/api/tf/html/python/tf_python.html
    self.bc.sendTransform ( \
      (self.transform.pose.position.x,
       self.transform.pose.position.y, 
       self.transform.pose.position.z), 
      (self.transform.pose.orientation.x,
       self.transform.pose.orientation.y,
       self.transform.pose.orientation.z,
       self.transform.pose.orientation.w),
      rospy.Time.now (),
      self.transform.child_frame, self.transform.parent_frame)


def main ():

  rospy.init_node ('tf_broadcaster_node', anonymous=True)

  print ('tf_broadcaster running...')

  thisNode = tfBroadcaster ()

  wait_rate = rospy.Rate (30)

  while not rospy.is_shutdown ():

    thisNode.broadcast ()

    wait_rate.sleep ()


if __name__ == '__main__':

  # This enables Ctrl+C kill at any time
  try:
    main ()
  except rospy.exceptions.ROSInterruptException, err:
    pass

