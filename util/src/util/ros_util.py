#!/usr/bin/env python

# Mabel Zhang
# 22 Jun 2017
#
# Utility functions for ROS stuff.
#


import rospy
import tf
from geometry_msgs.msg import Pose, PoseStamped, Point
from std_srvs.srv import Empty, Trigger, TriggerRequest

import numpy as np


def call_empty_rossrv (srv_name):

  print ('Waiting for rosservice %s' % srv_name)
  rospy.wait_for_service (srv_name)
  try:
    print ('Calling %s...' % srv_name)
    srv = rospy.ServiceProxy (srv_name, Empty)
    resp = srv ()
  except rospy.ServiceException, e:
    err_msg = 'ERROR: Call to rosservice %s failed' % srv_name
    print (err_msg)
    return False

  return True


# Copied from robotiq_takktile test_robotiq_control.py
# Returns (success, err_msg)
def call_trigger_rossrv (srv_name):

  print ('Waiting for rosservice %s' % srv_name)
  rospy.wait_for_service (srv_name)
  try:
    print ('Calling %s...' % srv_name)
    srv = rospy.ServiceProxy (srv_name, Trigger)
    req = TriggerRequest ()
    resp = srv (req)
  except rospy.ServiceException, e:
    err_msg = 'ERROR: Call to rosservice %s failed' % srv_name
    print (err_msg)
    return False, err_msg

  return resp.success, resp.message

  #if not resp.success:
  #  print ('ERROR: rosservice %s returned unsuccessful execution. Message: %s' % (
  #    srv_name, resp.message))


# Parameters:
#   msg: geometry_msgs/Point, or geometry_msgs/Vector3
def array_from_Point (msg):

  return np.array ([msg.x, msg.y, msg.z])


def Point_from_array (v):

  return Point (v[0], v[1], v[2])


# Returns 4 x 4 NumPy-matrix-equivalent of a geometry_msgs/Pose message
# Parameters:
#   msg: geometry_msgs/Pose
def matrix_from_Pose (msg):

  # R. 4 x 4 matrix
  mat = tf.transformations.quaternion_matrix ([msg.orientation.x, msg.orientation.y,
    msg.orientation.z, msg.orientation.w])

  # t. Last column
  mat [0:3, 3] = [msg.position.x, msg.position.y, msg.position.z]

  return mat


# Returns geometry_msgs/Pose message corresponding to a 4 x 4 matrix.
# Parameters:
#   mat: 4 x 4 NumPy array
def Pose_from_matrix (mat):

  msg = Pose ()

  # Last column
  msg.position.x = mat [0, 3]
  msg.position.y = mat [1, 3]
  msg.position.z = mat [2, 3]

  quat = tf.transformations.quaternion_from_matrix (mat)
  msg.orientation.x = quat [0]
  msg.orientation.y = quat [1]
  msg.orientation.z = quat [2]
  msg.orientation.w = quat [3]

  return msg


# Returns 4 x 4 NumPy-matrix-equivalent of a 7-tuple transformation
# Parameters:
#   tq: 7-tuple, (tx ty tz qx qy qz qw)
def matrix_from_7tuple (tq):

  mat = tf.transformations.quaternion_matrix ([tq[3], tq[4], tq[5], tq[6]])
  mat [0:3, 3] = [tq[0], tq[1], tq[2]]

  return mat


# Package a geometry_msgs/Pose into geometry_msgs/PoseStamped
def stamp_pose (pose, frame_id):

  pose_st = PoseStamped ()

  pose_st.pose = pose

  pose_st.header.frame_id = frame_id
  pose_st.header.stamp = rospy.Time.now ()

  return pose_st

