#!/usr/bin/env python

# Mabel Zhang
# 21 Sep 2018
#
# Euler rotation utility functions
# Used in depth_scene_rendering to run in Blender Python.
#


import numpy as np

# Local
# NOTE that quaternions are ordered (w, x, y, z), UNLIKE the version in ROS
#   tf.transformations, which is ordered (x, y, z, w)!
from util.tf_transformations import quaternion_from_euler, quaternion_matrix, \
  euler_matrix


def get_rand_rot (x_range=(0, 2*np.pi), y_range=(0, 2*np.pi),
  z_range=(0, 2*np.pi), axes='sxyz', qwFirst=False):

  rx = x_range[0] + np.random.rand () * (x_range[1] - x_range[0])
  ry = y_range[0] + np.random.rand () * (y_range[1] - y_range[0])
  rz = z_range[0] + np.random.rand () * (z_range[1] - z_range[0])

  # Make a quaternion (qw qx qy qz) out of rx, ry ,rz
  q = quaternion_from_euler (rx, ry, rz, axes=axes)

  #print (euler_matrix (rx, ry, rz, axes))
  #print (quaternion_matrix (q))

  if qwFirst:
    quat = q
  else:
    # Reorder to (qx qy qz qw)
    quat = (q[1], q[2], q[3], q[0])

  return [rx, ry, rz], quat

