#!/usr/bin/env python

# Mabel Zhang
# 14 Feb 2016
#
# Utility functions for quaternion operations
#

import tf

import numpy as np

from util.ansi_colors import ansi_colors


# Returns the quaternion of q_goal wrt q_wrt.
# Useful when need to set a goal pose for movement, e.g. q_wrt is orientation
#   of robot frame, q_goal is orientation of goal pose, and you want to get
#   goal orientation wrt robot frame's orientation.
# Both must be in the same frame, e.g. world frame.
# It is possible that robot frame is same as world frame, that's fine. We just
#   need both rotations to be expressed in the same frame to start with,
#   otherwise they don't even make sense, can't do calculations with them.
#   e.g. q_goal can be a vector you obtained by subtracting some point outside
#   of object an object by the center point of the object, then q_goal is in
#   world frame. You should express q_wrt in the same frame. If your robot
#   frame itself defines the world frame, that's fine, just pass in 0 0 0 1.
# Parameters:
#   q_wrt, q_goal: 4-tuples. Quaternions
def get_relative_rotation_q (q_wrt, q_goal):

  # Let a = q_wrt, b = q_goal.
  # q^w_a.inv() * q^w_b = q^a_w * q^w_b. w cancels out. = q^a_b, which is
  #   rotation of b wrt a.
  return (tf.transformations.quaternion_inverse (q_wrt) * q_goal)


# Rotation to make vector v_wrt look like vector v_goal
# Returns a quaternion (4-elt numpy array) that rotates (phyiscally moves)
#   v_wrt onto v_goal.
# Parameters:
#   v_wrt_orig, v_goal_orig: 3-tuples. Vectors.
def get_relative_rotation_v (v_wrt_orig, v_goal_orig):

  # Given two vectors, two lines define a plane. Then the cross product of the
  #   two vectors gives you the vector perpendicular to both. This is the 
  #   axis to rotate by.
  # The angle to rotate by is given by the angle difference btw the two
  #   vectors, which can be found using dot product.
  # Now the only remaining problem is sign.


  # This is the way I like the best, hard to find, had to click here from
  #   within stackexchange. Vastly superior than the top result from Google,
  #   which gives rotation matrices with columns like [1 0 2], wtF.
  # Ref: http://math.stackexchange.com/questions/1246679/expression-of-rotation-matrix-from-two-vectors?rq=1

  # Normalize. Must normalize, otherwise matrices using these as basis vectors
  #   won't be orthogonal!
  v_wrt = v_wrt_orig / np.linalg.norm (v_wrt_orig)
  v_goal = v_goal_orig / np.linalg.norm (v_goal_orig)
  #print ('a: [%g %g %g]' % (v_wrt[0], v_wrt[1], v_wrt[2]))
  #print ('b: [%g %g %g]' % (v_goal[0], v_goal[1], v_goal[2]))

  # If the two are parallel, cross product will be a vector of all 0's! Then
  #   dividing by it to normalize will result in nan's being returned.
  # Return identity or -180 instead, to indicate the two vectors were parallel.
  # If the two are parallel and in same direction:
  if np.all (np.abs (v_wrt - v_goal) < 1e-6):
    print ('%sWARN: quat_utils.py get_relative_rotation_v(): Two vectors are same. Returning identity!%s' % (
      ansi_colors.WARNING, ansi_colors.ENDC))
    return [0, 0, 0, 1], np.eye (4)

  # If the two are parallel and in opposite direction:
  elif np.all (np.abs (-v_wrt - v_goal) < 1e-6):
    # From two parallel vectors, cannot get a vector perpendicular to both, `.`
    #   there are infinite planes normal to both. The rotation from one vector
    #   to the other is simply 180 degs wrt any vector perpendicular to both!

    print ('%sWARN: quat_utils.py get_relative_rotation_v(): Two vectors are parallel and in opposite directions. Returning a 180-deg rotation wrt an arbitrary vector!%s' % (
      ansi_colors.WARNING, ansi_colors.ENDC))

    # Don't use a random vector, need predictable behavior in caller.
    arb_v = np.array ([1., 0., 0.])
    # If the arbitrary vector is parallel to v_goal, choose another one
    if np.dot (arb_v, v_goal) - 1 < 1e-6 or \
       np.dot (arb_v, v_goal) + 1 < 1e-6:
      arb_v = np.array ([0., 1., 0.])
    arb_n = np.cross (arb_v, v_goal)

    rot_quat = tf.transformations.quaternion_about_axis (np.pi, arb_n)
    rot_mat = tf.transformations.quaternion_matrix (rot_quat)

    # Note this rotation is not the unique answer, there are infinite axes to
    #   rotate 180 wrt, to get the rotation btw the two parallel vectors
    #   passed in!
    return rot_quat, rot_mat

  cross_prod = np.cross (v_wrt, v_goal)
  n = cross_prod / float (np.linalg.norm (cross_prod))
  #print ('n: [%g %g %g]' % (n[0], n[1], n[2]))

  # Matrix [a, nxa, n] form a basis, defined by a, n which is a vector
  #   perpendicular to a, and n x a which is a vector perpendicular to a and n.
  n_x_a = np.cross (n, v_wrt)
  a_basis = np.array ([[v_wrt[0], n_x_a[0], n[0]],
                       [v_wrt[1], n_x_a[1], n[1]],
                       [v_wrt[2], n_x_a[2], n[2]]])
  #print ('n x a: [%g %g %g]' % (n_x_a[0], n_x_a[1], n_x_a[2]))
  #print ('a_basis:')
  #print (a_basis)

  n_x_b = np.cross (n, v_goal)
  b_basis = np.array ([[v_goal[0], n_x_b[0], n[0]],
                       [v_goal[1], n_x_b[1], n[1]],
                       [v_goal[2], n_x_b[2], n[2]]])
  #print ('n x b: [%g %g %g]' % (n_x_b[0], n_x_b[1], n_x_b[2]))
  #print ('b_basis:')
  #print (b_basis)
  
  # Then to rotate a onto b, can do the same trick as inverting quaternions
  #   and multiplying two quaternions!
  #   i.e. rotate a back to world frame first (using a.inv()), then rotate to
  #   b!!!
  # The only difference is, quaternions multiply the new quat on the right,
  #   rotation matrices multiply the new matrix on the left.
  #   So while quaternion is a.inv() * b, rot mats are b * a.inv().
  #rot_mat = np.dot (b_basis, np.linalg.inv (a_basis))
  # a_basis is orthogonal, because it's made from 3 known independenet columns.
  #   So inverse is just transpose
  rot_mat = np.dot (b_basis, a_basis.T)

  # Append [0 0 0 1] to last row and last column, to make matrix 4 x 4
  rot_mat = np.append (rot_mat, np.array ([[0, 0, 0]]), 0)
  rot_mat = np.append (rot_mat, np.array ([[0], [0], [0], [1]]), 1)

  #print (rot_mat)

  # Convert matrix into quaternion
  rot_quat = tf.transformations.quaternion_from_matrix (rot_mat)

  return rot_quat, rot_mat



  # This doesn't work. Gives rotation matrix with columns like [1, 0, 2], WTH.
  '''
  # This calculation gives a matrix directly, no ambiguity of sign
  # Ref: http://math.stackexchange.com/questions/180418/calculate-rotation-matrix-to-align-vector-a-to-vector-b-in-3d

  # Normalize. Must normalize, otherwise dot product doesn't give cosine, it
  #   gives |a||b| cos!
  v_wrt = v_wrt_orig / np.linalg.norm (v_wrt_orig)
  v_goal = v_goal_orig / np.linalg.norm (v_goal_orig)

  v_perp = np.cross (v_wrt, v_goal)
  print ('v: [%g %g %g]' % (v_perp[0], v_perp[1], v_perp[2]))

  # If a == -b, I haven't implemented the case. See stackexchange link above.
  #   Pick a rotation of pi about any axis perpendicular to a.
  if np.linalg.norm (v_perp) == 0:
    print ('Cross product is 0. a and b are the same vector. Setting rot mat to identity.')
    rot_mat = np.eye (3)

  else:
    sin_of_ang = np.linalg.norm (v_perp)
    cos_of_ang = np.dot (v_wrt, v_goal)
    print ('s: %g' % (sin_of_ang))
    print ('c: %g' % (cos_of_ang))
 
    # Skew symmetric matrix of cross product v_perp
    skew_sym = np.array ([[0, -v_perp[2], v_perp[1]],
                          [v_perp[2], 0, -v_perp[0]],
                          [-v_perp[1], v_perp[0], 0]])
    print (skew_sym)
 
    #rot_mat = np.eye (3) + skew_sym + (skew_sym ** 2) * \
    #  ((1.0 - cos_of_ang) / (sin_of_ang ** 2))
    # This gives same outcome as above....... both have a 2 in the rotation
    #   matrix, that's not a rotation matrix!!!
    rot_mat = np.eye (3) + skew_sym * sin_of_ang + \
      (skew_sym ** 2) * (1.0 - cos_of_ang)
 
    # Append [0 0 0 1] to last row and last column, to make matrix 4 x 4
    rot_mat = np.append (rot_mat, np.array ([[0, 0, 0]]), 0)
    rot_mat = np.append (rot_mat, np.array ([[0], [0], [0], [1]]), 1)

  # Rotation of +x to +z:
  # Should be [0 0 -1
  #            0 1 0 
  #            1 0 0]

  print (rot_mat)

  # Convert matrix into quaternion
  rot_quat = tf.transformations.quaternion_from_matrix (rot_mat)

  return rot_quat
  '''


# Copied from my new continuumRepo active_chord active_util.py
# TODO: This does not guarantee order! i.e. the axis and angle does not tell
#   you whether it rotates n1 onto n2, or n2 onto n1! For that, use
#   get_relative_rotation_v().
# Parameters:
#   n1, n2: 3-elt 1D NumPy arrays. Unit normal vectors, each representing a
#     plane. The normal vector is the normal to the plane.
def calc_axis_angle_btw_vecs (n1, n2):

  # Two vectors make a plane.
  # Take the cross product btw them, and that gives you a normal to their
  #   common plane. Then just need to rotate wrt this normal, by the angle
  #   in between the two orig vectors. This will rotate n1 onto n2.

  # Make sure the inputs are unit vectors
  n1 /= np.linalg.norm (n1)
  n2 /= np.linalg.norm (n2)

  # Find the normal to the plane formed by n1 and n2
  axis = np.cross (n1, n2)

  # Find the angle btw the two vectors, by dot product.
  # dot(a, b) = |a| |b| cos theta
  # If a and b are unit vectors, then dot(a, b) = cos theta, and
  #   theta = arccos (dot (a, b)).
  angle = np.arccos (np.dot (n1, n2))

  return axis, angle


# Copied from my active_touch costs_util.py
# Calculates the angle of a Quaternion, offsetted from identity 0 0 0 w=1.
# Returns angle in range [-pi, pi]
def get_quaternion_angle (qx, qy, qz, qw):

  # Because Quaternions have two identical representations, q = -q, so we need
  #   to have a convention, to represent such equal quaternions in the same
  #   way. This is needed `.` in the following calculation for angle, we ignore
  #   sign of qx qy qz, and only look at sign of qw. If two equal
  #   quaternions are not represented the same way, i.e. one has +w, one has
  #   -w, then the calculated angle will be different, which is incorrect!!
  # Convention: qx is always positive.
  if qx < 0:
    qx = -qx
    qy = -qy
    qz = -qz
    qw = -qw

  # Quaternion distance
  #   Distance is btw (qx, qy, qz, qw) and origin (0, 0, 0, w=1)
  # Signed angle of rotation by shortest arc
  # Ref: http://stackoverflow.com/questions/23260939/distance-or-magnitude-between-two-quaternions
  # atan2(qw, qxyz_norm) gives pi for 0 0 0 1 quaternion!!! That's not right!
  #   It should give 0!! Testing on python shell with
  #   q = tf.transformations.quaternion_about_axis(1.9, (1,0,0))
  #   calc_mvmt_cost(0, 0, 0, *q)
  #   reveals that atan2(qxyz_norm, qw) is the correct argument order. It 
  #   returns angle 1.9. If you flip the order, it returns 1.2, which is wrong.
  angle = 2.0 * np.arctan2 (np.linalg.norm ((qx, qy, qz)), qw)

  # Checked this on python shell. It needs this correction
  if angle > np.pi:
    angle = angle - 2.0 * np.pi
  elif angle < -np.pi:
    # TODO 14 Jan 2016:
    # Wait... this seems wrong. angle < 0 here. This will make angle over 2*pi!
    #   It should be angle + 2.0 * np.pi, no? Test it
    #   > Tested some random rpy angles in active_chords package action_util.py
    #     This case NEVER happens! I think
    #     arctan2() above always gives range (0, 2*pi). So that's why even
    #     though this line is wrong, it never affects anything.
    #angle = -angle + 2.0 * np.pi
    # I think this line should be right. But since this case is never true,
    #   might as well remove the elif altogether.
    angle = angle + 2.0 * np.pi
  return angle


# Returns skew-symmetric matrix according to equations here, same ordering
#   of the 3 elts.
#   https://pythonpath.wordpress.com/2012/09/04/skew-with-numpy-operations/
#   https://en.wikipedia.org/wiki/Skew-symmetric_matrix
def skew_sym_mat (a, b, c):
  return np.array ([[0,-c,b],[c,0,-a],[-b,a,0]])


# Enforces qx to be positive, so all the quaternions used in your program
#   follow the same convention, so you have unique parameter representation for
#   the same rotation!
def standardize_quats (qx, qy, qz, qw):

  # Because Quaternions have two identical representations, q = -q, so we need
  #   to have a convention, to represent such equal quaternions in the same
  #   way. This is needed `.` in the following calculation for angle, we ignore
  #   sign of qx qy qz, and only look at sign of qw. If two equal
  #   quaternions are not represented the same way, i.e. one has +w, one has
  #   -w, then the calculated angle will be different, which is incorrect!!
  # Convention: qx is always positive.
  if qx < 0:
    qx = -qx
    qy = -qy
    qz = -qz
    qw = -qw

  return qx, qy, qz, qw


