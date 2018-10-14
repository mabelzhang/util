#!/usr/bin/env python

# Mabel Zhang
# 18 Sep 2018
#
# To be run in Blender Python. So do not import extra libraries that are not
#   known to Blender.
# Used by depth_scene_rendering package in Blender Python.
#
# Generate a random orientation in spherical coordinates (long, lat), or
#   (lng, lat), on the sphere. This point represents a rotation from
#   reference point (1, 0, 0) on the sphere, by convention.
# Tested in ./test_spherical_pose_generation.py
#

import numpy as np

# Local
from util.tf_transformations import rotation_matrix, quaternion_from_matrix, \
  euler_from_matrix


# lng, lat: Scalars
# Returns 4 x 4 rotation matrix
# Ref images from: https://stackoverflow.com/questions/5437865/longitude-latitude-to-quaternion
def spherical_matrix (lng, lat):

  # Fix z to be pointing up, then longitude is a rotation wrt z
  Rz = np.array ([[np.cos (lng), -np.sin (lng), 0, 0],
                  [np.sin (lng),  np.cos (lng), 0, 0],
                  [0, 0, 1, 0],
                  [0, 0, 0, 1]])

  # Lat is a rotation wrt either x or y, pick one
  # Rotation wrt y
  Ry = np.array ([[np.cos (lat), 0, np.sin (lat), 0],
                  [0, 1, 0, 0],
                  [-np.sin (lat), 0, np.cos (lat), 0],
                  [0, 0, 0, 1]])

  R = np.dot (Ry, Rz)

  return R


# Convert spherical coordinates to quaternion.
# NOTE the parameter ranges here are different from those passed to
#   position_from_spherical()!
# NOTE locations on unit sphere generated here do NOT correspond to those from
#   position_from_spherical(). To get matching ones, use get_rand_pose().
# Translated from https://github.com/moble/quaternion/blob/306630d69f382827ef097357ca6ee057a42c2103/quaternion.c#L19
#   https://stackoverflow.com/questions/5437865/longitude-latitude-to-quaternion
# Paramters:
#   lng: longitude, range (-pi, pi). Scalar or numpy vector same size as lat
#   lat: latitude, range (-pi/2, pi/2) Scalar or numpy vector same size as lng
# Returns a scalar if lat and lng are scalars, or 4 x n numpy array.
def quaternion_from_spherical (lng, lat, qwFirst=False):

  ct = np.cos (lat * 0.5)
  cp = np.cos (lng * 0.5)
  st = np.sin (lat * 0.5)
  sp = np.sin (lng * 0.5)

  if qwFirst:
    # (w, x, y, z)
    quat = np.array ([cp*ct, -sp*st, st*cp, sp*ct])
  else:
    # (x, y, z, w)
    quat = np.array ([sp*ct, cp*ct, -sp*st, st*cp])

  return quat


# Convert spherical coordinates to Cartesian coordinates.
# NOTE the ranges here is different from those passed to
#   quaternion_from_spherical()!
# NOTE locations on unit sphere generated here do NOT correspond to those from
#   quaternion_from_spherical(). To get matching ones, use get_rand_pose().
# Paramters:
#   lng: longitude, range (0, 2*pi). Scalar or numpy vector same size as lat
#   lat: latitude, range (0, pi) Scalar or numpy vector same size as lng
# Returns a scalar if lat and lng are scalars, or 3 x n numpy array.
# Ref: http://mathworld.wolfram.com/SphericalCoordinates.html
def position_from_spherical (lng, lat):

  x = np.cos (lng) * np.sin (lat)
  y = np.sin (lng) * np.sin (lat)
  z = np.cos (lat)

  return np.array ([x, y, z])


# Conversion to XYZ Euler (not coded):
# Spherical coordinates has z-axis fixed pointing up; longitude is wrt z,
#   followed by latitude wrt y or x. So will need to use a Euler convention
#   that rotates wrt z first, then y, and set the remaining axis (x) to zero.
#   https://stackoverflow.com/questions/5437865/longitude-latitude-to-quaternion
def euler_from_spherical (lng, lat):

  #return (0, lat, lng)
  return None


# Return 3-elt numpy array
# Default longitude range (-180, 180), latitude range (-90, 90)
def get_rand_position (lng_range=(-np.pi, np.pi),
  lat_range=(-0.5*np.pi, 0.5*np.pi)):

  lng = lng_range[0] + np.random.rand () * (lng_range[1] - lng_range[0])
  lat = lat_range[0] + np.random.rand () * (lat_range[1] - lat_range[0])

  return (position_from_spherical (lng, lat))


# Return 4-elt numpy array
# Default longitude range (-180, 180), latitude range (-90, 90)
def get_rand_rotation (lng_range=(-np.pi, np.pi),
  lat_range=(-0.5*np.pi, 0.5*np.pi), qwFirst=False):

  lng = lng_range[0] + np.random.rand () * (lng_range[1] - lng_range[0])
  lat = lat_range[0] + np.random.rand () * (lat_range[1] - lat_range[0])

  # Make a quaternion out of lng, lat
  return (quaternion_from_spherical (lng, lat, qwFirst))


# Get position and orientation with *MATCHING* location on unit sphere.
#   Orientation is computed directly from position, by vector between position
#   and center of sphere.
# Default long/lat range same as those for get_rand_position(), `.` orientation
#   is calculated from position.
# Parameters:
#   from_vec: Source vector the rotation will be calculated from. Canonical
#     reference vector is the x-vector, (1, 0, 0). But if calculating for a
#     camera pose that has z pointing out of camera, use z-vector, (0, 0, 1).
def get_rand_pose (lng_range=(-np.pi, np.pi),
  lat_range=(-0.5*np.pi, 0.5*np.pi), from_vec=[1,0,0], qwFirst=False):

  lng = lng_range[0] + np.random.rand () * (lng_range[1] - lng_range[0])
  lat = lat_range[0] + np.random.rand () * (lat_range[1] - lat_range[0])

  pos = position_from_spherical (lng, lat)


  # Center of sphere is at (0, 0, 0)
  # It should already be a unit vector, coming from spherical coord eqns
  orientation_vec = pos

  # Dot product, a dot b = |a||b|cos(theta), theta = acos((a dot b) / (|a||b|)).
  #   |a| == |b| == 1 if a and b are unit vectors, as they are here.
  #   Then theta = acos (a dot b)
  angle = np.arccos (np.dot (from_vec, orientation_vec))
  # Vector perpendicular to both vecs, is the rotation axis
  axis = np.cross (from_vec, orientation_vec)

  mat = rotation_matrix (angle, axis)

  # Find quaternion for the orientation vector, wrt default vector (1, 0, 0)
  # (w x y z)
  quat = quaternion_from_matrix (mat)
  euler = euler_from_matrix (mat)


  # DEBUG: If rotation matrix correct, these two should be exactly the same
  #print (pos)
  #print (np.dot (mat, from_vec + [0]))


  if not qwFirst:
    # (x y z w)
    quat = (quat[1], quat[2], quat[3], quat[0])

  return pos, quat, euler


# Returns n x 4 NumPy array
def get_ordered_pose (lng_range=(0, 2 * np.pi), lat_range=(0, np.pi),
  qwFirst=False):

  nLongs = 36
  nLats = 9

  # nLongs x nLats
  # Full longitude range (-180, 180)
  lng = np.tile (np.array (np.linspace (lng_range[0], lng_range[1], nLongs))
    .reshape (nLongs, 1), (1, nLats))
  # Remove last row, `.` 0  == 2 * np.pi, duplicate points
  lng = lng [0:lng.shape[0]-1, :]
  #print (lng)
  # Full latitude range (-90, 90)
  lat = np.tile (np.array (np.linspace (lat_range[0], lat_range[1], nLats))
    .reshape (1, nLats), (nLongs-1, 1))
  #print (lat)

  lng = lng.flatten ()
  lat = lat.flatten ()

  #print (lng)
  #print (lat)

  #lng -= np.pi
  #lat -= (0.5 * np.pi)

  # nLongs x nLats
  # Subtract by half the range for quaternion, to stay in range of function.
  #   This produces the same plot for both position and quaternion.
  return (position_from_spherical (lng, lat),
    quaternion_from_spherical (lng-np.pi, lat-(0.5*np.pi), qwFirst))

