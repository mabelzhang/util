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


# Convert spherical coordinates to quaternion.
# NOTE the ranges here is different from those passed to
#   position_from_spherical()!
# Translated from https://github.com/moble/quaternion/blob/306630d69f382827ef097357ca6ee057a42c2103/quaternion.c#L19
#   https://stackoverflow.com/questions/5437865/longitude-latitude-to-quaternion
# Paramters:
#   lng: longitude, range (-pi, pi). Scalar or numpy vector same size as lat
#   lat: latitude, range (-pi/2, pi/2) Scalar or numpy vector same size as lng
# Returns a scalar if lat and lng are scalars, or 4 x n numpy array.
def quaternion_from_spherical (lng, lat):

  ct = np.cos (lat * 0.5)
  cp = np.cos (lng * 0.5)
  st = np.sin (lat * 0.5)
  sp = np.sin (lng * 0.5)
  quat = np.array ([cp*ct, -sp*st, st*cp, sp*ct])

  return quat


# Convert spherical coordinates to Cartesian coordinates.
# NOTE the ranges here is different from those passed to
#   quaternion_from_spherical()!
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


# Return 4-elt numpy array
def get_rand_pose ():

  # Longitude range (-180, 180)
  lng = np.random.rand () * 2 * np.pi
  lng -= np.pi
  # Normally, latitude range (-90, 90). Truncate to (0, 90), so it is always
  #   above horizon, `.` tabletop
  lat = np.random.rand () * 0.5 * np.pi

  # Make a quaternion out of lng, lat
  return (position_from_spherical (lng, lat),
    quaternion_from_spherical (lng, lat))


# Returns n x 4 NumPy array
def get_ordered_pose ():

  nLongs = 36
  nLats = 9

  # nLongs x nLats
  # Full longitude range (-180, 180)
  lng = np.tile (np.array (np.linspace (0, 2 * np.pi, nLongs)).reshape (nLongs, 1), (1, nLats))
  # Remove last row, `.` 0  == 2 * np.pi, duplicate points
  lng = lng [0:lng.shape[0]-1, :]
  #print (lng)
  # Full latitude range (-90, 90)
  lat = np.tile (np.array (np.linspace (0, np.pi, nLats)).reshape (1, nLats), (nLongs-1, 1))
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
    quaternion_from_spherical (lng-np.pi, lat-(0.5*np.pi)))

