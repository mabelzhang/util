#!/usr/bin/env python

# Mabel Zhang
# 18 Sep 2018
#
# Test spherical orientation generation, visualize it.
# Testing in a separate file `.` spherical_pose_generation.py needs to run
#   in Blender Python, so must only contain minimum libraries known to Blender.
#   Visualization code requires libraries not in Blender.
#


import numpy as np

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Local
from spherical_pose_generation import get_ordered_pose, get_rand_pose
from tf_transformations import quaternion_matrix



def test_ordered_poses (topOnly=False):

  if topOnly:
    # Top hemisphere
    poses = get_ordered_pose (lat_range=(0, 0.5 * np.pi), qwFirst=True)
  else:
    # Full sphere
    poses = get_ordered_pose (qwFirst=True)

  # 3 x n
  pos = poses [0]
  # 4 x n
  quats = poses [1]

  return pos, quats


def test_rand_poses (n_poses, topOnly=False):

  pos = np.zeros ((3, n_poses))
  quats = np.zeros ((4, n_poses))

  if topOnly:
    lat_range = (0, 0.5*np.pi)
  else:
    lat_range = (0, np.pi)

  for i in range (n_poses):
    pos [:, i], quats [:, i], _ = get_rand_pose (
      lat_range=lat_range, qwFirst=True)

  return pos, quats



def main ():

  ORDERED = False
  TOP_ONLY = False

  if ORDERED:
    pos, quats = test_ordered_poses (topOnly=TOP_ONLY)
    if TOP_ONLY:
      out_name = 'test_spherical_pose_generation_top'
    else:
      out_name = 'test_spherical_pose_generation_full'

  else:
    n_rand_pts = 100
    pos, quats = test_rand_poses (n_rand_pts, topOnly=TOP_ONLY)

    if TOP_ONLY:
      out_name = 'test_spherical_pose_generation_rand_top'
    else:
      out_name = 'test_spherical_pose_generation_rand_full'


  #print quats.shape
  n_pts = quats.shape [1]


  fig = plt.figure (figsize=(15, 6))


  #####
  # Use quaternion rotation only, plot rotations on unit sphere

  # Ref 3D plot https://matplotlib.org/mpl_toolkits/mplot3d/tutorial.html
  ax = fig.add_subplot (131, projection='3d')

  # 3 x n
  XYZ = np.tile (np.array ([[0, 0, 0]]).T, (1, n_pts))
  # 3 x n
  UVW = np.zeros ((3, n_pts))

  for i in range (n_pts):

    mat = quaternion_matrix (quats [:, i])

    # Multiply the rotation by x-axis, to get a vector
    UVW [:, i] = np.dot (mat [0:3, 0:3], [1, 0, 0])


  # Ref https://matplotlib.org/examples/mplot3d/quiver3d_demo.html
  # First three xyzs are start point, last three are vector.
  # pivot specifies the part of the arrow that is at the grid point, 1st xyzs
  #   passed in. Arrow rotates about this point. Default is 'tip', regardless
  #   of what API says. Arrowheads end at first set of xyzs passed in. Other
  #   option is 'tail' or 'middle'.
  ax.quiver (
    UVW[0, :], UVW[1, :], UVW[2, :],
    -UVW[0, :], -UVW[1, :], -UVW[2, :],
    color='orange', length=0.4, arrow_length_ratio=0.5, pivot='tail')
  ax.scatter (UVW[0, :], UVW[1, :], UVW[2, :], c='orange')

  ax.set_title ('By quaternion only')
  ax.set_aspect (1)
  ax.set_xlim (-1, 1)
  ax.set_ylim (-1, 1)
  ax.set_zlim (-1, 1)



  #####
  # Use position only, plot positions on sphere

  ax = fig.add_subplot (132, projection='3d')

  ax.scatter (pos[0, :], pos[1, :], pos[2, :], c='red')

  ax.set_title ('By position only')
  ax.set_aspect (1)
  ax.set_xlim (-1, 1)
  ax.set_ylim (-1, 1)
  ax.set_zlim (-1, 1)


  #####
  # Overlap position and quaternion, to see if they overlap exactly
  # Result: They do NOT overlap exactly. So the best option to generate both
  #   position and quaternion is using the quaternion way, where quats variable
  #   gives the quaternions, UVW gives the positions.

  ax = fig.add_subplot (133, projection='3d')

  # Quaternion
  #ax.quiver (XYZ[0, :], XYZ[1, :], XYZ[2, :], UVW[0, :], UVW[1, :], UVW[2, :],
  #  color='orange', length=1, arrow_length_ratio=0.1, pivot='tail')
  ax.scatter (UVW[0, :], UVW[1, :], UVW[2, :], c='orange', alpha=0.5)

  # Position
  ax.scatter (pos[0, :], pos[1, :], pos[2, :], c='red', alpha=0.5)

  # Text label, for easier debugging of mismatches between positions and quats
  for i in range (pos.shape [1]):
    ax.text (UVW[0, i], UVW[1, i], UVW[2, i], str(i), color='orange')
    ax.text (pos[0, i], pos[1, i], pos[2, i], str(i), color='red')

  ax.set_title ('Overlay quaternion and position')
  ax.set_aspect (1)
  ax.set_xlim (-1, 1)
  ax.set_ylim (-1, 1)
  ax.set_zlim (-1, 1)


  fig.tight_layout ()

  '''
  # Save png
  fig.savefig (out_name + '.png')
  print ('Written plot to %s.png' % out_name)

  # Save eps
  fig.savefig (out_name + '.eps')
  print ('Written plot to %s.eps' % out_name)
  '''

  plt.show ()



if __name__ == '__main__':
  main ()

