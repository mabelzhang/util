#ifndef _CV_UTIL_H_
#define _CV_UTIL_H_

// Mabel Zhang
// 15 Sep 2018
//
// Utility functions for computer vision
//

// Eigen
#include <Eigen/Core>

// Parameters:
//   pts: 3 x n. Each column is (x y z) coordinate of a point in 3D.
//   K: 3 x 3 camera intrinsics matrix.
//   uv: 3 x n. Return value. 2D image coordinates in the top two rows.
void project_3d_pts_to_2d (Eigen::MatrixXf & pts, Eigen::MatrixXf & K,
  Eigen::MatrixXi & uv)
{
  uv = (K * pts).array ().round ().cast <int> ();
}

// Parameters:
//   pts: 3 x n. Each column is (x y z) coordinate of a point in 3D.
//   P: 3 x 4 camera projection matrix
//         [fx'  0  cx' Tx]
//     P = [ 0  fy' cy' Ty]
//         [ 0   0   1   0]
//   uv: 2 x n. Return value. 2D image coordinates in the top two rows.
void project_3d_pts_to_2d_homo (Eigen::MatrixXf & pts, Eigen::MatrixXf & P,
  Eigen::MatrixXi & uv)
{
  // Sanity check
  if (pts.cols () == 0)
  {
    uv = Eigen::MatrixXi (2, 0);
    return;
  }

  // Make matrices homogeneous
  // Concatenate a row of 1s to bottom of matrix, using comma operator
  //   Ref: https://stackoverflow.com/questions/21496157/eigen-how-to-concatenate-matrix-along-a-specific-dimension
  // 4 x n
  Eigen::MatrixXf pts4 (4, pts.cols ());
  pts4 << pts, Eigen::MatrixXf::Ones (1, pts.cols ());

  // Project points into image plane, to find (u, v) rectified image coords
  //   of the 3D points.
  //   [u v w]' = P * [X Y Z 1]'
  //   x = u / w
  //   y = v / w
  //   http://docs.ros.org/melodic/api/sensor_msgs/html/msg/CameraInfo.html
  // 3 x n = (3 x 4) * (4 x n)
  //std::cerr << "pts:" << std::endl << pts4 << std::endl;
  Eigen::MatrixXf uv3 = P * pts4;
  //std::cerr << "u, v, w:" << std::endl << uv3 << std::endl;

  // Divide by w, to get [u/w y/w 1]' in each column.
  // Replicate row to 3 rows, to be same dimension as divider
  uv3 = uv3.cwiseQuotient (uv3.row (2).replicate (uv3.rows (), 1));
  //std::cerr << "u/w, y/w, 1:" << std::endl << uv3 << std::endl;

  // 2 x n. Eliminate 3rd row
  // NOTE: can't do uv3 = uv3.topRows (2), first column all 0s. Need to
  //   reallocate a new matrix.
  uv = uv3.topRows (2).array ().round ().cast <int> ();
  //std::cerr << "Final image coordinates:" << std::endl << uv << std::endl;
}

#endif
