#ifndef LIN_ALG_UTIL_H
#define LIN_ALG_UTIL_H

// Mabel Zhang
// 6 Sep 2018
//
// Linear algebra utilities.
//

#include <Eigen/Core>

// Translated from project_pts_onto_line() in lin_alg_util.py
// Project a set of points onto a given line. Return the distance of the points
//   from the starting point (p1) of the line.
// Parameters:
//   pt1, pt2: 3-vectors. (pt2 - pt1) is a vector parallel to a line.
//     Eigen::Vectors are by default column vectors.
//   pts: n x 3. A set of n points.
// Returns n-vector.
//   Values mean distance of projection from pt1.
//   Positive value means projected onto (pt2 - pt1), i.e. in between points pt1
//     and pt2;
//   negative value means projected onto -(pt2 - pt1).
void project_pts_onto_line (Eigen::Vector3f pt1, Eigen::Vector3f pt2,
  Eigen::MatrixXf & pts, Eigen::VectorXf & projections)
{
  // Projection of a point onto a vector is equal to the dot product of the
  //   point and the unit vector of the vector.
  //   Note the point and the vector must have the same starting point (see pic
  //   in link), the starting point of the vector. So if the point is in terms
  //   of origin, it needs to be in terms of the starting point (pt1) of the
  //   vector.
  //  
  //  Ref: http://en.wikipedia.org/wiki/Vector_projection

  int nPts = pts.rows ();

  // Calc unit vector parallel to line
  // 1 x 3
  Eigen::Vector3f line = pt2 - pt1;
  float line_norm = line.norm ();

  //printf ("Tiled pt1: %ld x %ld\n", pt1.replicate (1, nPts).rows (),
  //  pt1.replicate (1, nPts).cols ());

  // n x 3
  // Eigen::Vectors are by default column vectors. pt1 is 3 x 1, replicate
  //   to 3 x n, transpose to n x 3.
  Eigen::MatrixXf diff = pts - pt1.replicate (1, nPts).transpose ();

  //printf ("diff: %ld x %ld\n", diff.rows (), diff.cols ());

  // If line has length 0, then di
  // TODO: Not tested!
  if (line_norm < 1e-6)
  {
    // Take norm to the right. On n x 3 array, that gives (n, ) result.
    // Ref Eigen cheat sheet
    //   https://eigen.tuxfamily.org/dox/group__QuickRefPage.html
    //   replicate (verticalTimes, horizontalTimes)
    // n x 3
    // rowwise() produces n x 1
    projections = diff.rowwise ().norm ();
  }
  else
  {
    Eigen::Vector3f line_unit = line / line_norm;
    
    // n x 1. Distance of projection from pt1.
    // Value is positive if point projects onto a point in between pt2 - pt1,
    //   i.e. in positive direction of the vector pt2 - pt1. Value is negative
    //   if point projects onto a point on negative direction of (pt2 - pt1).
    // Subtract pts by pt1, to put points wrt starting point of vector.
    //   Otherwise, since points are wrt origin [0 0 0], the result projection
    //   height will be in terms of origin, i.e. projection onto
    //   (pt2 - [0 0 0]).
    // (n x 1) = ((n x 3) * (1 x 3)^T)
    projections = diff * line_unit;
  }
}

#endif
