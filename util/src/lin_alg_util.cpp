// Mabel Zhang
// 6 Sep 2018
//
// Tests lin_alg_util.h
//

#include <iostream>
#include <util/lin_alg_util.h>

// Test project_pts_onto_line()
int main (int argc, char ** argv)
{
  Eigen::Vector3f pt1 = Eigen::Vector3f::Random (3);
  std::cerr << "pt1: " << std::endl << pt1 << std::endl;
  Eigen::Vector3f pt2 = Eigen::Vector3f::Random (3);
  std::cerr << "pt2: " << std::endl << pt2 << std::endl;
  Eigen::MatrixXf pts = Eigen::MatrixXf::Random (10, 3);
  std::cerr << "pts: " << std::endl << pts << std::endl;

  Eigen::VectorXf projections;
  project_pts_onto_line (pt1, pt2, pts, projections);
  std::cerr << "projections: " << std::endl << projections << std::endl;

  return 0;
}


