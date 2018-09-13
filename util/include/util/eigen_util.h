// Mabel Zhang
// 12 Sep 2018
//
// Convenience functions for Eigen.
// NOT TESTED. NOT USED after written.
//

#include <Eigen/Core>

// Ref: https://stackoverflow.com/questions/17036818/initialise-eigenvector-with-stdvector/17037695
void convert_vector_to_Eigen (std::vector <int> & vec, Eigen::VectorXi & eig)
{
  // Use built-in constructor
  eig = Eigen::VectorXi (vec.data ());
}

// Ref https://stackoverflow.com/questions/26094379/typecasting-eigenvectorxd-to-stdvector
void convert_Eigen_to_vector (Eigen::VectorXi & eig, std::vector <int> & vec)
{
  // Assign to raw data array
  vec.resize (eig.size ());
  Eigen::VectorXi::Map (&vec [0], eig.size ()) = eig;
}

