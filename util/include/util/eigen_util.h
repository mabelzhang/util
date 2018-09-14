// Mabel Zhang
// 12 Sep 2018
//
// Convenience functions for Eigen.
// NOT TESTED. NOT USED after written. Compiles.
//

#include <Eigen/Core>

// Ref Pengyao answer: https://stackoverflow.com/questions/17036818/initialise-eigenvector-with-stdvector/17037695
void convert_vector_to_Eigen (std::vector <int> & vec, Eigen::VectorXi & eig)
{
  eig = Eigen::Map <Eigen::VectorXi> (&vec [0], vec.size ());
}

// Ref https://stackoverflow.com/questions/26094379/typecasting-eigenvectorxd-to-stdvector
void convert_Eigen_to_vector (Eigen::VectorXi & eig, std::vector <int> & vec)
{
  // Assign to raw data array
  vec.resize (eig.size ());
  Eigen::VectorXi::Map (&vec [0], eig.size ()) = eig;
}

