// Mabel Zhang
// 12 Sep 2018
//
// Convenience functions for Eigen.
// NOT TESTED. NOT USED after written. Compiles.
//

#include <Eigen/Core>

#include <Eigen/Dense>
#include <vector>
#include <iostream>
#include <fstream>


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

// From https://stackoverflow.com/questions/34247057/how-to-read-csv-file-and-assign-to-eigen-matrix
template<typename M>
M load_csv_to_Eigen (const std::string & path)
{
  std::ifstream indata;
  indata.open (path.c_str ());

  std::string line;
  std::vector<double> values;
  uint rows = 0;

  while (std::getline (indata, line))
  {
    std::stringstream lineStream (line);
    std::string cell;

    // Read comma-separated line
    while (std::getline (lineStream, cell, ','))
    {
      // Convert string to double. Requires c++11
      values.push_back (std::stod (cell));
    }
    ++rows;
  }

  // Convert std::vector to Eigen::Matrix
  return Eigen::Map <const Eigen::Matrix <typename M::Scalar,
    M::RowsAtCompileTime, M::ColsAtCompileTime, Eigen::RowMajor>> (
    values.data (), rows, values.size () / rows);
}

