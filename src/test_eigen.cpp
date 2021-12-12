#include <iostream>
#include <Eigen/Core>
#include <Eigen/Dense>

using namespace std;
using namespace Eigen;

int main(){
  Eigen::Matrix<double, 2, 4> M;
  M << 1, 2, 0, 0, 2, 3, 0, 0;
  cout << M << endl; 
  FullPivLU<Matrix<double, 2, 4>> lu(M);
  Eigen::MatrixXd M_n = lu.kernel();
  // M_n.transposeInPlace();
  // cout << M_n << endl;
  // cout << M * M_n << endl;

   MatrixXf A = MatrixXf::Random(3, 2);
   cout << "Here is the matrix A:\n" << A << endl;
   VectorXf b = VectorXf::Random(3);
   cout << "Here is the right hand side b:\n" << b << endl;
   cout << "The least-squares solution is:\n"
        << A.bdcSvd(ComputeThinU | ComputeThinV).solve(b) << endl;

  
  return 0;
}