#include "SystemModels.hpp"

#include <sysexits.h>
#include <iostream>

using namespace ::std;
using namespace ::Eigen;

int main(int argc, const char* argv[])
{
  ProcessModel truePM(1, 0.1, true);
  cout << "Fd=\n" << truePM.getFD() << "\nQd=\n" << truePM.getQD() << endl;

  ObservationModel om(1, 0);

  cout << "H=" << om.getH() << endl;

  State x = State::Zero();

  cout << "H*x=" << om.getH() * x << endl;

  StateCovariance P;

  cout << "P=\n" << P << endl;

  MatrixNM C = P * om.getH().transpose();

  cout << "P*H'=\n" << Matrix2d::Identity() * om.getH().transpose() << endl;

  return EX_OK;

  
  State xTrue = State::Zero();

  for (int i = 0; i < 100; ++i)
    {
      xTrue = truePM.predict(xTrue);
      cout << xTrue.transpose() << endl;
    }
  
  return EX_OK;
}
