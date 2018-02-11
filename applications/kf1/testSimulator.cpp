#include "Simulator.hpp"

#include <sysexits.h>
#include <iostream>

using namespace ::std;

int main(int argc, const char* argv[])
{
  Parameters testParameters(1, 1e-6, 0.1);
  
  Simulator simulator(testParameters);

  State x0 = State::Zero();
  StateCovariance P0 = StateCovariance::Identity();
  
  simulator.initialize(x0, P0);
  
  for (int i = 0; i < 100; ++i)
    {
      simulator.step();

      cout << simulator.getX().transpose() << " " << simulator.getZ() << endl;
    }
  
  return EX_OK;
}
