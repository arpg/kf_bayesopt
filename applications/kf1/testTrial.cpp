#include "Trial.hpp"

#include <sysexits.h>
#include <iostream>

using namespace ::std;

int main(int argc, const char* argv[])
{
  Parameters correctParameters(1, 1, 1);
  Parameters wrongParameters(4, 4, 1);

  //  Trial t(1, correctParameters, wrongParameters);
  Trial t(1, correctParameters, correctParameters);

  t.run();

  cout << "Average NEES=" << t.getAverageNEES() << endl;
  
  return EX_OK;
}
