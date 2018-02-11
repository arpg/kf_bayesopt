#include "Trial.hpp"

#include <sysexits.h>
#include <iostream>

using namespace ::std;

int main(int argc, const char* argv[])
{
  Parameters testParameters(1, 5, 0.1);

  Trial t(1, testParameters, testParameters);

  t.run();
  
  return EX_OK;
}
