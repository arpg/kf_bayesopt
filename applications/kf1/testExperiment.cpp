#include "Experiment.hpp"

#include <sysexits.h>
#include <iostream>

using namespace ::std;

int main(int argc, const char* argv[])
{

  Experiment e(0);
 
  e.run();
  
  return EX_OK;
}
