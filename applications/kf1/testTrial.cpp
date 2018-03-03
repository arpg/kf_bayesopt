#include "Trial.hpp"

#include <sysexits.h>
#include <iostream>
#include <fstream>

using namespace ::std;

int main(int argc, const char* argv[])
{
  double q = 0.01; 
  ofstream wr("NEES_QPSD.csv",ios::out|ios::app);
  for(int i = 0; i<100;i++)
  {
    q+=0.04015;
    Parameters correctParameters(1, 0.1, 0.1);
    Parameters wrongParameters(q, 0.1, 0.1);//process noise, observation noise, deltaT

  //  Trial t(1, correctParameters, wrongParameters);
   Trial t(1, correctParameters, wrongParameters);

   t.run();

   cout << "Average NEES=" << t.getAverageNEES()<<","<<" QPSD is "<<q<< endl;

   wr<<t.getAverageNEES()<<","<<q<<endl;
  }
  wr.close();
  
  return EX_OK;
}
