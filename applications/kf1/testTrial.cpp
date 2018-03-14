#include "Trial.hpp"

#include <sysexits.h>
#include <iostream>
#include <fstream>

using namespace ::std;

int main(int argc, const char* argv[])
{
  double q = 0.01; //q=0.01
  ofstream wr("QPSD_NEES_NIS.csv",ios::out|ios::app);
  for(int i = 0; i<100;i++)
  {
    q+=0.04015;//0.04015
    Parameters correctParameters(1, 1, 0.1);//(1e-8, 1e-8, 0.1)
    Parameters wrongParameters(q,0.95, 0.1);//process noise, observation noise, deltaT//(1e-8, 1e-8, 0.1)

  //  Trial t(1, correctParameters, wrongParameters);
   Trial t(1, correctParameters, wrongParameters);

   t.run();

   double J_NEES = sqrt((log(t.getAverageNEES()/2))*(log(t.getAverageNEES()/2)));
   double J_NIS  = sqrt(log(t.getAverageNIS())*log(t.getAverageNIS()));

   //cout << "Average NEES after log =" << J_NEES<<","<<" QPSD is "<<q<< endl;
   cout << "Average NIS after log =" << J_NIS<<","<<" QPSD is "<<q<< endl;

   wr<<q<<","<<J_NEES<<","<<J_NIS<<endl;
  }
  wr.close();
  
  return EX_OK;
}
