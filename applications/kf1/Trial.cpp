#include "Trial.hpp"

// setprecision example
#include <iostream>     // std::cout, std::fixed
#include <iomanip> 
using namespace ::std;

Trial::Trial(int trialNumber, const Parameters& simulatorParameters, const Parameters& estimatorParameters):
  _trialNumber(trialNumber), _simulator(simulatorParameters), _estimator(estimatorParameters)
{
}

void Trial::run()
{
  int i ;
  
  for(i=0;i<Nsimruns;i++)
  {
  // TODO: Move these out and make them externally controllable
    State x0 = State::Zero();
    StateCovariance P0 = StateCovariance::Identity();//StateCovariance::Zeros()
  
    _simulator.initialize(x0, P0);
    _estimator.initialize(x0, P0);

    _averageNEES = 0;

   

    for (int step = 0; step < maxIterations; ++step)
    {
      _simulator.step(step);//step variable is for G*u;
      Observation z = _simulator.getZ();
      _estimator.step(z,step);
      /*
      cout << setprecision(12);
      cout << step << " " << _simulator.getX().transpose() << " " << z << " " << _estimator.getXEst().transpose()
	   << " " << _estimator.getPEst().diagonal().transpose();
      */
      // chi^2 estimate
      State xErr = _simulator.getX()-_estimator.getXEst();//_estimator.getXEst() - _simulator.getX();
      //double chi2 = xErr.transpose() * _estimator.getPEst().llt().solve(xErr);
      double chi2 = xErr.transpose()*_estimator.getPEst().inverse()*xErr;
      //OneNEES.push_back(chi2);
      rowNEES += chi2;
      //cout << " " << chi2 << endl;
     }
     AllNEES+= rowNEES;
     //AllNEES.push_back(OneNEES);
   }
  
  _averageNEES = AllNEES/(maxIterations*Nsimruns);
}
