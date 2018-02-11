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
  // TODO: Move these out and make them externally controllable
  State x0 = State::Zero();
  StateCovariance P0 = StateCovariance::Zero();
  
  _simulator.initialize(x0, P0);
  _estimator.initialize(x0, P0);

  double totalCHI2 = 0;

  int maxIterations = 1000;

  for (int step = 0; step < maxIterations; ++step)
    {
      _simulator.step();
      Observation z = _simulator.getZ();
      _estimator.step(z);
      /*
      cout << setprecision(12);
      cout << step << " " << _simulator.getX().transpose() << " " << z << " " << _estimator.getXEst().transpose()
	   << " " << _estimator.getPEst().diagonal().transpose();
      */

      // chi^2 estimate
      State xErr = _estimator.getXEst() - _simulator.getX();
      double chi2 = xErr.transpose() * _estimator.getPEst().llt().solve(xErr);
      totalCHI2 += chi2;
      //cout << " " << chi2 << endl;
    }

  cerr << totalCHI2 / maxIterations << endl;
}
