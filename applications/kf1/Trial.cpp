#include "Trial.hpp"

using namespace ::std;

Trial::Trial(int trialNumber, const Parameters& simulatorParameters, const Parameters& estimatorParameters):
  _trialNumber(trialNumber), _simulator(simulatorParameters), _estimator(estimatorParameters)
{
}

void Trial::run()
{
  // TODO: Move these out and make them externally controllable
  State x0 = State::Zero();
  StateCovariance P0 = 2 * StateCovariance::Identity();
  
  _simulator.initialize(x0, P0);
  _estimator.initialize(x0, P0);

  for (int step = 0; step < 10000; ++step)
    {
      _simulator.step();
      Observation z = _simulator.getZ();
      _estimator.step(z);
      //cout << step << " " << _simulator.getX().transpose() << " " << _estimator.getXEst().transpose()
      //   << " " << _estimator.getPEst().diagonal().transpose() << endl;

      // chi^2 estimate
      State xErr = _estimator.getXEst() - _simulator.getX();
      double chi2 = xErr.transpose() * _estimator.getPEst() * xErr;
      //cout << chi2 << endl;
    }
}
