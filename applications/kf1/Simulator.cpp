#include "Simulator.hpp"

Simulator::Simulator(const Parameters& parameters) :
  _processModel(parameters.processNoise, parameters.deltaT, true),
  _observationModel(parameters.observationNoise, true)
{
}

void Simulator::initialize(const State& x0, const StateCovariance& P0)
{
  Eigen::EigenMultivariateNormal<double> initialConditionSampler(x0, P0);

  _x = initialConditionSampler.samples(1);
}

void Simulator::step()
{
  _x = _processModel.predict(_x);
  _z = _observationModel.predict(_x);
}
