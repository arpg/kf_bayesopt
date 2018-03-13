#include "Simulator.hpp"

Simulator::Simulator(const Parameters& parameters) :
  _processModel(parameters.processNoise, parameters.deltaT, true),
  _observationModel(parameters.observationNoise, true)
{
}

void Simulator::initialize(const State& x0, const StateCovariance& P0)
{
  unsigned seed = std::chrono::system_clock::now().time_since_epoch().count(); //different seed for different initial random number
  Eigen::EigenMultivariateNormal<double> initialConditionSampler(x0,P0,false,seed);

  _x = initialConditionSampler.samples(1);
  //std::cout<<"x initial is "<<_x<<std::endl;//each time it will just initalize from the same point

}

void Simulator::step(int count)
{
  _x = _processModel.predict(_x,count);
  _z = _observationModel.predict(_x);
}
