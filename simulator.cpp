#include "simulator.h"

using namespace Eigen;

Simulator::Simulator(ReadParaVehicle& param_vehicle):observation_model_(param_vehicle, true), process_model_(param_vehicle, true){};

void Simulator::Initialize(const State& x0, const StateCovariance& P0)
{
  unsigned seed = std::chrono::system_clock::now().time_since_epoch().count(); //different seed for different initial random number
  Eigen::EigenMultivariateNormal<double> initialConditionSampler(Eigen::MatrixXd::Zero(N,1),P0,false,seed);

  x_ = x0 + initialConditionSampler.samples(1);
}

void Simulator::Step(int count)
{
  x_ = process_model_.Predict(x_,count);
  z_ = observation_model_.Predict(x_);
}

//compute the average and variance of random number. It should be similar to the parameters in initialConditionSampler
State Simulator::get_ini_state_noise(){
    return ini_state_noise_;
};

unsigned Simulator::get_seed(){
  return seed_;
}
