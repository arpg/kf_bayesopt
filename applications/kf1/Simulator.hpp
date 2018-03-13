#ifndef __SIMULATOR_HPP__
#define __SIMULATOR_HPP__

#include "SystemModels.hpp"
#include "Parameters.hpp"
#include <chrono>

class Simulator
{
public:

  Simulator(const Parameters& parameters);

  void initialize(const State& x0, const StateCovariance& P0);

  void step(int count);

  const State& getX() const
  {
    return _x;
  }

  const Observation& getZ() const
  {
    return _z;
  }

private:

  ProcessModel _processModel;

  ObservationModel _observationModel;

  State _x;

  Observation _z;
};

#endif // __SIMULATOR_HPP__
