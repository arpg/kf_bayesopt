#ifndef __SIMULATOR_HPP__
#define __SIMULATOR_HPP__

#include "system_models.hpp"
#include "read_para_vehicle.h"


class Simulator
{
public:

  Simulator(ReadParaVehicle& parameters);

  void Initialize(const State& x0, const StateCovariance& P0);

  void Step(int count);

  State get_ini_state_noise();

  unsigned get_seed();

  const State& get_x() const
  {
    return x_;
  }

  const Observation& get_z() const
  {
    return z_;
  }

private:

  ProcessModel process_model_;

  ObservationModel observation_model_;

  State x_;

  State ini_state_noise_;

  Observation z_;

  unsigned seed_;
};

#endif