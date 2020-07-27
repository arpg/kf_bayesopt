#ifndef __KALMAN_FILTER_HPP__
#define __KALMAN_FILTER_HPP__

#include "system_models.hpp"

class KalmanFilter
{
  public:

  KalmanFilter(ReadParaVehicle& param_vehicle);

  void Initialize(const State& x0, const StateCovariance& P0);
  
  void Step(const Observation& z,int count);

  const State& get_xest() const
  {
    return xest_;
  }

  const StateCovariance& get_pest() const
  {
    return pest_;
  }

  const Observation& get_residual() const
  {
    return nu_;
  }

  const ObservationNoiseCovariance& get_s() const
  {
    return sob_;
  }

  private:

  State xest_;
  StateCovariance pest_;

  Observation nu_;
  ObservationNoiseCovariance sob_;

  State xpred_;
  StateCovariance ppred_;

  ProcessModel process_model_;
  ObservationModel observation_model_;
};

#endif

