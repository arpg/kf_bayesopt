#ifndef __KALMAN_FILTER_HPP__
#define __KALMAN_FILTER_HPP__

#include "SystemModels.hpp"
#include "Parameters.hpp"

class KalmanFilter
{
public:

  KalmanFilter(const Parameters& parameters);

  void initialize(const State& x0, const StateCovariance& P0);
  
  void step(const Observation& z,int count);

  const State& getXEst() const
  {
    return _xEst;
  }

  const StateCovariance& getPEst() const
  {
    return _PEst;
  }

  const Observation& getIn() const
  {
    return _nu;
  }

  const Observation& getPyy() const
  {
    return _Pyykp1;
  }

private:

  State _xEst;
  StateCovariance _PEst;

  State _xPred;
  StateCovariance _PPred;

  ProcessModel _processModel;
  ObservationModel _observationModel;

  Observation _nu;
  Observation _Pyykp1;
};

#endif //  __KALMAN_FILTER_HPP__

