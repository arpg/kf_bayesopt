#pragma once

//#include "vehicleModel.h"
#include "lqrController.h"

class EKF
{
public:

  EKF(readPara_vehicle& param_vehicle);

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

  const Observation& getResidual() const
  {
    return _nu;
  }

  const ObservationNoiseCovariance& getS() const
  {
    return _Sob;
  }

private:

  State _xEst;
  StateCovariance _PEst;

  Observation _nu;
  ObservationNoiseCovariance _Sob;

  State _xPred;
  StateCovariance _PPred;

  processModel _processModel;
  observationModel _observationModel;

  controller _controller;
};