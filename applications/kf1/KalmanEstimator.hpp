#ifndef __KALMAN_FILTER_HPP__
#define __KALMAN_FILTER_HPP__

#include "SystemModels.hpp"

class KalmanFilter
{
public:

  KalmanFilter(const Parameters& parameters);

private:

  ProcessModel _processModel;
  ObservationModel _observationModel;
};

#endif //  __KALMAN_FILTER_HPP__

