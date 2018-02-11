#ifndef __TRIAL_HPP__
#define __TRIAL_HPP__

#include "Simulator.hpp"
#include "KalmanFilter.hpp"

class Trial
{
public:

  Trial(int trialNumber, const Parameters& simulatorParameters, const Parameters& estimatorParameters);

  void run();
  
private:

  int _trialNumber;
  
  // Simulator which generates the sequence
  Simulator _simulator;

  // Estimator which does the magic
  KalmanFilter _estimator;
};

#endif // __TRIAL_HPP__
