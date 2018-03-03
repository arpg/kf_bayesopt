#ifndef __TRIAL_HPP__
#define __TRIAL_HPP__

#include "Simulator.hpp"
#include "KalmanFilter.hpp"


class Trial
{
public:

  Trial(int trialNumber, const Parameters& simulatorParameters, const Parameters& estimatorParameters);

  void run();

  double getAverageNEES() const
  {
    return _averageNEES;
  }
  
private:

  int _trialNumber;

  int Nsimruns = 10;
  
  int maxIterations=201;
  
  // Simulator which generates the sequence
  Simulator _simulator;

  // Estimator which does the magic
  KalmanFilter _estimator;

  // Average NEES value
  double _averageNEES;

  double AllNEES = 0 ;
  double rowNEES = 0 ;
  //vector<vector<double> > AllNEES;
  //vector<double> OneNEES
};

#endif // __TRIAL_HPP__
