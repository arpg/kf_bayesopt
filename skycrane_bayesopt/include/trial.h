#pragma once

#include "vehicleSimulator.h"
#include "EKF.h"
#include <algorithm>

class trial
{
public:

  trial(readPara_vehicle& simulatorParameters,  readPara_vehicle& estimatorParameters);
  ~trial();

  void run();

  double getAverageNEES();
  double getAverageNIS();
  double getAverageToX0();
  void   setPlotFlag(bool flag);
  void   showAverageIniStateNoise(); //check if the average initial state noise and its covariance is similar to what we have set after a large run
  void   showVarainceIniStateNoise();
  std::vector<double> getStateHis();
  std::vector<double> getStateHis_gt();
  
private:

  // Simulator which generates the sequence
  Simulator _simulator;

  // Estimator which does the magic
  EKF _estimator;

  // Average NEES value
  double _averageNEES;
  double _averageNIS;
  double _averageToX0;

  double AllNEES = 0.0 ;
  double rowNEES = 0.0 ;
  double rowNIS  = 0.0 ;
  double AllNIS  = 0.0 ;
  double rowToX0 = 0.0 ;
  double AllToX0 = 0.0 ;

  bool plotFlag;

  std::vector<State> iniStateNoiseHis;

  std::vector<double> stateHis, stateHis_gt;//state history from kalman filter and ground truth history

  readPara_vehicle _param_vehicle;
};