#pragma once

#include "vehicleSimulator.h"
#include "EKF.h"

class trial
{
public:

  trial(readPara_vehicle& simulatorParameters,  readPara_vehicle& estimatorParameters);

  void run();

  double getAverageNEES();
  double getAverageNIS();
  
private:

  // Simulator which generates the sequence
  Simulator _simulator;

  // Estimator which does the magic
  EKF _estimator;

  // Average NEES value
  double _averageNEES;
  double _averageNIS;

  double AllNEES = 0 ;
  double rowNEES = 0 ;
  double rowNIS  = 0 ;
  double AllNIS  = 0 ;
  //vector<vector<double> > AllNEES;
  //vector<double> OneNEES

  readPara_vehicle _param_vehicle;
};