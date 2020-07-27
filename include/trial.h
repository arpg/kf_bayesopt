#ifndef __TRIAL_H__
#define __TRIAL_H__

#include "simulator.h"
#include "kalman_filter.h"

class Trial
{
public:

  Trial(ReadParaVehicle& simulator_parameters,  ReadParaVehicle& estimator_parameters);

  void Run();

  double get_average_nees() const
  {
    return average_nees_;
  }

  double get_average_nis() const
  {
    return average_nis_;
  }
  
  double det_p_ = 0;
  double det_s_ = 0;
private:
  
  // Simulator which generates the sequence
  Simulator simulator_;

  // Estimator which does the magic
  KalmanFilter estimator_;

  // Average NEES value
  double average_nees_;
  double average_nis_;

  double all_nees_ = 0 ;
  double row_nees_ = 0 ;
  double row_nis_  = 0 ;
  double all_nis_  = 0 ;

  double k_average_;

  ReadParaVehicle param_vehicle_;
};

#endif // __TRIAL_H__
