#include "trial.h"

#include <iostream>
#include <iomanip> 
#include <vector>
#include <sstream>
#include <fstream>
#include "matplotlibcpp.h"

namespace plt = matplotlibcpp;

Trial::Trial(ReadParaVehicle& simulator_parameters,  ReadParaVehicle& estimator_parameters)
    :  simulator_(simulator_parameters), estimator_(estimator_parameters), param_vehicle_(simulator_parameters){};


void Trial::Run()
{
  srand(time(NULL));
  k_average_ = param_vehicle_.max_iterations_*param_vehicle_.nsimruns_/param_vehicle_.cpu_core_number_;

  for(int i=0;i<param_vehicle_.nsimruns_/param_vehicle_.cpu_core_number_;i++)
  {

    State x0 = State::Zero();
    StateCovariance P0 = StateCovariance::Identity();
  
    simulator_.Initialize(x0, P0);
    estimator_.Initialize(x0, P0);

    average_nees_ = 0;
    average_nis_ = 0;

    for (int step = 0; step < param_vehicle_.max_iterations_; ++step)
    {

      simulator_.Step(step);
      Observation z = simulator_.get_z();
      estimator_.Step(z,step);

      // chi^2 estimate
      State xerror = simulator_.get_x()-estimator_.get_xest();

      double chi2_nees = 0, chi2_nis = 0;
      if(param_vehicle_.cost_choice_ == "JNEES")
          chi2_nees = xerror.transpose()*estimator_.get_pest().inverse()*xerror;
      else if(param_vehicle_.cost_choice_ == "JNIS"){   
          auto tmp  = estimator_.get_residual().transpose()*estimator_.get_s().inverse()*estimator_.get_residual();
          chi2_nis = tmp(0);
      }
      else{
          std::cout<<"not available choice............. "<<std::endl;
          exit(0);
      }

      row_nees_ += chi2_nees;
      row_nis_  += chi2_nis;

      all_nees_ += row_nees_;
      all_nis_  += row_nis_;

      row_nees_ = 0;
      row_nis_  = 0;

    }
     
  }
  
  average_nees_ = all_nees_/k_average_;
  average_nis_  = all_nis_/k_average_;
  
}
