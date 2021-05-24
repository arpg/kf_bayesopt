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

  double all_var_nees = 0, all_var_nis = 0;

  std::vector<double> e1,e2;
  std::vector<double> ub1,ub2;
  std::vector<double> lb1,lb2;
  std::vector<double> time_step;

  for(int i=0;i<param_vehicle_.nsimruns_/param_vehicle_.cpu_core_number_;i++)
  {

    State x0 = State::Zero();
    StateCovariance P0 = StateCovariance::Identity();
  
    simulator_.Initialize(x0, P0);
    estimator_.Initialize(x0, P0);

    average_nees_ = 0;
    average_nis_ = 0;

    std::vector<double> his_nees;
    std::vector<double> his_nis;
    his_nees.reserve(param_vehicle_.max_iterations_);
    his_nis.reserve(param_vehicle_.max_iterations_);

    for (int step = 0; step < param_vehicle_.max_iterations_; ++step)
    {

      simulator_.Step(step);
      Observation z = simulator_.get_z();
      estimator_.Step(z,step);

      // chi^2 estimate
      State xerror = simulator_.get_x()-estimator_.get_xest();

      double chi2_nees = 0, chi2_nis = 0;

      /*tmp check the NEES and NIS value at the same time */
      chi2_nees = xerror.transpose()*estimator_.get_pest().inverse()*xerror;
      auto tmp  = estimator_.get_residual().transpose()*estimator_.get_s().inverse()*estimator_.get_residual();
      chi2_nis = tmp(0);

      row_nees_ += chi2_nees;
      row_nis_  += chi2_nis;

      his_nees.push_back(chi2_nees);
      his_nis.push_back(chi2_nis);

      /*if nsumruns == 1, we'll plot the 95% confidence figure*/
      if( param_vehicle_.nsimruns_ == 1){
          time_step.push_back(step*param_vehicle_.dt_);
          e1.push_back(xerror(0,0));
          e2.push_back(xerror(1,0));
          StateCovariance curr_pest = estimator_.get_pest();
          lb1.push_back(-2*sqrt(curr_pest(0,0)));
          lb2.push_back(-2*sqrt(curr_pest(1,1)));
          ub1.push_back(2*sqrt(curr_pest(0,0)));
          ub2.push_back(2*sqrt(curr_pest(1,1)));
      }

    }
    all_nees_ += row_nees_;
    all_nis_  += row_nis_;

    double ave_nees = row_nees_/param_vehicle_.max_iterations_;
    double ave_nis  = row_nis_/param_vehicle_.max_iterations_;

    double var_nees = 0, var_nis = 0;

    //calculate variance
    for(int i = 0; i<param_vehicle_.max_iterations_; i++){
      var_nees += (his_nees[i] - ave_nees) * (his_nees[i] - ave_nees);
      var_nis += (his_nis[i] - ave_nis) * (his_nis[i] - ave_nis);
    }

    var_nees /= param_vehicle_.max_iterations_- 1;
    var_nis /= param_vehicle_.max_iterations_- 1;
    all_var_nees += var_nees;
    all_var_nis += var_nis;

    //clear
    his_nees.clear();
    his_nis.clear();

    row_nees_ = 0;
    row_nis_  = 0;

     
  }
  
  average_var_nees_ = all_var_nees * param_vehicle_.cpu_core_number_/param_vehicle_.nsimruns_;
  average_var_nis_ = all_var_nis * param_vehicle_.cpu_core_number_/param_vehicle_.nsimruns_;
  average_nees_ = all_nees_/k_average_;
  average_nis_  = all_nis_/k_average_;

   /*plot error and 95 percent confidence interval*/
  if( param_vehicle_.nsimruns_ == 1){
      plt::subplot(2,1,1);
      plt::plot(time_step,e1);
      plt::plot(time_step,lb1);
      plt::plot(time_step,ub1);
      plt::ylabel("xi");
      plt::title("error with 95 percent confidence interval");
      plt::subplot(2,1,2);
      plt::plot(time_step,e2);
      plt::plot(time_step,lb2);
      plt::plot(time_step,ub2);
      plt::ylabel("xidot");
      plt::show();
  }
  
}
