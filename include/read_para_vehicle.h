#pragma once

#include <iostream>
#include <string>
#include <vector>
#include <eigen3/Eigen/Dense>
#include "ros/ros.h"

class ReadParaVehicle{
  public:
      ReadParaVehicle(ros::NodeHandle nh, bool showReadInfo);
      ReadParaVehicle();

      /*computer parameter*/
      int cpu_core_number_ ;

      /*basic parameters*/
      int    state_dof_; //state degree of freedom is 2 here
      int    observation_dof_;//observation degree of freedom is 1 here

      /*simulation time parameter*/
      double t_start_;//start of the simulation time, set as 0 normally
      double dt_;
      double t_end_;//end of the simulation time


      /*initial state*/
      double xi0_;
      double xidot0_;
      double y0_;
      double ydot0_;
      std::vector<double> disturbance_;
      std::string P0_; //input "identity" or "zero"

      /*Number of running simulators and EKF max step*/
      int nsimruns_ = 10;
      int max_iterations_;

      /*ground truth vehivle measurement noise and process noise, for simulator*/
      int dim_pn_;
      int dim_on_;
      std::vector<double> pnoise_;
      std::vector<double> onoise_;

      /*optimization choice and cost choice*/
      std::string optimization_choice_; //optimize "processNoise" or "observationNoise" or "both"
      std::string cost_choice_;

      /*tmp*/
      double tmpf = 17.32;
};

