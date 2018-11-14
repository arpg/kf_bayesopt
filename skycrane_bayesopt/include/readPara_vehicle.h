#pragma once

#include <iostream>
#include <string>
#include <vector>
#include <eigen3/Eigen/Dense>
#include "ros/ros.h"

class readPara_vehicle{
  public:
      readPara_vehicle(ros::NodeHandle nh, bool showReadInfo);
      readPara_vehicle();

      /*computer parameter*/
      int CPUcoreNumber ;

      /*basic parameters*/
      double pi = 3.14159265359;
      double beta;//angle of the propellant to the body
      double g; //Gravity
      double rho;//Planet surface atmosphere density 
      double C_D; //Drag Coefficient
      double m_b; //mass of EDL system + rover (w/o heatshield or backshell)
      double m_f; //mass of fuel
      double w_b; //width of the sky crane body
      double h_b; //height of the sky crane body
      double d_b; //depth of the sky crane body
      double w_f; //width of the propellant housing
      double h_f; //height of the propellant housing
      double d_f; //housing depth
      int    stateDOF; //state degree of freedom is 6 here
      int    observationDOF;//observation degree of freedom is 4 here
      int    controlDOF;//control input degree of freedom. 2 Here
      int    bayesoptTotalSampleNumber; //When the bayesopt reach this number, we'll get a plot about the state

      /*simulation time parameter*/
      double t_start;//start of the simulation time, set as 0 normally
      double dt;
      double t_end;//end of the simulation time

      /*Parameter computed by the parameters above*/
      double h_cm; //height of the body center of the mass
      double w_cm; //width of the body center of the mass
      double Ieta; //moment of inertia 
      double Aside; //side area
      double Abot; //bottom or top area
      int length;//(t_end - t_start)/dt

      /*initial state*/
      double z0;
      double theta0;
      double xi0;
      double xidot0;
      double zdot0;
      double thetadot0;
      std::vector<double> disturbance;
      std::string P0; //input "identity" or "zero"
      std::string P0_BASE;

      /*Number of running simulators and EKF max step*/
      int Nsimruns = 10;
      int maxIterations;

      /*ground truth vehivle measurement noise and process noise, for simulator*/
      int dim_pn;
      int dim_on;
      std::vector<double> pnoise;
      std::vector<double> onoise;

      /*optimization choice and cost choice*/
      std::string optimizationChoice; //optimize "processNoise" or "observationNoise" or "both"
      std::string costChoice;

      /*controller*/
      std::vector<double> Klin;
};

