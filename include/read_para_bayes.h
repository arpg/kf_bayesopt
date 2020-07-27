#pragma once

#include <iostream>
#include <string>
#include <vector>
#include <eigen3/Eigen/Dense>
#include "ros/ros.h"

class ReadParaBayes{
  public:
    ReadParaBayes(ros::NodeHandle nh);
    ReadParaBayes();

/* Nonparametric process "parameters" */
    std::string kernel_name_ ;
    double kernel_theta_     ;
    double kernel_sigma_     ;
    std::string mean_name_   ;
    double mean_mu_          ;
    double mean_sigma_       ;
    double prior_alpha_      ;
    double prior_beta_       ;
    double sigma_            ;
    double noise_            ;
    double epsilon_          ;
    std::string surr_name_   ;
    std::string crit_name_   ;

    /* Algorithm parameters */
    int n_iterations_            ;
    int n_init_samples_          ;
    int n_iteration_relearn_     ;
    int force_jump_              ;
    int n_inner_evaluation_      ;
    int random_seed_             ;
    std::vector<double> lower_bound_;
    std::vector<double> upper_bound_;  

    /* Logging and Files */
    int verbose_level_            ;
    std::string log_filename_        ;
    std::string save_filename_       ;
    std::string load_filename_ ;
};

