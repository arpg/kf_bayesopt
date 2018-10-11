#pragma once

#include <iostream>
#include <string>
#include <vector>
#include <eigen3/Eigen/Dense>
#include "ros/ros.h"

class readPara_bayes{
  public:
    readPara_bayes(ros::NodeHandle nh);

/*Address config*/
    std::string slam_yaml_path ;//path to slam yaml file
    std::string rosbag_path;//path to rosbag

    //see medie folder commet about their use. The following three's value is got from launch file
    std::string start_signal_path;
    std::string cost_path;
    std::string optimization_history_path;

/* Nonparametric process "parameters" */
    std::string kernel_name ;
    double kernel_theta     ;
    double kernel_sigma     ;
    std::string mean_name   ;
    double mean_mu          ;
    double mean_sigma       ;
    double prior_alpha      ;
    double prior_beta       ;
    double sigma            ;
    double noise            ;
    double epsilon          ;
    std::string surr_name   ;
    std::string crit_name   ;

    /* Algorithm parameters */
    int n_iterations            ;
    int n_init_samples          ;
    int n_iteration_relearn     ;
    int force_jump              ;
    int n_inner_evaluation      ; /**< Used per dimmension */
    int random_seed             ;
    int slam_fail_threshold     ;
    int ransac_noise            ;
    int opt_dim                 ; //dimension of the parameters that need to be optimized
    std::vector<double> lower_bound;
    std::vector<double> upper_bound;  

    /* Logging and Files */
    int verbose_level            ;
    std::string log_filename        ;
    std::string save_filename       ;
    std::string load_filename ;
};

