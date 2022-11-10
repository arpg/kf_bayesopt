#include "read_para_bayes.h"

ReadParaBayes::ReadParaBayes(){};

ReadParaBayes::ReadParaBayes(ros::NodeHandle nh){

    /* Nonparametric process "parameters" */
    if (nh.getParam("surr_name", surr_name_))
        ROS_INFO("surr_name: %s", surr_name_.c_str());
    else
        ROS_INFO("No surrogate name message");

    if (nh.getParam("kernel_name", kernel_name_))
        ROS_INFO("surr_name: %s", kernel_name_.c_str());
    else
        ROS_INFO("No kernel name message");

    if (nh.getParam("crit_name", crit_name_))
        ROS_INFO("acquisition funtion: %s", crit_name_.c_str());
    else
        ROS_INFO("No acquisition function name message");
    
    if (nh.getParam("kernel_theta", kernel_theta_))
        ROS_INFO("kernel_theta: %f", kernel_theta_);
    else
        ROS_INFO("No kernel_theta message name");

    if (nh.getParam("kernel_sigma", kernel_sigma_))
        ROS_INFO("kernel_theta: %f", kernel_sigma_);
    else
        ROS_INFO("No kernel_sigma message name");

    if (nh.getParam("mean_name", mean_name_))
        ROS_INFO("mean_name: %s", mean_name_.c_str());
    else
        ROS_INFO("No mean name message");

    if (nh.getParam("mean_mu", mean_mu_))
        ROS_INFO("mean_mu: %f", mean_mu_);
    else
        ROS_INFO("No mean_mu message name");

    if (nh.getParam("mean_sigma", mean_sigma_))
        ROS_INFO("mean_sigma: %f", mean_sigma_);
    else
        ROS_INFO("No mean_sigma message name");

    if (nh.getParam("prior_alpha", prior_alpha_))
        ROS_INFO("prior_alpha: %f", prior_alpha_);
    else
        ROS_INFO("No prior_alpha message name");

    if (nh.getParam("prior_beta", prior_beta_))
        ROS_INFO("prior_beta: %f", prior_beta_);
    else
        ROS_INFO("No prior_beta message name");

    if (nh.getParam("sigma", sigma_))
        ROS_INFO("sigma: %f", sigma_);
    else
        ROS_INFO("No sigma message name");

    if (nh.getParam("epsilon", epsilon_))
        ROS_INFO("epsilon: %f", epsilon_);
    else
        ROS_INFO("No epsilon message name");

    if (nh.getParam("noise", noise_))
        ROS_INFO("noise: %f", noise_);
    else
        ROS_INFO("No noise message name");


    if (nh.getParam("lower_bound", lower_bound_)){
        ROS_INFO("Got lower bound ");
    }
    else{
        ROS_INFO("no lower bound");
    }

    if (nh.getParam("upper_bound", upper_bound_)){
        if(upper_bound_.size() != lower_bound_.size()){
            ROS_FATAL("bounding box size must agree! lower bound size %zu, upper bound size %zu", lower_bound_.size(), upper_bound_.size());
        }
        else{
            ROS_INFO("Got upper bound");
        }
   }


    /* Algorithm parameters */
    if (nh.getParam("n_init_samples", n_init_samples_)) 
        ROS_INFO("Got initial sample number: %d", n_init_samples_);
    else
        ROS_INFO("no initial sample message name");

    if (nh.getParam("n_iterations", n_iterations_))
        ROS_INFO("Got iteration number: %d", n_iterations_);
    else
        ROS_INFO("No iteration number message name");

    if (nh.getParam("n_iteration_relearn", n_iteration_relearn_))
        ROS_INFO("Got iteration relearn number: %d", n_iteration_relearn_);
    else
        ROS_INFO("No iteration relearn message name");

    if (nh.getParam("n_inner_evaluation", n_inner_evaluation_))
        ROS_INFO("Got n_inner_evaluation number: %d", n_inner_evaluation_);
    else
        ROS_INFO("No n_inner_evaluation message name");

    if (nh.getParam("verbose_level", verbose_level_))
        ROS_INFO("verbose_level: %d", verbose_level_);
    else
        ROS_INFO("No verbose_level message name");

    if (nh.getParam("force_jump", force_jump_))
        ROS_INFO("force_jump: %d", force_jump_);
    else
        ROS_INFO("No force_jump message name");

    /* Logging and Files */
    if (nh.getParam("random_seed", random_seed_)){
        if(random_seed_>=0)
            ROS_INFO("random seed>=0. We'll use a fixed seed");
        else
            ROS_INFO("random seed<0. A time based seed is used");
    }
    else{
        ROS_INFO("No random seed message name");
    }

    if (nh.getParam("log_filename", log_filename_))
        ROS_INFO("log_filename: %s", log_filename_.c_str());
    else
        ROS_INFO("No log_filename name message");

    if (nh.getParam("load_filename", load_filename_))
        ROS_INFO("load_filename: %s", load_filename_.c_str());
    else
        ROS_INFO("No load_filename name message");

    if (nh.getParam("save_filename", save_filename_))
        ROS_INFO("save_filename: %s", save_filename_.c_str());
    else
        ROS_INFO("No save_filename name message");
}
