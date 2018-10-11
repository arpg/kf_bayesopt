#include "readPara_bayes.h"

readPara_bayes::readPara_bayes(ros::NodeHandle nh){

    /*Address config*/
    if (nh.getParam("slam_yaml_path", slam_yaml_path))
        ROS_INFO("slam_yaml_path: %s", slam_yaml_path.c_str());
    else
        ROS_INFO("No yaml path of slam ");

    if (nh.getParam("rosbag_path", rosbag_path))
        ROS_INFO("rosbag_path: %s", rosbag_path.c_str());
    else
        ROS_INFO("No rosbag path of slam ");

    if (nh.getParam("start_signal", start_signal_path))
        ROS_INFO("start_signal_path: %s", start_signal_path.c_str());
    else
        ROS_INFO("No start_signal path of slam ");

    if (nh.getParam("cost", cost_path))
        ROS_INFO("cost_path: %s", cost_path.c_str());
    else
        ROS_INFO("No cost path of slam ");

    if (nh.getParam("optimization_history", optimization_history_path))
        ROS_INFO("optimization_history_path: %s", optimization_history_path.c_str());
    else
        ROS_INFO("No optimization_history_path path of slam ");

    /* Nonparametric process "parameters" */
    if (nh.getParam("surr_name", surr_name))
        ROS_INFO("surr_name: %s", surr_name.c_str());
    else
        ROS_INFO("No surrogate name message");

    if (nh.getParam("kernel_name", kernel_name))
        ROS_INFO("surr_name: %s", kernel_name.c_str());
    else
        ROS_INFO("No kernel name message");

    if (nh.getParam("crit_name", crit_name))
        ROS_INFO("acquisition funtion: %s", crit_name.c_str());
    else
        ROS_INFO("No acquisition function name message");
    
    if (nh.getParam("kernel_theta", kernel_theta))
        ROS_INFO("kernel_theta: %f", kernel_theta);
    else
        ROS_INFO("No kernel_theta message name");

    if (nh.getParam("kernel_sigma", kernel_sigma))
        ROS_INFO("kernel_theta: %f", kernel_sigma);
    else
        ROS_INFO("No kernel_sigma message name");

    if (nh.getParam("mean_name", mean_name))
        ROS_INFO("mean_name: %s", mean_name.c_str());
    else
        ROS_INFO("No mean name message");

    if (nh.getParam("mean_mu", mean_mu))
        ROS_INFO("mean_mu: %f", mean_mu);
    else
        ROS_INFO("No mean_mu message name");

    if (nh.getParam("mean_sigma", mean_sigma))
        ROS_INFO("mean_sigma: %f", mean_sigma);
    else
        ROS_INFO("No mean_sigma message name");

    if (nh.getParam("prior_alpha", prior_alpha))
        ROS_INFO("prior_alpha: %f", prior_alpha);
    else
        ROS_INFO("No prior_alpha message name");

    if (nh.getParam("prior_beta", prior_beta))
        ROS_INFO("prior_beta: %f", prior_beta);
    else
        ROS_INFO("No prior_beta message name");

    if (nh.getParam("sigma", sigma))
        ROS_INFO("sigma: %f", sigma);
    else
        ROS_INFO("No sigma message name");

    if (nh.getParam("epsilon", epsilon))
        ROS_INFO("epsilon: %f", epsilon);
    else
        ROS_INFO("No epsilon message name");

    if (nh.getParam("noise", noise))
        ROS_INFO("noise: %f", noise);
    else
        ROS_INFO("No noise message name");

    if (nh.getParam("opt_dim", opt_dim)){
        if(opt_dim<1)
            ROS_INFO("dimension of optimized parameter must be bigger than 1");
        else
            ROS_INFO("opt_dim: %d", opt_dim);
    }
    else
        ROS_INFO("No dimension of optimized parameter message");

    if (nh.getParam("lower_bound", lower_bound)){
        if(lower_bound.size() != opt_dim){
            ROS_INFO("bounding box size must be equal to opt_dim");
        }
        else{
            ROS_INFO("Got lower bound");
        }
   }

    if (nh.getParam("upper_bound", upper_bound)){
        if(upper_bound.size() != opt_dim){
            ROS_INFO("bounding box size must be equal to opt_dim");
        }
        else{
            ROS_INFO("Got upper bound");
        }
   }


    /* Algorithm parameters */
    if (nh.getParam("n_init_samples", n_init_samples)) 
        ROS_INFO("Got initial sample number: %d", n_init_samples);
    else
        ROS_INFO("no initial sample message name");

    if (nh.getParam("n_iterations", n_iterations))
        ROS_INFO("Got iteration number: %d", n_iterations);
    else
        ROS_INFO("No iteration number message name");

    if (nh.getParam("n_iteration_relearn", n_iteration_relearn))
        ROS_INFO("Got iteration relearn number: %d", n_iteration_relearn);
    else
        ROS_INFO("No iteration relearn message name");

    if (nh.getParam("n_inner_evaluation", n_inner_evaluation))
        ROS_INFO("Got n_inner_evaluation number: %d", n_inner_evaluation);
    else
        ROS_INFO("No n_inner_evaluation message name");

    if (nh.getParam("slam_fail_threshold", slam_fail_threshold))
        ROS_INFO("Got slam_fail_threshold number: %d", slam_fail_threshold);
    else
        ROS_INFO("No slam_fail_threshold message name");

    if (nh.getParam("ransac_noise", ransac_noise))//should be double, the original code needs to be modified. But that's ok, I don't think we need this number.
        ROS_INFO("Got ransac_noise number: %d", ransac_noise);
    else
        ROS_INFO("No ransac_noise message name");

    if (nh.getParam("verbose_level", verbose_level))
        ROS_INFO("verbose_level: %d", verbose_level);
    else
        ROS_INFO("No verbose_level message name");

    if (nh.getParam("force_jump", force_jump))
        ROS_INFO("force_jump: %d", force_jump);
    else
        ROS_INFO("No force_jump message name");

    /* Logging and Files */
    if (nh.getParam("random_seed", random_seed)){
        if(random_seed>=0)
            ROS_INFO("random seed>=0. We'll use a fixed seed");
        else
            ROS_INFO("random seed<0. A time based seed is used");
    }
    else{
        ROS_INFO("No random seed message name");
    }

    if (nh.getParam("log_filename", log_filename))
        ROS_INFO("log_filename: %s", log_filename.c_str());
    else
        ROS_INFO("No log_filename name message");

    if (nh.getParam("load_filename", load_filename))
        ROS_INFO("load_filename: %s", load_filename.c_str());
    else
        ROS_INFO("No load_filename name message");

    if (nh.getParam("save_filename", save_filename))
        ROS_INFO("save_filename: %s", save_filename.c_str());
    else
        ROS_INFO("No save_filename name message");
}
