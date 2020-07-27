#include "read_para_vehicle.h"

ReadParaVehicle::ReadParaVehicle(ros::NodeHandle nh, bool showReadInfo){
    /*If you want to modify this code, please remember to modify the code when showReadInfo is false also!*/
    if(showReadInfo){
        ROS_INFO("reading the parameters of the vehicle");
        /*computer parameter*/
        if (nh.getParam("CPUcoreNumber", cpu_core_number_))
            ROS_INFO("CPUcoreNumber: %d", cpu_core_number_);
        else
            ROS_INFO("No parameters CPUcoreNumber");

        /*basic parameter*/
        if (nh.getParam("stateDOF", state_dof_))
            ROS_INFO("stateDOF: %d", state_dof_);
        else
            ROS_INFO("No parameters stateDOF");

        if (nh.getParam("observationDOF", observation_dof_))
            ROS_INFO("observationDOF: %d", observation_dof_);
        else
            ROS_INFO("No parameters observationDOF");


        /*simulation time parameter*/
        if (nh.getParam("dt", dt_))
            ROS_INFO("dt: %f", dt_);
        else
            ROS_INFO("No parameters dt");

        if (nh.getParam("t_start", t_start_))
            ROS_INFO("t_start: %f", t_start_);
        else
            ROS_INFO("No parameters t_start");

        if (nh.getParam("t_end", t_end_))
            ROS_INFO("t_end: %f", t_end_);
        else
            ROS_INFO("No parameters t_end");

        /*Parameter computed by the parameters above*/
        max_iterations_ = (t_end_ - t_start_)/dt_;
        ROS_INFO("max iteration : %d", max_iterations_);

        /*initial state*/
        if (nh.getParam("xi0", xi0_))
            ROS_INFO("xi0: %f", xi0_);
        else
            ROS_INFO("No parameters xi0");

        if (nh.getParam("xidot0", xidot0_))
            ROS_INFO("xidot0: %f", xidot0_);
        else
            ROS_INFO("No parameters xidot0");

        if (nh.getParam("disturbance", disturbance_)){
            if(disturbance_.size() != state_dof_){
                ROS_INFO("disturbance size must equal to stateDOF");
            }
            else{
                ROS_INFO("got disturbance");
            }
        }

        if (nh.getParam("P0", P0_))
            ROS_INFO("P0: %s", P0_.c_str());
        else
            ROS_INFO("No parameters P0");

        /*Number of running simulators and EKF max step*/
        if (nh.getParam("Nsimruns", nsimruns_))
            ROS_INFO("Nsimruns: %d", nsimruns_);
        else
            ROS_INFO("No parameters Nsimruns");

        if (nsimruns_%cpu_core_number_ != 0)
            ROS_WARN("the nsimruns should be an integral multiple of cpu core number, or the final result may be slightly off");
       

        /*gorund truth vehicle noise parameter for simulator, optimizationChice and costChoice*/
        if (nh.getParam("dim_pn", dim_pn_))
            ROS_INFO("dim_pn: %d", dim_pn_);
        else
            ROS_INFO("No parameters dim_pn");
        
        if (nh.getParam("dim_on", dim_on_))
            ROS_INFO("dim_on: %d", dim_on_);
        else
            ROS_INFO("No parameters dim_on");

        if (nh.getParam("pnoise", pnoise_)){
            ROS_INFO("Got process noise");
        }
        else{
            ROS_FATAL("process noise must be set");
        }

        if (nh.getParam("onoise", onoise_)){
            ROS_INFO("Got observation noise");
        }
        else{
            ROS_FATAL("observation noise must be set");
        }

        dim_pn_ = pnoise_.size();
        dim_on_ = onoise_.size();

        if(nh.getParam("optimizationChoice",optimization_choice_))
            ROS_INFO("optimizationChoice is : %s", optimization_choice_.c_str());
        else
            ROS_INFO("no parameter optimizationChoice");

        if(nh.getParam("costChoice",cost_choice_))
            ROS_INFO("costChoice is : %s", cost_choice_.c_str());
        else
            ROS_INFO("no parameter costChoice");

    }
    else{
        /*computer parameter*/
        nh.getParam("CPUcoreNumber", cpu_core_number_);

        /*basic parameter*/
        nh.getParam("stateDOF", state_dof_);
        nh.getParam("observationDOF", observation_dof_);

        /*simulation time parameter*/
        nh.getParam("dt", dt_);
        nh.getParam("t_start", t_start_);
        nh.getParam("t_end", t_end_);

        /*initial state*/
        nh.getParam("xi0", xi0_);
        nh.getParam("xidot0", xidot0_);
        nh.getParam("P0", P0_);
        nh.getParam("disturbance", disturbance_);

        /*Number of running simulators and EKF max step*/
        nh.getParam("Nsimruns", nsimruns_);

        if (nsimruns_%cpu_core_number_ != 0)
            ROS_WARN("the nsimruns should be an integral multiple of cpu core number, or the final result may be slightly off");

        /*Parameter computed by the parameters above*/
        max_iterations_ = (t_end_ - t_start_)/dt_;

        /*gorund truth vehicle noise parameter for simulator*/
        nh.getParam("pnoise", pnoise_);
        nh.getParam("onoise", onoise_);
        dim_pn_ = pnoise_.size();
        dim_on_ = onoise_.size();

        /*optimization choice*/
        nh.getParam("optimizationChoice",optimization_choice_); //optimize "processNoise" or "observationNoise" or "both"

        /*cost choice, JNEES or JNIS*/
        nh.getParam("costChoice",cost_choice_);
    }
}

ReadParaVehicle::ReadParaVehicle(){};