#include "trial.h"
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float64.h>
#include <sysexits.h>
#include <iostream>
#include <fstream>
#include <thread>
#include <stdio.h>

using namespace std;

class runTrial{
public:
    runTrial(ros::NodeHandle& nh){
        nh_ = nh;
        pub_state_his =  nh.advertise<std_msgs::Float64MultiArray>("state_his",10);
    }
    void run(){

        bool show_read_info = true;
        ReadParaVehicle rv_gt(nh_, !show_read_info);
        ReadParaVehicle rv(nh_, !show_read_info);

        std::string filename("/home/zhaozhong/Desktop/nis_1d_dt01.csv");
        std::remove(filename.c_str());
        std::ofstream ofs;
        ofs.open (filename, std::ofstream::out | std::ofstream::app);

        double pnoise_est0 = 1.0, onoise_est0 = 0.1, pnoise_est1 = 1.0, onoise_est1 = 0.1;
        nh_.getParam("pnoise_est0", pnoise_est0);
        nh_.getParam("onoise_est0", onoise_est0);
        nh_.getParam("pnoise_est1", pnoise_est1);
        nh_.getParam("onoise_est1", onoise_est1);

        //std::cout<<"ros parameter "<<pnoise<<","<<onoise<<std::endl;

        rv.pnoise_[0] = pnoise_est0;
        rv.pnoise_[1] = pnoise_est1;
        rv.onoise_[0] = onoise_est0;
        rv.onoise_[1] = onoise_est1;

        std::cout<<"estimator process noise "<<rv.pnoise_[0]<<","<<rv.pnoise_[1]<<std::endl;
        std::cout<<"estimator observation noise "<<rv.onoise_[0]<<","<<rv.onoise_[1]<<std::endl;

        std::cout<<"simulator process noise "<<rv_gt.pnoise_[0]<<","<<rv_gt.pnoise_[1]<<std::endl;
        std::cout<<"simulator observation noise "<<rv_gt.onoise_[0]<<","<<rv_gt.onoise_[1]<<std::endl;
         

        for(int j = 1; j < 101; j++){
            //rv.pnoise_[0] = 0.05*j;
            // std::cout<<"estimator process noise "<<rv.pnoise_[0]<<","<<rv.pnoise_[1]<<std::endl;
            // std::cout<<"estimator observation noise "<<rv.onoise_[0]<<","<<rv.onoise_[1]<<std::endl;

            // std::cout<<"simulator process noise "<<rv_gt.pnoise_[0]<<","<<rv_gt.pnoise_[1]<<std::endl;
            // std::cout<<"simulator observation noise "<<rv_gt.onoise_[0]<<","<<rv_gt.onoise_[1]<<std::endl;

            std::vector<std::thread> th;
            std::vector<Trial> t;

            for(int i = 0; i<rv_gt.cpu_core_number_; i++)
                t.push_back(Trial(rv_gt,rv));   

            for(int i = 0; i<rv_gt.cpu_core_number_; i++)
                th.push_back(std::thread(&Trial::Run, &t[i])); 

            for(int i = 0; i<rv_gt.cpu_core_number_; i++) 
                th[i].join();

            std_msgs::Float64MultiArray state_his;

            state_his.layout.dim.push_back(std_msgs::MultiArrayDimension());
            state_his.layout.dim.push_back(std_msgs::MultiArrayDimension());
            state_his.layout.dim[0].size = N;
            state_his.layout.dim[0].label = "stateNumber"; 
            state_his.layout.dim[1].size = rv.max_iterations_;
            state_his.layout.dim[1].label = "iterations";

            //usleep(300000);
            //pub_state_his.publish(state_his);
            
            std_msgs::Float64 cost;

            if(rv.cost_choice_ == "JNEES" || rv.cost_choice_ == "CNEES"){
                double average_nees = 0.0;
                double average_var_nees = 0.0;
                for(int i = 0; i<rv_gt.cpu_core_number_; i++){
                    average_nees += t[i].get_average_nees()/rv_gt.cpu_core_number_;
                    average_var_nees += t[i].get_average_var_nees()/rv_gt.cpu_core_number_;
                }
                double tmp_J_NEES  = std::abs(log(average_nees/rv.state_dof_));
                double tmp_NEES_var = std::abs(log(0.5*average_var_nees/rv.state_dof_));
                double J_NEES = tmp_J_NEES;
                if(rv.cost_choice_ == "CNEES"){
                    J_NEES += tmp_NEES_var;
                }
                
                std::cout<<"cost JNEES "<<J_NEES<<", before log "<<average_nees<<std::endl;
                std::cout<<"variance NEES "<<", before log "<<average_var_nees<<", after log "<<std::abs(log(0.5*average_var_nees/rv.state_dof_))<<std::endl;
            }
            else if(rv.cost_choice_ == "JNIS" || rv.cost_choice_ == "CNIS"){
                double average_nis = 0.0;
                double average_var_nis = 0.0;
                for(int i = 0; i<rv_gt.cpu_core_number_; i++){
                    average_nis += t[i].get_average_nis()/rv_gt.cpu_core_number_;
                    average_var_nis += t[i].get_average_var_nis()/rv_gt.cpu_core_number_;
                }
                double tmp_J_NIS = std::abs(log(average_nis/rv.observation_dof_));
                double tmp_NIS_var = std::abs(log(0.5*average_var_nis/rv.observation_dof_));
                J_NIS  =  tmp_J_NIS;
                if(rv.cost_choice_ == "CNIS")
                    J_NIS += tmp_NIS_var;
                
                std::cout<<"average_nis "<<average_nis<<std::endl;
                std::cout<<"variance NIS "<<average_var_nis<<std::endl;
                std::cout<<"jnis and jvariance "<<tmp_J_NIS<<","<<tmp_NIS_var<<std::endl;
                ofs<<average_nis<<","<<average_var_nis<<std::endl;
            }
            
            //ofs<<rv.pnoise_[0]<<","<<J_NIS<<std::endl;
            
        }
        

        ofs.close();

    }

private:
    ros::NodeHandle nh_;
    ros::Publisher pub_state_his;
    double J_NEES,J_NIS;
};

int main(int nargs, char *args[])
{
    ros::init(nargs,args,"robot_kf");

    ros::NodeHandle n;

    runTrial rT(n);

    rT.run();
    
    return EX_OK;
}