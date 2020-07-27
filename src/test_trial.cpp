#include "trial.h"
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float64.h>
#include <sysexits.h>
#include <iostream>
#include <fstream>
#include <thread>

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

        rv.pnoise_[0] = 1.88725;
        rv.onoise_[0] = 0.747529;

        std::cout<<"estimator process noise "<<rv.pnoise_[0]<<std::endl;
        std::cout<<"estimator observation noise "<<rv.onoise_[0]<<std::endl;

        std::cout<<"simulator process noise "<<rv_gt.pnoise_[0]<<std::endl;
        std::cout<<"simulator observation noise "<<rv_gt.onoise_[0]<<std::endl;

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

        usleep(300000);
        pub_state_his.publish(state_his);
        
        std_msgs::Float64 cost;
        
        if(rv.cost_choice_ == "JNEES"){
            double average_nees = 0.0;
            for(int i = 0; i<rv_gt.cpu_core_number_; i++){
                average_nees += t[i].get_average_nees()/rv_gt.cpu_core_number_;
            }
            J_NEES  = std::abs(log(average_nees/rv.state_dof_));
            cost.data = J_NEES;
            std::cout<<"cost JNEES "<<J_NEES<<std::endl;
        }
        else if(rv.cost_choice_ == "JNIS"){
            double averahe_nis = 0.0;
            for(int i = 0; i<rv_gt.cpu_core_number_; i++){
                averahe_nis += t[i].get_average_nis()/rv_gt.cpu_core_number_;
            }
            J_NIS  = std::abs(log(averahe_nis/rv.observation_dof_));
            cost.data = J_NIS;
            std::cout<<"cost JNIS "<<J_NIS<<std::endl;
        }

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