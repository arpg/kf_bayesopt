#include <ros/ros.h>
#include "trial.h"
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float64.h>
#include <sysexits.h>
#include <iostream>
#include <fstream>
#include <thread>

using namespace std;

class RunTrial{
public:
    RunTrial(ros::NodeHandle& nh){
        nh_ = nh;
        pub_cost_ =  nh.advertise<std_msgs::Float64>("cost",10);
        pub_state_his_ =  nh.advertise<std_msgs::Float64MultiArray>("stateHis",10);
    }
    void ProcessNoiseCallback(const std_msgs::Float64MultiArray::ConstPtr& msg){
        std::cout<<"get the noise....................."<<std::endl;

        //if true, we'll show the information of the parameters we read from yaml
        bool show_read_info = true;
        ReadParaVehicle rv_gt(nh_, !show_read_info);
        ReadParaVehicle rv(nh_,!show_read_info);

        int npn = msg->layout.dim[0].size; //this is how many numbers that the process noise contain

        //msg->data is vector<double>, the first npn number is about process noise the rest is about measurement noise
        std::vector<double> pn(msg->data.begin(), msg->data.begin()+npn);
        std::vector<double> on(msg->data.begin()+npn, msg->data.end());

        //replace the estimator's process noise and observation noise by the data from bayesopt
        rv.pnoise_ = pn;
        rv.onoise_ = on;

        std::cout<<"process noise ";
        for(double np: rv.pnoise_){
            std::cout<<np<<" ";
        }
        std::cout<<std::endl;
        
        std::cout<<"observation noise "<<rv.onoise_[0]<<std::endl;

        std_msgs::Float64 cost;

        int it_num = 2;
        std::vector<double> pic_max(2,0.0);
        
        for(int k = 0; k<it_num; k++){
            //create thread and object array
            std::vector<std::thread> th;
            std::vector<Trial> t;

            for(int i = 0; i<rv_gt.cpu_core_number_; i++)
                t.push_back(Trial(rv_gt,rv));   


            //be careful we cannot put t.push_back into the same loop as th.push_back. When we create thread, they are hoped to be created continiously
            for(int i = 0; i<rv_gt.cpu_core_number_; i++)
                th.push_back(std::thread(&Trial::Run, &t[i])); 

            //joint the thread
            for(int i = 0; i<rv_gt.cpu_core_number_; i++) 
                th[i].join();

            
            if(rv.cost_choice_ == "JNEES"){
                double average_nees = 0.0;
                for(int i = 0; i<rv_gt.cpu_core_number_; i++){
                    average_nees += t[i].get_average_nees()/rv_gt.cpu_core_number_;
                }
                J_NEES  = std::abs(log(average_nees/rv.observation_dof_));
                //cost.data = J_NEES;
                pic_max[k] = J_NEES;
                std::cout<<"cost JNEES "<<J_NEES<<std::endl;
            }
            else if(rv.cost_choice_ == "JNIS"){
                double average_nis = 0.0;
                for(int i = 0; i<rv_gt.cpu_core_number_; i++){
                    average_nis += t[i].get_average_nis()/rv_gt.cpu_core_number_;
                }
                J_NIS  = std::abs(log(average_nis/rv.observation_dof_));

                pic_max[k] = J_NIS;
                std::cout<<"cost JNIS "<<J_NIS<<std::endl;
            }


            rv.dt_ = 1.0;
            rv_gt.dt_ = 1.0;
            rv.t_end_ = 201.0;
            rv_gt.t_end_ = 201.0;
            rv.max_iterations_ = (rv.t_end_- rv.t_start_)/rv.dt_;
            rv_gt.max_iterations_ = (rv_gt.t_end_ - rv_gt.t_start_)/rv_gt.dt_;
        }

        std::cout<<pic_max[0]<<","<<pic_max[1]<<std::endl;
        double max_cost = *max_element(pic_max.begin(), pic_max.end());
        cost.data = max_cost;

        pub_cost_.publish(cost);
        
        std::cout<<"publish the cost.....................\n \n"<<std::endl;
    }

private:
    ros::NodeHandle nh_;
    double J_NEES,J_NIS;
    ros::Publisher pub_cost_;
    ros::Publisher pub_state_his_;
    ros::Subscriber sub_noise_;
};

int main(int nargs, char *args[])
{
    ros::init(nargs,args,"robot_1d_kf");

    ros::NodeHandle n;

    RunTrial rT(n);

    ros::Subscriber sub = n.subscribe("noise",10, &RunTrial::ProcessNoiseCallback, &rT);

    ros::spin();
    
    return EX_OK;
}