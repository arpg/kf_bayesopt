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
    runTrial(ros::NodeHandle& _nh){
        nh = _nh;
        pub_cost =  nh.advertise<std_msgs::Float64>("cost",10);
        pub_stateHis =  nh.advertise<std_msgs::Float64MultiArray>("stateHis",10);
    }
    void processNoiseCallback(const std_msgs::Float64MultiArray::ConstPtr& msg){
        last_bayes_iteration++;
        std::cout<<"get the noise....................."<<std::endl;
        bool showReadInfo = true;//if true, we'll show the information of the parameters we read from yaml
        readPara_vehicle rV_gt(nh, !showReadInfo);//this will use ground truth(orignal process noise and measurement noise in the yaml file)
        readPara_vehicle rV(nh,!showReadInfo);//this will usw the measurment noise and process noise updated by bayesopt, see code below

        int Npn = msg->layout.dim[0].size; //this is how many numbers that the process noise contain

        //msg->data is vector<double>, the first Npn number is about process noise the rest is about measurement noise
        std::vector<double> pn(msg->data.begin(), msg->data.begin()+Npn);
        std::vector<double> on(msg->data.begin()+Npn, msg->data.end());

        rV.pnoise = pn;//replace the estimator's process noise and observation noise by the data from bayesopt
        rV.onoise = on;

        std::cout<<"process noise "<<rV.pnoise[0]<<","<<rV.pnoise[1]<<","<<rV.pnoise[2]<<std::endl;
        std::cout<<"observation noise "<<rV.onoise[0]<<","<<rV.onoise[1]<<","<<rV.onoise[2]<<","<<rV.onoise[3]<<std::endl;
        
        /*create thread and object array*/
        std::vector<std::thread> th;
        std::vector<trial> t;

        for(int i = 0; i<rV_gt.CPUcoreNumber; i++)
            t.push_back(trial(rV_gt,rV));   

        //If we reach the last bayesopt iteration, we'll check the lqr controller's result
        if(last_bayes_iteration == rV.bayesoptTotalSampleNumber)
            t[0].setPlotFlag(true);
        else
            t[0].setPlotFlag(false);

        //be careful we cannot put t.push_back into the same loop as th.push_back. When we create thread, they are hoped to be created continiously
        for(int i = 0; i<rV_gt.CPUcoreNumber; i++)
            th.push_back(std::thread(&trial::run, &t[i])); 
        /*joint the thread*/
        for(int i = 0; i<rV_gt.CPUcoreNumber; i++) 
            th[i].join();

        //pub the last bayesopt iteration to plot the state to see if the result converge
        if(last_bayes_iteration == rV.bayesoptTotalSampleNumber){
            std_msgs::Float64MultiArray stateHis;

            stateHis.data = t[0].getStateHis();

            stateHis.layout.dim.push_back(std_msgs::MultiArrayDimension());//to assign that layout has two dimensions
            stateHis.layout.dim.push_back(std_msgs::MultiArrayDimension());
            stateHis.layout.dim[0].size = N;
            stateHis.layout.dim[0].label = "stateNumber"; 
            stateHis.layout.dim[1].size = rV.maxIterations;
            stateHis.layout.dim[1].label = "iterations";

            usleep(500000);
            pub_stateHis.publish(stateHis);
        }

        std_msgs::Float64 cost;
        
        if(rV.costChoice == "JNEES"){
            double averageNEES = 0.0;
            for(int i = 0; i<rV_gt.CPUcoreNumber; i++){
                averageNEES += t[i].getAverageNEES()/rV_gt.CPUcoreNumber;
            }
            J_NEES  = std::abs(log(averageNEES/rV.observationDOF));
            cost.data = J_NEES;
            std::cout<<"cost JNEES "<<J_NEES<<std::endl;
        }
        else if(rV.costChoice == "JNIS"){
            double averageNIS = 0.0;
            for(int i = 0; i<rV_gt.CPUcoreNumber; i++){
                averageNIS += t[i].getAverageNIS()/rV_gt.CPUcoreNumber;
            }
            J_NIS  = std::abs(log(averageNIS/rV.observationDOF));
            cost.data = J_NIS;
            std::cout<<"cost JNIS "<<J_NIS<<std::endl;
        }
        else if(rV.costChoice == "withRespectToX0"){
            double averageToX0 = 0.0;
            for(int i = 0; i<rV_gt.CPUcoreNumber; i++){
                averageToX0 += t[i].getAverageToX0()/rV_gt.CPUcoreNumber;
            }
            diffX0  = std::abs(log(averageToX0/rV.stateDOF));
            cost.data = diffX0;
            std::cout<<"cost, the difference with respect to x0 "<<diffX0<<std::endl;
        }

        pub_cost.publish(cost);
        
        std::cout<<"publish the cost.....................\n \n"<<std::endl;//<<"pubCount"<<pubCount<<std::endl;
    }

private:
    ros::NodeHandle nh;
    double J_NEES,J_NIS,diffX0;
    ros::Publisher pub_cost;
    ros::Publisher pub_stateHis;
    ros::Subscriber sub_noise;
    int last_bayes_iteration = 0;
};

int main(int nargs, char *args[])
{
    ros::init(nargs,args,"skyCrane_EKF");

    ros::NodeHandle n;

    runTrial rT(n);

    ros::Subscriber sub = n.subscribe("noise",10, &runTrial::processNoiseCallback, &rT);

    ros::spin();
    
    return EX_OK;
}