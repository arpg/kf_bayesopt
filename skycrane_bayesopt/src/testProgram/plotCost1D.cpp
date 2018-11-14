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
        pub_cost_noise =  nh.advertise<std_msgs::Float64MultiArray>("costVsNoise",10);
    }
    void run(){

        bool showReadInfo = true;//if true, we'll show the information of the parameters we read from yaml
        readPara_vehicle rV_gt(nh, !showReadInfo);//this will use ground truth(orignal process noise and measurement noise in the yaml file)
        readPara_vehicle rV(nh, !showReadInfo);//this will usw the measurment noise and process noise updated by bayesopt, see code below

        rV.pnoise[0] = 0.0001;
        rV.pnoise[1] = 0.0001;
        rV.pnoise[2] = 0.0001;

        int runTimes = 1000;
        for(int i = 0; i<runTimes; i++){

            std::cout<<"estimator process noise "<<rV.pnoise[0]<<","<<rV.pnoise[1]<<","<<rV.pnoise[2]<<std::endl;
            std::cout<<"estimator observation noise "<<rV.onoise[0]<<","<<rV.onoise[1]<<","<<rV.onoise[2]<<","<<rV.onoise[3]<<std::endl;

            std::cout<<"simulator process noise "<<rV_gt.pnoise[0]<<","<<rV_gt.pnoise[1]<<","<<rV_gt.pnoise[2]<<std::endl;
            std::cout<<"simulator observation noise "<<rV_gt.onoise[0]<<","<<rV_gt.onoise[1]<<","<<rV_gt.onoise[2]<<","<<rV_gt.onoise[3]<<std::endl;
            
            /*create thread and object array*/
            std::vector<std::thread> th;
            std::vector<trial> t;

            for(int i = 0; i<rV_gt.CPUcoreNumber; i++)
                t.push_back(trial(rV_gt,rV));   

            t[0].setPlotFlag(true);//just choose one run so we can get the state to plot
            //be careful we cannot put t.push_back into the same loop as th.push_back. When we create thread, they are hoped to be created continiously
            for(int i = 0; i<rV_gt.CPUcoreNumber; i++)
                th.push_back(std::thread(&trial::run, &t[i])); 
            /*joint the thread*/
            for(int i = 0; i<rV_gt.CPUcoreNumber; i++) 
                th[i].join();

            //pub the last bayesopt iteration to plot the state to see if the result converge
            std_msgs::Float64MultiArray cost_noise;
            std::vector<double> cost_noise_data;

            cost_noise_data.push_back(rV.pnoise[0]);
            //cost_noise_data.push_back(i);//if you want to see the result of running one parameter for runtimes and plot, use this line
            
            cost_noise.layout.dim.push_back(std_msgs::MultiArrayDimension());//to assign that layout has two dimensions
            cost_noise.layout.dim[0].size = 2;
            if(i<runTimes - 1)
                cost_noise.layout.dim[0].label = "NotStartPlot"; 
            else
                cost_noise.layout.dim[0].label = "startPlot"; 
            
            if(rV.costChoice == "JNEES"){
                double averageNEES = 0.0;
                for(int i = 0; i<rV_gt.CPUcoreNumber; i++){
                    averageNEES += t[i].getAverageNEES()/rV_gt.CPUcoreNumber;
                }
                J_NEES  = std::abs(log(averageNEES/rV.observationDOF));
                cost_noise_data.push_back(J_NEES);
                std::cout<<"cost JNEES "<<J_NEES<<std::endl;
            }
            else if(rV.costChoice == "JNIS"){
                double averageNIS = 0.0;
                for(int i = 0; i<rV_gt.CPUcoreNumber; i++){
                    averageNIS += t[i].getAverageNIS()/rV_gt.CPUcoreNumber;
                }
                J_NIS  = std::abs(log(averageNIS/rV.observationDOF));
                cost_noise_data.push_back(J_NIS);
                std::cout<<"cost JNIS "<<J_NIS<<std::endl;
            }
            else if(rV.costChoice == "withRespectToX0"){
                double averageToX0 = 0.0;
                for(int i = 0; i<rV_gt.CPUcoreNumber; i++){
                    averageToX0 += t[i].getAverageToX0()/rV_gt.CPUcoreNumber;
                }
                diffX0  = std::abs(log(averageToX0/rV.stateDOF));
                cost_noise_data.push_back(diffX0);
                std::cout<<"cost, the difference with respect to x0 "<<diffX0<<std::endl;
            }
            cost_noise.data = cost_noise_data;

            //usleep(100000);
            pub_cost_noise.publish(cost_noise);

            rV.pnoise[0] += 0.0001;
            rV.pnoise[1] += 0.0001;
            rV.pnoise[2] += 0.0001;
        }

    }

private:
    ros::NodeHandle nh;
    ros::Publisher pub_cost_noise;
    double J_NEES,J_NIS, diffX0;
};

int main(int nargs, char *args[])
{
    ros::init(nargs,args,"skyCrane_EKF");

    ros::NodeHandle n;

    runTrial rT(n);

    rT.run();
    
    return EX_OK;
}