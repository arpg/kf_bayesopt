#include "trial.h"
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float64.h>
#include <sysexits.h>
#include <iostream>
#include <fstream>

using namespace std;

class runTrial{
public:
    runTrial(ros::NodeHandle& _nh){
        nh = _nh;
        pub_cost =  nh.advertise<std_msgs::Float64>("cost",10);
    }
    void processNoiseCallback(const std_msgs::Float64MultiArray::ConstPtr& msg){
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
        
        trial t(rV_gt,rV);

        t.run();

        std_msgs::Float64 cost;
        if(rV.costChoice == "JNEES"){
            J_NEES = sqrt((log(t.getAverageNEES()/rV.stateDOF))*(log(t.getAverageNEES()/rV.stateDOF)));
            cost.data = J_NEES;
            std::cout<<"JNEES "<<J_NEES<<std::endl;
        }
        else{
            J_NIS  = sqrt((log(t.getAverageNIS()/rV.observationDOF))*(log(t.getAverageNIS()/rV.observationDOF)));
            cost.data = J_NIS;
            std::cout<<"JNIS "<<J_NIS<<std::endl;
        }

        pub_cost.publish(cost);
        
        std::cout<<"publish the cost.....................\n \n"<<std::endl;//<<"pubCount"<<pubCount<<std::endl;
    }

private:
    ros::NodeHandle nh;
    double J_NEES,J_NIS;
    ros::Publisher pub_cost;
    ros::Subscriber sub_noise;
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