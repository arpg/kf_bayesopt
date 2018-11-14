#define _USE_MATH_DEFINES
#include <cmath>
#include <algorithm>
#include <unistd.h>
#include <fstream>
#include <iomanip>
#include <map>
#include <Eigen/Dense>
#include <boost/math/constants/constants.hpp>
#include <boost/numeric/ublas/assignment.hpp>
#include "bayesopt/bayesopt.hpp"
#include "specialtypes.hpp"
#include "param_loader.hpp"
#include "readPara_bayes.h"
#include "readPara_vehicle.h"
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float64.h>

using namespace std;
using namespace Eigen;

#define dim_pn 3
#define dim_on 4

enum optimizationChoice{
  processNoise, //0
  observationNoise, //1
  all, //2
  processNoise1D, //3 optimize the first element of process noise, we can then visualize the result for 1d and 2d
  processNoise2D //4 optimize the second element of process noise
};

class skyCrane: public bayesopt::ContinuousModel
{
public:
    skyCrane(bayesopt::Parameters par, int Dim, ros::NodeHandle& _nh, readPara_vehicle& _rv):
      ContinuousModel(Dim,par) {
          rv        = _rv;
          dim       = Dim;
          nh        = _nh;
          pub_noise = nh.advertise<std_msgs::Float64MultiArray>("noise", 10);
          pub_sgAc  = nh.advertise<std_msgs::Float64MultiArray>("sgAc", 10);
          //subscribe in a class. If outside of a class, we need use object to take the place this pointer
          sub_cost  = nh.subscribe<std_msgs::Float64>("cost", 10, &skyCrane::costCallback,this);

          //to use switch case to decide what parameters to optimize 
          choice.insert(std::pair<std::string,optimizationChoice>("processNoise",processNoise));
          choice.insert(std::pair<std::string,optimizationChoice>("observationNoise",observationNoise));
          choice.insert(std::pair<std::string,optimizationChoice>("all",all));
          choice.insert(std::pair<std::string,optimizationChoice>("processNoise1D",processNoise1D));
          choice.insert(std::pair<std::string,optimizationChoice>("processNoise2D",processNoise2D));
          //mModel->getPrediction(10);
      }

    double evaluateSample(const vectord& xin)
    {
      //std::cout<<"xin "<<xin[0]<<xin[1]<<xin[2]<<std::endl;
      cost = -1;
      std_msgs::Float64MultiArray noise_msgs;

      noise_msgs.layout.dim.push_back(std_msgs::MultiArrayDimension());//to assign that layout has two dimensions
      noise_msgs.layout.dim.push_back(std_msgs::MultiArrayDimension());
      noise_msgs.layout.dim[0].size = dim_pn;
      noise_msgs.layout.dim[0].label = "processNoiseDimension"; 
      noise_msgs.layout.dim[1].size = dim_on;
      noise_msgs.layout.dim[1].label = "measurmentNoiseDimension";

      std::vector<double> data;//data to publish, including process noise and observation
      std::vector<double> dataOn;//data of observationNoise
      //std::cout<<choice.find(rv.optimizationChoice)->second<<" is the optmizationChoice....."<<std::endl;
      //switch case cannot be used for string so I use map
      switch(choice.find(rv.optimizationChoice)->second){
        case processNoise:
          data.insert(data.end(),xin.begin(),xin.end());
          dataOn = rv.onoise; //if just optimize the process noise, we use the default observation noise in the yaml file
          data.insert(data.end(),dataOn.begin(),dataOn.end());
          break;
        case observationNoise:
          data = rv.pnoise;
          dataOn.insert(dataOn.end(),xin.begin(),xin.end()); //optimize observation noise, process noise will use default ones
          data.insert(data.end(),dataOn.begin(),dataOn.end());
          break;
        case all:
          data.insert(data.end(),xin.begin(),xin.end());
          break;
        case processNoise1D:
          data.insert(data.end(),xin.begin(),xin.end()); //code looks the same as optimize the processNoise but now xin should 1d
          data.insert(data.end(),xin.begin(),xin.end());//data.push_back(rv.pnoise[1]); //temporarily change to dagnonal search
          data.insert(data.end(),xin.begin(),xin.end());//data.push_back(rv.pnoise[2]);
          dataOn = rv.onoise; //if just optimize the process noise, we use the default observation noise in the yaml file
          data.insert(data.end(),dataOn.begin(),dataOn.end());
          break;
        case processNoise2D:
          data.insert(data.end(),xin.begin(),xin.end()); //code looks the same as optimize the processNoise but now xin should 2d
          data.push_back(rv.pnoise[2]);
          dataOn = rv.onoise; //if just optimize the process noise, we use the default observation noise in the yaml file
          data.insert(data.end(),dataOn.begin(),dataOn.end());
          break;
        default:
          std::cout<<"Please choose optimization method!!!"<<std::endl;
          ros::shutdown();
          break;
      }
      //the vector data is always like this, std::vector<double> data = {DataOfprocessNoise,DataOfObservationNoise}
      noise_msgs.data = data;

      //maybe ros system cannot response that fast. We need publish one message once or twice then we need sleep for a short time
      usleep(300000);
      pub_noise.publish(noise_msgs);
      //std::cout<<"noise_mags data "<<noise_msgs.data[0]<<std::endl;

      std::cout<<"publish the noise........................."<<std::endl;

      usleep(200000);
      getAndPubSgAcDate();//this is the data from last iteration

      //when a message is published, the corresponding call back function will be stored into a queue.
      //when a ros spin is called, it will call the first one in the queue.
      //ros spinOnce will only check the if the queue has call back function for several hundred msecs
      //if the cost function costs too much time(publish to fast), then the ros spinOnce will just return because there is not enough
      //time for it to wait and check. Then when the cost function is finished, the callback function will not be executed
      //Here I add one more condition, when cost is -1 we continue check if there is callback function, the callback function
      //can change the cost so that we can jump out from the will loop. This can make us not lost one call back function 
      //when the cost function is too long.
      while(cost == -1){
        ros::spinOnce();
      }
      return cost;
    };

    void costCallback(const std_msgs::Float64::ConstPtr& msg){
      cost = msg->data;
      std::cout<<"get the cost, now change it to "<<cost<<"..........................\n \n"<<std::endl;
    }

    bool checkReachability(const vectord &query)
    {return true;};

  void getAndPubSgAcDate(){
    std_msgs::Float64MultiArray sgAc_msgs;
    sgAc_msgs.layout.dim.push_back(std_msgs::MultiArrayDimension());//to assign that layout has two dimensions
    sgAc_msgs.layout.dim[0].size = dim;
    sgAc_msgs.layout.dim[0].label = "dimension to plot"; 

    if(dim == 1){
      std::vector<double> x       = get1Dx();
      std::vector<double> y       = get1Dmean();
      std::vector<double> sl      = get1DStdLowerBound();
      std::vector<double> su      = get1DStdUpperBound();
      std::vector<double> c       = get1DAcquisitionFunctionValue();
      std::vector<double> sampleX = getSampleX(); //points that are really sampled(run the cost function and get the cost)
      std::vector<double> sampleY = getSampleY();

      sgAc_msgs.layout.dim.push_back(std_msgs::MultiArrayDimension());//to assign that layout has two dimensions
      sgAc_msgs.layout.dim[1].size = x.size();
      sgAc_msgs.layout.dim[1].label = "size";
      //data structure is x axis value, mean, lower bound, upper bound, and citeria
      sgAc_msgs.data.insert(sgAc_msgs.data.end(),  x.begin(),      x.end());
      sgAc_msgs.data.insert(sgAc_msgs.data.end(),  y.begin(),      y.end());
      sgAc_msgs.data.insert(sgAc_msgs.data.end(), sl.begin(),      sl.end());
      sgAc_msgs.data.insert(sgAc_msgs.data.end(), su.begin(),      su.end());
      sgAc_msgs.data.insert(sgAc_msgs.data.end(),  c.begin(),      c.end());
      sgAc_msgs.data.insert(sgAc_msgs.data.end(), sampleX.begin(), sampleX.end());
      sgAc_msgs.data.insert(sgAc_msgs.data.end(), sampleY.begin(), sampleY.end());

      pub_sgAc.publish(sgAc_msgs);
    }
    else if(dim == 2){
      std::vector<double> xx  = get1Dx(); //we just need 1D x because the coordinate is x square 
      std::vector<std::vector<double> > yy   = get2Dmean();
      std::vector<std::vector<double> > slsl = get2DStdLowerBound();
      std::vector<std::vector<double> > susu = get2DStdUpperBound();
      std::vector<std::vector<double> > cc   = get2DAcquisitionFunctionValue();
      std::vector<double> sampleX = getSampleX(); //points that are really sampled(run the cost function and get the cost)
      std::vector<double> sampleY = getSampleY();

      //std::cout<<"xx size "<<xx.size()<<std::endl;

      sgAc_msgs.layout.dim.push_back(std_msgs::MultiArrayDimension());//to assign that layout has two dimensions
      sgAc_msgs.layout.dim[1].size = xx.size();
      sgAc_msgs.layout.dim[1].label = "size";

      sgAc_msgs.data.insert(sgAc_msgs.data.end(),  xx.begin(),      xx.end());//in fact we just need one row of x becasue the coordinate is x square
      //copy all the other data to msgs
      //yy is like yy[i][j] is the mean value whose coordinate is (x[i], x[j]). Similar for the others
      //Normally we don't plot upper bound and lower bound for 2D or it will be hard to visualize
      for(int i = 0; i < xx.size(); i++)
        sgAc_msgs.data.insert(sgAc_msgs.data.end(),  yy[i].begin(),      yy[i].end()); 
      // for(int i = 0; i < xx.size(); i++)
      //   sgAc_msgs.data.insert(sgAc_msgs.data.end(),  slsl[i].begin(),      slsl[i].end()); 
      // for(int i = 0; i < xx.size(); i++)
      //   sgAc_msgs.data.insert(sgAc_msgs.data.end(),  susu[i].begin(),      susu[i].end()); 
      for(int i = 0; i < xx.size(); i++)
        sgAc_msgs.data.insert(sgAc_msgs.data.end(),  cc[i].begin(),      cc[i].end()); 

      sgAc_msgs.data.insert(sgAc_msgs.data.end(), sampleX.begin(), sampleX.end());
      sgAc_msgs.data.insert(sgAc_msgs.data.end(), sampleY.begin(), sampleY.end());
      
      pub_sgAc.publish(sgAc_msgs);
    }
  } 

private:
    ros::NodeHandle nh;
    ros::Publisher pub_noise;
    ros::Publisher pub_sgAc; //publish surrogate model value and acuiqsition function value 
    ros::Subscriber sub_cost;
    readPara_vehicle rv;
    std::map<std::string, optimizationChoice> choice;
    int dim;//dimention of optimized parameters;
    double cost;
};

int main(int nargs, char *args[])
{
  ros::init(nargs,args,"skyCrane_bayes");

  ros::NodeHandle n;

  readPara_bayes re(n);
  readPara_vehicle rv(n,false); //false: won't show the parameters that are read

  bayesopt::Parameters par;
  if(nargs > 1){
      if(!bayesopt::utils::ParamLoader::load(args[1], par)){
          std::cout << "ERROR: provided file \"" << args[1] << "\" does not exist" << std::endl;
          return -1;
      }
  }
  else{
      par = initialize_parameters_to_default();
      //par.crit_name = "cEIa"; //use annealed version, hope it won't accept a bad result that easily. Not work
      par.n_init_samples = re.n_init_samples;
      par.n_iterations = re.n_iterations;
      par.random_seed = re.random_seed;
      par.verbose_level = re.verbose_level;
      par.surr_name = re.surr_name;
      //par.epsilon = 0.2;
      //par.RANSAC_Noise = 2;
      par.noise = re.noise;
      //std::cout<<std::setprecision(10)<<re.noise<<"............................"<<std::endl;
      par.force_jump = re.force_jump;
      //about noise see the source code: in bayesoptbase about line 123 we can see if (std::pow(mYPrev - yNext,2) < mParameters.noise)
      //so if the yNext is similar to mYPrev, we say it stucks.
      } 
  skyCrane sC(par,re.opt_dim,n, rv);
  vectord result(re.opt_dim);

  boost::numeric::ublas::vector<double> lowerBound(re.opt_dim);
  boost::numeric::ublas::vector<double> upperBound(re.opt_dim);
  for(int i = 0; i<re.opt_dim; i++){
    lowerBound(i) = re.lower_bound[i];
    upperBound(i) = re.upper_bound[i];
  }
  sC.setBoundingBox(lowerBound,upperBound); 

  sC.optimize(result);
 
return 0;
}
