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
#include "read_para_bayes.h"
#include "read_para_vehicle.h"
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float64.h>

using namespace std;
using namespace Eigen;

enum optimizationChoice{
  processNoise, //0
  observationNoise, //1
  all //2
};

class Robot1D: public bayesopt::ContinuousModel
{
public:
    Robot1D(bayesopt::Parameters par, int dim, ros::NodeHandle& nh, ReadParaVehicle& rv):
      ContinuousModel(dim,par) {
          rv_        = rv;
          dim_       = dim;
          nh_        = nh;
          pub_noise_ = nh.advertise<std_msgs::Float64MultiArray>("noise", 10);
          pub_sg_ac_  = nh.advertise<std_msgs::Float64MultiArray>("sgAc", 10);
          //subscribe in a class. If outside of a class, we need use object to take the place this pointer
          sub_cost_  = nh.subscribe<std_msgs::Float64>("cost", 10, &Robot1D::CostCallback,this);

          //to use switch case to decide what parameters to optimize 
          choice_.insert(std::pair<std::string,optimizationChoice>("processNoise",processNoise));
          choice_.insert(std::pair<std::string,optimizationChoice>("observationNoise",observationNoise));
          choice_.insert(std::pair<std::string,optimizationChoice>("all",all));
      }

    double evaluateSample(const vectord& xin)
    {
      if(!ros::ok())
        exit(0);
      cost_ = -1;
      it_counter_++;
      std_msgs::Float64MultiArray noise_msgs;

      //to assign that layout has two dimensions
      noise_msgs.layout.dim.push_back(std_msgs::MultiArrayDimension());
      noise_msgs.layout.dim.push_back(std_msgs::MultiArrayDimension());
      noise_msgs.layout.dim[0].size = rv_.pnoise_.size();
      noise_msgs.layout.dim[0].label = "processNoiseDimension"; 
      noise_msgs.layout.dim[1].size = rv_.onoise_.size();
      noise_msgs.layout.dim[1].label = "measurmentNoiseDimension";

      std::vector<double> data;//data to publish, including process noise and observation
      std::vector<double> dataOn;//data of observationNoise

      //switch case cannot be used for string so I use map
      switch(choice_.find(rv_.optimization_choice_)->second){
        case processNoise:
          data.insert(data.end(),xin.begin(),xin.end());
          dataOn = rv_.onoise_; //if just optimize the process noise, we use the default observation noise in the yaml file
          data.insert(data.end(),dataOn.begin(),dataOn.end());
          break;
        case observationNoise:
          data = rv_.pnoise_;
          dataOn.insert(dataOn.end(),xin.begin(),xin.end()); //optimize observation noise, process noise will use default ones
          data.insert(data.end(),dataOn.begin(),dataOn.end());
          break;
        case all:
          data.insert(data.end(),xin.begin(),xin.end());
          break;
        default:
          std::cout<<"Please choose optimization method!!!"<<std::endl;
          ros::shutdown();
          break;
      }

      //the vector data is always like this, std::vector<double> data = {DataOfprocessNoise,DataOfObservationNoise}
      std::cout<<"data is ";
      for(auto d:data){
        std::cout<<d<<",";
      }
      std::cout<<std::endl;
      noise_msgs.data = data;

      //maybe ros system cannot response that fast. We need publish one message once or twice then we need sleep for a short time
      if(first_run_){
        usleep(500000);//300000 previously
        first_run_ = false;
      }
      pub_noise_.publish(noise_msgs);

      std::cout<<"publish the noise........................."<<std::endl;

      /*UNCOMMENT if you want to see the plot online*/
      //usleep(200000);
      //get_pub_sg_ac_data();//this is the data from last iteration

      /*when a message is published, the corresponding call back function will be stored into a queue.
      when a ros spin is called, it will call the first one in the queue.
      ros spinOnce will only check the if the queue has call back function for several hundred msecs
      if the cost function costs too much time(publish to fast), then the ros spinOnce will just return because there is not enough
      time for it to wait and check. Then when the cost function is finished, the callback function will not be executed
      Here I add one more condition, when cost is -1 we continue check if there is callback function, the callback function
      can change the cost so that we can jump out from the will loop. This can make us not lost one call back function 
      when the cost function is too long.*/
      while(cost_ == -1 && ros::ok()){
        ros::spinOnce();
      }

      if(cost_< min_cost_)
        min_cost_ = cost_;
      return cost_;
    };

    void CostCallback(const std_msgs::Float64::ConstPtr& msg){
      cost_ = msg->data;
      std::cout<<"get the cost, now change it to "<<cost_<<"..........................\n \n"<<std::endl;
    }

    bool checkReachability(const vectord &query)
    {return true;}

    void get_pub_sg_ac_data(){
    std_msgs::Float64MultiArray sg_ac_msgs;
    sg_ac_msgs.layout.dim.push_back(std_msgs::MultiArrayDimension());//to assign that layout has two dimensions
    sg_ac_msgs.layout.dim[0].size = dim_;
    sg_ac_msgs.layout.dim[0].label = "dimension to plot"; 

    if(dim_ == 1){
      std::vector<double> x       = get1Dx();
      std::vector<double> y       = get1Dmean();
      std::vector<double> sl      = get1DStdLowerBound();
      std::vector<double> su      = get1DStdUpperBound();
      std::vector<double> c       = get1DAcquisitionFunctionValue();
      std::vector<double> sampleX = getSampleX(); //points that are really sampled(run the cost function and get the cost)
      std::vector<double> sampleY = getSampleY();

      sg_ac_msgs.layout.dim.push_back(std_msgs::MultiArrayDimension());//to assign that layout has two dimensions
      sg_ac_msgs.layout.dim[1].size = x.size();
      sg_ac_msgs.layout.dim[1].label = "size";
      //data structure is x axis value, mean, lower bound, upper bound, and criteria
      sg_ac_msgs.data.insert(sg_ac_msgs.data.end(),  x.begin(),      x.end());
      sg_ac_msgs.data.insert(sg_ac_msgs.data.end(),  y.begin(),      y.end());
      sg_ac_msgs.data.insert(sg_ac_msgs.data.end(), sl.begin(),      sl.end());
      sg_ac_msgs.data.insert(sg_ac_msgs.data.end(), su.begin(),      su.end());
      sg_ac_msgs.data.insert(sg_ac_msgs.data.end(),  c.begin(),      c.end());
      sg_ac_msgs.data.insert(sg_ac_msgs.data.end(), sampleX.begin(), sampleX.end());
      sg_ac_msgs.data.insert(sg_ac_msgs.data.end(), sampleY.begin(), sampleY.end());

      //There is a very strange phenomenon. After the last iteration I publish the data, the matplot figure wondow will
      //freeze(no response). I think it is because ros spin. Loop is in ros spin so I cannot do anything operating the figure. 
      //The way I solve this is by passing a last iteration symbol to
      //python code and tell it this is the last iteration(dim[2] == 1). Then you can see in the python code
      //I'll have a 9999...(very long pause) to make the figure window not freeze.
      sg_ac_msgs.layout.dim.push_back(std_msgs::MultiArrayDimension());//to assign that layout has two dimensions
      if(it_counter_ == mParameters.n_init_samples + mParameters.n_iterations -1) //mParameter is a public member of BayesoptBase class, which has been inherited
        sg_ac_msgs.layout.dim[2].size = 1;
      else 
        sg_ac_msgs.layout.dim[2].size = 0;
      sg_ac_msgs.layout.dim[2].label = "keep active";

      pub_sg_ac_.publish(sg_ac_msgs);
    }
    else if(dim_ == 2){
      std::vector<double> xx  = get2Dx(); //we just need 1D x because the coordinate is x square 
      std::vector<std::vector<double> > yy   = get2Dmean();
      std::vector<std::vector<double> > slsl = get2DStdLowerBound();
      std::vector<std::vector<double> > susu = get2DStdUpperBound();
      std::vector<std::vector<double> > cc   = get2DAcquisitionFunctionValue();
      std::vector<double> sampleX = getSampleX(); //points that are really sampled(run the cost function and get the cost)
      std::vector<double> sampleY = getSampleY();

      //std::cout<<"xx size "<<xx.size()<<std::endl;
      if(xx.size() != 0){
        sg_ac_msgs.layout.dim.push_back(std_msgs::MultiArrayDimension());//to assign that layout has two dimensions
        sg_ac_msgs.layout.dim[1].size = xx.size()/2;
        sg_ac_msgs.layout.dim[1].label = "size";
        //std::cout<<"the size of sampled X is "<<xx.size()<<std::endl;

        sg_ac_msgs.data.insert(sg_ac_msgs.data.end(),  xx.begin(),      xx.begin()+xx.size()/2);//in fact we just need one row of x becasue the coordinate is x square

        //copy all the other data to msgs
        //yy is like yy[i][j] is the mean value whose coordinate is (x[i], x[j]). Similar for the others
        //Normally we don't plot upper bound and lower bound for 2D or it will be hard to visualize
        for(int i = 0; i < xx.size()/2; i++){
          sg_ac_msgs.data.insert(sg_ac_msgs.data.end(),  yy[i].begin(),      yy[i].end());
        }

        for(int i = 0; i < xx.size()/2; i++){
          sg_ac_msgs.data.insert(sg_ac_msgs.data.end(),  cc[i].begin(),      cc[i].end());
        } 

        sg_ac_msgs.data.insert(sg_ac_msgs.data.end(), sampleX.begin(), sampleX.end());
        sg_ac_msgs.data.insert(sg_ac_msgs.data.end(), sampleY.begin(), sampleY.end());


        sg_ac_msgs.layout.dim.push_back(std_msgs::MultiArrayDimension());//to assign that layout has two dimensions

        //The matplot will freeze after the last iteration, so we need give a sigal to let it keep active(not exit the program)
        if(it_counter_ == mParameters.n_init_samples + mParameters.n_iterations -1){
          sg_ac_msgs.layout.dim[2].size = 1;
          std::cout<<"should keep active"<<std::endl;
        }
        else 
          sg_ac_msgs.layout.dim[2].size = 0;
        sg_ac_msgs.layout.dim[2].label = "keep active";
        
        pub_sg_ac_.publish(sg_ac_msgs);
      }
    }
  } 
  double min_cost_ = 99999;
private:
    ros::NodeHandle nh_;
    ros::Publisher pub_noise_;
    ros::Publisher pub_sg_ac_; //publish surrogate model value and acuiqsition function value 
    ros::Subscriber sub_cost_;
    ReadParaVehicle rv_;
    std::map<std::string, optimizationChoice> choice_;
    int dim_;//dimention of optimized parameters;
    double cost_;
    int it_counter_ = 0; //iteration number
    bool first_run_ = true;
};

int main(int nargs, char *args[])
{
  ros::init(nargs,args,"robot1d_bayes");

  ros::NodeHandle n;

  ReadParaBayes re(n);
  ReadParaVehicle rv(n,false); //false: won't show the parameters that are read

  bayesopt::Parameters par;
  if(nargs > 1){
      if(!bayesopt::utils::ParamLoader::load(args[1], par)){
          std::cout << "ERROR: provided file \"" << args[1] << "\" does not exist" << std::endl;
          return -1;
      }
  }
  else{
        par = initialize_parameters_to_default();
        par.n_init_samples = re.n_init_samples_;
        par.n_iterations = re.n_iterations_;
        par.random_seed = re.random_seed_;
        par.verbose_level = re.verbose_level_;
        par.surr_name = re.surr_name_;
        par.noise = re.noise_;
        par.force_jump = re.force_jump_;
        par.init_method = 1;//1 for Latin Hypercube Sampling, default, 2 for Sobol Sequence, >2 for uniform sampling. >2 is not good
      }

  int opt_dim = re.lower_bound_.size();
  Robot1D sc(par,opt_dim,n, rv);
  vectord result(opt_dim);

  boost::numeric::ublas::vector<double> lower_bound(opt_dim);
  boost::numeric::ublas::vector<double> upper_bound(opt_dim);
  for(int i = 0; i<opt_dim; i++){
    lower_bound(i) = re.lower_bound_[i];
    upper_bound(i) = re.upper_bound_[i];
  }
  sc.setBoundingBox(lower_bound,upper_bound); 

  sc.optimize(result);

  std::string filename("/home/zhaozhong/Desktop/opt_result.csv");
  std::cout<<"final optimization result ";
  for(auto r:result)
    std::cout<<r<<",";
  std::cout<<std::endl;
  //std::remove(filename.c_str());
  std::ofstream ofs;
  ofs.open (filename, std::ofstream::out | std::ofstream::app);
  //double opt_cost = sc.evaluateSample(result);//the opt cost is random so we cant run the program and get the cost again. It would be different
  for(auto r:result)
    ofs<<r<<",";
  ofs<<sc.min_cost_<<std::endl;
 
return 0;
}
