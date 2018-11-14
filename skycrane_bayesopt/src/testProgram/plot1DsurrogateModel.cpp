/*
-------------------------------------------------------------------------
   This file is part of BayesOpt, an efficient C++ library for 
   Bayesian optimization.

   Copyright (C) 2011-2015 Ruben Martinez-Cantin <rmcantin@unizar.es>
 
   BayesOpt is free software: you can redistribute it and/or modify it 
   under the terms of the GNU Affero General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   BayesOpt is distributed in the hope that it will be useful, but 
   WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU Affero General Public License for more details.

   You should have received a copy of the GNU Affero General Public License
   along with BayesOpt.  If not, see <http://www.gnu.org/licenses/>.
------------------------------------------------------------------------
*/

#include <cmath>
#include <algorithm>
#include <unistd.h>
#include <boost/math/constants/constants.hpp>
#include <boost/numeric/ublas/assignment.hpp>
#include "bayesopt/bayesopt.hpp"
#include "specialtypes.hpp"
#include "param_loader.hpp"
#include "ros/ros.h"
#include <std_msgs/Float64MultiArray.h>

class ExampleOneD: public bayesopt::ContinuousModel
{
public:
  ExampleOneD(bayesopt::Parameters par, ros::NodeHandle& _nh, int _dim):
    ContinuousModel(1,par) {
      nh = _nh;
      dim = _dim;
      pub_sgAc  = nh.advertise<std_msgs::Float64MultiArray>("sgAc", 10);
    }

  double evaluateSample(const vectord& xin)
  {
    if (xin.size() != 1)
      {
	std::cout << "WARNING: This only works for 1D inputs." << std::endl
		  << "WARNING: Using only first component." << std::endl;
      }

    usleep(200000);    
    getAndPubSgAcDate();//Get and Pub surrogate model and acquisition function data. this is the data from last iteration

    double x = xin(0);
    return (x-0.3)*(x-0.3) + sin(20*x)*0.2;
  };

  bool checkReachability(const vectord &query)
  {return true;};

  void printOptimal()
  {
    std::cout << "Optimal:" << 0.23719 << std::endl;
  }

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
      std::vector<std::vector<double> > xx   = get2Dx();
      std::vector<std::vector<double> > yy   = get2Dmean();
      std::vector<std::vector<double> > slsl = get2DStdLowerBound();
      std::vector<std::vector<double> > susu = get2DStdUpperBound();
      std::vector<std::vector<double> > cc   = get2DAcquisitionFunctionValue();

      sgAc_msgs.layout.dim.push_back(std_msgs::MultiArrayDimension());//to assign that layout has two dimensions
      sgAc_msgs.layout.dim[1].size = xx.size();
      sgAc_msgs.layout.dim[1].label = "size";

      sgAc_msgs.data.insert(sgAc_msgs.data.end(),  xx[0].begin(),      xx[0].end());//in fact we just need one row of x becasue the coordinate is x square
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
      
      pub_sgAc.publish(sgAc_msgs);
    }
} 

private:
    ros::NodeHandle nh;
    ros::Publisher pub_sgAc; //publish surrogate model value and acuiqsition function value 
    int dim;
};

int main(int nargs, char *args[])
{
    ros::init(nargs,args,"plot1DsurrogateModel");

    ros::NodeHandle n;

    bayesopt::Parameters parameters;
    if(nargs > 1){
      if(!bayesopt::utils::ParamLoader::load(args[1], parameters)){
          std::cout << "ERROR: provided file \"" << args[1] << "\" does not exist" << std::endl;
          return -1;
      }
    }
    else{
    parameters.n_init_samples = 10;
    parameters.n_iterations = 80;
    parameters.surr_name = "sStudentTProcessNIG";
    parameters.crit_name = "cEI";//"cHedge(cEI,cLCB,cExpReturn,cOptimisticSampling)";
    //bayesopt::utils::ParamLoader::save("bo_oned.txt", parameters);
  }
  int dimention = 1;
  
  ExampleOneD opt(parameters,n,1);
  vectord result(1);
  opt.optimize(result);
  
  std::cout << "Result:" << result << std::endl;
  opt.printOptimal();

  return 0;
}
