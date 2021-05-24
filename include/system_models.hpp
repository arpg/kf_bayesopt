#ifndef __SYSTEM_MODELS_HPP__
#define __SYSTEM_MODELS_HPP__

#include <cmath>
#include <vector>
#include "eigenmvn.h"
#include "continuous2discrete.hpp"
#include "read_para_vehicle.h"
#include <chrono>

#define N 2
#define Me 1 //1 before

typedef Eigen::Matrix<double, N, 1> State;
typedef Eigen::Matrix<double, N, N> StateCovariance;
typedef Eigen::Matrix<double, N, N> ProcessNoiseCovariance;
typedef Eigen::Matrix<double, N, N> JacobianProcess;

typedef Eigen::Matrix<double, Me, 1> Observation;
typedef Eigen::Matrix<double, Me, Me> ObservationNoiseCovariance;
typedef Eigen::Matrix<double, Me, N> JacobianObservation;

typedef Eigen::Matrix<double, N, Me> MatrixNM;
typedef Eigen::Matrix<double, Me, N> MatrixMN;

class ProcessModel
{
public:

  ProcessModel(ReadParaVehicle& param_vehicle, bool sample_process_noise = false)
  {
    sample_process_noise_ = sample_process_noise;
    Eigen::Matrix2d fc, qc;
    
    fc = Eigen::Matrix2d::Zero();
    
    fc(0, 1) = 1.0;//17.32;//when use 17.32, q(0,0) will be 3.3*10^-4 * 17.32^2 = 0.1, equal to qd(1,1)

    qc(1, 1) = param_vehicle.pnoise_[0];
    // qc(0, 0) = param_vehicle.pnoise_[0];
    // qc(0, 1) = param_vehicle.pnoise_[0];
    // qc(1, 0) = param_vehicle.pnoise_[0];

    //std::cout<<"qc is "<<qc<<std::endl;

    g_(0,0)   = 0.5 * param_vehicle.dt_ * param_vehicle.dt_;//0.5 * param_vehicle.dt_ * param_vehicle.dt_;
    g_(1,0)   = param_vehicle.dt_;//param_vehicle.dt_;
      
    double j = 0; 

    for(int i = 0;i<2000;i++)
      {
        u_.push_back(2*cos(0.75*j));//originally use 2*cos(0.75*j)
        j += param_vehicle.dt_; 
        //if(i<10) {std::cout<<"j is "<<j<<std::endl;std::cout<<"u is "<<u[i]<<std::endl;}
      }
    //
    continuousToDiscrete(fd_, qd_, fc, qc, param_vehicle.dt_);

    // if(sample_process_noise == false){
    //   std::cout<<"estimator"<<std::endl;
    //   std::cout<<"_Qd is \n"<<qd_<<std::endl;
    //   std::cout<<"_Fd is \n"<<fd_<<std::endl;
    //   ros::shutdown();
    //   exit(0);
    // }else{
    //   std::cout<<"simulator"<<std::endl;
    //   std::cout<<"_Qd is \n"<<qd_<<std::endl;
    //   std::cout<<"_Fd is \n"<<fd_<<std::endl;
    // }

    if (sample_process_noise == true)
      {
        unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
        noise_sampler_ = Eigen::EigenMultivariateNormal<double>(State::Zero(),qd_,false,seed);//qd_
      }
   //std::cout<<"_Qd is \n"<<_Qd<<std::endl;
  }

  State Predict(const State& xEst, int count)
  {
    xpred_ = fd_ * xEst+g_*u_[count];
    //if(count<10) { std::cout<<xPred<<std::endl; std::cout<<"u is... "<<u[count]<<std::endl; }//std::cout<<"u is "<<u[count]<<std::endl;
    //std::cout<<"count is "<<count<<std::endl;
    if (sample_process_noise_ == true)
      {
         State noi = noise_sampler_.samples(1);
         if(count<3)
         {
            //std::cout<<"xPred is "<<xPred<<std::endl;
            //std::cout<<"_Fd is \n"<<_Fd<<std::endl;
            //std::cout<<"G is \n"<<G<<std::endl;
            //std::cout<<"u is \n"<<u[count]<<std::endl;
            //std::cout<<"noise is \n"<<noi<<std::endl;
         }
          xpred_ += noi;
      }
   
    return xpred_;
  }



  State get_prediction(){
      return xpred_;
  }

  //jacobianProcess
  JacobianProcess get_f(){
      return fd_;
  }

  //processNoiseCovariance
  ProcessNoiseCovariance get_qd(){
      return qd_;
  }


private:

  Eigen::Matrix2d fd_;
  Eigen::Matrix2d qd_;
  Eigen::Matrix<double,2,1> g_; 
  State xpred_;
  std::vector<double> u_;
  std::vector<double> tvec_;

  bool sample_process_noise_;
  
  Eigen::EigenMultivariateNormal<double> noise_sampler_;
};

class ObservationModel
{
public:

  ObservationModel(ReadParaVehicle& param_vehicle, bool sample_observation_noise = false)
  {
    sample_observation_noise_ = sample_observation_noise;
    //original
    h_(0, 0) = 1;
    h_(0, 1) = 0;
    r_(0, 0) = param_vehicle.onoise_[0];//param_vehicle.onoise_[0]
    
    if (sample_observation_noise_ == true)
      {
        unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
        noise_sampler_ = Eigen::EigenMultivariateNormal<double>(Observation::Zero(),r_,false,seed);
      }
  }

  Observation Predict(const State& x)
  {
    observation_ = h_ * x;
    
    if (sample_observation_noise_ == true)
      {
        observation_ += noise_sampler_.samples(1);
      }

    return observation_;
  }

  const MatrixMN& get_h() const
  {
    return h_;
  }

  const ObservationNoiseCovariance& get_r() const
  {
    return r_;
  }

  Observation get_observation(){
      return observation_;
  }

private:

  MatrixMN h_;
  ObservationNoiseCovariance r_;

  Observation observation_;

  bool sample_observation_noise_;
  
  Eigen::EigenMultivariateNormal<double> noise_sampler_;
};

#endif
