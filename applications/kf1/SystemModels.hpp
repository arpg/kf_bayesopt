#ifndef __SYSTEM_MODELS_HPP__
#define __SYSTEM_MODELS_HPP__

#include <cmath>
#include <vector>
#include "eigenmvn.h"
#include "continuousToDiscrete.hpp"

#define N 2
#define M 1

typedef Eigen::Matrix<double, N, 1> State;
typedef Eigen::Matrix<double, N, N> StateCovariance;
typedef Eigen::Matrix<double, N, N> ProcessNoiseCovariance;

typedef Eigen::Matrix<double, M, M> Observation;
typedef Eigen::Matrix<double, M, M> ObservationNoiseCovariance;

typedef Eigen::Matrix<double, N, M> MatrixNM;
typedef Eigen::Matrix<double, M, N> MatrixMN;

class ProcessModel
{
public:

  ProcessModel(double processNoise, double deltaT, bool sampleProcessNoise = false)
  {
    _sampleProcessNoise = sampleProcessNoise;
    Eigen::Matrix2d Fc, Qc;
    
    Fc = Eigen::Matrix2d::Zero();
    
    Fc(0, 1) = 1;
    Qc(1, 1) = processNoise;
    //
    G(0,0)   = 0.005;//just for test dt =0.1
    G(1,0)   = 0.1;
      
    double j = 0; 

    for(int i = 0;i<2000;i++)
      {
        u.push_back(2*cos(0.75*j));
        j += deltaT; 
        //if(i<10) {std::cout<<"j is "<<j<<std::endl;std::cout<<"u is "<<u[i]<<std::endl;}
      }
    //
    continuousToDiscrete(_Fd, _Qd, Fc, Qc, deltaT);

    if (_sampleProcessNoise == true)
      {
	_noiseSampler.setMean(State::Zero());
	_noiseSampler.setCovar(_Qd);
      }
  }

  State predict(const State& xEst, int count)
  {
    State xPred = _Fd * xEst+G*u[count];
    //if(count<10) { std::cout<<xPred<<std::endl; }//std::cout<<"u is "<<u[count]<<std::endl;
    //std::cout<<"count is "<<count<<std::endl;
    if (_sampleProcessNoise == true)
      {
	xPred += _noiseSampler.samples(1);
      }
   
    return xPred;
  }

  const Eigen::Matrix2d& getFD() const
  {
    return _Fd;
  }

  const Eigen::Matrix2d& getQD() const
  {
    return _Qd;
  }


private:

  Eigen::Matrix2d _Fd;
  Eigen::Matrix2d _Qd;
  Eigen::Matrix<double,2,1> G; 
  std::vector<double> u;
  std::vector<double> tvec;

  bool _sampleProcessNoise;
  
  Eigen::EigenMultivariateNormal<double> _noiseSampler;
};

class ObservationModel
{
public:

  ObservationModel(double observationNoise, bool sampleObservationNoise = false)
  {
    _sampleObservationNoise = sampleObservationNoise;
    _H(0, 0) = 1;
    _H(0, 1) = 0;
    _R(0, 0) = observationNoise;
    
    if (_sampleObservationNoise == true)
      {
	_noiseSampler.setMean(Observation::Zero());
	_noiseSampler.setCovar(_R);
      }
  }

  Observation predict(const State& x)
  {
    Observation z = _H * x;
    
    if (_sampleObservationNoise == true)
      {
	z += _noiseSampler.samples(1);
	//std::cout << _noiseSampler.samples(10000).transpose() << std::endl;
	//exit(0);
      }

    return z;
  }

  const MatrixMN& getH() const
  {
    return _H;
  }

  const ObservationNoiseCovariance& getR() const
  {
    return _R;
  }


private:

  MatrixMN _H;
  ObservationNoiseCovariance _R;

  bool _sampleObservationNoise;
  
  Eigen::EigenMultivariateNormal<double> _noiseSampler;
};

#endif // __SYSTEM_MODELS_HPP__
