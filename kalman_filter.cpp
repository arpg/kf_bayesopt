#include "kalman_filter.h"

using namespace ::Eigen;
using namespace ::std;

KalmanFilter::KalmanFilter(ReadParaVehicle& param_vehicle) :
  process_model_(param_vehicle, false),//false, no noise; true, noise
  observation_model_(param_vehicle, false)
{
}

void KalmanFilter::Initialize(const State& x0, const StateCovariance& P0)
{
  xest_ = x0;
  pest_ = P0;
}

void KalmanFilter::Step(const Observation& z,int count)
{
  // Predict
  process_model_.Predict(xest_,count);
  xpred_ = process_model_.get_prediction();
  ppred_ = process_model_.get_f()*pest_*process_model_.get_f().transpose() + process_model_.get_qd();

  // Compute the Kalman gain
  Eigen::Matrix<double, N, Me> ctmp = ppred_ * observation_model_.get_h().transpose();  
  sob_   = observation_model_.get_h() * ctmp + observation_model_.get_r();
  Eigen::Matrix<double, N, Me> k_gain = ctmp * sob_.inverse();
  
  //Innovation residual
  nu_ = z - observation_model_.Predict(xpred_);
  
  xest_ = xpred_ + k_gain * nu_;

  // Update the covariance using the Joseph form; should give some slight numerical stability
  Eigen::MatrixXd Xtmp = MatrixXd::Identity(N,N) - k_gain * observation_model_.get_h(); 
  pest_ = Xtmp*ppred_;
}
