#include "KalmanFilter.hpp"

using namespace ::Eigen;
using namespace ::std;

KalmanFilter::KalmanFilter(const Parameters& parameters) :
  _processModel(parameters.processNoise, parameters.deltaT, false),//false, no noise; true, noise
  _observationModel(parameters.observationNoise, false)//false
{
}
//    processNoise      observationNoise is set in parameters.hpp
void KalmanFilter::initialize(const State& x0, const StateCovariance& P0)
{
  _xEst = x0;
  _PEst = P0;
}

void KalmanFilter::step(const Observation& z,int count)
{
  // Predict
  _xPred = _processModel.predict(_xEst,count);
  const Matrix2d& F = _processModel.getFD();
  _PPred = F * _PEst * F.transpose() + _processModel.getQD();// QD comes from PDF lecture 22 , the 7th slide. QD here is 1D process Moise

  // Compute the Kalman gain
  MatrixNM C = _PPred * _observationModel.getH().transpose();  
  ObservationNoiseCovariance S = _observationModel.getH() * C + _observationModel.getR();//R comes from PDF lecture 22 , the 7th slide. R here is 1D observation noise
  MatrixNM K = C * S.inverse();

  // Update the mean
  Observation nu = z - _observationModel.predict(_xPred);
  _xEst = _xPred + K * nu;

  // Update the covariance using the Joseph form; should give some slight numerical stability
  Matrix2d X = Matrix2d::Identity() - K * _observationModel.getH();
  _PEst = X * _PPred; //* X.transpose() + K * _observationModel.getR() * K.transpose();

  // Alternative versions
  // _PEst = X * _PPred;
  // _PEst = _PPred - K * S * K.transpose();
}
