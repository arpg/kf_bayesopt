#include "EKF.h"

using namespace Eigen;

EKF::EKF(readPara_vehicle& param_vehicle)
        :_observationModel(param_vehicle),
        _processModel(param_vehicle){};

void EKF::initialize(const State& x0, const StateCovariance& P0){

    State perturb_x0;
    perturb_x0<<0.0,0.2,0.0,0.0,0.0,0.001;//just use a certain initial state to test(same as Matlab code)
    _xEst = x0 + perturb_x0;
    _PEst = P0;
};

void EKF::step(const Observation& z,int count){
    // Predict
    _processModel.setPrediction(_xEst,count);
    _xPred = _processModel.getPrediction();
    //std::cout<<"_xPred in EKF step "<<_xPred.transpose()<<std::endl;

    //Jacobian of process JacobianProcess F = _processModel.getF();
    _processModel.setJacobian();
    //std::cout<<"dynamic jacobian in EKF step is \n"<<_processModel.getF()<<std::endl;
    _PPred = _processModel.getF()*_PEst*_processModel.getF().transpose() + _processModel.getW();

    //check if positive definite
    //  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigensolver(_PPred);
    //  std::cout << "The eigenvalues of P are:\n" << eigensolver.eigenvalues().transpose() << std::endl;

   //setObservation
    _observationModel.setObservation(_xPred,count);
   //Jacobian of measurement , get H and R. Jacobian need use the new observation, so we need set it first
   _observationModel.setJacobian();
   //computer kalman gain, 6 by 6 matrix inverse
    Eigen::Matrix<double, N, Me> Ctmp  = _PPred * _observationModel.getH().transpose();
    _Sob   = _observationModel.getH() * Ctmp + _observationModel.getR();
    //std::cout<<"H is \n"<<_observationModel.getH()<<std::endl;

    Eigen::Matrix<double, N, Me> Kgain = Ctmp * _Sob.inverse(); 

    //Innovation residual
    _nu = z - _observationModel.getObservation();
    //std::cout<<"residual "<<_nu.transpose()<<","<<"z "<<z.transpose()<<", _observationModel.getObservation()  "
    //<<_observationModel.getObservation().transpose()<<std::endl;
    
    //update state estimation
    _xEst = _xPred + Kgain * _nu; 
    //std::cout<<"_xEst in EKF step "<<_xEst.transpose()<<std::endl;

    //update state covariance
    Eigen::MatrixXd Xtmp = MatrixXd::Identity(N,N) - Kgain * _observationModel.getH(); 
    _PEst = Xtmp*_PPred;
    //std::cout<<"_PEst in EKF step is \n"<<_PEst<<std::endl;
}