#include "EKF.h"

using namespace Eigen;

EKF::EKF(readPara_vehicle& param_vehicle)
        :_observationModel(param_vehicle, false), //false. No noise added. Kalman filter don't need ad noise. The simulator need it
        _processModel(param_vehicle, false), _controller(param_vehicle){};

void EKF::initialize(const State& x0, const StateCovariance& P0){
    _xEst = x0;
    _PEst = P0;
};

void EKF::step(const Observation& z,int count){
    //std::cout<<"\n \n count "<<count<<std::endl;
    //setFeedback control
    _controller.setFeedbackControl(_xEst,count);
    Control feedbackUin = _controller.getFeedbackControl();
    //std::cout<<"feedbackUin "<<feedbackUin(0,0)<<"  "<<feedbackUin(1,0)<<std::endl;
    // Predict
    // std::cout<<"_xEst k in EKF step "<<_xEst.transpose()<<std::endl;
    _processModel.setPrediction(feedbackUin, _xEst, count);
    _xPred = _processModel.getPrediction();
    //std::cout<<"_xPred in EKF step "<<_xPred.transpose()<<std::endl;

    //Jacobian of process JacobianProcess F = _processModel.getF();
    _processModel.setJacobian();
    //std::cout<<"dynamic jacobian in EKF step is \n"<<_processModel.getF()<<std::endl;
    _PPred = _processModel.getF()*_PEst*_processModel.getF().transpose() + _processModel.getW();
    //std::cout<<"_PPred \n "<<_PPred<<std::endl;
    //std::cout<<"Dyn jacobian \n"<<_processModel.getF()<<std::endl;
    //check if positive definite
    //  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigensolver(_PPred);
    //  std::cout << "The eigenvalues of P are:\n" << eigensolver.eigenvalues().transpose() << std::endl;

    //setFeedback control, need feedback control input at time k+1
    _controller.setFeedbackControl(_xPred,count);
    feedbackUin = _controller.getFeedbackControl();
    //std::cout<<"feedbackUin "<<feedbackUin(0,0)<<"  "<<feedbackUin(1,0)<<std::endl;
   //setObservation
    _observationModel.setObservation(feedbackUin, _xPred, count);
   //Jacobian of measurement , get H and R. Jacobian need use the new observation, so we need set it first
   _observationModel.setJacobian();
   //computer kalman gain, 6 by 6 matrix inverse
    Eigen::Matrix<double, N, Me> Ctmp  = _PPred * _observationModel.getH().transpose();
    _Sob   = _observationModel.getH() * Ctmp + _observationModel.getR();
    //std::cout<<"R is \n"<<_observationModel.getR()<<std::endl;
    //std::cout<<"H is \n"<<_observationModel.getH()<<std::endl;

    Eigen::Matrix<double, N, Me> Kgain = Ctmp * _Sob.inverse(); 
    //std::cout<<"Kalman gain is \n"<<Kgain<<std::endl;

    //Innovation residual
    _nu = z - _observationModel.getObservation();
    //std::cout<<"residual "<<_nu.transpose()<<std::endl;//<<","<<"z "<<z.transpose()<<", _observationModel.getObservation()  "
    //<<_observationModel.getObservation().transpose()<<std::endl;
    
    //update state estimation
    _xEst = _xPred + Kgain * _nu; 
    //std::cout<<"_xEst in EKF step "<<_xEst.transpose()<<"\n"<<std::endl;

    //update state covariance
    Eigen::MatrixXd Xtmp = MatrixXd::Identity(N,N) - Kgain * _observationModel.getH(); 
    _PEst = Xtmp*_PPred;
    //std::cout<<"_PEst in EKF step is \n"<<_PEst<<std::endl;//<<"\n and its inverse \n"<<_PEst.inverse()<<std::endl;
}
