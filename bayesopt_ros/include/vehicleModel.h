#pragma once

#include <cmath>
#include <Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions>
#include <vector>
#include <boost/numeric/odeint.hpp>
#include "eigenmvn.h"
#include "readPara_vehicle.h"

#define N 6
#define Me 4 //in ros, something is defined as M. Avoid the confliction
#define P 3 
#define C 2

typedef Eigen::Matrix<double, N, 1> State;
typedef Eigen::Matrix<double, N, N> StateCovariance;
typedef Eigen::Matrix<double, N, N> ProcessNoiseCovariance;//has to be 6 so that I can add Q to FPF in EKF
typedef Eigen::Matrix<double, N, N> JacobianProcess;

typedef Eigen::Matrix<double, Me, 1> Observation;
typedef Eigen::Matrix<double, Me, Me> ObservationNoiseCovariance;
typedef Eigen::Matrix<double, Me, N> JacobianObservation;

typedef Eigen::Matrix<double, C, 1> Control;
typedef Eigen::Matrix<double, P, 1> P_Noise;

typedef std::vector<double> state_type;//cannot use eigen matrix(such as State) as state type. Will show error, only can use build in type C++

struct simpleSkyCrane_NLDynamics{
    Control u;//control input from thrust
    P_Noise w;//process noise
    readPara_vehicle pV;//param of vehicle. Seems not a very nice way to use parameter because those parameters are constant, I don't hope to pass the object every time. However, I cannot find a better solution yet
    simpleSkyCrane_NLDynamics(Control param_u, P_Noise param_w, readPara_vehicle paramV) : u( param_u ),w(param_w),pV(paramV) {}
    
    void operator()( const state_type &x , state_type &xdot, const double t ) const 
    {
        double xidot = x[1], zdot = x[3], theta = x[4], thetadot = x[5], T1 = u(0,0), T2 = u(1,0);
        double alpha = atan2(zdot,xidot);//angle of attack
        double Vt    = sqrt(xidot*xidot + zdot*zdot); //total velocity
        
        //drag force
        double Aexposed = pV.Aside * cos(theta - alpha) + pV.Abot*sin(theta-alpha);
        double Fdxi     = 0.5 * pV.rho*pV.C_D*Aexposed*xidot*Vt;
        double Fdz      = 0.5 * pV.rho*pV.C_D *Aexposed*zdot*Vt;
        //std::cout<<"noise "<<w(0,0)<<","<<w(1,0)<<","<<w(2,0)<<std::endl;
        xdot[0] = xidot;
        xdot[1] = (T1*(cos(pV.beta)*sin(theta)+sin(pV.beta)*cos(theta)) 
                +  T2*(cos(pV.beta)*sin(theta)-sin(pV.beta)*cos(theta)) - Fdxi)/(pV.m_f + pV.m_b) + w(0,0);
        xdot[2] = zdot;
        xdot[3] = (T1*(cos(pV.beta)*cos(theta) - sin(pV.beta)*sin(theta))
                +  T2*(cos(pV.beta)*cos(theta) + sin(pV.beta)*sin(theta)) - Fdz) /(pV.m_f + pV.m_b) - pV.g + w(1,0);
        xdot[4] = thetadot;
        xdot[5] = (1.0/pV.Ieta)*( (T1-T2)*cos(pV.beta)*pV.w_b*0.5
                + (T2-T1)*sin(pV.beta)*(pV.h_cm)) + w(2,0);
        // std::cout<<"t is "<<t<<", state x is "<<x[0]<<","<<x[1]<<","<<x[2]<<","
        //     <<x[3]<<","<<x[4]<<","<<x[5]<<",control input is "<<u(0,0)<<","<<u(1,0)<<std::endl;
        // std::cout<<"alpha "<<alpha<<","<<"Vt "<<Vt<<","
        //     <<"Aexposed "<<Aexposed<<","<<"Fdxi "<<Fdxi<<","<<"Fdz"<<Fdz<<std::endl;
        //std::cout<<"xdot "<<xdot[0]<<","<<xdot[1]<<","<<xdot[2]<<","<<xdot[3]<<","<<xdot[4]<<","<<xdot[5]<<std::endl;
    }
};


class processModel{
public:
    processModel(readPara_vehicle& _pV);
    void setPrediction(const State& xEst, int _count);
    void setJacobian();
    JacobianProcess getF();
    ProcessNoiseCovariance getW();
    State getPrediction();

private:
    Eigen::MatrixXd wtilde0, uIn;
    readPara_vehicle pV;
    State xPred;
    JacobianProcess jacobianProcess;
    ProcessNoiseCovariance processNoiseCovariance;
    int count;

    void SimpleSkyCrane_DynJacobians_F();
    void SimpleSkyCrane_DynJacobians_W();
};

class observationModel{
public:
    observationModel(readPara_vehicle& _pV);
    void setObservation(const State& x, int count);
    void setJacobian();
    JacobianObservation getH();
    ObservationNoiseCovariance getR();
    Observation getObservation();

private:
    Eigen::MatrixXd utilde0, uIn;
    readPara_vehicle pV;
    State xPred;
    Observation observation;
    JacobianObservation jacobianObservation;
    ObservationNoiseCovariance observationNoiseCovariance;
    int count;
    
    void SimpleSkyCrane_MeasJacobians_H();
    void SimpleSkyCrane_MeasJacobians_R();
};