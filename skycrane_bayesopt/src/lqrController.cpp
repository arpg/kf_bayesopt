#include "lqrController.h"
//Again, repeat calculation and waste of space because matrix such as utilde0 use a lot space(Next time use inherit)
controller::controller(readPara_vehicle& _pV):pV(_pV){
    //Thrust of the rocket. Now its open loop so each time the input is just a constant.
    double Tnom = 0.5* pV.g * (pV.m_b + pV.m_f)/cos(pV.beta);
    uNomHist = Eigen::MatrixXd::Zero(C,pV.length); 
    uNomHist.fill(1.0);
    uNomHist = Tnom * uNomHist;

    xRef(0,0) = pV.xi0;
    xRef(1,0) = pV.xidot0;
    xRef(2,0) = pV.z0;
    xRef(3,0) = pV.zdot0;
    xRef(4,0) = pV.theta0;
    xRef(5,0) = pV.thetadot0;//LQR is going to try to maintain this state

    for(int i = 0; i<pV.controlDOF; i++){
        std::vector<double> klintmp(pV.Klin.begin()+i*pV.stateDOF,pV.Klin.begin()+(i+1)*pV.stateDOF);
        double* ptr = &klintmp[0];
        Eigen::Map<Eigen::VectorXd> Klintmp(ptr, klintmp.size());
        Klin.row(i) = Klintmp;
    }
}

void controller::setFeedbackControl(State& xEst, int count){
    uttCL = uNomHist.col(count) - (Klin*(xEst - xRef));
}

Control controller::getFeedbackControl(){
    return uttCL;
}