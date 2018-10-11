#include "vehicleModel.h"

using namespace Eigen;
using namespace boost::numeric::odeint;

//drawback, two many computation repeat again and again, should save them as members before(then more space will be used, pros and cons). 
//Considering it is not online code, should be OK for now
processModel::processModel(readPara_vehicle& _pV):pV(_pV){

    MatrixXd Stilde = MatrixXd::Zero(P,P);
    MatrixXd rn     = MatrixXd::Random(P,pV.length);
    
    //Original matlab code has

    //Qtilde = diag([1e-2 1e-2 1e-2]); %%nominal nonlinear DT AWGN covars
    //Stilde = chol(Qtilde);

    //where R = chol(A) produces an upper triangular matrix R from the diagonal and upper triangle of matrix A,
    // satisfying the equation R'*R=A. For a diagonal matrix, R should be just the square root of A
    for(int i = 0; i<P; i++)
        Stilde(i,i) = sqrt(pV.pnoise[i]); 

    wtilde0 = Stilde * rn;

    //Thrust of the rocket. Now its open loop so each time the input is just a constant.
    double Tnom = 0.5* pV.g * (pV.m_b + pV.m_f)/cos(pV.beta);
    uIn = MatrixXd::Zero(C,pV.length); 
    uIn.fill(1.0);
    uIn = Tnom * uIn;
}

void processModel::setPrediction(const State& xEst, int _count){
    count = _count;
    //std::cout<<"xEst "<<xEst.transpose()<<"\n"<<"count is "<<count<<std::endl;
    //Normally we use state space to get prediction(like State xPred = _Fd * xEst+G*u[count];), but now it is nonlinear 
    // directly slove ODE equation to get the prediction. things like ode45 will do the work from CT to DT
    runge_kutta_dopri5<state_type> stepper;
    std::vector<double> tmpState(xEst.data(),xEst.data()+xEst.rows()*xEst.cols());//assign eigen matrix value to std vector, xEst.rows()*xEst.cols() is the size of matrix
    //simpleSkyCrane_NLDynamics tmp(uIn.col(count),wtilde0.col(count));
    double t = 0.0; //according to the orignal code, we didn't use t here so we can set it as any number
    for ( int i = 0; i < 8; ++i){ //orignal code seems it has 41 steps, I don't know why, here I need use 8 steps to make it the same as original code's deltaT(0.0025).
        stepper.do_step(simpleSkyCrane_NLDynamics{uIn.col(count),wtilde0.col(count),pV}, tmpState , t, pV.dt/8 );//simpleSkyCrane_NLDynamics{xEst,uIn.col(count),wtilde0.col(count)}
        t += (pV.dt/8);
        //cout << tmpState[0] << endl;
    } //need to check if 41 is OK
    double* ptr = &tmpState[0];
    Eigen::Map<State> xPrediction(ptr,tmpState.size());//map stdvector to eigen matrix(vector).
    xPred = xPrediction;
    //std::cout<<"xPred in predition"<<xPred.transpose()<<std::endl;
};

//According to slides 10, we need Omega*Q*Omega, where Q should be identity matrix with diagnol value wtilde(0, count) (Q is time varing)
//Omega is from Gamma, accoding to slides 5, Camma should be a 6 by 3 matrix which can be used to add xidotdot, zdotdot, thetadotdot to state
//So Gamma should be [0 0 0;1 0 0;0 0 0; 0 1 0;0 0 0; 0 0 1], so Gamma is constant, not time varing here
//W = Omega*Q*Omega
void processModel::SimpleSkyCrane_DynJacobians_W(){
    ProcessNoiseCovariance CT_processNoiseCovariance;
    CT_processNoiseCovariance.fill(0.0);
    CT_processNoiseCovariance(1,1) = wtilde0(0,count) *wtilde0(0,count)* pV.dt * pV.dt;
    CT_processNoiseCovariance(3,3) = wtilde0(1,count) *wtilde0(1,count)* pV.dt * pV.dt;
    CT_processNoiseCovariance(5,5) = wtilde0(2,count) *wtilde0(2,count)* pV.dt * pV.dt;
    processNoiseCovariance = CT_processNoiseCovariance;//no need for exponential to convert to DT, it is already DT approximation
};

void processModel::SimpleSkyCrane_DynJacobians_F(){
    //std::cout<<"xPred in JacobianF"<<xPred.transpose()<<std::endl;
    JacobianProcess CT_jacobian = MatrixXd::Zero(N, N);//jacobian of state
    JacobianProcess iden = MatrixXd::Identity(N,N);

    double xidot = xPred(1,0), zdot = xPred(3,0), theta = xPred(4,0);
    double T1 = uIn(0,count), T2 = uIn(1,count);

    double alpha = atan2(zdot,xidot);//angle of attack
    double Vt    = sqrt(xidot*xidot + zdot*zdot); //total velocity 

    //dimensional constants
    double cd = 0.5*pV.rho*pV.C_D;
    double nc_m = -0.5*pV.C_D*pV.rho/(pV.m_f+pV.m_b);
    double oo_mt = 1.0/(pV.m_f+pV.m_b);

    //area trig terms:
    double Aside_cthetaaalpha = pV.Aside*cos(theta-alpha);
    double Aside_sthetaalpha  = pV.Aside*sin(theta-alpha);
    double Abot_cthetaalpha   = pV.Abot*cos(theta-alpha);
    double Abot_sthetaalpha   = pV.Abot*sin(theta-alpha);

    // std::cout<<"alpha "<<alpha<<","<<"Vt "<<Vt<<","
    //        <<"cd "<<cd<<","<<"nc_m "<<nc_m<<","<<"oo_mt"<<oo_mt<<","
    //        <<"Aside_cthetaaalpha"<<Aside_cthetaaalpha<<","<<"Aside_sthetaalpha"<<Aside_sthetaalpha<<","
    //        <<"Abot_cthetaalpha"<<Abot_cthetaalpha<<","<<"Abot_sthetaalpha"<<Abot_sthetaalpha<<std::endl;

    CT_jacobian(0,1) = 1; //fdot(xi) = xidot, while derivatives to other numbers are 0
    CT_jacobian(2,3) = 1; //fdot(z)  = zdot,
    CT_jacobian(4,5) = 1; //fdot(theta) = thetadot

    if(Vt>0){
    //my deriviation of CT_jacobian(1,1), the last part should be Abot_cthetaalpha - Aside_sthetaalpha(original is add)
        CT_jacobian(1,1) = nc_m*((Aside_cthetaaalpha + Abot_sthetaalpha)// derivative of xidotdot to xi dot
                            *((2*xidot*xidot + zdot*zdot)/Vt) + (xidot*zdot/Vt)*(Abot_cthetaalpha - Aside_sthetaalpha));
        CT_jacobian(1,3) = nc_m*((Aside_cthetaaalpha + Abot_sthetaalpha)//derivative , xidotdot to zdot
                            *((xidot*zdot)/Vt) + (xidot*xidot/Vt)*(-Abot_cthetaalpha + Aside_sthetaalpha));
        CT_jacobian(3,1) = nc_m*((Aside_cthetaaalpha + Abot_sthetaalpha)//derivative , zdotdot to xidot
                            *((xidot*zdot)/Vt) + (zdot*zdot/Vt)*(Abot_cthetaalpha - Aside_sthetaalpha));
        CT_jacobian(3,3) = nc_m*((Aside_cthetaaalpha + Abot_sthetaalpha)//derivative , zdotdot to zdot
                            *((2*xidot*xidot + zdot*zdot)/Vt) + (zdot*xidot/Vt)*(-Abot_cthetaalpha + Aside_sthetaalpha));
    }

    CT_jacobian(1,4) = oo_mt*(T1*(cos(pV.beta)*cos(theta) - sin(pV.beta)*sin(theta)) 
                        + T2*(cos(pV.beta)*cos(theta) + sin(pV.beta)*sin(theta)) 
                        + cd*xidot*Vt*(Aside_sthetaalpha - Abot_cthetaalpha));//derivative , xidotdot to theta
    CT_jacobian(3,4) = oo_mt*(-T1*(cos(pV.beta)*sin(theta) + sin(pV.beta)*cos(theta))
                        + T2*(sin(pV.beta)*cos(theta) - cos(pV.beta)*sin(theta))
                        + cd*zdot*Vt*(Aside_sthetaalpha - Abot_cthetaalpha)); //derivative, zdotdot to theta

    //std::cout<<"Jacobian in CT is \n"<<CT_jacobian<<std::endl;

    //To discrete time(DT), check slides lecture27_final 10th slide
    jacobianProcess = iden + CT_jacobian*pV.dt;
}

void processModel::setJacobian(){
    SimpleSkyCrane_DynJacobians_F();
    SimpleSkyCrane_DynJacobians_W();
}

JacobianProcess processModel::getF(){
    return jacobianProcess;
}

ProcessNoiseCovariance processModel::getW(){
    return processNoiseCovariance;
}

State processModel::getPrediction(){
    return xPred;
}

observationModel::observationModel(readPara_vehicle& _pV):pV(_pV){
    MatrixXd Rtilde = MatrixXd::Zero(Me,Me);
    MatrixXd rn     = MatrixXd::Random(Me,pV.length);
    for(int i = 0; i<Me; i++)
        Rtilde(i,i) = sqrt(pV.onoise[i]); 
    utilde0 = Rtilde * rn;

    double Tnom = 0.5* pV.g * (pV.m_b + pV.m_f)/cos(pV.beta);
    uIn = MatrixXd::Zero(C,pV.length); //need change this and make it the same as the matlab input sequential, but why control input is a constant
    uIn.fill(1.0);
    uIn = Tnom * uIn;
}
//similar to process model construction function
void observationModel::setObservation(const State& x, int _count){
    count = _count;
    xPred = x;
    double xi = x(0,0), xidot = x(1,0), z = x(2,0), zdot = x(3,0), theta = x(4,0), thetadot = x(5,0);
    double T1 = uIn(0,count), T2 = uIn(1,count); 

    double alpha = atan2(zdot,xidot);//angle of attack
    double Vt    = sqrt(xidot*xidot + zdot*zdot); //total velocity    
    //drag force
    double Aexposed = pV.Aside * cos(theta - alpha) + pV.Abot*sin(theta-alpha);
    double Fdxi     = 0.5 * pV.rho*pV.C_D*Aexposed*xidot*Vt;
    
    //mesurement, we can measure inertial translation xi, altitude above surface z, angular velocity thetadot, accleration xidotdot
    observation(0,0) = xi + utilde0(0,count);
    observation(1,0) = z  + utilde0(1,count);
    observation(2,0) = thetadot + utilde0(2,count);
    observation(3,0) = (T1*(cos(pV.beta)*sin(theta) + sin(pV.beta)*cos(theta))
            +  T2*(cos(pV.beta)*sin(theta) - sin(pV.beta)*cos(theta))- Fdxi) /(pV.m_f + pV.m_b) + utilde0(3,count);
    //Notice here measurement has relationship with control input
    //Normally a system we meet is like 
    //xdot = Ax+Bu
    //y    = Cx+Du
    //Most of the time D is zero, however, here we can see if this is a linear system, D is not zero because T1 and T1 are used   
}

void observationModel::SimpleSkyCrane_MeasJacobians_H(){
    JacobianObservation CT_jacobian = MatrixXd::Zero(Me, N);

    double xidot = xPred(1,0), zdot = xPred(3,0), theta = xPred(4,0);
    double T1 = uIn(0,count), T2 = uIn(1,count);

    //dimensional constants
    double cd = 0.5*pV.rho*pV.C_D;
    double nc_m = -0.5*pV.C_D*pV.rho/(pV.m_f+pV.m_b);
    double oo_mt = 1.0/(pV.m_f+pV.m_b);

    double alpha = atan2(zdot,xidot);//angle of attack
    double Vt    = sqrt(xidot*xidot + zdot*zdot); //total velocity
    //std::cout<<xidot<<"  "<<zdot<<"  "<<Vt<<std::endl;    
    //drag force
    double Aexposed = pV.Aside * cos(theta - alpha) + pV.Abot*sin(theta-alpha);
    double Fdxi     = 0.5 * pV.rho*pV.C_D*Aexposed*xidot*Vt;

    //area trig terms:
    double Aside_cthetaaalpha = pV.Aside*cos(theta-alpha);
    double Aside_sthetaalpha  = pV.Aside*sin(theta-alpha);
    double Abot_cthetaalpha   = pV.Abot*cos(theta-alpha);
    double Abot_sthetaalpha   = pV.Abot*sin(theta-alpha);

    // std::cout<<"alpha "<<alpha<<","<<"Vt "<<Vt<<","
    //        <<"cd "<<cd<<","<<"nc_m "<<nc_m<<","<<"oo_mt"<<oo_mt<<","
    //        <<"Aside_cthetaaalpha"<<Aside_cthetaaalpha<<","<<"Aside_sthetaalpha"<<Aside_sthetaalpha<<","
    //        <<"Abot_cthetaalpha"<<Abot_cthetaalpha<<","<<"Abot_sthetaalpha"<<Abot_sthetaalpha<<std::endl;

    CT_jacobian(0,0) = 1;
    CT_jacobian(1,2) = 1;
    CT_jacobian(2,5) = 1;

    if(Vt>0){
        CT_jacobian(3,1) = nc_m*((Aside_cthetaaalpha + Abot_sthetaalpha)// derivative of xidotdot to xi dot
                            *((2*xidot*xidot + zdot*zdot)/Vt) + (xidot*zdot/Vt)*(Abot_cthetaalpha - Aside_sthetaalpha));
        CT_jacobian(3,3) = nc_m*((Aside_cthetaaalpha + Abot_sthetaalpha)//derivative , xidotdot to zdot
                                *((xidot*zdot)/Vt) + (xidot*xidot/Vt)*(-Abot_cthetaalpha + Aside_sthetaalpha));
    }
    CT_jacobian(3,4) = oo_mt*(T1*(cos(pV.beta)*cos(theta) - sin(pV.beta)*sin(theta)) 
                        + T2*(cos(pV.beta)*cos(theta) + sin(pV.beta)*sin(theta)) 
                        + cd*xidot*Vt*(Aside_sthetaalpha - Abot_cthetaalpha));//derivative , xidotdot to theta

    //To discrete time(DT)
    jacobianObservation = CT_jacobian;
    //Seems there is no need to do anything for H(I remember from CT to DT, H just keep as it is). 
    //Also see the matlab code, directly Htildett = Ctilde;
}

//similar to process noise covariance, however, not sure if I should multiply dt^2. There is no slides about R from CT to DT
void observationModel::SimpleSkyCrane_MeasJacobians_R(){
    ObservationNoiseCovariance CT_observationNoiseCovariance;
    CT_observationNoiseCovariance.fill(0.0);
    CT_observationNoiseCovariance(0,0) = utilde0(0,count)*utilde0(0,count) * pV.dt * pV.dt;
    CT_observationNoiseCovariance(1,1) = utilde0(1,count)*utilde0(1,count) * pV.dt * pV.dt;
    CT_observationNoiseCovariance(2,2) = utilde0(2,count)*utilde0(2,count) * pV.dt * pV.dt;
    CT_observationNoiseCovariance(3,3) = utilde0(3,count)*utilde0(3,count) * pV.dt * pV.dt;
    observationNoiseCovariance = CT_observationNoiseCovariance;//no need for exponential to convert to DT, it is already DT approximation    
}
  
void observationModel::setJacobian(){
    SimpleSkyCrane_MeasJacobians_H();
    SimpleSkyCrane_MeasJacobians_R();
}

Observation observationModel::getObservation(){
    return observation;
}

JacobianObservation observationModel::getH(){
    return jacobianObservation;
}

ObservationNoiseCovariance observationModel::getR(){
    return observationNoiseCovariance;
}