#include "trial.h"
#include <iomanip>

trial::trial(readPara_vehicle& simulatorParameters,  readPara_vehicle& estimatorParameters)
    :  _simulator(simulatorParameters), _estimator(estimatorParameters)
    ,_param_vehicle(simulatorParameters){};

void trial::run(){
    srand(time(NULL));
    State x0, x0_dist;
    x0(0,0) = _param_vehicle.xi0       ;//+ _param_vehicle.disturbance[0]; 
    x0(1,0) = _param_vehicle.xidot0    ;//+ _param_vehicle.disturbance[1]; 
    x0(2,0) = _param_vehicle.z0        ;//+ _param_vehicle.disturbance[2]; 
    x0(3,0) = _param_vehicle.zdot0     ;//+ _param_vehicle.disturbance[3];
    x0(4,0) = _param_vehicle.theta0    ;//+ _param_vehicle.disturbance[4]; 
    x0(5,0) = _param_vehicle.thetadot0 ;//+ _param_vehicle.disturbance[5];
    x0_dist(0,0) = _param_vehicle.xi0       + _param_vehicle.disturbance[0]; 
    x0_dist(1,0) = _param_vehicle.xidot0    + _param_vehicle.disturbance[1]; 
    x0_dist(2,0) = _param_vehicle.z0        + _param_vehicle.disturbance[2]; 
    x0_dist(3,0) = _param_vehicle.zdot0     + _param_vehicle.disturbance[3];
    x0_dist(4,0) = _param_vehicle.theta0    + _param_vehicle.disturbance[4]; 
    x0_dist(5,0) = _param_vehicle.thetadot0 + _param_vehicle.disturbance[5];
    
    StateCovariance P0; //For EKF P0
    if(_param_vehicle.P0 == "zero")
        P0.fill(0.0);
    else if(_param_vehicle.P0 == "identity"){
        P0 = Eigen::MatrixXd::Identity(N,N);
        P0.diagonal()<<2.0,0.5,1.0,1.0,0.01,0.005;
    }
    //std::vector<double> his_rowNIS; //to calculate the middle point value, uncomment this
    //std::cout<<"x0 transpose "<<x0.transpose()<<", x0_dist transpose "<<x0_dist.transpose();
    for(int i = 0; i < _param_vehicle.Nsimruns/_param_vehicle.CPUcoreNumber; ++i){
        _averageNEES = 0.0;

        _simulator.initialize(x0,P0);
        _estimator.initialize(x0_dist,P0);
        
        iniStateNoiseHis.push_back(_simulator.getIniStateNoise());//have double about if the noise has the desired mean and variance, so save the history and check
        for(int step = 0; step<_param_vehicle.maxIterations; step++){

            if(plotFlag && i==0){
                State _x = _estimator.getXEst();
                State _x_gt = _simulator.getX();
                for(int j = 0; j<N; j++){
                    stateHis.push_back(_x(j,0));
                    stateHis_gt.push_back(_x_gt(j,0));
                }
            }

            _simulator.step(step);//step variable is for G*u;
            Observation z = _simulator.getZ();
            //std::cout<<z.transpose()<<std::endl;
            _estimator.step(z,step);
            // std::cout << std::setprecision(12);
            //std::cout << step << "\n" << _simulator.getX().transpose()<<"\n" << _estimator.getXEst().transpose()<<"\n"<<std::endl;//<< " " << z << " " 
            //std::cout<<step<<"\n"<< _estimator.getPEst().inverse()<<std::endl;

            // chi^2 estimate, NEES and NIS
            State xerror = _estimator.getXEst() - _simulator.getX();

            //std::cout<<_estimator.getXEst().transpose()<<"        "<<_simulator.getX().transpose()<<std::endl;
           // std::cout<<"xErr "<<xerror.transpose()<<std::endl;

            double chi2NEES = 0, chi2NIS = 0, toX0 = 0;
            if(_param_vehicle.costChoice == "JNEES") //for string, don't use c_str() to check if ti equals to each other, just use string itself
                chi2NEES = xerror.transpose()*_estimator.getPEst().inverse()*xerror;
            else if(_param_vehicle.costChoice == "JNIS")   
                chi2NIS  = _estimator.getResidual().transpose()*_estimator.getS().inverse()*_estimator.getResidual();
            else if(_param_vehicle.costChoice == "withRespectToX0")
                toX0     = (_estimator.getXEst() - x0).transpose() * (_estimator.getXEst() - x0);

            //std::cout<<"error in each step"<<chi2NEES<<" "<<chi2NIS<<std::endl;
            rowNEES += chi2NEES;
            rowNIS  += chi2NIS;
            rowToX0 += toX0;
            //std::cout << " " << chi2 << std::endl;
        }
        AllNEES += rowNEES;
        AllNIS  += rowNIS;
        AllToX0 += rowToX0;

        // his_rowNIS.push_back(rowNIS); to find the middle number of data, uncomment this
        rowNEES = 0.0;
        rowNIS  = 0.0; //rememeber to clear. Then we can add the second iteration
        rowToX0 = 0.0;
    }
    //find middle number of his_rowNIS. The result shows the middle point value is also as random as the JNIS
    // double middle_NIS;
    // int si = his_rowNIS.size();
    // std::sort(his_rowNIS.begin(), his_rowNIS.end());
    // if(si/2.0 == 0) // if even then middle the the average of middle two number
    //     middle_NIS = (his_rowNIS[si/2] + his_rowNIS[si/2 - 1])/2.0;
    // else
    //     middle_NIS = his_rowNIS[si/2];
    // middle_NIS /= _param_vehicle.maxIterations;
    // double M_NIS  = std::abs(log(middle_NIS/_param_vehicle.observationDOF));
    // std::cout<<"middle of NIS (after log..) "<<M_NIS<<std::endl;

    //std::cout<<"AllNees "<<AllNEES<<", AllNIS "<<AllNIS<<", AllToX0 "<<AllToX0<<std::endl;
    _averageNEES = AllNEES/(_param_vehicle.maxIterations*_param_vehicle.Nsimruns/_param_vehicle.CPUcoreNumber);
    _averageNIS  = AllNIS/(_param_vehicle.maxIterations*_param_vehicle.Nsimruns/_param_vehicle.CPUcoreNumber);
    _averageToX0 = AllToX0/(_param_vehicle.maxIterations*_param_vehicle.Nsimruns/_param_vehicle.CPUcoreNumber);
    //std::cout<<"averageNEES "<<_averageNEES<<", averageNEES "<<_averageNIS<<", averageToX0 "<<_averageToX0<<std::endl;

    //showAverageIniStateNoise();
    //showVarainceIniStateNoise();
}

double trial::getAverageNEES()
{
    return _averageNEES;
}

double trial::getAverageNIS()
{
    return _averageNIS;
}

double trial::getAverageToX0()
{
    return _averageToX0;
}

void trial::setPlotFlag(bool flag){
    plotFlag = flag;
}

void  trial::showAverageIniStateNoise(){
    State tmp = Eigen::MatrixXd::Zero(N,1);
    for(State i:iniStateNoiseHis){
        //std::cout<<"in compute average function "<<i.transpose()<<std::endl;
        tmp += i;
    }
    std::cout<<"average noise of initial State is "<<(tmp/_param_vehicle.Nsimruns).transpose()<<std::endl;
}
//we have 6 states
void  trial::showVarainceIniStateNoise(){
    State tmp   = Eigen::MatrixXd::Zero(N,1);
    State tmp2  = Eigen::MatrixXd::Zero(N,1);
    for(State i:iniStateNoiseHis)
        tmp += i;
    tmp /=  _param_vehicle.Nsimruns;
    //compute varaince for each parameter
    for(State i:iniStateNoiseHis){
        for(int j = 0; j<N; j++){
            tmp2(j,0) += (i(j,0) - tmp(j,0)) * (i(j,0) - tmp(j,0));
        }
    }
    std::cout<<"noise variance for each state is "<<(tmp2/_param_vehicle.Nsimruns).transpose()<<std::endl;
}
//state history for plotting
std::vector<double> trial::getStateHis(){
    return stateHis;
}

std::vector<double> trial::getStateHis_gt(){
    return stateHis_gt;
}

trial::~trial(){};
