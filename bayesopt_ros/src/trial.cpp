#include "trial.h"
#include <iomanip>

trial::trial(readPara_vehicle& simulatorParameters,  readPara_vehicle& estimatorParameters)
    :  _simulator(simulatorParameters), _estimator(estimatorParameters)
    ,_param_vehicle(simulatorParameters){};

void trial::run(){
    
    for(int i = 0; i < _param_vehicle.Nsimruns; ++i){
        
        State x0;
        x0(0,0) = _param_vehicle.xi0; 
        x0(1,0) = _param_vehicle.xidot0; 
        x0(2,0) = _param_vehicle.z0; 
        x0(3,0) = _param_vehicle.zdot0;
        x0(4,0) = _param_vehicle.theta0; 
        x0(5,0) = _param_vehicle.thetadot0;
        
        StateCovariance P0;
        if(_param_vehicle.P0 == "zero")
            P0.fill(0.0);
        else
            P0 = Eigen::MatrixXd::Identity(N,N);

        _averageNEES = 0.0;

        _simulator.initialize(x0,P0);
        _estimator.initialize(x0,P0);

        for(int step = 0; step<_param_vehicle.maxIterations; step++){
            _simulator.step(step);//step variable is for G*u;
            Observation z = _simulator.getZ();
            //std::cout<<z.transpose()<<std::endl;
            _estimator.step(z,step);
            // std::cout << std::setprecision(12);
            // std::cout << step << " " << _simulator.getX().transpose() << " " << z << " " << _estimator.getXEst().transpose()
            //  << " " << _estimator.getPEst().diagonal().transpose()<<std::endl;

            // chi^2 estimate, NEES and NIS
            State xerror = _estimator.getXEst() - _simulator.getX();
            //std::cout<<_estimator.getXEst().transpose()<<"        "<<_simulator.getX().transpose()<<std::endl;
            //std::cout<<"xErr "<<xErr.transpose()<<std::endl;
            double chi2NEES = 0, chi2NIS = 0;
            if(_param_vehicle.costChoice == "JNEES") //for string, don't use c_str() to check if ti equals to each other, just use string itself
                chi2NEES = xerror.transpose()*_estimator.getPEst().inverse()*xerror;
            else    
                chi2NIS  = _estimator.getResidual().transpose()*_estimator.getS().inverse()*_estimator.getResidual();
            //OneNEES.push_back(chi2);
            rowNEES += chi2NEES;
            rowNIS  += chi2NIS;
            //std::cout << " " << chi2 << std::endl;
        }
        AllNEES += rowNEES;
        AllNIS  += rowNIS;
    }
    _averageNEES = AllNEES/(_param_vehicle.maxIterations*_param_vehicle.Nsimruns);
    _averageNIS  = AllNIS/(_param_vehicle.maxIterations*_param_vehicle.Nsimruns);
}

double trial::getAverageNEES()
{
    return _averageNEES;
}

double trial::getAverageNIS()
{
    return _averageNIS;
}
