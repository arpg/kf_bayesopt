#include "vehicleSimulator.h"

using namespace Eigen;

Simulator::Simulator(readPara_vehicle& param_vehicle)
        :_observationModel(param_vehicle, true), //simulator, add noise. Simulator should add noise
        _processModel(param_vehicle, true), _controller(param_vehicle){};

//initial state is a point near x0
void Simulator::initialize(const State& x0, const StateCovariance& P0)
{
  seed = rand();//std::chrono::system_clock::now().time_since_epoch().count(); //different seed for different initial random number
  Eigen::EigenMultivariateNormal<double> initialConditionSampler(Eigen::MatrixXd::Zero(N,1),P0,false,seed);

  iniStateNoise = initialConditionSampler.samples(1);
  //iniStateNoise<<1.0,0.1,0.1,0.1,0.1,0.05;
  //std::cout<<"iniStateNoise"<<iniStateNoise.transpose();

  _x = x0 + iniStateNoise;
  //_x = x0;
  //std::cout<<"initial simulator state "<<_x.transpose()<<std::endl;

  /*initialize random seed*/
  int seed_ob = rand();
  int seed_pr = rand();
  
  _observationModel.setSeed(seed_ob);//use these lines then simulator initial state, process/observation noise will use the same seed.
  _processModel.setSeed(seed_pr);
  _observationModel.setNoise();
  _processModel.setNoise();
}

void Simulator::step(int count)
{
  _controller.setFeedbackControl(_x,count);
  Control feedbackUin = _controller.getFeedbackControl();

  _processModel.setPrediction(feedbackUin, _x,count);
  _x = _processModel.getPrediction();
  //std::cout<<"_x in simulator step \n"<<_x.transpose()<<std::endl;
  _controller.setFeedbackControl(_x,count);
  feedbackUin = _controller.getFeedbackControl();

  _observationModel.setObservation(feedbackUin, _x,count);
  _z = _observationModel.getObservation();
  //std::cout<<"_z in simulator step \n"<<_z.transpose()<<std::endl;
}

//compute the average and variance of random number. It should be similar to the parameters in initialConditionSampler
State Simulator::getIniStateNoise(){
    return iniStateNoise;
};

unsigned Simulator::getSeed(){
  return seed;
}




