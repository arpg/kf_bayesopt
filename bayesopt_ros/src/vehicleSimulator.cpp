#include "vehicleSimulator.h"

using namespace Eigen;

Simulator::Simulator(readPara_vehicle& param_vehicle)
        :_observationModel(param_vehicle),
        _processModel(param_vehicle){};

//initial state is a point near x0
void Simulator::initialize(const State& x0, const StateCovariance& P0)
{
  unsigned seed = std::chrono::system_clock::now().time_since_epoch().count(); //different seed for different initial random number
  Eigen::EigenMultivariateNormal<double> initialConditionSampler(x0,P0,false,seed);

  _x = initialConditionSampler.samples(1);
}

void Simulator::step(int count)
{
  _processModel.setPrediction(_x,count);
  _x = _processModel.getPrediction();
  //std::cout<<"_x in simulator step \n"<<_x.transpose()<<std::endl;
  _observationModel.setObservation(_x,count);
  _z = _observationModel.getObservation();
  //std::cout<<"_z in simulator step \n"<<_z.transpose()<<std::endl;
}


