#include "ProcessModel.hpp"

#include "continuousToDiscrete.hpp"

using namespace ::Eigen;

ProcessModel::ProcessModel(double processNoise, double deltaT, bool sampleProcessNoise)
{
  _sampleProcessNoise = sampleProcessNoise;
  Matrix2d Fc, Qc;

  Fc = Matrix2d::Zero();

  Fc(0, 1) = 1;
  Qc(1, 1) = processNoise;

  continuousToDiscrete(_Fd, _Qd, Fc, Qc, deltaT);
   
  _processNoiseSampler.setMean(State::Zero());
  _processNoiseSampler.setCovar(_Qd);
}

State ProcessModel::predict(const State& xEst)
{
  State xPred = _Fd * xEst;

  if (_sampleProcessNoise == true)
    {
      xPred += _processNoiseSampler.samples(1);
    }

  return xPred;
}
