#ifndef __PARAMETERS_HPP__
#define __PARAMETERS_HPP__

struct Parameters
{
  double processNoise;
  double observationNoise;
  double deltaT;

  Parameters()
  {
    processNoise = 0;
    observationNoise = 0;
    deltaT = 0;
  }

  Parameters(double pn, double on, double dT)
  {
    processNoise = pn;
    observationNoise = on;
    deltaT = dT;
  }
};

#endif // __PARAMETERS_HPP__
