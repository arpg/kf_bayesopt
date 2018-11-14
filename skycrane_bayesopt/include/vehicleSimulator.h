#pragma once

//#include "vehicleModel.h"
#include "lqrController.h"

class Simulator{

public:
    Simulator(readPara_vehicle& param_vehicle);

    void initialize(const State& x0, const StateCovariance& P0_BASE);

    void step(int count);

    State getIniStateNoise();

    unsigned getSeed();

    const State& getX() const
    {
    return _x;
    }

    const Observation& getZ() const
    {
    return _z;
    }

private:

    processModel _processModel;

    observationModel _observationModel;

    State _x;

    State iniStateNoise;

    Observation _z;

    controller _controller;

    unsigned seed;
};