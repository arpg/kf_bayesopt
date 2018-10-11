#pragma once

#include "vehicleModel.h"
#include <chrono>

class Simulator{

public:
    Simulator(readPara_vehicle& param_vehicle);

    void initialize(const State& x0, const StateCovariance& P0);

    void step(int count);

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

    Observation _z;
};