#pragma once

#include "vehicleModel.h"

class controller{
public:
    controller(readPara_vehicle& _pV);
    void setFeedbackControl(State& xEst, int count);
    Control getFeedbackControl();
private:
    readPara_vehicle& pV;
    Eigen::MatrixXd utilde0, wtilde0, uNomHist;
    State xRef;
    Control uttCL;
    Eigen::Matrix<double, C, N> Klin;
};