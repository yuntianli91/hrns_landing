#include "simulator/sensors/altimeter.h"

namespace MyFusion{

void Altimeter::setParam(double bias, double sigma){
    bias_ = bias;
    sigma_ = sigma / R_m; // convert from m to rad
    flagInit_ = true;
}

AltData Altimeter::getMeasurement(ImuMotionData currMotion){
    if(!flagInit_){
        cout << "WARNING: parameters untizlied! Please call setParam()\n";
    }
    
    AltData tmp;

    std::random_device rd;
    std::default_random_engine rg(rd());
    std::normal_distribution<double> noise(0., 1.);    

    tmp.timeStamp_ = currMotion.time_stamp_;
    tmp.range_ = currMotion.tnb_.y() / currMotion.qnb_.toRotationMatrix()(1, 1) + sigma_ * noise(rg);

    return tmp;
}

}