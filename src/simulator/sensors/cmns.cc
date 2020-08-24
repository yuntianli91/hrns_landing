#include "simulator/sensors/cmns.h"

namespace MyFusion
{

void CMNS::setParam(double bias, double sigma){
    bias_ = bias;
    sigma_ = sigma;
    flagInit_ = true;
}

CmnsData CMNS::getMeasurement(ImuMotionData currMotion){
    if(!flagInit_){
        cout << "WARNING: parameters untizlied! Please call setParam()\n";
    }
    
    CmnsData tmp;

    std::random_device rd;
    std::default_random_engine rg(rd());
    std::normal_distribution<double> noise(0., 1.);    

    Vec3d posNoise(noise(rg), noise(rg), noise(rg));

    tmp.timeStamp_ = currMotion.time_stamp_;
    tmp.pos_ = currMotion.tnb_ + sigma_ * posNoise;

    return tmp;
}
    
} // namespace MyFusionclass