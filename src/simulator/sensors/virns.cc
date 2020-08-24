#include "simulator/sensors/virns.h"

namespace MyFusion{

void VIRNS::setParams(double bias, double sigma){
    bias_ = bias;
    sigma_ = sigma;
    flagInit_ = true;
}

VirnsData VIRNS::getRelativeMeasurement(ImuMotionData currMotion){
    if(!flagInit_){
        cout << "WARNING: Noise parameters unset !\n";
    }

    VirnsData tmp;

    if(flagFirst){
        tmp.timeStamp_ = currMotion.time_stamp_;
        tmp.dPos_ = Vec3d(0., 0., 0.);
        flagFirst = false;            
    }
    else{
        Vec3d dp = currMotion.tnb_ - lastMotion_.tnb_;

        std::random_device rd; // random seed
        std::default_random_engine rg(rd()); // random engine
        std::normal_distribution<double> noise(0., 1.);

        Vec3d dpNoise(noise(rg), noise(rg), noise(rg));
        dp += sigma_ * dpNoise;

        tmp.timeStamp_ = currMotion.time_stamp_;
        tmp.dPos_ = dp;        
    }
    
    lastMotion_ = currMotion;
    return tmp;
}

}