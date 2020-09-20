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
    std::random_device rd;
    std::default_random_engine rg(rd());
    std::normal_distribution<double> stdGau(0., 1.);

    if(flagFirst){
        tmp.timeStamp_ = currMotion.time_stamp_;
        curP_ = FrameConvert::geo2mcmf(currMotion.tnb_);

        tmp.dPos_ = Vec3d(0., 0., 0.);
        flagFirst = false;            
    }
    else{
        tmp.timeStamp_ = currMotion.time_stamp_;
        curP_ = FrameConvert::geo2mcmf(currMotion.tnb_);
        tmp.dPos_ = curP_ - lastP_ + Vec3d::Ones() * sigma_ * stdGau(rg);
    }
    
    lastP_ = curP_; // reset lastP
    return tmp; // return result
}

}