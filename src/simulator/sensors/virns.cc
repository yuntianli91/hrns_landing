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
        curP_ = geo2mcmf(currMotion.tnb_);

        tmp.dPos_ = Vec3d(0., 0., 0.);
        flagFirst = false;            
    }
    else{
        tmp.timeStamp_ = currMotion.time_stamp_;
        curP_ = geo2mcmf(currMotion.tnb_);
        tmp.dPos_ = curP_ - lastP_;
    }
    
    lastP_ = curP_; // reset lastP
    return tmp; // return result
}

Vec3d VIRNS::geo2mcmf(Vec3d geo){
    Vec3d tmp;
    double lat = geo.x();
    double alt = geo.y(); 
    double lon = geo.z(); 
    
    tmp.x() = (R_m + alt) * cos(lat) * cos(lon);
    tmp.y() = (R_m + alt) * cos(lat) * sin(lon);
    tmp.z() = (R_m + alt) * sin(lat);

    return tmp;
}

Vec3d VIRNS::mcmf2geo(Vec3d mcmf){
    Vec3d tmp;

    return tmp;
}

}