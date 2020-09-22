#ifndef CMNS_H_
#define CMNS_H_
#include "commonHeaders.h"
#include "simulator/sensors/imu_base.h"
#include "simulator/sensorNoise.hpp"

namespace MyFusion{

struct CmnsData{
    double timeStamp_;
    Vec2d pos_; // latitude and longitude
};

class CMNS{
public:
    CMNS(double bias, double sigma){
        setParam(bias, sigma);
    }

    ~CMNS(){}

    void setParam(double bias, double sigma);

    CmnsData getMeasurement(ImuMotionData currMotion);

    double bias_;
    double sigma_;
    bool flagInit_ = false;
};

}

#endif