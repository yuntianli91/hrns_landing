#ifndef CMNS_H_
#define CMNS_H_
#include "commonHeaders.h"
#include "simulator/sensors/imu_base.h"

namespace MyFusion{

struct CmnsData{
    double timeStamp_;
    Vec3d pos_;
};

class CMNS{
public:
    CMNS(double bias, double sigma):bias_(bias), sigma_(sigma){
        flagInit_ = true;
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