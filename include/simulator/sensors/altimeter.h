#ifndef ALTIMETER_H_
#define ALTIMETER_H_
#include "commonHeaders.h"
#include "simulator/sensors/imu_base.h"

namespace MyFusion
{
struct AltData{
    double timeStamp_;
    double range_;
};

class Altimeter{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    Altimeter(double bias, double sigma){
        setParam(bias, sigma);
    }

    ~Altimeter(){}

    void setParam(double bias, double sigma);

    AltData getMeasurement(ImuMotionData currMotion);

    double bias_;
    double sigma_;
    bool flagInit_ = false;    
};

} // namespace MyFusion



#endif