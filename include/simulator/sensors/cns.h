#ifndef CNS_H_
#define CNS_H_
#include "commonHeaders.h"
#include "utilities/utilities.hpp"
#include "simulator/sensors/imu_base.h"
using namespace std;

namespace MyFusion
{

struct CnsData{
    double timeStamp_;
    Eigen::Quaterniond qnb_;
    Vec3d eulerAngle_;
};

class CNS{
public:
    CNS(double bias, double sigma):bias_(bias){
        sigma_ = sigma / 3600.; // arcsec->deg
        sigma_ = sigma_ / 180. * M_PI; // deg->rad
        flag_init_ = true;
    }
    ~CNS(){};

    CnsData getMeasurements(ImuMotionData currMotion);

    double sigma_; // noise sigma of measurements
    double bias_; // bias of measurements

    bool flag_init_ = false;


};

} // namespace myFusion

#endif