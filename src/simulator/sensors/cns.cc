#include "simulator/sensors/cns.h"

namespace MyFusion{

CnsData CNS::getMeasurements(ImuMotionData currMotion){
    // generate standard white noise
    std::random_device rd; // generate seed for random engine   
    std::default_random_engine rg(rd()); // create random eigine with seed rd();
    std::normal_distribution<double> noise(0.0, 1.0);

    Eigen::Quaterniond dq(1., 0.5 * sigma_ * noise(rg), 0.5 * sigma_ * noise(rg), 0.5 * sigma_ * noise(rg));
    dq.normalize();

    CnsData tmp;
    tmp.timeStamp_ = currMotion.time_stamp_;    
    tmp.qnb_ = (currMotion.qnb_ * dq).normalized();    

    // convert to geo and compute euler angles
    Eigen::Matrix3d Cgb = AttUtility::getCge(currMotion.tnb_) * tmp.qnb_.toRotationMatrix();
    tmp.eulerAngle_ = AttUtility::R2Euler(Cgb);

    return tmp;
}

}