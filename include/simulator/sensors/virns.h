#ifndef VIRNS_H_
#define VIRNS_H_
#include "commonHeaders.h"
#include "simulator/sensors/imu_base.h"

namespace MyFusion{

struct VirnsData{
    double timeStamp_;
    Vec3d dPos_;
    Vec3d pos_;
};

class VIRNS{
public:
    VIRNS(double bias, double sigma):bias_(bias), sigma_(sigma){
        flagInit_ = true;
    }
    ~VIRNS(){}


    void setParams(double bias, double sigma);
    /**
     * @brief Get the Relative Measurement dp
     * 
     * @param currMotion 
     * @return Vec3d 
     */
    VirnsData getRelativeMeasurement(ImuMotionData currMotion);
    
    ImuMotionData lastMotion_;
    double bias_, sigma_; // noise bias and sigma
    bool flagInit_ = false;
    bool flagFirst = true;
};

}

#endif