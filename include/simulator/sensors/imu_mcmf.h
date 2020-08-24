#ifndef IMU_MDMF_H_
#define IMU_MCMF_H_

#include "./imu_base.h"

namespace MyFusion{

class IMU_MCMF : public IMU_BASE{
public:
    IMU_MCMF(ImuParam params);
    // ~IMU_MCMF(){};
    
    void oneStepIntegration();

}; 

}


#endif