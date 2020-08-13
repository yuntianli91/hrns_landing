#ifndef IMU_MDMF_H_
#define IMU_MCMF_H_

#include "simulator/sensors/imu_base.h"

namespace myFusion{

class IMU_MCMF:public IMU_BASE{
public:
    using IMU_BASE::IMU_BASE;

    void midIntegration();

}; 

}


#endif