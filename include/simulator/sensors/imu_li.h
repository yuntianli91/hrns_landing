#ifndef IMU_LI_H_
#define IMU_LI_H_

#include "simulator/sensors/imu_base.h"

namespace MyFusion{

class IMU_LI:public IMU_BASE{
public:
    using IMU_BASE::IMU_BASE;

    void oneStepIntegration();

};

}

#endif