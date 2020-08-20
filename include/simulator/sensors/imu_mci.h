#ifndef IMU_MCI_H_
#define IMU_MCI_H_
#include "./imu_base.h"


namespace myFusion{

class IMU_MCI:public IMU_BASE{
public:
    IMU_MCI(ImuParam params);
    // ~IMU_MCI(){};
    void oneStepIntegration();

};

}
#endif