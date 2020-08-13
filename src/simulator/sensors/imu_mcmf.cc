#include "simulator/sensors/imu_mcmf.h"

namespace myFusion{

void IMU_MCMF::midIntegration(){
    // 
    Vec3d gyr_mid = 0.5 * (gyr_0_ + gyr_1_) - gyr_bias_;

}
    
}