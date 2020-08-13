#ifndef IMU_G_H_
#define IMU_G_H_

#include "./imu_base.h"

namespace myFusion{

class IMU_G:public IMU_BASE{
public:
    using IMU_BASE::IMU_BASE;

    void oneStepIntegration();

    /**
     * @brief generate trajectory date 
     * 
     * @param initPose : initiate pose
     * @param a_b_all : all accelerations in B frame 
     * @param omega_gb_all : all angular rates with respect to G frame in B frame
     * @return ImuMotionData 
     */
    vector<ImuMotionData> trajGenerator(ImuMotionData initPose, vector<Vec3d> a_b_all, vector<Vec3d> omega_gb_all);

    Eigen::Vector3d pos_n_; // pos in navigation frame

};

}


#endif