#include "constParams.h"
#include "simulator/sensors/imu_li.h"

namespace myFusion{

void IMU_LI::oneStepIntegration()
{
    // quaterniond update
    Eigen::Vector3d mid_gyr = 0.5 * (gyr_0_ + gyr_1_) - gyr_bias_;
        
    Eigen::Quaterniond qnb0 = qnb_;
    Eigen::Quaterniond dq(1., 0.5 * mid_gyr.x() * time_step_, 0.5 * mid_gyr.y() * time_step_, 0.5 * mid_gyr.z() * time_step_);
    dq.normalize();
        
    qnb_ = qnb0 * dq;
    // pos update
    Eigen::Vector3d gw = Vec3d::Zero(); // geavity vector
    gw.z() = -computeG(tnb_.z());
    // gw.z() = -1.622;

    Eigen::Vector3d acc_w = 0.5 * (qnb0 * (acc_0_ - acc_bias_) + qnb_ * (acc_1_ - acc_bias_)) + gw;
    tnb_ += vel_ * time_step_ + 0.5 * acc_w * time_step_ * time_step_; 
    // vel update
    vel_ += acc_w * time_step_;
}

}