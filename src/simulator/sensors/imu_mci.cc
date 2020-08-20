#include "simulator/sensors/imu_mci.h"

namespace myFusion{

IMU_MCI::IMU_MCI(ImuParam params):IMU_BASE(params){
    
}

void IMU_MCI::oneStepIntegration(){
    double h_m = tnb_.norm(); // distance to center
    double alt = h_m - R_m;
    double lon = atan2(tnb_.y(), tnb_.x());    
    double lat = atan2(tnb_.z(), tnb_.x());

    // update quaternion
    Eigen::Quaterniond dq(1., 0.5 * gyr_0_.x() * time_step_, 0.5 * gyr_0_.y() * time_step_, 0.5 * gyr_0_.z() * time_step_);
    dq.normalize();

    Eigen::Quaterniond qnb0 = qnb_;
    qnb_ = qnb0 * dq;
    qnb_.normalize();
    
    // calculate geavity
    Eigen::Vector3d g_n(0., 0., 0.);    
    g_n.x() = -computeG(alt) * cos(lat);
    g_n.z() = -computeG(alt) * sin(lat);

    //update position and velocity 
    Eigen::Vector3d acc_m = qnb0 * acc_0_ - gyr_0_.cross(vel_) + g_n;
    tnb_ += vel_ * time_step_ + 0.5 * acc_m * time_step_ * time_step_;
    vel_ += acc_m * time_step_;
}

}