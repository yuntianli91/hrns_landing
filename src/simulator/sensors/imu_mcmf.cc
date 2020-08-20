#include "simulator/sensors/imu_mcmf.h"

namespace myFusion{

IMU_MCMF::IMU_MCMF(ImuParam params):IMU_BASE(params){
//     setParams(params);
}

void IMU_MCMF::oneStepIntegration(){
    double h_m = tnb_.norm(); // distance to center
    double alt = h_m - R_m;
    double lon = atan2(tnb_.y(), tnb_.x());    
    double lat = atan2(tnb_.z(), tnb_.x());
    pos_ = Eigen::Vector3d(lat, lon, alt);

    Eigen::Vector3d w_im(0., 0., W_im);

    // update quaternion
    Eigen::Vector3d w_mb = gyr_0_ - qnb_.conjugate() * w_im;
    Eigen::Quaterniond dq(1., 0.5 * w_mb.x() * time_step_, 0.5 * w_mb.y() * time_step_, 0.5 * w_mb.z() * time_step_);
    dq.normalize();

    Eigen::Quaterniond qnb0 = qnb_;
    qnb_ = qnb0 * dq;
    qnb_.normalize();
    
    // calculate geavity
    Eigen::Vector3d g_g(0., 0., 0.);    
    g_g.z() = computeG(alt);
    // compute qmg
    Eigen::AngleAxisd r_vec_1(lon, Eigen::Vector3d(0., 0., 1.)); // z
    Eigen::AngleAxisd r_vec_2(- lat, Eigen::Vector3d(0., 1., 0.)); // y
    Eigen::AngleAxisd r_vec_3(-0.5 * M_PI, Eigen::Vector3d(0., 1., 0.)); // x
    Eigen::Quaterniond qmg(r_vec_1 * r_vec_2 * r_vec_3);

    Eigen::Vector3d g_m = qmg * g_g;

    // double g_scale = computeG(alt);
    // Eigen::Vector3d g_m(0., 0., 0.);
    // g_m.x() = -g_scale * cos(lat) * cos(lon);
    // g_m.y() = -g_scale * cos(lat) * sin(lon);
    // g_m.z() = -g_scale * sin(lat);

    //update position and velocity 
    Eigen::Vector3d acc_m = qnb0 * acc_0_ - 2.0 * w_im.cross(vel_) + g_m;
    tnb_ += vel_ * time_step_ + 0.5 * acc_m * time_step_ * time_step_;
    vel_ += acc_m * time_step_;
}

    
}