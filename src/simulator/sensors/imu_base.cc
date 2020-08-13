#include "simulator/sensors/imu_base.h"

namespace myFusion{

IMU_BASE::IMU_BASE(ImuParam param){
    setParams(param);
}

void IMU_BASE::setParams(ImuParam param)
{
    acc_bias_ = Vec3d::Ones() * param.acc_b_; 
    gyr_bias_ = Vec3d::Ones() * param.gyr_b_;
    acc_n_ = param.acc_n_; gyr_n_ = param.gyr_n_;
    acc_w_ = param.acc_w_; gyr_w_ = param.gyr_w_;

    time_step_ = param.time_step_;
    
    init_flag_ = true;       
}

void IMU_BASE::oneStepPropagate(ImuMotionData &data){
    // 
    if(!init_flag_){
        std::cout << "Please initiate imu parameters first !\n";
        return;
    }

    // extract imu measurements and states
    acc_1_ = data.acc_; gyr_1_ = data.gyr_;
    // add noise to trajectory data  
    addNoise();
    // check if it's the first measurement
    if(first_flag_){
        tnb_ = data.tnb_; vel_ = data.vel_;
        qnb_ = data.qnb_;
        cout << "Integration type: " << intType << endl;
        first_flag_ = false;
    }
    else{
        oneStepIntegration();
    }
    // construct
    data.tnb_ = tnb_; data.vel_ = vel_; 
    data.qnb_ = qnb_; data.Rnb_ = qnb_.toRotationMatrix();
    
    data.acc_ = acc_1_; data.gyr_ = gyr_1_;
    data.acc_bias_ = acc_bias_; 
    data.gyr_bias_ = gyr_bias_;
    // update
    acc_0_ = acc_1_; gyr_0_ = gyr_1_; 
}

void IMU_BASE::addNoise(){
    if(!init_flag_){
        std::cout << "Please initiate imu parameters first !\n";
        return;
    }
    // generate standard white noise
    std::random_device rd; // generate seed for random eigine   
    std::default_random_engine rg(rd()); // create random eigine with seed rd();
    std::normal_distribution<double> noise(0.0, 1.0);

    // ================= add noise to measurements
    Eigen::Vector3d acc_noise(noise(rg), noise(rg), noise(rg));
    acc_noise *= acc_n_;
    // cout << "acc_noise times deviation: " << acc_noise << endl;
    acc_1_ += acc_noise / sqrt(time_step_)  + acc_bias_;

    Eigen::Vector3d gyr_noise(noise(rg), noise(rg), noise(rg));
    gyr_noise *= gyr_n_;
    gyr_1_ += gyr_noise / sqrt(time_step_)  + gyr_bias_;

    // ================= update bias with random walk
    Eigen::Vector3d acc_walk_noise(noise(rg), noise(rg), noise(rg));
    acc_walk_noise *= acc_w_;
    acc_bias_ += acc_walk_noise * sqrt(time_step_);

    Eigen::Vector3d gyr_walk_noise(noise(rg), noise(rg), noise(rg));
    gyr_walk_noise *= gyr_w_;
    gyr_bias_ += gyr_walk_noise * sqrt(time_step_);
}

double IMU_BASE::computeG(double h, CELESTIAL body){
    double scale = 0.0;
    
    switch (body)
    {
    case EARTH:
        scale = (1. + h / R_e);
        return (g0_e / (scale * scale));
        break;
    case MOON:
        scale = (1. + h / R_m);
        return (g0_m / (scale * scale));
        break;
    default:
        cout << "Wrong celestial type!\n";
        break;
    }
}

}