#ifndef IMU_BASE_H_
#define IMU_BASE_H_

#include "commonHeaders.h"

using namespace std;

enum CELESTIAL{
    EARTH = 0,
    MOON = 1,
};

namespace myFusion{

/**
 * @brief Struct of IMU Parameters
 * 
 */
struct ImuParam{
    double acc_b_, gyr_b_; // bias
    double acc_n_, gyr_n_; // noise
    double acc_w_, gyr_w_; // random walk
    double time_step_; // time_step of imu
};

/**
 * @brief Struct of IMU Data
 * 
 */
struct ImuMotionData{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    double time_stamp_;  // current time stamp
    
    Eigen::Vector3d tnb_; // translation from b-frame to w-frame, e.g., posotion in w-frame;
    Eigen::Matrix3d Rnb_; // rotation transform from b-fram to w-frame
    Eigen::Quaterniond qnb_; // quaternion rotate w-frame to b-frame (q^w_b = R^w_b但前者表示w->q的旋转，后者表示b->w的坐标变换)

    Eigen::Vector3d acc_; // linear acceleration
    Eigen::Vector3d vel_; // linear velocity
    Eigen::Vector3d gyr_; // angular rate

    Eigen::Vector3d acc_bias_; // accelerometer bias
    Eigen::Vector3d gyr_bias_; // gyroscope bias

    Eigen::Vector3d pos_; // pos in local frame (used in geometric reference frame)
};

class IMU_BASE{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    
    IMU_BASE() = delete;
    IMU_BASE(ImuParam param);
    ~IMU_BASE(){}

    /**
     * @brief Set parameters of imu
     * 
     * @param param struct of imu parameters 
     */
    void setParams(ImuParam param);
    
    /**
     * @brief add noise to current measurement and update bias
     * 
     * @param data 
     */
    void addNoise();

    /**
     * @brief propagate for one step
     * 
     * @param data io, received ImuMotionData from trajectory generator 
     *          and return ImuMotionData from IMU simulator. 
     */
    void oneStepPropagate(ImuMotionData &data);

    double computeG(double h, CELESTIAL body=MOON);

    // /**
    //  * @brief trajectory generator
    //  * 
    //  * @param t 
    //  * @return IMUMotionData 
    //  */
    // virtual ImuMotionData motionModel(double t) = 0;

    virtual void oneStepIntegration() = 0;

    /**
     * @brief Set the Integration type
     * 
     * @param type 
     */
    void setIntType(int type){intType = type;}

protected:
    Vec3d acc_bias_, gyr_bias_; //bias 
    double acc_n_, gyr_n_; // noise
    double acc_w_, gyr_w_; // random walk
    // init state
    // Vec3d init_twb_, init_vel_;
    // Mat3d init_Rwb_;
    // current state
    Vec3d tnb_, vel_;
    Qd qnb_;
    Vec3d pos_; // pos in local frame(used in geometric frame)
    // measurements for mid_integration
    Vec3d acc_0_, gyr_0_; // last measurements
    Vec3d acc_1_, gyr_1_; // current measurements

    double time_step_; // sample step of IMU

    bool init_flag_ = false; // flag of initialization
    bool first_flag_ = true; // flag of first measurement

    int intType = 1; // 0- euler; 1-mid

};

}

#endif